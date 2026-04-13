/**
 * @file ina_emg.c
 * @brief ADC continuous-mode driver for INA analog EMG input.
 */

#include "ina_emg.h"
#include "esp_adc/adc_continuous.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ina_emg";

// We'll convert raw 12-bit ADC counts to int16 centered around 0.
// ADC returns 0..4095; midscale is 2048. Output = raw - 2048, then shifted
// left by 4 to fill the int16 range. That gives roughly -32768..+32767.
#define ADC_BITS            12
#define ADC_MIDSCALE        2048

static int16_t convert_sample(uint32_t raw12) {
    int32_t centered = (int32_t)(raw12 & 0x0FFF) - ADC_MIDSCALE;
    centered <<= 4;  // 12-bit → 16-bit sign-extended
    if (centered >  32767) centered =  32767;
    if (centered < -32768) centered = -32768;
    return (int16_t)centered;
}

esp_err_t ina_emg_init(ina_emg_t *dev, uint32_t sample_rate_hz) {
    memset(dev, 0, sizeof(*dev));
    dev->sample_rate_hz = sample_rate_hz ? sample_rate_hz : INA_EMG_DEFAULT_RATE_HZ;

    adc_continuous_handle_cfg_t pool_cfg = {
        .max_store_buf_size = INA_EMG_BUF_SAMPLES * sizeof(adc_digi_output_data_t) * 2,
        .conv_frame_size    = INA_EMG_BUF_SAMPLES * sizeof(adc_digi_output_data_t),
    };
    adc_continuous_handle_t h = NULL;
    esp_err_t ret = adc_continuous_new_handle(&pool_cfg, &h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "adc_continuous_new_handle failed: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_digi_pattern_config_t pattern = {
        .atten      = ADC_ATTEN_DB_12,          // full-scale ~3.3 V
        .channel    = INA_EMG_ADC_CHANNEL,
        .unit       = ADC_UNIT_1,
        .bit_width  = ADC_BITWIDTH_12,
    };
    adc_continuous_config_t cfg = {
        .sample_freq_hz = dev->sample_rate_hz,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
        .pattern_num    = 1,
        .adc_pattern    = &pattern,
    };

    ret = adc_continuous_config(h, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "adc_continuous_config failed: %s", esp_err_to_name(ret));
        adc_continuous_deinit(h);
        return ret;
    }

    dev->handle = h;
    ESP_LOGI(TAG, "Initialized on ADC1 CH%d (GPIO2) @ %lu Hz",
             INA_EMG_ADC_CHANNEL, dev->sample_rate_hz);
    return ESP_OK;
}

esp_err_t ina_emg_start(ina_emg_t *dev) {
    if (!dev->handle) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = adc_continuous_start((adc_continuous_handle_t)dev->handle);
    if (ret == ESP_OK) {
        dev->running = true;
        ESP_LOGI(TAG, "Conversion started");
    }
    return ret;
}

esp_err_t ina_emg_stop(ina_emg_t *dev) {
    if (!dev->handle) return ESP_ERR_INVALID_STATE;
    adc_continuous_stop((adc_continuous_handle_t)dev->handle);
    dev->running = false;
    ESP_LOGI(TAG, "Conversion stopped");
    return ESP_OK;
}

void ina_emg_deinit(ina_emg_t *dev) {
    if (dev->handle) {
        if (dev->running) ina_emg_stop(dev);
        adc_continuous_deinit((adc_continuous_handle_t)dev->handle);
        dev->handle = NULL;
    }
}

int ina_emg_read_samples(ina_emg_t *dev, int16_t *out, int max) {
    if (!dev->handle || !dev->running) return 0;

    // Stack buffer to receive one DMA chunk
    static adc_digi_output_data_t buf[INA_EMG_BUF_SAMPLES];
    uint32_t bytes_read = 0;
    uint32_t want_bytes = (uint32_t)max * sizeof(adc_digi_output_data_t);
    if (want_bytes > sizeof(buf)) want_bytes = sizeof(buf);

    esp_err_t ret = adc_continuous_read(
        (adc_continuous_handle_t)dev->handle,
        (uint8_t *)buf, want_bytes, &bytes_read, 0 /* non-blocking */);
    if (ret != ESP_OK || bytes_read == 0) return 0;

    int n = bytes_read / sizeof(adc_digi_output_data_t);
    if (n > max) n = max;
    for (int i = 0; i < n; i++) {
        // On ESP32-C3, adc_digi_output_data_t.type2.data holds the 12-bit value
        out[i] = convert_sample(buf[i].type2.data);
    }
    return n;
}
