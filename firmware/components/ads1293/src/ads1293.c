/**
 * @file ads1293.c
 * @brief TI ADS1293 SPI driver implementation for ESP-IDF
 *
 * Configuration logic ported from CleverHand-interface:
 *   clvHd_module_ADS1293EMG.hpp / clvHd_module_ADS1293EMG.cpp
 */

#include "ads1293.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "ads1293";

// =============================================================================
// Low-level SPI register access
// =============================================================================

uint8_t ads1293_read_reg(ads1293_t *dev, uint8_t reg) {
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), 0x00 };
    uint8_t rx[2] = { 0 };
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_polling_transmit(dev->spi, &t);
    return rx[1];
}

void ads1293_write_reg(ads1293_t *dev, uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), val };
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
    };
    spi_device_polling_transmit(dev->spi, &t);
}

void ads1293_read_regs(ads1293_t *dev, uint8_t start_reg, uint8_t *buf, int count) {
    // First byte: register address with read bit, followed by count dummy bytes
    uint8_t tx[count + 1];
    uint8_t rx[count + 1];
    memset(tx, 0, sizeof(tx));
    tx[0] = start_reg | 0x80;

    spi_transaction_t t = {
        .length = (uint32_t)((count + 1) * 8),
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_polling_transmit(dev->spi, &t);
    memcpy(buf, &rx[1], count);
}

// =============================================================================
// Initialization
// =============================================================================

esp_err_t ads1293_init(ads1293_t *dev, const ads1293_hw_config_t *hw) {
    memset(dev, 0, sizeof(ads1293_t));
    dev->hw = *hw;

    // Initialize SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = hw->gpio_mosi,
        .miso_io_num = hw->gpio_miso,
        .sclk_io_num = hw->gpio_sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };

    esp_err_t ret = spi_bus_initialize(hw->spi_host, &buscfg, SPI_DMA_DISABLED);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE means bus already initialized (shared bus)
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_device_interface_config_t devcfg = {
        .mode = 0,                          // SPI Mode 0 (CPOL=0 CPHA=0)
        .clock_speed_hz = hw->clock_hz,
        .spics_io_num = hw->gpio_cs,
        .queue_size = 1,
    };

    ret = spi_bus_add_device(hw->spi_host, &devcfg, &dev->spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set default ADC max values
    dev->fast_adc_max = 0x8000;
    for (int i = 0; i < 3; i++) {
        dev->precise_adc_max[i] = 0x800000;
    }

    // Give the chip time to come out of power-on reset. On cold boot the
    // ADS1293 may not respond to SPI for a few ms.
    vTaskDelay(pdMS_TO_TICKS(50));

    // Verify device identity — retry for up to ~500 ms in case the chip is slow
    // to come out of reset. 0xFF means MISO is floating (no response yet).
    uint8_t revid = 0xFF;
    for (int attempt = 0; attempt < 50; attempt++) {
        revid = ads1293_read_revid(dev);
        if (revid == ADS1293_EXPECTED_REVID) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (revid != ADS1293_EXPECTED_REVID) {
        ESP_LOGE(TAG, "REVID mismatch after 50 retries: got 0x%02X, expected 0x%02X",
                 revid, ADS1293_EXPECTED_REVID);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "ADS1293 detected (REVID=0x%02X)", revid);
    dev->initialized = true;
    return ESP_OK;
}

void ads1293_deinit(ads1293_t *dev) {
    if (dev->spi) {
        ads1293_stop(dev);
        spi_bus_remove_device(dev->spi);
        dev->spi = NULL;
    }
    dev->initialized = false;
}

uint8_t ads1293_read_revid(ads1293_t *dev) {
    return ads1293_read_reg(dev, ADS1293_REVID_REG);
}

// =============================================================================
// Internal: R2/R3 encoding helpers
// =============================================================================

static uint8_t encode_r2(int r2) {
    switch (r2) {
        case 4:  return 0x01;
        case 5:  return 0x02;
        case 6:  return 0x04;
        case 8:  return 0x08;
        default: return 0x01;   // default to 4
    }
}

static uint8_t encode_r3(int r3) {
    switch (r3) {
        case 4:   return 0x01;
        case 6:   return 0x02;
        case 8:   return 0x04;
        case 12:  return 0x08;
        case 16:  return 0x10;
        case 32:  return 0x20;
        case 64:  return 0x40;
        case 128: return 0x80;
        default:  return 0x01;  // default to 4
    }
}

// =============================================================================
// ADC max computation (ported from update_adc_max)
// =============================================================================

static void update_adc_max(ads1293_t *dev, const ads1293_filter_config_t *f) {
    for (int i = 0; i < 3; i++) {
        switch (f->R2) {
            case 4:
                dev->fast_adc_max = 0x8000;
                dev->precise_adc_max[i] = 0x800000;
                if (f->R3[i] == 6 || f->R3[i] == 12)
                    dev->precise_adc_max[i] = 0xF30000;
                break;
            case 5:
                dev->fast_adc_max = 0xC350;
                dev->precise_adc_max[i] = 0xC35000;
                if (f->R3[i] == 8 || f->R3[i] == 16)
                    dev->precise_adc_max[i] = 0xB964F0;
                break;
            case 6:
                dev->fast_adc_max = 0xF300;
                dev->precise_adc_max[i] = 0xF30000;
                if (f->R3[i] == 8 || f->R3[i] == 16)
                    dev->precise_adc_max[i] = 0xE6A900;
                break;
            case 8:
                dev->fast_adc_max = 0x8000;
                dev->precise_adc_max[i] = 0x800000;
                if (f->R3[i] == 6 || f->R3[i] == 12)
                    dev->precise_adc_max[i] = 0xF30000;
                break;
        }
    }
}

// =============================================================================
// Configuration
// =============================================================================

esp_err_t ads1293_configure(ads1293_t *dev, const ads1293_config_t *cfg) {
    if (!dev->initialized) return ESP_ERR_INVALID_STATE;

    // 1. Enter power-down mode
    ads1293_write_reg(dev, ADS1293_CONFIG_REG, ADS1293_MODE_POWER_DOWN);

    // 2. Configure clock (stop, select source, no output)
    uint8_t osc_val = cfg->use_internal_clock ? 0x00 : 0x02;  // bit1 = extern
    ads1293_write_reg(dev, ADS1293_OSC_CN_REG, osc_val);

    // 3. Route channels
    for (int ch = 0; ch < 3; ch++) {
        uint8_t pos = cfg->channels[ch].pos_input & 0x07;
        uint8_t neg = cfg->channels[ch].neg_input & 0x07;
        uint8_t val = pos | (neg << 3);
        if (pos == neg) val |= 0xC0;
        ads1293_write_reg(dev, ADS1293_FLEX_CH0_CN_REG + ch, val);
    }

    // 4. Enable/shutdown INA+SDM+ADC per channel
    //    AFE_SHDN_CN bits: [0]=INA0, [1]=INA1, [2]=INA2, [3]=SDM0, [4]=SDM1, [5]=SDM2
    //    0 = enabled, 1 = shutdown
    uint8_t shdn = 0;
    shdn |= (cfg->channels[0].enabled ? 0 : 0x09);  // bits 0,3
    shdn |= (cfg->channels[1].enabled ? 0 : 0x12);  // bits 1,4
    shdn |= (cfg->channels[2].enabled ? 0 : 0x24);  // bits 2,5
    ads1293_write_reg(dev, ADS1293_AFE_SHDN_CN_REG, shdn);

    // 5. Configure resolution (bits 0-2) and frequency (bits 3-5) in AFE_RES_REG
    uint8_t afe_res = 0;
    for (int ch = 0; ch < 3; ch++) {
        if (cfg->channels[ch].high_res)  afe_res |= (1 << ch);
        if (cfg->channels[ch].high_freq) afe_res |= (1 << (ch + 3));
    }
    ads1293_write_reg(dev, ADS1293_AFE_RES_REG, afe_res);

    // 6. Set decimation filters
    // R1: one bit per channel (1=R1 of 2, 0=R1 of 4)
    uint8_t r1_val = 0;
    for (int ch = 0; ch < 3; ch++) {
        if (cfg->filters.R1[ch] == 2) r1_val |= (1 << ch);
    }
    ads1293_write_reg(dev, ADS1293_R1_RATE_REG, r1_val);

    // R2: one-hot global
    ads1293_write_reg(dev, ADS1293_R2_RATE_REG, encode_r2(cfg->filters.R2));

    // R3: one-hot per channel
    ads1293_write_reg(dev, ADS1293_R3_RATE_CH0_REG, encode_r3(cfg->filters.R3[0]));
    ads1293_write_reg(dev, ADS1293_R3_RATE_CH1_REG, encode_r3(cfg->filters.R3[1]));
    ads1293_write_reg(dev, ADS1293_R3_RATE_CH2_REG, encode_r3(cfg->filters.R3[2]));

    // 7. Right-Leg Drive routing (RLD_CN, 0x0C)
    //    [6] RLD_BW, [5:4] RLD_CAPDRIVE, [3] SHDN_RLD, [2:0] SELRLD
    uint8_t rld_val = 0;
    if (cfg->rld.bw_high)                 rld_val |= (1 << 6);
    rld_val |= (cfg->rld.cap_drive & 0x3) << 4;
    if (cfg->rld.route == 0 || cfg->rld.route > 6) {
        rld_val |= (1 << 3);              // SHDN_RLD = 1 (powered down)
    } else {
        rld_val |= (cfg->rld.route & 0x07);   // SELRLD = INx, SHDN_RLD = 0
    }
    ads1293_write_reg(dev, ADS1293_RLD_CN_REG, rld_val);

    // 8. Start clock
    osc_val |= 0x04;  // bit2 = start
    ads1293_write_reg(dev, ADS1293_OSC_CN_REG, osc_val);

    // 9. Update ADC max values for conversion math
    update_adc_max(dev, &cfg->filters);

    ESP_LOGI(TAG, "Configured: ch=[%d,%d,%d] R1=[%d,%d,%d] R2=%d R3=[%d,%d,%d] rate=%.0f Hz  RLD=%s",
             cfg->channels[0].enabled, cfg->channels[1].enabled, cfg->channels[2].enabled,
             cfg->filters.R1[0], cfg->filters.R1[1], cfg->filters.R1[2],
             cfg->filters.R2,
             cfg->filters.R3[0], cfg->filters.R3[1], cfg->filters.R3[2],
             ads1293_get_sample_rate(cfg, 0),
             (cfg->rld.route == 0 || cfg->rld.route > 6)
                 ? "off"
                 : (cfg->rld.route == 1 ? "IN1" :
                    cfg->rld.route == 2 ? "IN2" :
                    cfg->rld.route == 3 ? "IN3" :
                    cfg->rld.route == 4 ? "IN4" :
                    cfg->rld.route == 5 ? "IN5" : "IN6"));

    return ESP_OK;
}

// =============================================================================
// Start / Stop
// =============================================================================

esp_err_t ads1293_start(ads1293_t *dev) {
    if (!dev->initialized) return ESP_ERR_INVALID_STATE;
    ads1293_write_reg(dev, ADS1293_CONFIG_REG, ADS1293_MODE_START_CONV);
    ESP_LOGI(TAG, "Conversion started");
    return ESP_OK;
}

esp_err_t ads1293_stop(ads1293_t *dev) {
    if (!dev->initialized) return ESP_ERR_INVALID_STATE;
    ads1293_write_reg(dev, ADS1293_CONFIG_REG, ADS1293_MODE_STANDBY);
    ESP_LOGI(TAG, "Conversion stopped");
    return ESP_OK;
}

// =============================================================================
// Data Reading
// =============================================================================

esp_err_t ads1293_read_data(ads1293_t *dev, ads1293_sample_t *sample) {
    // Bulk read 16 bytes starting from DATA_STATUS_REG (0x30)
    // Layout:
    //   [0]    DATA_STATUS
    //   [1:2]  CH0 fast (16-bit BE)
    //   [3:4]  CH1 fast (16-bit BE)
    //   [5:6]  CH2 fast (16-bit BE)
    //   [7:9]  CH0 precise (24-bit BE)
    //   [10:12] CH1 precise (24-bit BE)
    //   [13:15] CH2 precise (24-bit BE)
    uint8_t raw[ADS1293_DATA_BLOCK_SIZE];
    ads1293_read_regs(dev, ADS1293_DATA_STATUS_REG, raw, ADS1293_DATA_BLOCK_SIZE);

    sample->status = raw[0];

    // Parse fast values (16-bit big-endian)
    for (int ch = 0; ch < 3; ch++) {
        int offset = 1 + ch * 2;
        sample->fast[ch] = (int16_t)((raw[offset] << 8) | raw[offset + 1]);
    }

    // Parse precise values (24-bit big-endian, sign-extend to int32)
    for (int ch = 0; ch < 3; ch++) {
        int offset = 7 + ch * 3;
        int32_t val = ((int32_t)raw[offset] << 16) |
                      ((int32_t)raw[offset + 1] << 8) |
                      ((int32_t)raw[offset + 2]);
        // Sign-extend from 24-bit to 32-bit
        if (val & 0x800000) val |= 0xFF000000;
        sample->precise[ch] = val;
    }

    return ESP_OK;
}

bool ads1293_data_ready(ads1293_t *dev, uint8_t *status) {
    uint8_t s = ads1293_read_reg(dev, ADS1293_DATA_STATUS_REG);
    if (status) *status = s;
    // Bits 2-4: fast data ready (ch0,ch1,ch2)
    // Bits 5-7: precise data ready (ch0,ch1,ch2)
    return (s & 0xFC) != 0;
}

// =============================================================================
// Sample Rate Calculation
// =============================================================================

float ads1293_get_sample_rate(const ads1293_config_t *cfg, int channel) {
    if (channel < 0 || channel > 2) return 0.0f;
    float base_freq = cfg->channels[channel].high_freq ? 204800.0f : 102400.0f;
    float r1 = (float)cfg->filters.R1[channel];
    float r2 = (float)cfg->filters.R2;
    float r3 = (float)cfg->filters.R3[channel];
    return base_freq / (r1 * r2 * r3);
}
