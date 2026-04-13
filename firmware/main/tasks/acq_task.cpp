#include "acq_task.hpp"
#include "config.hpp"
#include "timestamp.hpp"
#include "packet_types.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "acq";

struct AcqContext {
    ads1293_t *adc;
    MonomodRingBuffer *ring;
    SemaphoreHandle_t tx_sem;
    esp_timer_handle_t timer;
    uint32_t sequence;
    volatile bool running;
};

static AcqContext s_ctx;

static void IRAM_ATTR acq_timer_cb(void *arg) {
    AcqContext *ctx = (AcqContext *)arg;
    if (!ctx->running) return;

    ads1293_sample_t sample;
    if (ads1293_read_data(ctx->adc, &sample) != ESP_OK) return;

    // Forward every read — the ADS1293 status bit can fail to set with certain
    // clock/filter configurations, but the data registers still contain valid
    // samples. If no new conversion happened, the Python host sees duplicate
    // values (easy to spot). This is less aggressive than gating on status.
    SampleFrame frame = {};
    frame.timestamp_us = axon::get_timestamp_us();
    frame.sequence = ctx->sequence++;
    frame.num_channels = ADS1293_NUM_CHANNELS;
    frame.adc_type = AXON_ADC_TYPE_ADS1293;
    frame.samples[0] = sample.precise[0];
    frame.samples[1] = sample.precise[1];
    frame.samples[2] = sample.precise[2];

    ctx->ring->push_overwrite(frame);
    xSemaphoreGiveFromISR(ctx->tx_sem, NULL);
}

void acq_task_start(ads1293_t *adc, MonomodRingBuffer *ring,
                    SemaphoreHandle_t tx_sem, TaskHandle_t *handle) {
    s_ctx.adc = adc;
    s_ctx.ring = ring;
    s_ctx.tx_sem = tx_sem;
    s_ctx.sequence = 0;
    s_ctx.running = true;

    // Compute timer interval from default config
    // Default: 102400 / (2*4*4) = 3200 Hz → 312.5 µs
    uint64_t interval_us = 312;  // ~3200 Hz

    esp_timer_create_args_t timer_args = {
        .callback = acq_timer_cb,
        .arg = &s_ctx,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "acq_poll",
    };
    esp_timer_create(&timer_args, &s_ctx.timer);
    esp_timer_start_periodic(s_ctx.timer, interval_us);

    ESP_LOGI(TAG, "ACQ started: poll every %llu us", interval_us);
    *handle = NULL;  // Timer-based, no FreeRTOS task handle
}

void acq_task_stop(TaskHandle_t handle) {
    (void)handle;
    s_ctx.running = false;
    if (s_ctx.timer) {
        esp_timer_stop(s_ctx.timer);
        esp_timer_delete(s_ctx.timer);
        s_ctx.timer = NULL;
    }
    ESP_LOGI(TAG, "ACQ stopped. Frames: %lu", s_ctx.sequence);
}

// =============================================================================
// INA (analog EMG) acquisition task
// =============================================================================
struct INAContext {
    ina_emg_t *ina;
    MonomodRingBuffer *ring;
    SemaphoreHandle_t tx_sem;
    uint32_t sequence;
    volatile bool running;
};
static INAContext s_ina_ctx;

static void ina_task_func(void *arg) {
    INAContext *ctx = (INAContext *)arg;
    const int CHUNK = 64;
    int16_t buf[CHUNK];
    while (ctx->running) {
        int n = ina_emg_read_samples(ctx->ina, buf, CHUNK);
        if (n <= 0) {
            vTaskDelay(pdMS_TO_TICKS(5));  // nothing ready yet
            continue;
        }
        for (int i = 0; i < n; i++) {
            SampleFrame frame = {};
            frame.timestamp_us = axon::get_timestamp_us();
            frame.sequence = ctx->sequence++;
            frame.num_channels = AXON_INA_NUM_CHANNELS;
            frame.adc_type = AXON_ADC_TYPE_INA;
            frame.samples[0] = (int32_t)buf[i];   // only ch0
            ctx->ring->push_overwrite(frame);
        }
        xSemaphoreGive(ctx->tx_sem);
    }
    ESP_LOGI(TAG, "INA ACQ task exiting (frames: %lu)", ctx->sequence);
    vTaskDelete(NULL);
}

void acq_task_start_ina(ina_emg_t *ina, MonomodRingBuffer *ring,
                        SemaphoreHandle_t tx_sem, TaskHandle_t *handle) {
    s_ina_ctx.ina = ina;
    s_ina_ctx.ring = ring;
    s_ina_ctx.tx_sem = tx_sem;
    s_ina_ctx.sequence = 0;
    s_ina_ctx.running = true;
    xTaskCreate(ina_task_func, "acq_ina", 4096, &s_ina_ctx, 22, handle);
    ESP_LOGI(TAG, "ACQ (INA) started");
}

void acq_task_stop_ina(TaskHandle_t handle) {
    (void)handle;
    s_ina_ctx.running = false;
    vTaskDelay(pdMS_TO_TICKS(50));  // let it exit
    ESP_LOGI(TAG, "ACQ (INA) stopped");
}
