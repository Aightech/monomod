/**
 * @file ina_emg.h
 * @brief Single-channel analog EMG driver for ESP32-C3 using internal ADC.
 *
 * Used by the INA331+LIS3DH submodule where the INA output is an analog
 * voltage on GPIO2 (D0). We read it via the ESP32-C3's ADC1 in continuous
 * DMA mode.
 *
 *   Sample rate target: 3200 Hz (matches ADS1293 default for consistency)
 *   Resolution:         12-bit (sign-extended to int16 for wire format)
 *   Channel:            ADC1_CH2 (GPIO2 = D0)
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "hal/adc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define INA_EMG_ADC_CHANNEL     ADC_CHANNEL_2     // GPIO2 on ESP32-C3
#define INA_EMG_DEFAULT_RATE_HZ 3200
#define INA_EMG_BUF_SAMPLES     512               // DMA pool depth

typedef struct {
    void *handle;                 // adc_continuous_handle_t
    uint32_t sample_rate_hz;
    bool running;
} ina_emg_t;

/**
 * @brief Initialize ADC in continuous mode at the given sample rate.
 */
esp_err_t ina_emg_init(ina_emg_t *dev, uint32_t sample_rate_hz);

/**
 * @brief Start continuous conversion. Samples accumulate in DMA buffer.
 */
esp_err_t ina_emg_start(ina_emg_t *dev);

/**
 * @brief Stop conversion and flush DMA.
 */
esp_err_t ina_emg_stop(ina_emg_t *dev);

/**
 * @brief Deinitialize driver.
 */
void ina_emg_deinit(ina_emg_t *dev);

/**
 * @brief Drain available samples from the DMA buffer.
 *
 * @param dev      Driver handle
 * @param samples  Output buffer for int16 samples
 * @param max      Maximum samples to read
 * @return Number of samples actually read (0 if none available)
 */
int ina_emg_read_samples(ina_emg_t *dev, int16_t *samples, int max);

#ifdef __cplusplus
}
#endif
