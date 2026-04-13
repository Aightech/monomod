/**
 * @file lis3dh.h
 * @brief STMicro LIS3DH 3-axis accelerometer over SPI.
 *
 * Shares the SPI bus with ADS1293 (same CS pin). Minimal driver used for
 *   - module detection at boot (read WHO_AM_I, expect 0x33)
 *   - optional live accel readings (10 Hz, ±2g)
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LIS3DH_WHOAMI       0x33
#define LIS3DH_REG_WHOAMI   0x0F

typedef struct {
    spi_device_handle_t spi;
    bool initialized;
} lis3dh_t;

typedef struct {
    int gpio_mosi, gpio_miso, gpio_sclk, gpio_cs;
    int clock_hz;
    spi_host_device_t spi_host;
} lis3dh_hw_config_t;

/**
 * @brief Probe for LIS3DH on the given SPI bus. On success the device
 * handle is left registered on the bus for subsequent reads.
 *
 * @return ESP_OK if WHO_AM_I reads 0x33
 */
esp_err_t lis3dh_init(lis3dh_t *dev, const lis3dh_hw_config_t *hw);

/**
 * @brief Release the SPI device handle.
 */
void lis3dh_deinit(lis3dh_t *dev);

/**
 * @brief Read WHO_AM_I register (for re-verification / diagnostics).
 */
uint8_t lis3dh_read_whoami(lis3dh_t *dev);

/**
 * @brief Enable the accelerometer: 100 Hz, ±2g, X/Y/Z all enabled.
 */
esp_err_t lis3dh_enable(lis3dh_t *dev);

/**
 * @brief Read latest accelerometer sample (raw int16, LSB/16384 ≈ 1 g).
 */
esp_err_t lis3dh_read_accel(lis3dh_t *dev, int16_t *x, int16_t *y, int16_t *z);

#ifdef __cplusplus
}
#endif
