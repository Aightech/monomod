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

// Auxiliary ADC (3 inputs; ADC1 = pin used for the amplified EMG signal).
// 10-bit, left-justified two's complement, output rate == accelerometer ODR.
#define LIS3DH_REG_OUT_ADC1_L   0x08
#define LIS3DH_REG_TEMP_CFG     0x1F   // bit7 ADC_EN, bit6 TEMP_EN

// CTRL_REG1 ODR codes (bits [7:4])
#define LIS3DH_ODR_100HZ        0x5
#define LIS3DH_ODR_200HZ        0x6
#define LIS3DH_ODR_400HZ        0x7
#define LIS3DH_ODR_1344HZ       0x9    // 1.344 kHz in normal mode

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

/**
 * @brief Set the output data rate (CTRL_REG1 ODR field), keeping X/Y/Z enabled,
 * normal (high-res capable) mode. Use e.g. LIS3DH_ODR_1344HZ for EMG.
 */
esp_err_t lis3dh_set_odr(lis3dh_t *dev, uint8_t odr_code);

/**
 * @brief Enable the auxiliary ADC (ADC_EN in TEMP_CFG_REG). Required before
 * reading ADC1. The device must already be out of power-down (ODR != 0).
 */
esp_err_t lis3dh_enable_adc(lis3dh_t *dev);

/**
 * @brief Read auxiliary input ADC1 (raw int16, 10-bit left-justified two's
 * complement). On this board ADC1 carries the amplified EMG signal.
 */
esp_err_t lis3dh_read_adc1(lis3dh_t *dev, int16_t *out);

#ifdef __cplusplus
}
#endif
