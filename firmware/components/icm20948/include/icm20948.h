/**
 * @file icm20948.h
 * @brief TDK InvenSense ICM-20948 9-axis IMU driver (I2C) for ESP-IDF
 *
 * Works with the Adafruit ICM-20948 breakout (default I2C address 0x69).
 *
 * Provides raw accelerometer + gyroscope data. The magnetometer (AK09916)
 * is accessible via an internal I2C master pass-through — not used here for
 * simplicity (can be added later).
 *
 * Register layout uses banked addressing: REG_BANK_SEL selects which of 4
 * banks the other registers map into.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// I2C addresses (Adafruit breakout default is 0x69)
#define ICM20948_I2C_ADDR_LOW   0x68
#define ICM20948_I2C_ADDR_HIGH  0x69
#define ICM20948_I2C_ADDR       ICM20948_I2C_ADDR_HIGH

// WHO_AM_I expected value
#define ICM20948_WHOAMI         0xEA

// Accelerometer full-scale range
typedef enum {
    ICM20948_ACCEL_2G  = 0,
    ICM20948_ACCEL_4G  = 1,
    ICM20948_ACCEL_8G  = 2,
    ICM20948_ACCEL_16G = 3,
} icm20948_accel_range_t;

// Gyroscope full-scale range
typedef enum {
    ICM20948_GYRO_250DPS  = 0,
    ICM20948_GYRO_500DPS  = 1,
    ICM20948_GYRO_1000DPS = 2,
    ICM20948_GYRO_2000DPS = 3,
} icm20948_gyro_range_t;

/** IMU sample (raw ADC counts) */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
} icm20948_raw_t;

/** Driver handle */
typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    uint8_t addr;
    uint8_t current_bank;
    icm20948_accel_range_t accel_range;
    icm20948_gyro_range_t  gyro_range;
    bool initialized;
} icm20948_t;

/**
 * @brief Initialize ICM-20948 on I2C bus
 *
 * Performs soft reset, wakes from sleep, verifies WHO_AM_I, and applies
 * default ranges (±2g accel, ±250 dps gyro).
 *
 * @param dev Driver handle (caller-allocated)
 * @param bus I2C master bus handle (already initialized)
 * @param addr I2C address (0x68 or 0x69)
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if device not detected
 */
esp_err_t icm20948_init(icm20948_t *dev, i2c_master_bus_handle_t bus, uint8_t addr);

/** Deinitialize and release I2C device resource */
void icm20948_deinit(icm20948_t *dev);

/** Read WHO_AM_I register (should return 0xEA) */
uint8_t icm20948_whoami(icm20948_t *dev);

/** Configure accelerometer and gyroscope full-scale ranges */
esp_err_t icm20948_set_ranges(icm20948_t *dev,
                              icm20948_accel_range_t accel_range,
                              icm20948_gyro_range_t gyro_range);

/**
 * @brief Read raw accel + gyro + temperature (14 bytes, one I2C transaction)
 */
esp_err_t icm20948_read_raw(icm20948_t *dev, icm20948_raw_t *out);

/** Convert raw accel to g (depends on configured range) */
float icm20948_accel_to_g(icm20948_t *dev, int16_t raw);

/** Convert raw gyro to deg/s (depends on configured range) */
float icm20948_gyro_to_dps(icm20948_t *dev, int16_t raw);

#ifdef __cplusplus
}
#endif
