/**
 * @file icm20948.c
 * @brief TDK ICM-20948 driver implementation
 */

#include "icm20948.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "icm20948";

// =============================================================================
// Register addresses (banked)
// =============================================================================

// Common to all banks
#define REG_BANK_SEL        0x7F

// User Bank 0
#define WHO_AM_I            0x00
#define USER_CTRL           0x03
#define PWR_MGMT_1          0x06
#define PWR_MGMT_2          0x07
#define INT_PIN_CFG         0x0F
#define ACCEL_XOUT_H        0x2D
#define GYRO_XOUT_H         0x33
#define TEMP_OUT_H          0x39

// User Bank 2
#define GYRO_CONFIG_1       0x01
#define GYRO_SMPLRT_DIV     0x00
#define ACCEL_CONFIG        0x14
#define ACCEL_SMPLRT_DIV_2  0x11

// =============================================================================
// Low-level I2C helpers
// =============================================================================

static esp_err_t write_reg(icm20948_t *dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev->dev, buf, 2, 100);
}

static esp_err_t read_reg(icm20948_t *dev, uint8_t reg, uint8_t *val) {
    return i2c_master_transmit_receive(dev->dev, &reg, 1, val, 1, 100);
}

static esp_err_t read_regs(icm20948_t *dev, uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_master_transmit_receive(dev->dev, &reg, 1, buf, len, 100);
}

static esp_err_t select_bank(icm20948_t *dev, uint8_t bank) {
    if (dev->current_bank == bank) return ESP_OK;
    esp_err_t ret = write_reg(dev, REG_BANK_SEL, bank << 4);
    if (ret == ESP_OK) dev->current_bank = bank;
    return ret;
}

// =============================================================================
// Initialization
// =============================================================================

esp_err_t icm20948_init(icm20948_t *dev, i2c_master_bus_handle_t bus, uint8_t addr) {
    memset(dev, 0, sizeof(icm20948_t));
    dev->bus = bus;
    dev->addr = addr;
    dev->current_bank = 0xFF;  // force first select

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400000,
    };
    esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &dev->dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Select Bank 0
    ret = select_bank(dev, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bank select failed — device not responding");
        return ESP_ERR_NOT_FOUND;
    }

    // Verify WHO_AM_I
    uint8_t whoami = 0;
    ret = read_reg(dev, WHO_AM_I, &whoami);
    if (ret != ESP_OK || whoami != ICM20948_WHOAMI) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: got 0x%02X, expected 0x%02X",
                 whoami, ICM20948_WHOAMI);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "ICM-20948 detected (WHO_AM_I=0x%02X) at I2C 0x%02X", whoami, addr);

    // Soft reset
    write_reg(dev, PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(50));
    dev->current_bank = 0xFF;  // reset restored bank 0
    select_bank(dev, 0);

    // Wake up (auto-select best clock source)
    write_reg(dev, PWR_MGMT_1, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Enable both accel and gyro (PWR_MGMT_2 = 0x00 enables all)
    write_reg(dev, PWR_MGMT_2, 0x00);

    // Set default ranges: ±2g accel, ±250 dps gyro
    ret = icm20948_set_ranges(dev, ICM20948_ACCEL_2G, ICM20948_GYRO_250DPS);
    if (ret != ESP_OK) return ret;

    dev->initialized = true;
    return ESP_OK;
}

void icm20948_deinit(icm20948_t *dev) {
    if (!dev->initialized) return;
    if (dev->dev) {
        i2c_master_bus_rm_device(dev->dev);
        dev->dev = NULL;
    }
    dev->initialized = false;
}

uint8_t icm20948_whoami(icm20948_t *dev) {
    select_bank(dev, 0);
    uint8_t v = 0;
    read_reg(dev, WHO_AM_I, &v);
    return v;
}

// =============================================================================
// Configuration
// =============================================================================

esp_err_t icm20948_set_ranges(icm20948_t *dev,
                              icm20948_accel_range_t accel_range,
                              icm20948_gyro_range_t gyro_range) {
    esp_err_t ret = select_bank(dev, 2);
    if (ret != ESP_OK) return ret;

    // ACCEL_CONFIG bits [2:1] = full-scale select, bit [0] = DLPF enable
    uint8_t accel_cfg = ((uint8_t)accel_range << 1) | 0x01;  // enable DLPF
    ret = write_reg(dev, ACCEL_CONFIG, accel_cfg);
    if (ret != ESP_OK) return ret;

    // GYRO_CONFIG_1 bits [2:1] = full-scale select, bit [0] = DLPF enable
    uint8_t gyro_cfg = ((uint8_t)gyro_range << 1) | 0x01;
    ret = write_reg(dev, GYRO_CONFIG_1, gyro_cfg);
    if (ret != ESP_OK) return ret;

    dev->accel_range = accel_range;
    dev->gyro_range = gyro_range;

    // Back to Bank 0 for data reads
    return select_bank(dev, 0);
}

// =============================================================================
// Data reading
// =============================================================================

esp_err_t icm20948_read_raw(icm20948_t *dev, icm20948_raw_t *out) {
    esp_err_t ret = select_bank(dev, 0);
    if (ret != ESP_OK) return ret;

    // Read 14 bytes: accel(6) + gyro(6) + temp(2), all big-endian
    uint8_t buf[14];
    ret = read_regs(dev, ACCEL_XOUT_H, buf, 14);
    if (ret != ESP_OK) return ret;

    out->accel_x = (int16_t)((buf[0] << 8) | buf[1]);
    out->accel_y = (int16_t)((buf[2] << 8) | buf[3]);
    out->accel_z = (int16_t)((buf[4] << 8) | buf[5]);
    out->gyro_x  = (int16_t)((buf[6] << 8) | buf[7]);
    out->gyro_y  = (int16_t)((buf[8] << 8) | buf[9]);
    out->gyro_z  = (int16_t)((buf[10] << 8) | buf[11]);
    out->temp    = (int16_t)((buf[12] << 8) | buf[13]);

    return ESP_OK;
}

// =============================================================================
// Conversion
// =============================================================================

float icm20948_accel_to_g(icm20948_t *dev, int16_t raw) {
    // LSB/g depends on range (sheet: 16384, 8192, 4096, 2048 for ±2,±4,±8,±16g)
    static const float lsb_per_g[4] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
    return (float)raw / lsb_per_g[dev->accel_range];
}

float icm20948_gyro_to_dps(icm20948_t *dev, int16_t raw) {
    // LSB/(dps) for ±250,500,1000,2000 dps: 131, 65.5, 32.8, 16.4
    static const float lsb_per_dps[4] = {131.0f, 65.5f, 32.8f, 16.4f};
    return (float)raw / lsb_per_dps[dev->gyro_range];
}
