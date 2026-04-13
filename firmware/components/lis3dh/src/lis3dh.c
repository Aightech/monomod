/**
 * @file lis3dh.c
 * @brief STMicro LIS3DH SPI driver (minimal).
 *
 * Register read protocol:
 *   byte 0: [R/W=1 | MS=auto-inc | addr[5:0]]  -> for single: 0x80 | addr
 *   byte 1: 0xFF (dummy, MISO returns register value)
 *
 * Register write protocol:
 *   byte 0: [R/W=0 | MS=0 | addr[5:0]]
 *   byte 1: data
 */

#include "lis3dh.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "lis3dh";

// Register addresses
#define REG_CTRL_REG1       0x20
#define REG_CTRL_REG4       0x23
#define REG_OUT_X_L         0x28

// CTRL_REG1 bits
#define CTRL1_ODR_100HZ     (0x5 << 4)  // 100 Hz
#define CTRL1_LP_EN         (1 << 3)    // low-power mode (8-bit data)  — not used
#define CTRL1_Z_EN          (1 << 2)
#define CTRL1_Y_EN          (1 << 1)
#define CTRL1_X_EN          (1 << 0)

// CTRL_REG4 bits (defaults: ±2g, normal mode, MSB-right, 12-bit)
#define CTRL4_BDU           (1 << 7)    // block data update

static uint8_t read_reg(lis3dh_t *dev, uint8_t reg) {
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), 0x00 };
    uint8_t rx[2] = { 0 };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rx };
    spi_device_polling_transmit(dev->spi, &t);
    return rx[1];
}

static void write_reg(lis3dh_t *dev, uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { (uint8_t)(reg & 0x3F), val };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
    spi_device_polling_transmit(dev->spi, &t);
}

static void read_regs(lis3dh_t *dev, uint8_t reg, uint8_t *buf, int count) {
    // Set MS bit (auto-increment) for multi-byte read
    uint8_t cmd = reg | 0x80 | 0x40;
    uint8_t tx[1 + 6] = { cmd };    // up to 6 bytes (xl/xh/yl/yh/zl/zh)
    uint8_t rx[1 + 6] = { 0 };
    if (count > 6) count = 6;
    spi_transaction_t t = {
        .length = (1 + count) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_polling_transmit(dev->spi, &t);
    memcpy(buf, &rx[1], count);
}

esp_err_t lis3dh_init(lis3dh_t *dev, const lis3dh_hw_config_t *hw) {
    memset(dev, 0, sizeof(*dev));

    spi_bus_config_t bus = {
        .mosi_io_num = hw->gpio_mosi,
        .miso_io_num = hw->gpio_miso,
        .sclk_io_num = hw->gpio_sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    esp_err_t ret = spi_bus_initialize(hw->spi_host, &bus, SPI_DMA_DISABLED);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_device_interface_config_t devcfg = {
        .mode           = 0,      // SPI Mode 0
        .clock_speed_hz = hw->clock_hz,
        .spics_io_num   = hw->gpio_cs,
        .queue_size     = 1,
    };
    ret = spi_bus_add_device(hw->spi_host, &devcfg, &dev->spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        return ret;
    }

    uint8_t id = read_reg(dev, LIS3DH_REG_WHOAMI);
    if (id != LIS3DH_WHOAMI) {
        ESP_LOGW(TAG, "WHO_AM_I = 0x%02X, expected 0x%02X", id, LIS3DH_WHOAMI);
        spi_bus_remove_device(dev->spi);
        dev->spi = NULL;
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "LIS3DH detected (WHO_AM_I = 0x%02X)", id);
    dev->initialized = true;
    return ESP_OK;
}

void lis3dh_deinit(lis3dh_t *dev) {
    if (dev->spi) {
        spi_bus_remove_device(dev->spi);
        dev->spi = NULL;
    }
    dev->initialized = false;
}

uint8_t lis3dh_read_whoami(lis3dh_t *dev) {
    if (!dev->initialized) return 0;
    return read_reg(dev, LIS3DH_REG_WHOAMI);
}

esp_err_t lis3dh_enable(lis3dh_t *dev) {
    if (!dev->initialized) return ESP_ERR_INVALID_STATE;
    // ODR = 100 Hz, normal power, all 3 axes enabled
    write_reg(dev, REG_CTRL_REG1, CTRL1_ODR_100HZ | CTRL1_X_EN | CTRL1_Y_EN | CTRL1_Z_EN);
    // BDU enabled so multi-byte reads are atomic
    write_reg(dev, REG_CTRL_REG4, CTRL4_BDU);
    return ESP_OK;
}

esp_err_t lis3dh_read_accel(lis3dh_t *dev, int16_t *x, int16_t *y, int16_t *z) {
    if (!dev->initialized) return ESP_ERR_INVALID_STATE;
    uint8_t buf[6];
    read_regs(dev, REG_OUT_X_L, buf, 6);
    *x = (int16_t)((buf[1] << 8) | buf[0]);
    *y = (int16_t)((buf[3] << 8) | buf[2]);
    *z = (int16_t)((buf[5] << 8) | buf[4]);
    return ESP_OK;
}
