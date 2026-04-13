/**
 * @file ads1293.h
 * @brief TI ADS1293 3-Channel EMG AFE Driver for ESP-IDF
 *
 * Register map and configuration logic ported from CleverHand-interface
 * clvHd_module_ADS1293EMG_registers.hpp / clvHd_module_ADS1293EMG.hpp
 *
 * SPI protocol: Read = reg | 0x80, Write = reg & 0x7F
 * SPI mode 0 (CPOL=0 CPHA=0), MSB first, max 20 MHz
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// ADS1293 Register Map
// =============================================================================

typedef enum {
    ADS1293_CONFIG_REG          = 0x00,
    ADS1293_FLEX_CH0_CN_REG     = 0x01,
    ADS1293_FLEX_CH1_CN_REG     = 0x02,
    ADS1293_FLEX_CH2_CN_REG     = 0x03,
    ADS1293_FLEX_PACE_CN_REG    = 0x04,
    ADS1293_FLEX_VBAT_CN_REG    = 0x05,
    ADS1293_LOD_CN_REG          = 0x06,
    ADS1293_LOD_EN_REG          = 0x07,
    ADS1293_LOD_CURRENT_REG     = 0x08,
    ADS1293_LOD_AC_CN_REG       = 0x09,
    ADS1293_CMDET_EN_REG        = 0x0A,
    ADS1293_CMDET_CN_REG        = 0x0B,
    ADS1293_RLD_CN_REG          = 0x0C,
    ADS1293_WILSON_EN1_REG      = 0x0D,
    ADS1293_WILSON_EN2_REG      = 0x0E,
    ADS1293_WILSON_EN3_REG      = 0x0F,
    ADS1293_WILSON_CN_REG       = 0x10,
    ADS1293_REF_CN_REG          = 0x11,
    ADS1293_OSC_CN_REG          = 0x12,
    ADS1293_AFE_RES_REG         = 0x13,
    ADS1293_AFE_SHDN_CN_REG     = 0x14,
    ADS1293_AFE_FAULT_CN_REG    = 0x15,
    // 0x16 not defined
    ADS1293_AFE_PACE_CN_REG     = 0x17,
    ADS1293_ERROR_LOD_REG       = 0x18,
    ADS1293_ERROR_STATUS_REG    = 0x19,
    ADS1293_ERROR_RANGE1_REG    = 0x1A,
    ADS1293_ERROR_RANGE2_REG    = 0x1B,
    ADS1293_ERROR_RANGE3_REG    = 0x1C,
    ADS1293_ERROR_SYNC_REG      = 0x1D,
    ADS1293_ERROR_MISC_REG      = 0x1E,
    ADS1293_DIGO_STRENGTH_REG   = 0x1F,
    // 0x20 not defined
    ADS1293_R2_RATE_REG         = 0x21,
    ADS1293_R3_RATE_CH0_REG     = 0x22,
    ADS1293_R3_RATE_CH1_REG     = 0x23,
    ADS1293_R3_RATE_CH2_REG     = 0x24,
    ADS1293_R1_RATE_REG         = 0x25,
    ADS1293_DIS_EFILTER_REG     = 0x26,
    ADS1293_DRDYB_SRC_REG       = 0x27,
    ADS1293_SYNCB_CN_REG        = 0x28,
    ADS1293_MASK_DRDYB_REG      = 0x29,
    ADS1293_MASK_ERR_REG        = 0x2A,
    // 0x2B..0x2C not defined
    ADS1293_RESERVED_0x2D_REG   = 0x2D,
    ADS1293_ALARM_FILTER_REG    = 0x2E,
    ADS1293_CH_CNFG_REG         = 0x2F,
    ADS1293_DATA_STATUS_REG     = 0x30,
    ADS1293_DATA_CH0_PACE_REG   = 0x31,  // 2 bytes (16-bit fast)
    ADS1293_DATA_CH1_PACE_REG   = 0x33,
    ADS1293_DATA_CH2_PACE_REG   = 0x35,
    ADS1293_DATA_CH0_ECG_REG    = 0x37,  // 3 bytes (24-bit precise)
    ADS1293_DATA_CH1_ECG_REG    = 0x3A,
    ADS1293_DATA_CH2_ECG_REG    = 0x3D,
    ADS1293_REVID_REG           = 0x40,
} ads1293_reg_t;

// Operating modes (CONFIG_REG values)
#define ADS1293_MODE_START_CONV  0x01
#define ADS1293_MODE_STANDBY     0x02
#define ADS1293_MODE_POWER_DOWN  0x04

// Expected REVID value
#define ADS1293_EXPECTED_REVID   0x01

// Data block: 16 bytes from DATA_STATUS_REG
#define ADS1293_DATA_BLOCK_SIZE  16

// =============================================================================
// Configuration Structures
// =============================================================================

/** Per-channel configuration */
typedef struct {
    bool    enabled;
    uint8_t pos_input;      // 1-6 (electrode), 0 = not connected
    uint8_t neg_input;      // 1-6 (electrode), 0 = not connected
    bool    high_res;       // true = 24-bit, false = 16-bit
    bool    high_freq;      // true = 204.8 kHz base, false = 102.4 kHz base
} ads1293_channel_config_t;

/** Filter configuration */
typedef struct {
    int R1[3];              // Per-channel: 2 or 4
    int R2;                 // Global: 4, 5, 6, or 8
    int R3[3];              // Per-channel: 4, 6, 8, 12, 16, 32, 64, or 128
} ads1293_filter_config_t;

/** Right-Leg Drive (RLD) configuration */
typedef struct {
    uint8_t route;        // 0 = disabled, 1..6 = drive into IN1..IN6
    bool    bw_high;      // false = 50 kHz BW, true = 200 kHz BW
    uint8_t cap_drive;    // 0..3 (low / med-low / med-high / high)
} ads1293_rld_config_t;

/** Full device configuration */
typedef struct {
    ads1293_channel_config_t channels[3];
    ads1293_filter_config_t  filters;
    ads1293_rld_config_t     rld;
    bool use_internal_clock;
} ads1293_config_t;

/** Raw sample data from a single read */
typedef struct {
    uint8_t status;         // DATA_STATUS register
    int32_t precise[3];     // 24-bit values, sign-extended to int32
    int16_t fast[3];        // 16-bit values
} ads1293_sample_t;

/** Hardware config for SPI */
typedef struct {
    int gpio_mosi;
    int gpio_miso;
    int gpio_sclk;
    int gpio_cs;
    int clock_hz;
    spi_host_device_t spi_host;
} ads1293_hw_config_t;

/** Driver handle */
typedef struct {
    spi_device_handle_t spi;
    ads1293_hw_config_t hw;
    int32_t fast_adc_max;
    int32_t precise_adc_max[3];
    bool initialized;
} ads1293_t;

// =============================================================================
// Default Configuration
// =============================================================================

#define ADS1293_DEFAULT_CONFIG() { \
    .channels = { \
        { .enabled = true, .pos_input = 1, .neg_input = 2, .high_res = true, .high_freq = false }, \
        { .enabled = true, .pos_input = 3, .neg_input = 4, .high_res = true, .high_freq = false }, \
        { .enabled = true, .pos_input = 5, .neg_input = 6, .high_res = true, .high_freq = false }, \
    }, \
    .filters = { \
        .R1 = {2, 2, 2}, \
        .R2 = 4, \
        .R3 = {4, 4, 4}, \
    }, \
    .rld = { .route = 0, .bw_high = false, .cap_drive = 0 }, \
    .use_internal_clock = true, \
}

// =============================================================================
// API
// =============================================================================

/**
 * @brief Initialize SPI bus and ADS1293 device
 *
 * @param dev Driver handle (caller-allocated)
 * @param hw Hardware pin/bus configuration
 * @return ESP_OK on success
 */
esp_err_t ads1293_init(ads1293_t *dev, const ads1293_hw_config_t *hw);

/**
 * @brief Deinitialize and release SPI resources
 */
void ads1293_deinit(ads1293_t *dev);

/**
 * @brief Read device revision ID (should return 0x01)
 */
uint8_t ads1293_read_revid(ads1293_t *dev);

/**
 * @brief Read a single register
 */
uint8_t ads1293_read_reg(ads1293_t *dev, uint8_t reg);

/**
 * @brief Write a single register
 */
void ads1293_write_reg(ads1293_t *dev, uint8_t reg, uint8_t val);

/**
 * @brief Read multiple contiguous registers
 */
void ads1293_read_regs(ads1293_t *dev, uint8_t start_reg, uint8_t *buf, int count);

/**
 * @brief Apply full configuration
 *
 * Executes the setup sequence: POWER_DOWN → clock → routing →
 * enable → resolution → frequency → filters → clock start
 */
esp_err_t ads1293_configure(ads1293_t *dev, const ads1293_config_t *cfg);

/**
 * @brief Start continuous conversion
 */
esp_err_t ads1293_start(ads1293_t *dev);

/**
 * @brief Stop conversion (enter standby)
 */
esp_err_t ads1293_stop(ads1293_t *dev);

/**
 * @brief Read all data registers in a single SPI transaction (16 bytes)
 *
 * Reads DATA_STATUS + 3×fast(16-bit) + 3×precise(24-bit).
 * Returns raw integer values — host-side conversion to volts.
 *
 * @param dev Driver handle
 * @param sample Output sample data
 * @return ESP_OK on success
 */
esp_err_t ads1293_read_data(ads1293_t *dev, ads1293_sample_t *sample);

/**
 * @brief Check if new data is available (reads DATA_STATUS register)
 *
 * @param dev Driver handle
 * @param status Output: DATA_STATUS byte
 * @return true if any channel has new data
 */
bool ads1293_data_ready(ads1293_t *dev, uint8_t *status);

/**
 * @brief Get the effective sample rate for a given configuration
 *
 * @param cfg Configuration
 * @param channel Channel index (0-2) — each may differ if R1/R3 differ
 * @return Sample rate in Hz
 */
float ads1293_get_sample_rate(const ads1293_config_t *cfg, int channel);

#ifdef __cplusplus
}
#endif
