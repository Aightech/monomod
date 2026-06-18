#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "board_pins.h"   // silk D-label -> GPIO map (XIAO / Super Mini)

// =============================================================================
// MONOMOD Board Configuration (pins via silk D-labels; see monomod_board)
// =============================================================================

#define BOARD_NAME      "MONOMOD_C3"
#define FW_VERSION_MAJ  1
#define FW_VERSION_MIN  0
#define FW_VERSION_PATCH 0
#define FW_VERSION_STR  "1.0.0"

// -----------------------------------------------------------------------------
// ADS1293 SPI — SPI2_HOST (only user SPI available on C3)
// -----------------------------------------------------------------------------
#define PIN_ADS_MOSI    PIN_D9          // PICO
#define PIN_ADS_MISO    PIN_D10         // POCI
#define PIN_ADS_SCLK    PIN_D8
#define PIN_ADS_CS      PIN_D7
#define ADS_SPI_HOST    SPI2_HOST
#define ADS_SPI_FREQ_HZ 4000000         // 4 MHz (conservative; ADS1293 max 20 MHz)

// -----------------------------------------------------------------------------
// ICM-20948 I2C (Adafruit breakout) — cast to gpio_num_t for the I2C bus config
// -----------------------------------------------------------------------------
#define PIN_I2C_SDA       ((gpio_num_t)PIN_D4)
#define PIN_I2C_SCL       ((gpio_num_t)PIN_D5)
#define I2C_FREQ_HZ       400000        // 400 kHz
#define IMU_I2C_ADDR      0x69          // Adafruit breakout default (AD0 high)
                                        // Use 0x68 if AD0 jumper is cut

// -----------------------------------------------------------------------------
// Network
// -----------------------------------------------------------------------------
#define UDP_DATA_PORT   5000
#define TCP_CTRL_PORT   5001
#define MDNS_SERVICE    "_monomod._udp"
#define MDNS_HOSTNAME   "monomod"

// -----------------------------------------------------------------------------
// Task Priorities (single-core C3 — no core pinning)
// WiFi/LWIP runs at 23 (ESP-IDF default)
// -----------------------------------------------------------------------------
#define PRIO_ACQ        22
#define PRIO_TX         18
#define PRIO_IMU        10
#define PRIO_TCP        8

// Stack sizes
#define STACK_ACQ       4096
#define STACK_TX        8192
#define STACK_IMU       4096
#define STACK_TCP       4096

// -----------------------------------------------------------------------------
// ADS1293 defaults
// -----------------------------------------------------------------------------
#define ADS1293_NUM_CHANNELS    3
#define ADS1293_DEFAULT_R1      2
#define ADS1293_DEFAULT_R2      4
#define ADS1293_DEFAULT_R3      4
