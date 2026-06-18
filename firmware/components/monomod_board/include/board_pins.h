/**
 * @file board_pins.h
 * @brief Silk "D" label → GPIO mapping per ESP32-C3 carrier board.
 *
 * Firmware refers to the board's printed pin labels (PIN_D0..PIN_D10) instead of
 * raw GPIO numbers, so the same code compiles for different carriers. Pick the
 * board with `idf.py menuconfig` → "MONOMOD Board", or set
 * `CONFIG_MONOMOD_BOARD_SUPERMINI_C3=y` in sdkconfig.defaults.
 *
 * Macros are plain integers (usable for SPI/led_strip int fields). For APIs that
 * want a gpio_num_t (e.g. I2C bus config) cast at the use site: (gpio_num_t)PIN_Dx.
 */

#pragma once

#if defined(CONFIG_MONOMOD_BOARD_SUPERMINI_C3)

/* ESP32-C3 Super Mini */
#define MONOMOD_BOARD_NAME  "ESP32-C3 Super Mini"
#define PIN_D0   5
#define PIN_D1   6
#define PIN_D2   7
#define PIN_D3   8
#define PIN_D4   9
#define PIN_D5   10
#define PIN_D6   20
#define PIN_D7   1
#define PIN_D8   2
#define PIN_D9   3
#define PIN_D10  4

#elif defined(CONFIG_MONOMOD_BOARD_XIAO_S3)

/* Seeed XIAO ESP32-S3 */
#define MONOMOD_BOARD_NAME  "Seeed XIAO ESP32-S3"
#define PIN_D0   1
#define PIN_D1   2
#define PIN_D2   3
#define PIN_D3   4
#define PIN_D4   5
#define PIN_D5   6
#define PIN_D6   43
#define PIN_D7   44
#define PIN_D8   7
#define PIN_D9   8
#define PIN_D10  9

#else  /* default: Seeed XIAO ESP32-C3 */

#define MONOMOD_BOARD_NAME  "Seeed XIAO ESP32-C3"
#define PIN_D0   2
#define PIN_D1   3
#define PIN_D2   4
#define PIN_D3   5
#define PIN_D4   6
#define PIN_D5   7
#define PIN_D6   21
#define PIN_D7   20
#define PIN_D8   8
#define PIN_D9   9
#define PIN_D10  10

#endif
