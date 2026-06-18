/**
 * @file main.c
 * @brief LIS3DH SPI bring-up test for a XIAO ESP32-C3 module.
 *
 * Purpose: confirm the LIS3DH is soldered correctly and answers on the SPI bus.
 *
 * Behavior:
 *   - NeoPixel BLUE   while probing.
 *   - NeoPixel RED    (slow blink) while the LIS3DH is NOT detected. The probe
 *     retries every second, so you can rework the joint and watch it go green
 *     without reflashing.
 *   - NeoPixel GREEN  once WHO_AM_I == 0x33. Then it streams live accel readings
 *     over the USB serial console — tilt/tap the board to confirm it's truly
 *     alive (values should change, ~16384 LSB ≈ 1 g on the axis facing down).
 *
 * Hot-swap: while streaming it keeps polling WHO_AM_I. Unplug a module and it
 * drops back to the red probe state, then auto-detects the next one you plug in
 * — no reboot needed.
 *
 * Wiring (XIAO ESP32-C3 silk -> GPIO):
 *   MOSI/PICO  D10 -> GPIO10
 *   MISO/POCI  D9  -> GPIO9
 *   SCLK       D8  -> GPIO8
 *   CS         D7  -> GPIO20
 *   NeoPixel   D0  -> GPIO2
 * (The amplified EMG on the LIS3DH ADC1 input is not used by this test.)
 */

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"

#include "lis3dh.h"
#include "board_pins.h"

static const char *TAG = "lis3dh_test";

// ---- Pin map (XIAO ESP32-C3) ------------------------------------------------
// Silk D-labels; GPIOs resolved per board in monomod_board (XIAO / Super Mini)
#define PIN_MOSI        PIN_D10  // PICO
#define PIN_MISO        PIN_D9   // POCI
#define PIN_SCLK        PIN_D8
#define PIN_CS          PIN_D7
#define LIS3DH_SPI_HOST SPI2_HOST
#define LIS3DH_SPI_HZ   1000000 // 1 MHz — conservative for bring-up (LIS3DH max 10 MHz)

#define PIN_NEOPIXEL    PIN_D0
#define NEOPIXEL_COUNT  1

// ---- NeoPixel ---------------------------------------------------------------
static led_strip_handle_t s_strip;

static void neopixel_init(void) {
    led_strip_config_t strip_cfg = {
        .strip_gpio_num   = PIN_NEOPIXEL,
        .max_leds         = NEOPIXEL_COUNT,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model        = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src       = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,  // 10 MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_strip));
}

static void neopixel_set(uint8_t r, uint8_t g, uint8_t b) {
    led_strip_set_pixel(s_strip, 0, r, g, b);
    led_strip_refresh(s_strip);
}

// ---- Main -------------------------------------------------------------------
void app_main(void) {
    ESP_LOGI(TAG, "LIS3DH SPI bring-up test");
    ESP_LOGI(TAG, "SPI%d: MOSI=%d MISO=%d SCLK=%d CS=%d @ %d Hz",
             LIS3DH_SPI_HOST + 1, PIN_MOSI, PIN_MISO, PIN_SCLK, PIN_CS, LIS3DH_SPI_HZ);

    neopixel_init();

    lis3dh_t dev;
    const lis3dh_hw_config_t hw = {
        .gpio_mosi = PIN_MOSI,
        .gpio_miso = PIN_MISO,
        .gpio_sclk = PIN_SCLK,
        .gpio_cs   = PIN_CS,
        .clock_hz  = LIS3DH_SPI_HZ,
        .spi_host  = LIS3DH_SPI_HOST,
    };

    // Outer loop: probe -> stream -> (module unplugged) -> probe again, so you
    // can hot-swap modules on the SPI connector without rebooting.
    while (true) {
        // ---- Probe: retry until a module answers (rework / swap live) -------
        neopixel_set(0, 0, 40);  // blue: probing
        bool blink = false;
        while (lis3dh_init(&dev, &hw) != ESP_OK) {
            ESP_LOGW(TAG, "LIS3DH NOT detected (WHO_AM_I != 0x%02X). "
                          "Check soldering / CS / MISO. Retrying...", LIS3DH_WHOAMI);
            blink = !blink;
            neopixel_set(blink ? 60 : 8, 0, 0);  // red blink
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        ESP_LOGI(TAG, "LIS3DH DETECTED — WHO_AM_I = 0x%02X. SPI bus OK.",
                 lis3dh_read_whoami(&dev));
        neopixel_set(0, 60, 0);  // green: detected

        // ---- Stream: read accel + watch for the module being removed -------
        // Values are raw int16 (±2g, ~16384 LSB/g). An unplugged module reads
        // all-0xFF (x=y=z=-1) and WHO_AM_I stops returning 0x33.
        lis3dh_enable(&dev);
        uint32_t n = 0;
        int misses = 0;
        while (true) {
            // Confirm the module is still present before trusting the data.
            if (lis3dh_read_whoami(&dev) != LIS3DH_WHOAMI) {
                // Debounce: require a few bad reads so a single SPI glitch
                // doesn't kick us out of streaming.
                if (++misses >= 3) {
                    ESP_LOGW(TAG, "Module removed — re-probing for the next one...");
                    break;
                }
            } else {
                misses = 0;
                int16_t x, y, z;
                if (lis3dh_read_accel(&dev, &x, &y, &z) == ESP_OK) {
                    ESP_LOGI(TAG, "accel  x=%6d  y=%6d  z=%6d  (%.2f %.2f %.2f g)",
                             x, y, z, x / 16384.0f, y / 16384.0f, z / 16384.0f);
                }
                // gentle green heartbeat so you can see the loop is alive
                neopixel_set(0, (n++ & 1) ? 60 : 15, 0);
            }
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        // Release the SPI device handle; the next lis3dh_init() re-adds it.
        lis3dh_deinit(&dev);
    }
}
