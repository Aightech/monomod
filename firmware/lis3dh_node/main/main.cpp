/**
 * @file main.cpp
 * @brief MONOMOD LIS3DH node — streams accel + EMG to the GUI over WiFi.
 *
 * Hardware: XIAO ESP32-C3 + a single LIS3DH on SPI. The LIS3DH provides BOTH
 * data sources, so all SPI reads happen in one acquisition task (single bus
 * owner). Acquisition is decoupled from transmission via lock-free SPSC rings so
 * a blocking/slow WiFi send never stalls (and never silently drops) sampling:
 *   - acq_task (prio 22): reads ADC1 (EMG) + accel, pushes into rings. No network.
 *   - tx_task  (prio 18): drains rings, batches, and sends UDP packets.
 *     EMG  → MONOMOD_TYPE_DATA_RAW / MONOMOD_ADC_TYPE_INA (1 ch, 16-bit)
 *     Accel→ MONOMOD_TYPE_IMU (accel filled, gyro = 0)
 *
 * Talks the existing MONOMOD protocol (UDP data :5000, TCP control :5001, mDNS
 * _monomod._udp), so the stock driver/GUI discover and stream from it unchanged.
 *
 * Wiring is by silk D-label; GPIOs resolved per board in monomod_board.
 * NeoPixel status: red=no sensor, blue=waiting for WiFi, dim cyan=idle,
 * green pulse=streaming.
 */

#include <cstdio>
#include <cstring>
#include <atomic>
#include <fcntl.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "led_strip.h"

#include "lis3dh.h"
#include "board_pins.h"
#include "packet_types.h"
#include "crc16.h"
#include "timestamp.hpp"
#include "ring_buffer.hpp"
#include "network_manager.hpp"
#include "wifi_provisioning.hpp"

static const char *TAG = "node";

// ---- Pin map (silk D-labels; GPIOs resolved per board in monomod_board) ------
#define PIN_MOSI        PIN_D10     // PICO
#define PIN_MISO        PIN_D9      // POCI
#define PIN_SCLK        PIN_D8
#define PIN_CS          PIN_D7
#define LIS_SPI_HOST    SPI2_HOST
#define LIS_SPI_HZ      2000000     // 2 MHz
#define PIN_NEOPIXEL    PIN_D0

// ---- Acquisition / transmit ------------------------------------------------
#define EMG_RATE_HZ          1000   // ADC1 poll rate (LIS3DH ODR set to 1.344 kHz)
#define ACCEL_RATE_HZ        100    // accel/IMU packet rate
#define TX_BATCH_WINDOW_MS   20     // tx wakes this often to drain the rings
#define TX_MAX_SAMPLES_PKT   32     // EMG samples per UDP packet (~MTU-safe)

// ---- Rings (single producer: acq_task; single consumer: tx_task) -----------
using EmgRing = RingBuffer<SampleFrame, 512>;   // ~1 kHz, 0.5 s buffer
using AccRing = RingBuffer<SampleFrame, 64>;    // ~100 Hz
static EmgRing s_emg_ring;
static AccRing s_acc_ring;

// ---- Globals ----------------------------------------------------------------
static led_strip_handle_t s_strip;
static lis3dh_t           s_lis;
static NetworkManager     s_net;
static WifiProvisioning   s_wifi;
static monomod_device_state_t s_state = MONOMOD_STATE_BOOT;
static TaskHandle_t       s_acq = nullptr;
static TaskHandle_t       s_tx  = nullptr;
static std::atomic<bool>  s_running{false};

// Runtime-configurable rates (defaults above; set via START / SET_IMU):
//   s_emg_rate  — EMG poll rate in Hz (host START sample_rate)
//   s_accel_div — emit 1 accel sample per N EMG samples (host SET_IMU rate_div)
static int s_emg_rate  = EMG_RATE_HZ;
static int s_accel_div = EMG_RATE_HZ / ACCEL_RATE_HZ;

// Pick the smallest LIS3DH ODR >= the EMG poll rate (the aux-ADC output rate
// equals the ODR, so the ODR must keep up with polling to avoid duplicates).
static uint8_t odr_for_rate(int hz) {
    if (hz <= 100) return LIS3DH_ODR_200HZ;
    if (hz <= 350) return LIS3DH_ODR_400HZ;   // covers 200/250
    return LIS3DH_ODR_1344HZ;                  // 500/1000
}

// =============================================================================
// NeoPixel
// =============================================================================
static void neopixel_init() {
    led_strip_config_t strip_cfg = {};
    strip_cfg.strip_gpio_num   = PIN_NEOPIXEL;
    strip_cfg.max_leds         = 1;
    strip_cfg.led_pixel_format = LED_PIXEL_FORMAT_GRB;
    strip_cfg.led_model        = LED_MODEL_WS2812;
    strip_cfg.flags.invert_out = false;

    led_strip_rmt_config_t rmt_cfg = {};
    rmt_cfg.clk_src         = RMT_CLK_SRC_DEFAULT;
    rmt_cfg.resolution_hz   = 10 * 1000 * 1000;
    rmt_cfg.flags.with_dma  = false;

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_strip));
}

static void neopixel_set(uint8_t r, uint8_t g, uint8_t b) {
    led_strip_set_pixel(s_strip, 0, r, g, b);
    led_strip_refresh(s_strip);
}

// =============================================================================
// Packet senders (MONOMOD protocol) — called only from tx_task
// =============================================================================
static void send_emg_frames(const SampleFrame *frames, size_t n, uint16_t *seq) {
    uint8_t pkt[MONOMOD_HEADER_SIZE + sizeof(monomod_data_header_t)
                + TX_MAX_SAMPLES_PKT * MONOMOD_INA_SAMPLE_SIZE + MONOMOD_CRC_SIZE];

    monomod_header_t *hdr = (monomod_header_t *)pkt;
    hdr->magic = MONOMOD_MAGIC;
    hdr->type = MONOMOD_TYPE_DATA_RAW;
    hdr->seq = (*seq)++;
    hdr->timestamp = frames[0].timestamp_us;

    monomod_data_header_t *dh = (monomod_data_header_t *)(pkt + MONOMOD_HEADER_SIZE);
    dh->ch_mask = 0x1;                 // single channel
    dh->n_samples = (uint8_t)n;
    dh->adc_type = MONOMOD_ADC_TYPE_INA;

    uint8_t *p = pkt + MONOMOD_HEADER_SIZE + sizeof(monomod_data_header_t);
    for (size_t i = 0; i < n; i++) {
        int16_t v = (int16_t)frames[i].samples[0];   // 16-bit big-endian
        *p++ = (uint8_t)((v >> 8) & 0xFF);
        *p++ = (uint8_t)(v & 0xFF);
    }

    size_t len = p - pkt;
    crc16_append(pkt, len);
    len += MONOMOD_CRC_SIZE;
    s_net.send_udp(pkt, len);
}

static void send_imu(int16_t ax, int16_t ay, int16_t az, uint16_t *seq) {
    uint8_t pkt[MONOMOD_HEADER_SIZE + sizeof(monomod_imu_payload_t) + MONOMOD_CRC_SIZE];

    monomod_header_t *hdr = (monomod_header_t *)pkt;
    hdr->magic = MONOMOD_MAGIC;
    hdr->type = MONOMOD_TYPE_IMU;
    hdr->seq = (*seq)++;
    hdr->timestamp = monomod::get_timestamp_us();

    monomod_imu_payload_t *pl = (monomod_imu_payload_t *)(pkt + MONOMOD_HEADER_SIZE);
    pl->accel_x = ax; pl->accel_y = ay; pl->accel_z = az;
    pl->gyro_x = 0;   pl->gyro_y = 0;   pl->gyro_z = 0;   // LIS3DH has no gyro

    size_t len = MONOMOD_HEADER_SIZE + sizeof(monomod_imu_payload_t);
    crc16_append(pkt, len);
    len += MONOMOD_CRC_SIZE;
    s_net.send_udp(pkt, len);
}

static void send_status() {
    uint8_t pkt[MONOMOD_HEADER_SIZE + sizeof(monomod_status_payload_t) + MONOMOD_CRC_SIZE];

    monomod_header_t *hdr = (monomod_header_t *)pkt;
    hdr->magic = MONOMOD_MAGIC;
    hdr->type = MONOMOD_TYPE_STATUS;
    hdr->seq = 0;
    hdr->timestamp = monomod::get_timestamp_us();

    monomod_status_payload_t *st = (monomod_status_payload_t *)(pkt + MONOMOD_HEADER_SIZE);
    memset(st, 0, sizeof(*st));
    uint8_t flags = MONOMOD_FLAG_IMU_ENABLED;
    if (s_wifi.is_connected())                  flags |= MONOMOD_FLAG_WIFI_CONNECTED;
    if (s_state == MONOMOD_STATE_STREAMING)     flags |= MONOMOD_FLAG_STREAMING;
    if (s_state == MONOMOD_STATE_ERROR)         flags |= MONOMOD_FLAG_ERROR;
    st->flags = flags;
    st->rssi = s_wifi.wifi()->get_rssi();
    st->packets_sent = s_net.udp_packets_sent();
    st->packets_dropped = s_emg_ring.overflow_count();   // samples lost to ring-full
    st->state = (uint8_t)s_state;
    st->sample_rate_hz = s_emg_rate;
    st->channel_mask = 0x1;
    // battery_pct/battery_mv left 0 — no battery gauge on this board.

    size_t len = MONOMOD_HEADER_SIZE + sizeof(monomod_status_payload_t);
    crc16_append(pkt, len);
    len += MONOMOD_CRC_SIZE;
    s_net.send_udp(pkt, len);
}

// =============================================================================
// Acquisition task — single SPI owner. ONLY reads sensors and pushes to rings.
// =============================================================================
static void acq_task(void *arg) {
    int rate = s_emg_rate > 0 ? s_emg_rate : EMG_RATE_HZ;
    TickType_t period = pdMS_TO_TICKS(1000 / rate);
    if (period < 1) period = 1;
    const int accel_div = s_accel_div > 0 ? s_accel_div : 1;
    uint32_t emg_seq = 0;
    int accel_cnt = 0;

    ESP_LOGI(TAG, "ACQ started: EMG %d Hz, accel ~%d Hz (div %d)",
             rate, rate / accel_div, accel_div);

    TickType_t last = xTaskGetTickCount();
    while (s_running.load(std::memory_order_acquire)) {
        vTaskDelayUntil(&last, period);

        int16_t emg;
        if (lis3dh_read_adc1(&s_lis, &emg) == ESP_OK) {
            SampleFrame f = {};
            f.timestamp_us = monomod::get_timestamp_us();
            f.sequence = emg_seq++;
            f.num_channels = 1;
            f.adc_type = MONOMOD_ADC_TYPE_INA;
            f.samples[0] = emg;
            s_emg_ring.push(f);    // drop-on-full; overflow_count() tracks loss
        }

        if (++accel_cnt >= accel_div) {
            accel_cnt = 0;
            int16_t x, y, z;
            if (lis3dh_read_accel(&s_lis, &x, &y, &z) == ESP_OK) {
                SampleFrame a = {};
                a.timestamp_us = monomod::get_timestamp_us();
                a.num_channels = 3;
                a.samples[0] = x; a.samples[1] = y; a.samples[2] = z;
                s_acc_ring.push(a);
            }
        }
    }

    s_acq = nullptr;
    vTaskDelete(nullptr);
}

// =============================================================================
// TX task — drains the rings, batches, sends. The only task that calls the net.
// =============================================================================
static void tx_task(void *arg) {
    uint16_t emg_seq = 0, imu_seq = 0;
    esp_task_wdt_add(NULL);     // a stuck sendto now trips the watchdog

    while (s_running.load(std::memory_order_acquire)) {
        vTaskDelay(pdMS_TO_TICKS(TX_BATCH_WINDOW_MS));
        esp_task_wdt_reset();

        // EMG: drain everything accumulated, split into MTU-safe packets.
        size_t count = 0;
        const SampleFrame *frames = s_emg_ring.peek_contiguous(count);
        while (frames && count > 0) {
            size_t batch = (count > TX_MAX_SAMPLES_PKT) ? TX_MAX_SAMPLES_PKT : count;
            send_emg_frames(frames, batch, &emg_seq);
            s_emg_ring.consume(batch);
            esp_task_wdt_reset();
            frames = s_emg_ring.peek_contiguous(count);
        }

        // Accel: one IMU packet per sample.
        SampleFrame a;
        while (s_acc_ring.pop(a)) {
            send_imu((int16_t)a.samples[0], (int16_t)a.samples[1],
                     (int16_t)a.samples[2], &imu_seq);
        }
    }

    esp_task_wdt_delete(NULL);
    s_tx = nullptr;
    vTaskDelete(nullptr);
}

// =============================================================================
// Streaming start/stop with deterministic teardown
// =============================================================================
static void streaming_start() {
    s_emg_ring.reset();
    s_acc_ring.reset();
    s_running.store(true, std::memory_order_release);
    xTaskCreate(acq_task, "acq", 4096, nullptr, 22, &s_acq);
    xTaskCreate(tx_task,  "tx",  8192, nullptr, 18, &s_tx);
    s_state = MONOMOD_STATE_STREAMING;
    ESP_LOGI(TAG, "START");
}

static void streaming_stop() {
    s_running.store(false, std::memory_order_release);
    // Wait for both tasks to actually exit (each nulls its handle before
    // vTaskDelete). Bounded ~1 s — covers a tx blocked in sendto.
    for (int i = 0; i < 100 && (s_acq || s_tx); i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    s_state = MONOMOD_STATE_IDLE;
    ESP_LOGI(TAG, "STOP (emg_overflow=%lu)", (unsigned long)s_emg_ring.overflow_count());
}

// =============================================================================
// Command handler (TCP control). Response built WITHOUT CRC — NetworkManager
// appends it. resp_len = 0 means "no response".
// =============================================================================
static void handle_command(const uint8_t *packet, size_t pkt_len,
                           uint8_t *response, size_t *resp_len) {
    if (pkt_len < MONOMOD_HEADER_SIZE + 1) return;

    const uint8_t *payload = packet + MONOMOD_HEADER_SIZE;
    uint8_t cmd_id = payload[0];

    monomod_header_t *rhdr = (monomod_header_t *)response;
    rhdr->magic = MONOMOD_MAGIC;
    rhdr->type = MONOMOD_TYPE_ACK;
    rhdr->seq = 0;
    rhdr->timestamp = monomod::get_timestamp_us();
    uint8_t *rpayload = response + MONOMOD_HEADER_SIZE;

    monomod_ack_payload_t ack = { .cmd_id = cmd_id, .status = MONOMOD_ACK_SUCCESS };

    switch (cmd_id) {
        case MONOMOD_CMD_PING:
            break;

        case MONOMOD_CMD_GET_INFO: {
            monomod_device_info_t info = {};
            info.fw_major = 1;
            info.fw_minor = 0;
            info.fw_patch = 0;
            info.adc_type = MONOMOD_ADC_TYPE_INA;
            info.num_channels = MONOMOD_INA_NUM_CHANNELS;
            info.sample_size = MONOMOD_INA_SAMPLE_SIZE;
            info.max_sample_rate = EMG_RATE_HZ;
            info.features = MONOMOD_FEATURE_IMU;
            memcpy(rpayload, &ack, sizeof(ack));
            memcpy(rpayload + sizeof(ack), &info, sizeof(info));
            *resp_len = MONOMOD_HEADER_SIZE + sizeof(ack) + sizeof(info);
            return;
        }

        case MONOMOD_CMD_START: {
            if (s_state == MONOMOD_STATE_STREAMING) { ack.status = MONOMOD_ACK_BUSY; break; }
            if (!s_lis.initialized)                 { ack.status = MONOMOD_ACK_HW_ERROR; break; }
            // Apply requested EMG sample rate (1 kHz is the practical ceiling).
            size_t plen = pkt_len - MONOMOD_HEADER_SIZE;   // includes cmd_id
            if (plen >= 1 + sizeof(monomod_cmd_start_params_t)) {
                const monomod_cmd_start_params_t *sp =
                    (const monomod_cmd_start_params_t *)(payload + 1);
                if (sp->sample_rate >= 50 && sp->sample_rate <= 1000) {
                    s_emg_rate = (int)sp->sample_rate;
                }
            }
            lis3dh_set_odr(&s_lis, odr_for_rate(s_emg_rate));
            streaming_start();
            break;
        }

        case MONOMOD_CMD_STOP:
            if (s_state == MONOMOD_STATE_STREAMING) streaming_stop();
            break;

        case MONOMOD_CMD_SET_IMU: {
            // rate_div = emit 1 accel sample per N EMG samples (set by the host
            // from emg_rate / imu_rate). Applied on the next START.
            size_t plen = pkt_len - MONOMOD_HEADER_SIZE;
            if (plen >= 1 + sizeof(monomod_cmd_set_imu_params_t)) {
                const monomod_cmd_set_imu_params_t *ip =
                    (const monomod_cmd_set_imu_params_t *)(payload + 1);
                if (ip->rate_div >= 1) s_accel_div = ip->rate_div;
            }
            break;
        }

        case MONOMOD_CMD_REBOOT:
            memcpy(rpayload, &ack, sizeof(ack));
            *resp_len = MONOMOD_HEADER_SIZE + sizeof(ack);
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_restart();
            return;

        default:
            ESP_LOGW(TAG, "Unknown cmd 0x%02X", cmd_id);
            ack.status = MONOMOD_ACK_NOT_SUPPORTED;
            break;
    }

    memcpy(rpayload, &ack, sizeof(ack));
    *resp_len = MONOMOD_HEADER_SIZE + sizeof(ack);
}

// =============================================================================
// Network bring-up (idempotent — also used for late connect after provisioning)
// =============================================================================
static bool try_start_network() {
    if (s_net.is_initialized() || !s_wifi.is_connected()) return s_net.is_initialized();
    NetworkConfig cfg;
    cfg.adc_type = MONOMOD_ADC_TYPE_INA;
    cfg.num_channels = MONOMOD_INA_NUM_CHANNELS;
    if (s_net.init(cfg) != ESP_OK) return false;
    s_net.set_command_callback(handle_command);
    s_net.start();
    ESP_LOGI(TAG, "Network started");
    return true;
}

// =============================================================================
// Main
// =============================================================================
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "MONOMOD LIS3DH node starting (%s)", MONOMOD_BOARD_NAME);

    // Route stdin/stdout through USB-Serial/JTAG so WiFi provisioning lines
    // (WIFI+ADD:ssid,password) reach the firmware.
    usb_serial_jtag_driver_config_t usb_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_serial_jtag_driver_install(&usb_cfg);
    usb_serial_jtag_vfs_use_driver();
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);
    fcntl(fileno(stdin), F_SETFL, O_NONBLOCK);

    // NVS (WiFi credentials)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    neopixel_init();
    neopixel_set(0, 0, 40);  // blue: probing

    // ---- Probe the LIS3DH (retry so a board can be reworked live) ----------
    const lis3dh_hw_config_t hw = {
        .gpio_mosi = PIN_MOSI, .gpio_miso = PIN_MISO,
        .gpio_sclk = PIN_SCLK, .gpio_cs   = PIN_CS,
        .clock_hz  = LIS_SPI_HZ, .spi_host = LIS_SPI_HOST,
    };
    bool blink = false;
    while (lis3dh_init(&s_lis, &hw) != ESP_OK) {
        ESP_LOGW(TAG, "LIS3DH not detected — check soldering / CS / MISO. Retrying...");
        blink = !blink;
        neopixel_set(blink ? 60 : 8, 0, 0);  // red blink
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "LIS3DH detected (WHO_AM_I=0x%02X)", lis3dh_read_whoami(&s_lis));

    // Configure for streaming: high ODR + auxiliary ADC for EMG.
    lis3dh_enable(&s_lis);                       // BDU + X/Y/Z, 100 Hz
    lis3dh_set_odr(&s_lis, LIS3DH_ODR_1344HZ);   // bump to 1.344 kHz for EMG
    lis3dh_enable_adc(&s_lis);                   // ADC1 = amplified EMG input

    // ---- WiFi ----
    ESP_LOGI(TAG, "Initializing WiFi...");
    if (s_wifi.init() == ESP_OK) {
        if (s_wifi.connect_from_nvs(15000) != ESP_OK) {
            ESP_LOGW(TAG, "No stored WiFi. Provision over USB: WIFI+ADD:ssid,password");
        }
    }
    try_start_network();

    s_state = MONOMOD_STATE_IDLE;
    ESP_LOGI(TAG, "Ready. WiFi: %s", s_wifi.is_connected() ? "connected" : "disconnected");

    // ---- Main loop: USB serial provisioning + late connect + status LED ----
    char line[160];
    int line_pos = 0;
    uint32_t pulse = 0;
    uint32_t last_status_ms = 0, last_reconnect_ms = 0;
    while (true) {
        int c = getchar();
        while (c != EOF) {
            if (c == '\r' || c == '\n') {
                if (line_pos > 0) {
                    line[line_pos] = '\0';
                    s_wifi.process_serial_command(line);
                    line_pos = 0;
                }
            } else if (line_pos < (int)sizeof(line) - 1) {
                line[line_pos++] = (char)c;
            }
            c = getchar();
        }

        vTaskDelay(pdMS_TO_TICKS(100));
        try_start_network();  // picks up a just-provisioned connection

        uint32_t now_ms = monomod::get_timestamp_ms();

        // STATUS telemetry ~1 Hz (only when a host is connected)
        if (s_net.has_client() && (now_ms - last_status_ms) >= 1000) {
            last_status_ms = now_ms;
            send_status();
        }

        // WiFi / link-loss handling: stop streaming on drop, periodically retry,
        // recover to IDLE so the host can re-START after reconnecting.
        bool wifi_ok = s_wifi.is_connected();
        if (s_state == MONOMOD_STATE_STREAMING && (!wifi_ok || s_net.link_error())) {
            ESP_LOGW(TAG, "Link lost during streaming — stopping");
            streaming_stop();
            s_net.clear_link_error();
            s_state = MONOMOD_STATE_ERROR;
        }
        if (!wifi_ok && (now_ms - last_reconnect_ms) >= 10000) {
            last_reconnect_ms = now_ms;
            ESP_LOGI(TAG, "WiFi down — attempting reconnect");
            s_wifi.connect_from_nvs(8000);
        } else if (wifi_ok && s_state == MONOMOD_STATE_ERROR) {
            s_state = MONOMOD_STATE_IDLE;   // recovered
        }

        // Status LED
        if (s_state == MONOMOD_STATE_STREAMING) {
            neopixel_set(0, (pulse++ & 1) ? 60 : 15, 0);   // green pulse
        } else if (s_wifi.is_connected()) {
            neopixel_set(0, 8, 8);                          // dim cyan: idle/ready
        } else {
            neopixel_set(0, 0, 40);                         // blue: waiting for WiFi
        }
    }
}
