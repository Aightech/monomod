/**
 * @file main.cpp
 * @brief MONOMOD Firmware Entry Point
 *
 * Standalone wireless EMG module: ADS1293 + BNO085 + ESP32-C3
 */

#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"

#include "config.hpp"
#include "ads1293.h"
#include "ina_emg.h"
#include "lis3dh.h"
#include "icm20948.h"
#include "packet_types.h"
#include "crc16.h"
#include "ring_buffer.hpp"
#include "timestamp.hpp"
#include "wifi_provisioning.hpp"

#include "acq_task.hpp"
#include "tx_task.hpp"
#include "imu_task.hpp"
#include "network_manager.hpp"

static const char *TAG = "monomod";

// =============================================================================
// Global state
// =============================================================================

// EMG module auto-detect result
typedef enum {
    EMG_MODULE_NONE = 0,
    EMG_MODULE_ADS1293,
    EMG_MODULE_INA_LIS,   // INA331 + LIS3DH combo submodule
} emg_module_t;

static emg_module_t    g_emg_module = EMG_MODULE_NONE;
static ads1293_t       g_adc;
static ads1293_config_t g_adc_config = ADS1293_DEFAULT_CONFIG();
static ina_emg_t       g_ina;
static lis3dh_t        g_lis;
static MonomodRingBuffer g_ring_buffer;
static icm20948_t      g_imu;
static i2c_master_bus_handle_t g_i2c_bus = NULL;

static axon_device_state_t g_state = AXON_STATE_BOOT;
// (packet stats tracked by NetworkManager and ring buffer)

// Task handles
static TaskHandle_t g_acq_task = NULL;
static TaskHandle_t g_tx_task = NULL;
static TaskHandle_t g_imu_task = NULL;
static SemaphoreHandle_t g_tx_sem = NULL;

// Network + WiFi
static NetworkManager g_net;
static WifiProvisioning g_wifi;

// =============================================================================
// Command Handler
// =============================================================================

static void handle_command(const uint8_t *packet, size_t pkt_len,
                           uint8_t *response, size_t *resp_len) {
    // packet includes header (9 bytes) + payload; CRC already stripped
    if (pkt_len < AXON_HEADER_SIZE + 1) return;

    const uint8_t *payload = packet + AXON_HEADER_SIZE;
    size_t len = pkt_len - AXON_HEADER_SIZE;

    uint8_t cmd_id = payload[0];
    const uint8_t *params = payload + 1;
    size_t params_len = len - 1;

    // Build response header
    axon_header_t *rhdr = (axon_header_t *)response;
    rhdr->magic = AXON_MAGIC;
    rhdr->type = AXON_TYPE_ACK;
    rhdr->seq = 0;
    rhdr->timestamp = axon::get_timestamp_us();
    uint8_t *rpayload = response + AXON_HEADER_SIZE;

    // Default: ACK with success
    axon_ack_payload_t ack = { .cmd_id = cmd_id, .status = AXON_ACK_SUCCESS };

    switch (cmd_id) {
        case AXON_CMD_PING:
            ESP_LOGI(TAG, "PING");
            break;

        case AXON_CMD_GET_INFO: {
            axon_device_info_t info = {};
            info.fw_major = FW_VERSION_MAJ;
            info.fw_minor = FW_VERSION_MIN;
            info.fw_patch = FW_VERSION_PATCH;
            info.features = AXON_FEATURE_IMU;
            if (g_emg_module == EMG_MODULE_INA_LIS) {
                info.adc_type = AXON_ADC_TYPE_INA;
                info.num_channels = AXON_INA_NUM_CHANNELS;
                info.sample_size = AXON_INA_SAMPLE_SIZE;
                info.max_sample_rate = 4000;
            } else {
                // Default: ADS1293 (also works as fallback when nothing detected)
                info.adc_type = AXON_ADC_TYPE_ADS1293;
                info.num_channels = ADS1293_NUM_CHANNELS;
                info.sample_size = AXON_ADS1293_SAMPLE_SIZE;
                info.max_sample_rate = 25600;
            }
            memcpy(rpayload, &ack, sizeof(ack));
            memcpy(rpayload + sizeof(ack), &info, sizeof(info));
            *resp_len = AXON_HEADER_SIZE + sizeof(ack) + sizeof(info);
            return;
        }

        case AXON_CMD_START: {
            if (g_state == AXON_STATE_STREAMING) {
                ack.status = AXON_ACK_BUSY;
                break;
            }
            // Parse start params
            if (params_len >= sizeof(axon_cmd_start_params_t)) {
                const axon_cmd_start_params_t *sp =
                    (const axon_cmd_start_params_t *)params;
                ESP_LOGI(TAG, "START: rate=%lu ch_mask=0x%08lx",
                         sp->sample_rate, sp->ch_mask);
                // TODO: apply sample rate and channel mask to config
            }

            // Create ring buffer + semaphore
            if (!g_tx_sem) g_tx_sem = xSemaphoreCreateBinary();
            g_ring_buffer.reset();

            if (g_emg_module == EMG_MODULE_ADS1293) {
                // Configure and start ADS1293
                if (ads1293_configure(&g_adc, &g_adc_config) != ESP_OK) {
                    ack.status = AXON_ACK_HW_ERROR;
                    break;
                }
                ads1293_start(&g_adc);
                acq_task_start(&g_adc, &g_ring_buffer, g_tx_sem, &g_acq_task);
            } else if (g_emg_module == EMG_MODULE_INA_LIS) {
                // Start the ADC continuous conversion + INA acq task
                if (ina_emg_start(&g_ina) != ESP_OK) {
                    ack.status = AXON_ACK_HW_ERROR;
                    break;
                }
                acq_task_start_ina(&g_ina, &g_ring_buffer, g_tx_sem, &g_acq_task);
            } else {
                ESP_LOGE(TAG, "No EMG module detected — cannot START");
                ack.status = AXON_ACK_HW_ERROR;
                break;
            }

            tx_task_start(&g_ring_buffer, &g_net, g_tx_sem, &g_tx_task);
            if (g_imu.initialized) {
                imu_task_start(&g_imu, &g_net, 100, &g_imu_task);
            }

            g_state = AXON_STATE_STREAMING;
            break;
        }

        case AXON_CMD_STOP: {
            if (g_state != AXON_STATE_STREAMING) break;

            // Stop tasks
            if (g_emg_module == EMG_MODULE_ADS1293) {
                acq_task_stop(g_acq_task);
                ads1293_stop(&g_adc);
            } else if (g_emg_module == EMG_MODULE_INA_LIS) {
                acq_task_stop_ina(g_acq_task);
                ina_emg_stop(&g_ina);
            }
            tx_task_stop(g_tx_task);
            if (g_imu_task) imu_task_stop(g_imu_task);
            g_acq_task = NULL;
            g_tx_task = NULL;
            g_imu_task = NULL;

            g_state = AXON_STATE_IDLE;
            ESP_LOGI(TAG, "STOP");
            break;
        }

        case AXON_CMD_READ_REGISTER: {
            if (params_len < 1) { ack.status = AXON_ACK_INVALID_PARAM; break; }
            uint8_t reg = params[0];
            uint8_t val = ads1293_read_reg(&g_adc, reg);
            axon_read_reg_response_t rr = { .reg_addr = reg, .value = val };
            memcpy(rpayload, &ack, sizeof(ack));
            memcpy(rpayload + sizeof(ack), &rr, sizeof(rr));
            *resp_len = AXON_HEADER_SIZE + sizeof(ack) + sizeof(rr);
            ESP_LOGI(TAG, "READ_REG 0x%02X = 0x%02X", reg, val);
            return;
        }

        case AXON_CMD_WRITE_REGISTER: {
            if (params_len < 2) { ack.status = AXON_ACK_INVALID_PARAM; break; }
            uint8_t reg = params[0];
            uint8_t val = params[1];
            ads1293_write_reg(&g_adc, reg, val);
            ESP_LOGI(TAG, "WRITE_REG 0x%02X = 0x%02X", reg, val);
            break;
        }

        case AXON_CMD_REBOOT:
            ESP_LOGI(TAG, "REBOOT requested");
            memcpy(rpayload, &ack, sizeof(ack));
            *resp_len = AXON_HEADER_SIZE + sizeof(ack);
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_restart();
            return;

        default:
            ESP_LOGW(TAG, "Unknown cmd 0x%02X", cmd_id);
            ack.status = AXON_ACK_NOT_SUPPORTED;
            break;
    }

    memcpy(rpayload, &ack, sizeof(ack));
    *resp_len = AXON_HEADER_SIZE + sizeof(ack);
}

// =============================================================================
// Main
// =============================================================================

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "MONOMOD %s starting...", FW_VERSION_STR);

    // Make stdin non-blocking so we can poll for serial commands in main loop
    setvbuf(stdin, NULL, _IONBF, 0);
    fcntl(fileno(stdin), F_SETFL, O_NONBLOCK);

    // 1. Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // 2. Auto-detect EMG submodule:
    //    a) Probe ADS1293 on SPI (REVID = 0x01)
    //    b) Else probe LIS3DH on SPI (WHO_AM_I = 0x33) → INA+LIS module
    //    c) Else fall back to bare-ADC mode: just read GPIO2 (D0).
    //       Useful for any DIY setup that puts an analog signal on that pin.
    ads1293_hw_config_t ads_hw = {
        .gpio_mosi = PIN_ADS_MOSI, .gpio_miso = PIN_ADS_MISO,
        .gpio_sclk = PIN_ADS_SCLK, .gpio_cs   = PIN_ADS_CS,
        .clock_hz  = ADS_SPI_FREQ_HZ, .spi_host = ADS_SPI_HOST,
    };
    ESP_LOGI(TAG, "Probing EMG module on SPI...");
    ret = ads1293_init(&g_adc, &ads_hw);
    if (ret == ESP_OK) {
        g_emg_module = EMG_MODULE_ADS1293;
        ESP_LOGI(TAG, "→ ADS1293 module detected (3-ch, 24-bit SPI)");
    } else {
        // ADS1293 not found; release its SPI handle and try LIS3DH on same bus
        ads1293_deinit(&g_adc);
        lis3dh_hw_config_t lis_hw = {
            .gpio_mosi = PIN_ADS_MOSI, .gpio_miso = PIN_ADS_MISO,
            .gpio_sclk = PIN_ADS_SCLK, .gpio_cs   = PIN_ADS_CS,
            .clock_hz  = ADS_SPI_FREQ_HZ, .spi_host = ADS_SPI_HOST,
        };
        bool lis_present = (lis3dh_init(&g_lis, &lis_hw) == ESP_OK);
        if (lis_present) {
            ESP_LOGI(TAG, "→ INA+LIS module detected (LIS3DH on SPI + analog EMG on D0)");
            lis3dh_enable(&g_lis);
        } else {
            ESP_LOGW(TAG, "No SPI module — falling back to bare-ADC mode on GPIO2 (D0)");
        }
        // INA path: works whether or not LIS3DH is present.
        if (ina_emg_init(&g_ina, 3200) == ESP_OK) {
            g_emg_module = EMG_MODULE_INA_LIS;
            ESP_LOGI(TAG, "→ INA mode active (1-ch analog @ 3200 Hz on GPIO2/D0%s)",
                     lis_present ? " + LIS3DH accel" : "");
        } else {
            ESP_LOGE(TAG, "ina_emg_init failed — no EMG source available");
        }
    }

    // 3. Initialize I2C bus + BNO085
    i2c_master_bus_config_t i2c_cfg = {};
    i2c_cfg.i2c_port = I2C_NUM_0;
    i2c_cfg.sda_io_num = PIN_I2C_SDA;
    i2c_cfg.scl_io_num = PIN_I2C_SCL;
    i2c_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_cfg.glitch_ignore_cnt = 7;
    i2c_cfg.flags.enable_internal_pullup = true;
    ret = i2c_new_master_bus(&i2c_cfg, &g_i2c_bus);
    if (ret == ESP_OK) {
        ret = icm20948_init(&g_imu, g_i2c_bus, IMU_I2C_ADDR);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "ICM-20948 not found — IMU disabled (check wiring, try addr 0x68)");
        } else {
            ESP_LOGI(TAG, "ICM-20948 ready (+/-2g, +/-250 dps)");
        }
    } else {
        ESP_LOGW(TAG, "I2C bus init failed — IMU disabled");
    }

    // 4. Initialize WiFi
    ESP_LOGI(TAG, "Initializing WiFi...");
    ret = g_wifi.init();
    if (ret == ESP_OK) {
        ret = g_wifi.connect_from_nvs(15000);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "No stored WiFi or connection failed");
            ESP_LOGI(TAG, "Use serial: WIFI+ADD:ssid,password");
        }
    }

    // 5. Initialize Network Manager (if WiFi connected)
    if (g_wifi.is_connected()) {
        NetworkConfig net_cfg;
        ret = g_net.init(net_cfg);
        if (ret == ESP_OK) {
            g_net.set_command_callback(handle_command);
            g_net.start();
        }
    }

    // Go to IDLE if any EMG module was detected; otherwise ERROR (but WiFi
    // and provisioning continue to work for diagnosis).
    g_state = (g_emg_module != EMG_MODULE_NONE) ? AXON_STATE_IDLE : AXON_STATE_ERROR;

    ESP_LOGI(TAG, "Init complete. WiFi: %s, State: %s",
             g_wifi.is_connected() ? "connected" : "disconnected",
             g_state == AXON_STATE_IDLE ? "IDLE" : "ERROR");

    ESP_LOGI(TAG, "Serial commands: WIFI+ADD:ssid,password | WIFI+STATUS");

    // Serial input buffer for USB-CDC command provisioning
    char line_buf[160];
    int line_pos = 0;
    uint32_t last_status_ms = 0;

    // Main loop: serial commands + periodic status + late WiFi connect
    while (true) {
        // Non-blocking read from stdin (USB Serial/JTAG console)
        int c = getchar();
        while (c != EOF) {
            if (c == '\r' || c == '\n') {
                if (line_pos > 0) {
                    line_buf[line_pos] = '\0';
                    g_wifi.process_serial_command(line_buf);
                    line_pos = 0;
                }
            } else if (line_pos < (int)sizeof(line_buf) - 1) {
                line_buf[line_pos++] = (char)c;
            }
            c = getchar();
        }

        vTaskDelay(pdMS_TO_TICKS(50));

        // Check for late WiFi connection (user just provisioned)
        if (!g_net.is_initialized() && g_wifi.is_connected()) {
            NetworkConfig net_cfg;
            if (g_net.init(net_cfg) == ESP_OK) {
                g_net.set_command_callback(handle_command);
                g_net.start();
                ESP_LOGI(TAG, "Network manager started");
            }
        }

        // Periodic status log (every 5s when streaming)
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        if (g_state == AXON_STATE_STREAMING && (now - last_status_ms) >= 5000) {
            last_status_ms = now;
            ESP_LOGI(TAG, "Streaming: ring=%u/%u overflow=%lu udp=%lu err=%lu",
                     (unsigned)g_ring_buffer.size(),
                     (unsigned)g_ring_buffer.capacity(),
                     g_ring_buffer.overflow_count(),
                     g_net.udp_packets_sent(),
                     g_net.udp_send_errors());
        }
    }
}
