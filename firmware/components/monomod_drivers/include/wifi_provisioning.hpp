/**
 * @file wifi_provisioning.hpp
 * @brief Simplified WiFi provisioning for MONOMOD
 *
 * NVS credential storage only (no SD card). Supports:
 * - Storing/loading credentials from NVS
 * - USB serial command interface: WIFI+ADD:ssid,password
 * - AP fallback mode
 */

#pragma once

#include <cstdint>
#include "esp_err.h"
#include "wifi_manager.hpp"

class WifiProvisioning {
public:
    WifiProvisioning();

    esp_err_t init();

    /// Try connecting using stored NVS credentials. Returns ESP_OK if connected.
    esp_err_t connect_from_nvs(uint32_t timeout_ms = 15000);

    /// Store credentials and connect
    esp_err_t add_and_connect(const char* ssid, const char* password,
                              uint32_t timeout_ms = 15000);

    /// Process a USB serial command line (e.g. "WIFI+ADD:ssid,password")
    /// Returns true if the line was a WiFi command
    bool process_serial_command(const char* line);

    bool is_connected() const;
    WiFiManager* wifi() { return &m_wifi; }

private:
    WiFiManager m_wifi;
    bool m_initialized;

    esp_err_t load_credentials(char* ssid, size_t ssid_len,
                               char* password, size_t pass_len);
    esp_err_t save_credentials(const char* ssid, const char* password);
};
