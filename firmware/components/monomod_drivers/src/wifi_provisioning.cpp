#include "wifi_provisioning.hpp"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <cstring>

static const char *TAG = "wifi_prov";
static const char *NVS_NAMESPACE = "monomod_wifi";

WifiProvisioning::WifiProvisioning()
    : m_initialized(false) {}

esp_err_t WifiProvisioning::init() {
    esp_err_t ret = m_wifi.init();
    if (ret != ESP_OK) return ret;
    m_initialized = true;
    return ESP_OK;
}

esp_err_t WifiProvisioning::load_credentials(char *ssid, size_t ssid_len,
                                              char *password, size_t pass_len) {
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (ret != ESP_OK) return ret;

    ret = nvs_get_str(nvs, "ssid", ssid, &ssid_len);
    if (ret != ESP_OK) { nvs_close(nvs); return ret; }

    ret = nvs_get_str(nvs, "pass", password, &pass_len);
    nvs_close(nvs);
    return ret;
}

esp_err_t WifiProvisioning::save_credentials(const char *ssid, const char *password) {
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (ret != ESP_OK) return ret;

    nvs_set_str(nvs, "ssid", ssid);
    nvs_set_str(nvs, "pass", password);
    nvs_commit(nvs);
    nvs_close(nvs);

    ESP_LOGI(TAG, "Credentials saved for '%s'", ssid);
    return ESP_OK;
}

esp_err_t WifiProvisioning::connect_from_nvs(uint32_t timeout_ms) {
    char ssid[33] = {};
    char password[65] = {};

    esp_err_t ret = load_credentials(ssid, sizeof(ssid), password, sizeof(password));
    if (ret != ESP_OK || ssid[0] == '\0') {
        ESP_LOGW(TAG, "No stored WiFi credentials");
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Connecting to stored network '%s'...", ssid);
    return m_wifi.connect(ssid, password, timeout_ms);
}

esp_err_t WifiProvisioning::add_and_connect(const char *ssid, const char *password,
                                             uint32_t timeout_ms) {
    save_credentials(ssid, password);
    ESP_LOGI(TAG, "Connecting to '%s'...", ssid);
    return m_wifi.connect(ssid, password, timeout_ms);
}

bool WifiProvisioning::process_serial_command(const char *line) {
    // Format: WIFI+ADD:ssid,password
    if (strncmp(line, "WIFI+ADD:", 9) == 0) {
        const char *args = line + 9;
        const char *comma = strchr(args, ',');
        if (!comma) {
            ESP_LOGW(TAG, "Invalid format. Use: WIFI+ADD:ssid,password");
            return true;
        }
        char ssid[33] = {};
        char pass[65] = {};
        int ssid_len = comma - args;
        if (ssid_len > 32) ssid_len = 32;
        memcpy(ssid, args, ssid_len);
        strncpy(pass, comma + 1, sizeof(pass) - 1);

        // Trim trailing newline/CR
        char *end = pass + strlen(pass) - 1;
        while (end >= pass && (*end == '\n' || *end == '\r')) *end-- = '\0';

        add_and_connect(ssid, pass);
        return true;
    }

    if (strncmp(line, "WIFI+STATUS", 11) == 0) {
        auto status = m_wifi.get_status();
        ESP_LOGI(TAG, "WiFi: %s | IP: %s | RSSI: %d",
                 status.connected ? status.ssid : "disconnected",
                 status.has_ip ? status.ip_addr : "none",
                 status.rssi);
        return true;
    }

    return false;
}

bool WifiProvisioning::is_connected() const {
    return m_wifi.is_connected();
}
