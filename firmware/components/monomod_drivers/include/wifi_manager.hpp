#pragma once

#include <cstdint>
#include <cstring>
#include <vector>
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

/**
 * @brief WiFi Manager for ESP32-S3
 *
 * Provides station mode WiFi functionality:
 * - Network scanning
 * - Connection to access point
 * - Connection status monitoring
 * - Basic connectivity testing
 */
class WiFiManager {
public:
    // Event bits for connection status
    static constexpr int WIFI_CONNECTED_BIT = BIT0;
    static constexpr int WIFI_FAIL_BIT = BIT1;

    // Maximum scan results
    static constexpr int MAX_SCAN_RESULTS = 20;

    // Maximum connection retries
    static constexpr int MAX_RETRY = 5;

    struct Config {
        int max_retry;          // Max connection retries (default 5)
    };

    struct APInfo {
        char ssid[33];          // SSID (max 32 chars + null)
        int8_t rssi;            // Signal strength in dBm
        wifi_auth_mode_t auth;  // Authentication mode
        uint8_t channel;        // WiFi channel
        uint8_t bssid[6];       // MAC address
    };

    struct ConnectionStatus {
        bool connected;
        bool has_ip;
        char ip_addr[16];       // IP address string
        char gateway[16];       // Gateway address
        char netmask[16];       // Netmask
        int8_t rssi;            // Current signal strength
        char ssid[33];          // Connected SSID
    };

    explicit WiFiManager(const Config& config = {.max_retry = MAX_RETRY});
    ~WiFiManager();

    // Disable copy
    WiFiManager(const WiFiManager&) = delete;
    WiFiManager& operator=(const WiFiManager&) = delete;

    /**
     * @brief Initialize WiFi subsystem
     * @return ESP_OK on success
     */
    esp_err_t init();

    /**
     * @brief Deinitialize WiFi and release resources
     */
    void deinit();

    // =========================================================================
    // Scanning
    // =========================================================================

    /**
     * @brief Scan for available networks
     * @param results Vector to store scan results
     * @param timeout_ms Scan timeout in milliseconds
     * @return ESP_OK on success
     */
    esp_err_t scan(std::vector<APInfo>& results, uint32_t timeout_ms = 5000);

    /**
     * @brief Get auth mode as string
     */
    static const char* auth_mode_str(wifi_auth_mode_t auth);

    // =========================================================================
    // Connection
    // =========================================================================

    /**
     * @brief Connect to an access point
     * @param ssid Network SSID
     * @param password Network password (empty for open networks)
     * @param timeout_ms Connection timeout in milliseconds
     * @return ESP_OK on success
     */
    esp_err_t connect(const char* ssid, const char* password, uint32_t timeout_ms = 15000);

    /**
     * @brief Connect using BSSID (MAC address) targeting
     * @param ssid Network SSID
     * @param password Network password
     * @param bssid AP MAC address (6 bytes)
     * @param channel WiFi channel (0 for auto)
     * @param timeout_ms Connection timeout
     * @return ESP_OK on success
     *
     * @note Use this for weak signals - targeting a specific BSSID helps
     */
    esp_err_t connect_with_bssid(const char* ssid, const char* password,
                                  const uint8_t* bssid, uint8_t channel = 0,
                                  uint32_t timeout_ms = 15000);

    /**
     * @brief Find AP info by SSID in scan results
     * @param results Scan results from scan()
     * @param ssid SSID to find
     * @return Pointer to APInfo if found, nullptr otherwise
     */
    static const APInfo* find_ap_by_ssid(const std::vector<APInfo>& results, const char* ssid);

    /**
     * @brief Disconnect from current network
     * @return ESP_OK on success
     */
    esp_err_t disconnect();

    /**
     * @brief Check if connected to an AP
     */
    bool is_connected() const;

    /**
     * @brief Get current connection status
     */
    ConnectionStatus get_status();

    /**
     * @brief Get current RSSI (signal strength)
     * @return RSSI in dBm, or 0 if not connected
     */
    int8_t get_rssi();

    // =========================================================================
    // Connectivity Test
    // =========================================================================

    /**
     * @brief Test connectivity by performing DNS lookup
     * @param hostname Hostname to resolve (default: google.com)
     * @return ESP_OK if DNS resolution succeeds
     */
    esp_err_t test_dns(const char* hostname = "google.com");

    /**
     * @brief Test connectivity with HTTP GET request
     * @param url URL to fetch (default: http://httpbin.org/ip)
     * @param response Buffer to store response (optional)
     * @param response_len Response buffer length
     * @return ESP_OK if request succeeds
     */
    esp_err_t test_http(const char* url = "http://httpbin.org/ip",
                        char* response = nullptr, size_t response_len = 0);

    bool is_initialized() const { return m_initialized; }

private:
    Config m_config;
    bool m_initialized;
    bool m_connected;
    int m_retry_count;
    EventGroupHandle_t m_event_group;
    esp_netif_t* m_netif;

    // Static event handler (called by ESP-IDF event loop)
    static void event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data);

    // Instance event handler
    void handle_event(esp_event_base_t event_base, int32_t event_id, void* event_data);
};
