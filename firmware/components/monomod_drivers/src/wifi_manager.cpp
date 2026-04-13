#include "wifi_manager.hpp"

#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/dns.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_http_client.h"
#include <cstdio>

static const char* TAG = "WiFiManager";

// ============================================================================
// Constructor / Destructor
// ============================================================================

WiFiManager::WiFiManager(const Config& config)
    : m_config(config)
    , m_initialized(false)
    , m_connected(false)
    , m_retry_count(0)
    , m_event_group(nullptr)
    , m_netif(nullptr)
{
}

WiFiManager::~WiFiManager()
{
    deinit();
}

// ============================================================================
// Initialization
// ============================================================================

esp_err_t WiFiManager::init()
{
    if (m_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing WiFi...");

    // Initialize NVS (required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create event group
    m_event_group = xEventGroupCreate();
    if (!m_event_group) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_ERR_NO_MEM;
    }

    // Initialize TCP/IP stack
    ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create default event loop
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create default WiFi station interface
    m_netif = esp_netif_create_default_wifi_sta();
    if (!m_netif) {
        ESP_LOGE(TAG, "Failed to create WiFi STA netif");
        return ESP_FAIL;
    }

    // Initialize WiFi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register event handlers
    ret = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &WiFiManager::event_handler, this, nullptr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register WiFi event handler");
        return ret;
    }

    ret = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &WiFiManager::event_handler, this, nullptr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register IP event handler");
        return ret;
    }

    // Set WiFi mode to station
    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Start WiFi
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Disable power save mode for more reliable connections
    esp_wifi_set_ps(WIFI_PS_NONE);

    // Set country code (affects allowed channels and power levels)
    wifi_country_t country = {
        .cc = "GB",         // United Kingdom
        .schan = 1,
        .nchan = 13,
        .max_tx_power = 20,
        .policy = WIFI_COUNTRY_POLICY_MANUAL,
    };
    esp_wifi_set_country(&country);

    // Set maximum TX power (in 0.25 dBm units, 80 = 20 dBm)
    esp_wifi_set_max_tx_power(84);  // ~21 dBm - maximum
    int8_t tx_power;
    esp_wifi_get_max_tx_power(&tx_power);
    ESP_LOGI(TAG, "TX power set to: %.2f dBm", tx_power * 0.25f);

    m_initialized = true;
    ESP_LOGI(TAG, "WiFi initialized successfully");
    return ESP_OK;
}

void WiFiManager::deinit()
{
    if (!m_initialized) {
        return;
    }

    ESP_LOGI(TAG, "Deinitializing WiFi...");

    if (m_connected) {
        disconnect();
    }

    esp_wifi_stop();
    esp_wifi_deinit();

    if (m_netif) {
        esp_netif_destroy_default_wifi(m_netif);
        m_netif = nullptr;
    }

    if (m_event_group) {
        vEventGroupDelete(m_event_group);
        m_event_group = nullptr;
    }

    m_initialized = false;
}

// ============================================================================
// Event Handler
// ============================================================================

void WiFiManager::event_handler(void* arg, esp_event_base_t event_base,
                                 int32_t event_id, void* event_data)
{
    WiFiManager* self = static_cast<WiFiManager*>(arg);
    if (self) {
        self->handle_event(event_base, event_id, event_data);
    }
}

void WiFiManager::handle_event(esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGD(TAG, "WiFi STA started");
                break;

            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI(TAG, "Connected to AP");
                break;

            case WIFI_EVENT_STA_DISCONNECTED: {
                wifi_event_sta_disconnected_t* event =
                    (wifi_event_sta_disconnected_t*)event_data;

                // Check if we've already signaled failure - prevent duplicate handling
                EventBits_t bits = xEventGroupGetBits(m_event_group);
                if (bits & WIFI_FAIL_BIT) {
                    // Already failed, ignore further disconnect events
                    break;
                }

                ESP_LOGW(TAG, "Disconnected from AP, reason: %d", event->reason);
                m_connected = false;

                if (m_retry_count < m_config.max_retry) {
                    m_retry_count++;
                    ESP_LOGI(TAG, "Retrying connection (%d/%d)...",
                             m_retry_count, m_config.max_retry);
                    // Disconnect first to clear any pending state
                    esp_wifi_disconnect();
                    // Longer delay before retry - needed for auth timeouts
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    esp_wifi_connect();
                } else {
                    ESP_LOGE(TAG, "Max retries reached, connection failed");
                    xEventGroupSetBits(m_event_group, WIFI_FAIL_BIT);
                }
                break;
            }

            default:
                break;
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
            ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
            m_connected = true;
            m_retry_count = 0;
            xEventGroupSetBits(m_event_group, WIFI_CONNECTED_BIT);
        }
    }
}

// ============================================================================
// Scanning
// ============================================================================

esp_err_t WiFiManager::scan(std::vector<APInfo>& results, uint32_t timeout_ms)
{
    if (!m_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting WiFi scan...");
    results.clear();

    // Configure scan
    wifi_scan_config_t scan_config = {};
    scan_config.ssid = nullptr;
    scan_config.bssid = nullptr;
    scan_config.channel = 0;  // Scan all channels
    scan_config.show_hidden = false;
    scan_config.scan_type = WIFI_SCAN_TYPE_ACTIVE;
    scan_config.scan_time.active.min = 100;
    scan_config.scan_time.active.max = 300;

    // Start blocking scan
    esp_err_t ret = esp_wifi_scan_start(&scan_config, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Scan start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Get number of APs found
    uint16_t ap_count = 0;
    ret = esp_wifi_scan_get_ap_num(&ap_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get AP count: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Found %d access points", ap_count);

    if (ap_count == 0) {
        return ESP_OK;
    }

    // Limit results
    if (ap_count > MAX_SCAN_RESULTS) {
        ap_count = MAX_SCAN_RESULTS;
    }

    // Get AP records
    wifi_ap_record_t* ap_records = new wifi_ap_record_t[ap_count];
    ret = esp_wifi_scan_get_ap_records(&ap_count, ap_records);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get AP records: %s", esp_err_to_name(ret));
        delete[] ap_records;
        return ret;
    }

    // Convert to APInfo
    for (int i = 0; i < ap_count; i++) {
        APInfo info = {};
        strncpy(info.ssid, (char*)ap_records[i].ssid, sizeof(info.ssid) - 1);
        info.rssi = ap_records[i].rssi;
        info.auth = ap_records[i].authmode;
        info.channel = ap_records[i].primary;
        memcpy(info.bssid, ap_records[i].bssid, 6);
        results.push_back(info);
    }

    delete[] ap_records;
    return ESP_OK;
}

const char* WiFiManager::auth_mode_str(wifi_auth_mode_t auth)
{
    switch (auth) {
        case WIFI_AUTH_OPEN:            return "OPEN";
        case WIFI_AUTH_WEP:             return "WEP";
        case WIFI_AUTH_WPA_PSK:         return "WPA";
        case WIFI_AUTH_WPA2_PSK:        return "WPA2";
        case WIFI_AUTH_WPA_WPA2_PSK:    return "WPA/WPA2";
        case WIFI_AUTH_WPA3_PSK:        return "WPA3";
        case WIFI_AUTH_WPA2_WPA3_PSK:   return "WPA2/WPA3";
        case WIFI_AUTH_WAPI_PSK:        return "WAPI";
        default:                        return "UNKNOWN";
    }
}

// ============================================================================
// Connection
// ============================================================================

esp_err_t WiFiManager::connect(const char* ssid, const char* password, uint32_t timeout_ms)
{
    if (!m_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!ssid || strlen(ssid) == 0) {
        ESP_LOGE(TAG, "SSID is required");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Connecting to '%s'...", ssid);

    // Clear event bits
    xEventGroupClearBits(m_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    m_retry_count = 0;
    m_connected = false;

    // Configure WiFi
    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    if (password && strlen(password) > 0) {
        strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    }

    // Disable all advanced security features for maximum compatibility
    // Some routers have issues with PMF negotiation
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;  // Accept any auth mode
    wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_UNSPECIFIED;
    wifi_config.sta.pmf_cfg.capable = false;
    wifi_config.sta.pmf_cfg.required = false;

    esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi config: %s", esp_err_to_name(ret));
        return ret;
    }

    // Start connection
    ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_connect failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait for connection or failure
    EventBits_t bits = xEventGroupWaitBits(m_event_group,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE, pdFALSE,
                                            pdMS_TO_TICKS(timeout_ms));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to '%s'", ssid);
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to '%s'", ssid);
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "Connection timeout");
        esp_wifi_disconnect();
        return ESP_ERR_TIMEOUT;
    }
}

esp_err_t WiFiManager::connect_with_bssid(const char* ssid, const char* password,
                                           const uint8_t* bssid, uint8_t channel,
                                           uint32_t timeout_ms)
{
    if (!m_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!ssid || strlen(ssid) == 0) {
        ESP_LOGE(TAG, "SSID is required");
        return ESP_ERR_INVALID_ARG;
    }

    if (!bssid) {
        ESP_LOGE(TAG, "BSSID is required");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Connecting to '%s' (BSSID: %02X:%02X:%02X:%02X:%02X:%02X, CH: %d)...",
             ssid, bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5], channel);

    // Clear event bits
    xEventGroupClearBits(m_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    m_retry_count = 0;
    m_connected = false;

    // Configure WiFi with BSSID targeting
    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    if (password && strlen(password) > 0) {
        strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    }

    // Set BSSID and channel for targeted connection
    memcpy(wifi_config.sta.bssid, bssid, 6);
    wifi_config.sta.bssid_set = true;
    wifi_config.sta.channel = channel;

    // Disable all advanced security features for maximum compatibility
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_UNSPECIFIED;
    wifi_config.sta.pmf_cfg.capable = false;
    wifi_config.sta.pmf_cfg.required = false;

    esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi config: %s", esp_err_to_name(ret));
        return ret;
    }

    // Start connection
    ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_connect failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait for connection or failure
    EventBits_t bits = xEventGroupWaitBits(m_event_group,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE, pdFALSE,
                                            pdMS_TO_TICKS(timeout_ms));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to '%s'", ssid);
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to '%s'", ssid);
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "Connection timeout");
        esp_wifi_disconnect();
        return ESP_ERR_TIMEOUT;
    }
}

const WiFiManager::APInfo* WiFiManager::find_ap_by_ssid(const std::vector<APInfo>& results,
                                                         const char* ssid)
{
    for (const auto& ap : results) {
        if (strcmp(ap.ssid, ssid) == 0) {
            return &ap;
        }
    }
    return nullptr;
}

esp_err_t WiFiManager::disconnect()
{
    if (!m_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Disconnecting...");
    m_connected = false;
    return esp_wifi_disconnect();
}

bool WiFiManager::is_connected() const
{
    return m_connected;
}

WiFiManager::ConnectionStatus WiFiManager::get_status()
{
    ConnectionStatus status = {};

    if (!m_initialized) {
        return status;
    }

    status.connected = m_connected;

    if (m_connected) {
        // Get AP info
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            strncpy(status.ssid, (char*)ap_info.ssid, sizeof(status.ssid) - 1);
            status.rssi = ap_info.rssi;
        }

        // Get IP info
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(m_netif, &ip_info) == ESP_OK) {
            status.has_ip = ip_info.ip.addr != 0;
            snprintf(status.ip_addr, sizeof(status.ip_addr), IPSTR, IP2STR(&ip_info.ip));
            snprintf(status.gateway, sizeof(status.gateway), IPSTR, IP2STR(&ip_info.gw));
            snprintf(status.netmask, sizeof(status.netmask), IPSTR, IP2STR(&ip_info.netmask));
        }
    }

    return status;
}

int8_t WiFiManager::get_rssi()
{
    if (!m_connected) {
        return 0;
    }

    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        return ap_info.rssi;
    }
    return 0;
}

// ============================================================================
// Connectivity Tests
// ============================================================================

esp_err_t WiFiManager::test_dns(const char* hostname)
{
    if (!m_connected) {
        ESP_LOGE(TAG, "Not connected");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Resolving '%s'...", hostname);

    struct addrinfo hints = {};
    struct addrinfo* res = nullptr;

    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;

    int err = getaddrinfo(hostname, "80", &hints, &res);
    if (err != 0 || res == nullptr) {
        ESP_LOGE(TAG, "DNS lookup failed for '%s': %d", hostname, err);
        return ESP_FAIL;
    }

    struct sockaddr_in* addr_in = (struct sockaddr_in*)res->ai_addr;
    char ip_str[16];
    inet_ntoa_r(addr_in->sin_addr, ip_str, sizeof(ip_str));
    ESP_LOGI(TAG, "Resolved '%s' -> %s", hostname, ip_str);

    freeaddrinfo(res);
    return ESP_OK;
}

esp_err_t WiFiManager::test_http(const char* url, char* response, size_t response_len)
{
    if (!m_connected) {
        ESP_LOGE(TAG, "Not connected");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "HTTP GET: %s", url);

    // Simple buffer for response
    char local_buf[256] = {0};

    esp_http_client_config_t config = {};
    config.url = url;
    config.timeout_ms = 10000;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to init HTTP client");
        return ESP_FAIL;
    }

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        int64_t content_len = esp_http_client_get_content_length(client);
        ESP_LOGI(TAG, "HTTP Status: %d, Content-Length: %lld", status, content_len);

        if (status >= 200 && status < 300) {
            // Read response body
            int read_len = esp_http_client_read(client, local_buf, sizeof(local_buf) - 1);
            if (read_len > 0) {
                local_buf[read_len] = '\0';
                ESP_LOGI(TAG, "Response: %.100s%s", local_buf, read_len > 100 ? "..." : "");

                if (response && response_len > 0) {
                    strncpy(response, local_buf, response_len - 1);
                    response[response_len - 1] = '\0';
                }
            }
            err = ESP_OK;
        } else {
            ESP_LOGE(TAG, "HTTP request failed with status %d", status);
            err = ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG, "HTTP request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return err;
}
