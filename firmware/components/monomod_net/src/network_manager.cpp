/**
 * @file network_manager.cpp
 * @brief Network Manager implementation for MONOMOD
 *
 * UDP streaming + TCP control + mDNS discovery. Notes:
 * - mDNS service: _monomod._udp
 * - ADC type: ADS1293
 * - No core pinning (ESP32-C3 is single-core)
 */

#include "network_manager.hpp"
#include <cstring>
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "mdns.h"
#include "crc16.h"

// Self-contained defaults (previously pulled from the main firmware's
// config.hpp; inlined so this is a standalone, reusable component).
#ifndef FW_VERSION_STR
#define FW_VERSION_STR  "1.0.0"
#endif
#ifndef STACK_TCP
#define STACK_TCP       4096
#endif
#ifndef PRIO_TCP
#define PRIO_TCP        8
#endif

static const char *TAG = "NET";

NetworkManager::NetworkManager() {
    memset(&client_addr_, 0, sizeof(client_addr_));
    memset(full_hostname_, 0, sizeof(full_hostname_));
}

NetworkManager::~NetworkManager() {
    deinit();
}

esp_err_t NetworkManager::init(const NetworkConfig &config) {
    if (initialized_) return ESP_OK;

    config_ = config;

    // Hostname with full MAC suffix (all 6 octets) — collision-proof across a fleet
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(full_hostname_, sizeof(full_hostname_), "%s-%02X%02X%02X%02X%02X%02X",
             config_.hostname, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    ESP_LOGI(TAG, "Init network (hostname: %s)", full_hostname_);

    esp_err_t ret = init_udp();
    if (ret != ESP_OK) return ret;

    ret = init_tcp();
    if (ret != ESP_OK) { deinit(); return ret; }

    ret = init_mdns();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "mDNS init failed (non-fatal)");
    }

    initialized_ = true;
    ESP_LOGI(TAG, "Network ready — UDP:%d TCP:%d", config_.udp_port, config_.tcp_port);
    return ESP_OK;
}

void NetworkManager::deinit() {
    stop();
    if (udp_socket_ >= 0)  { close(udp_socket_);  udp_socket_ = -1; }
    if (tcp_client_ >= 0)  { close(tcp_client_);  tcp_client_ = -1; }
    if (tcp_listen_ >= 0)  { close(tcp_listen_);  tcp_listen_ = -1; }
    mdns_free();
    initialized_ = false;
}

// =============================================================================
// Socket Init
// =============================================================================

esp_err_t NetworkManager::init_udp() {
    udp_socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_socket_ < 0) {
        ESP_LOGE(TAG, "UDP socket failed: errno %d", errno);
        return ESP_FAIL;
    }

    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(config_.udp_port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(udp_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "UDP bind failed: errno %d", errno);
        close(udp_socket_);
        udp_socket_ = -1;
        return ESP_FAIL;
    }

    // Blocking send with short timeout — small 23-byte IMU packets always
    // slot into the TX queue, but larger batched data packets arrived faster
    // than the queue could drain when the socket was non-blocking. With a
    // blocking timeout, sendto briefly waits for queue space instead of
    // dropping the packet.
    struct timeval send_to = { .tv_sec = 0, .tv_usec = 50000 };  // 50 ms
    setsockopt(udp_socket_, SOL_SOCKET, SO_SNDTIMEO, &send_to, sizeof(send_to));
    int bcast = 1;
    setsockopt(udp_socket_, SOL_SOCKET, SO_BROADCAST, &bcast, sizeof(bcast));
    int sndbuf = 65536;
    setsockopt(udp_socket_, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    return ESP_OK;
}

esp_err_t NetworkManager::init_tcp() {
    tcp_listen_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (tcp_listen_ < 0) return ESP_FAIL;

    int opt = 1;
    setsockopt(tcp_listen_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(config_.tcp_port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(tcp_listen_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(tcp_listen_); tcp_listen_ = -1;
        return ESP_FAIL;
    }
    if (listen(tcp_listen_, 1) < 0) {
        close(tcp_listen_); tcp_listen_ = -1;
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t NetworkManager::init_mdns() {
    esp_err_t ret = mdns_init();
    if (ret != ESP_OK) return ret;

    mdns_hostname_set(full_hostname_);
    mdns_instance_name_set("MONOMOD EMG Device");

    // Register _monomod._udp service
    mdns_service_add(NULL, "_monomod", "_udp", config_.udp_port, NULL, 0);

    // TXT records
    char ch_str[4];
    snprintf(ch_str, sizeof(ch_str), "%d", config_.num_channels);
    const char *adc_str =
        (config_.adc_type == MONOMOD_ADC_TYPE_INA)     ? "INA"     :
        (config_.adc_type == MONOMOD_ADC_TYPE_ADS1299) ? "ADS1299" :
        (config_.adc_type == MONOMOD_ADC_TYPE_RHD2132) ? "RHD2132" : "ADS1293";
    mdns_txt_item_t txt[] = {
        { "version", FW_VERSION_STR },
        { "adc",     (char *)adc_str },
        { "ch",      ch_str },
    };
    mdns_service_txt_set("_monomod", "_udp", txt, 3);

    ESP_LOGI(TAG, "mDNS: %s.local (_monomod._udp)", full_hostname_);
    return ESP_OK;
}

// =============================================================================
// TCP Server
// =============================================================================

esp_err_t NetworkManager::start() {
    if (!initialized_ || tcp_running_) return ESP_OK;

    tcp_running_ = true;
    xTaskCreate(tcp_task_func, "tcp_srv", STACK_TCP, this, PRIO_TCP, &tcp_task_);
    ESP_LOGI(TAG, "TCP server started");
    return ESP_OK;
}

void NetworkManager::stop() {
    if (!tcp_running_) return;
    tcp_running_ = false;
    if (tcp_client_ >= 0) { shutdown(tcp_client_, SHUT_RDWR); close(tcp_client_); tcp_client_ = -1; }
    vTaskDelay(pdMS_TO_TICKS(100));
    tcp_task_ = nullptr;
}

void NetworkManager::tcp_task_func(void *arg) {
    static_cast<NetworkManager *>(arg)->tcp_task_run();
    vTaskDelete(nullptr);
}

void NetworkManager::tcp_task_run() {
    while (tcp_running_) {
        struct timeval timeout = { .tv_sec = 1, .tv_usec = 0 };
        setsockopt(tcp_listen_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        struct sockaddr_in addr;
        socklen_t len = sizeof(addr);
        int client = accept(tcp_listen_, (struct sockaddr *)&addr, &len);

        if (client < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            if (tcp_running_) ESP_LOGW(TAG, "Accept error: %d", errno);
            continue;
        }

        tcp_client_ = client;

        // Set as UDP target (same host, UDP port)
        client_addr_ = addr;
        client_addr_.sin_port = htons(config_.udp_port);
        client_valid_ = true;

        char ip[16];
        inet_ntop(AF_INET, &addr.sin_addr, ip, sizeof(ip));
        ESP_LOGI(TAG, "Client connected: %s", ip);

        handle_tcp_client(client);

        close(tcp_client_);
        tcp_client_ = -1;
        client_valid_ = false;   // stop targeting a disconnected client over UDP
        ESP_LOGI(TAG, "Client disconnected");
    }
}

void NetworkManager::handle_tcp_client(int sock) {
    // Short recv timeout so we can periodically re-check tcp_running_.
    // Idle (no-command) periods are normal during streaming — we do NOT
    // close the connection when the timeout fires; we just loop.
    struct timeval timeout = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    int flag = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    // TCP keepalive — detect truly-dead connections within ~90 s.
    int ka_on = 1;
    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &ka_on, sizeof(ka_on));
    int ka_idle = 60, ka_intvl = 10, ka_cnt = 3;
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &ka_idle, sizeof(ka_idle));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &ka_intvl, sizeof(ka_intvl));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &ka_cnt, sizeof(ka_cnt));

    // TCP is a byte stream: a command may arrive split across recv() calls or
    // (in theory) coalesced. Accumulate into rx_buf_ and extract complete
    // packets. The host is synchronous (send → wait for ACK), so the common
    // case is exactly one packet per accumulation; we resync on MAGIC and
    // validate by CRC over the buffer, waiting for more bytes if it doesn't yet.
    size_t acc = 0;
    while (tcp_running_) {
        ssize_t n = recv(sock, rx_buf_ + acc, sizeof(rx_buf_) - acc, 0);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;  // idle, not dead
            break;  // real error
        }
        if (n == 0) break;  // peer closed
        acc += (size_t)n;

        for (;;) {
            // Resync to the 2-byte MAGIC at the front of the accumulator.
            if (acc >= 1 && rx_buf_[0] != MONOMOD_MAGIC_BYTE0) {
                memmove(rx_buf_, rx_buf_ + 1, --acc); continue;
            }
            if (acc >= 2 && rx_buf_[1] != MONOMOD_MAGIC_BYTE1) {
                memmove(rx_buf_, rx_buf_ + 1, --acc); continue;
            }
            if (acc < MONOMOD_HEADER_SIZE + 1 + MONOMOD_CRC_SIZE) break;  // need more

            if (crc16_verify(rx_buf_, acc)) {           // complete, valid packet
                if (cmd_callback_) {
                    size_t resp_len = 0;
                    cmd_callback_(rx_buf_, acc - MONOMOD_CRC_SIZE, tx_buf_, &resp_len);
                    if (resp_len > 0) {
                        crc16_append(tx_buf_, resp_len);
                        send(sock, tx_buf_, resp_len + MONOMOD_CRC_SIZE, 0);
                    }
                }
                acc = 0;
                break;
            }
            if (acc >= sizeof(rx_buf_)) {               // full and still invalid
                ESP_LOGW(TAG, "RX buffer full / CRC invalid — resync");
                acc = 0;
            }
            break;  // otherwise wait for more bytes
        }
    }
}

// =============================================================================
// UDP
// =============================================================================

esp_err_t NetworkManager::send_udp(const uint8_t *data, size_t len) {
    if (!client_valid_ || udp_socket_ < 0) return ESP_ERR_INVALID_STATE;

    ssize_t sent = sendto(udp_socket_, data, len, 0,
                          (struct sockaddr *)&client_addr_, sizeof(client_addr_));
    if (sent < 0) {
        // A link-down errno means WiFi dropped — flag it so the app can stop
        // streaming and recover instead of discarding samples silently.
        if (errno == ENETDOWN || errno == ENETUNREACH || errno == EHOSTUNREACH) {
            link_error_ = true;
        }
        // Rate-limited logging — repeated errors (WiFi hiccup, queue full)
        // should be visible but not spam the log every ~10ms.
        static uint32_t last_err_log = 0;
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        if (now - last_err_log > 1000) {
            ESP_LOGW("NET", "sendto(%u bytes) failed: errno=%d", (unsigned)len, errno);
            last_err_log = now;
        }
        udp_err_++;
        return ESP_FAIL;
    }
    udp_sent_++;
    return ESP_OK;
}

esp_err_t NetworkManager::send_tcp(const uint8_t *data, size_t len) {
    if (tcp_client_ < 0) return ESP_ERR_INVALID_STATE;
    return (send(tcp_client_, data, len, 0) > 0) ? ESP_OK : ESP_FAIL;
}
