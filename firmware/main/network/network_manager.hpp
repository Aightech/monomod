/**
 * @file network_manager.hpp
 * @brief Network Manager — UDP streaming + TCP control + mDNS discovery
 *
 * Adapted from axonCtrl NetworkManager for MONOMOD.
 */

#pragma once

#include <stdint.h>
#include <functional>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "lwip/sockets.h"
#include "packet_types.h"

struct NetworkConfig {
    uint16_t udp_port = AXON_UDP_DATA_PORT;     // 5000
    uint16_t tcp_port = AXON_TCP_CTRL_PORT;     // 5001
    const char *hostname = "monomod";
    uint8_t adc_type = AXON_ADC_TYPE_ADS1293;
    uint8_t num_channels = AXON_ADS1293_NUM_CHANNELS;
};

using CommandCallback = std::function<void(const uint8_t *data, size_t len,
                                           uint8_t *response, size_t *response_len)>;

class NetworkManager {
public:
    NetworkManager();
    ~NetworkManager();

    NetworkManager(const NetworkManager &) = delete;
    NetworkManager &operator=(const NetworkManager &) = delete;

    esp_err_t init(const NetworkConfig &config = {});
    void deinit();

    /// Start TCP server task
    esp_err_t start();
    void stop();

    bool is_initialized() const { return initialized_; }
    bool has_client() const { return client_valid_; }

    // UDP
    esp_err_t send_udp(const uint8_t *data, size_t len);

    // TCP
    esp_err_t send_tcp(const uint8_t *data, size_t len);

    void set_command_callback(CommandCallback cb) { cmd_callback_ = cb; }

    // Stats
    uint32_t udp_packets_sent() const { return udp_sent_; }
    uint32_t udp_send_errors() const { return udp_err_; }

private:
    esp_err_t init_udp();
    esp_err_t init_tcp();
    esp_err_t init_mdns();

    static void tcp_task_func(void *arg);
    void tcp_task_run();
    void handle_tcp_client(int client_socket);

    NetworkConfig config_;
    char full_hostname_[32];

    bool initialized_ = false;

    int udp_socket_ = -1;
    int tcp_listen_ = -1;
    int tcp_client_ = -1;

    struct sockaddr_in client_addr_;
    bool client_valid_ = false;

    TaskHandle_t tcp_task_ = nullptr;
    volatile bool tcp_running_ = false;

    CommandCallback cmd_callback_;

    uint8_t rx_buf_[512];
    uint8_t tx_buf_[512];

    uint32_t udp_sent_ = 0;
    uint32_t udp_err_ = 0;
};
