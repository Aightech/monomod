#include "imu_task.hpp"
#include "config.hpp"
#include "packet_types.h"
#include "crc16.h"
#include "timestamp.hpp"
#include "esp_log.h"
#include <cstring>

static const char *TAG = "imu";

struct ImuContext {
    icm20948_t *imu;
    NetworkManager *net;
    int rate_hz;
    uint16_t seq;
    volatile bool running;
};

static ImuContext s_imu_ctx;

static void imu_task_func(void *arg) {
    ImuContext *ctx = (ImuContext *)arg;
    TickType_t period = pdMS_TO_TICKS(1000 / ctx->rate_hz);

    ESP_LOGI(TAG, "IMU task started at %d Hz", ctx->rate_hz);

    while (ctx->running) {
        icm20948_raw_t raw;
        if (icm20948_read_raw(ctx->imu, &raw) != ESP_OK) {
            vTaskDelay(period);
            continue;
        }

        // Build IMU packet with standard 12-byte raw payload
        uint8_t packet[AXON_HEADER_SIZE + sizeof(axon_imu_payload_t) + AXON_CRC_SIZE];

        axon_header_t *hdr = (axon_header_t *)packet;
        hdr->magic = AXON_MAGIC;
        hdr->type = AXON_TYPE_IMU;
        hdr->seq = ctx->seq++;
        hdr->timestamp = axon::get_timestamp_us();

        axon_imu_payload_t *pl =
            (axon_imu_payload_t *)(packet + AXON_HEADER_SIZE);
        pl->accel_x = raw.accel_x;
        pl->accel_y = raw.accel_y;
        pl->accel_z = raw.accel_z;
        pl->gyro_x  = raw.gyro_x;
        pl->gyro_y  = raw.gyro_y;
        pl->gyro_z  = raw.gyro_z;

        // CRC
        size_t pkt_len = AXON_HEADER_SIZE + sizeof(axon_imu_payload_t);
        crc16_append(packet, pkt_len);
        pkt_len += AXON_CRC_SIZE;

        ctx->net->send_udp(packet, pkt_len);

        vTaskDelay(period);
    }

    ESP_LOGI(TAG, "IMU task exiting");
    vTaskDelete(NULL);
}

void imu_task_start(icm20948_t *imu, NetworkManager *net,
                    int rate_hz, TaskHandle_t *handle) {
    s_imu_ctx.imu = imu;
    s_imu_ctx.net = net;
    s_imu_ctx.rate_hz = (rate_hz > 0) ? rate_hz : 100;
    s_imu_ctx.seq = 0;
    s_imu_ctx.running = true;

    xTaskCreate(imu_task_func, "imu", STACK_IMU, &s_imu_ctx, PRIO_IMU, handle);
}

void imu_task_stop(TaskHandle_t handle) {
    (void)handle;
    s_imu_ctx.running = false;
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "IMU task stopped");
}
