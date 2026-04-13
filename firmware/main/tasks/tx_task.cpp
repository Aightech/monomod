#include "tx_task.hpp"
#include "config.hpp"
#include "packet_types.h"
#include "crc16.h"
#include "timestamp.hpp"
#include "esp_log.h"
#include <cstring>

static const char *TAG = "tx";

struct TxContext {
    MonomodRingBuffer *ring;
    NetworkManager *net;
    SemaphoreHandle_t sem;
    uint16_t seq;
    volatile bool running;
};

static TxContext s_tx_ctx;

// Batch window: how long to wait between packet sends.
// At 3200 SPS, 30ms → ~96 samples per batch.
#define TX_BATCH_WINDOW_MS  30

// Cap samples per UDP packet to keep packet size small (~350 bytes).
// Some WiFi/router combinations silently drop larger UDP datagrams.
// A batch of 96 samples will be split into ~3 packets of 32 samples each.
//   32 samples × 3ch × 3B = 288 bytes + 17B headers + 2B CRC = 307 bytes
#define TX_MAX_SAMPLES_PER_PKT  32

static size_t send_one_packet(TxContext *ctx, const SampleFrame *frames,
                              size_t count, uint8_t *packet) {
    // Read packing info from the first frame (all frames in a batch share it)
    uint8_t adc_type = frames[0].adc_type;
    uint8_t n_ch     = frames[0].num_channels;

    // Header
    axon_header_t *hdr = (axon_header_t *)packet;
    hdr->magic = AXON_MAGIC;
    hdr->type = AXON_TYPE_DATA_RAW;
    hdr->seq = ctx->seq++;
    hdr->timestamp = frames[0].timestamp_us;

    // Data header — channel mask = low n_ch bits
    axon_data_header_t *dhdr = (axon_data_header_t *)(packet + AXON_HEADER_SIZE);
    dhdr->ch_mask = (1U << n_ch) - 1;
    dhdr->n_samples = (uint8_t)count;
    dhdr->adc_type = adc_type;

    uint8_t *data = packet + AXON_HEADER_SIZE + sizeof(axon_data_header_t);

    if (adc_type == AXON_ADC_TYPE_ADS1293) {
        // 24-bit signed samples, big-endian, interleaved
        for (size_t s = 0; s < count; s++) {
            for (uint8_t ch = 0; ch < n_ch; ch++) {
                int32_t v = frames[s].samples[ch];
                *data++ = (uint8_t)((v >> 16) & 0xFF);
                *data++ = (uint8_t)((v >>  8) & 0xFF);
                *data++ = (uint8_t)( v        & 0xFF);
            }
        }
    } else if (adc_type == AXON_ADC_TYPE_INA) {
        // 16-bit signed samples, big-endian, 1 channel
        for (size_t s = 0; s < count; s++) {
            int16_t v = (int16_t)frames[s].samples[0];
            *data++ = (uint8_t)((v >> 8) & 0xFF);
            *data++ = (uint8_t)( v       & 0xFF);
        }
    } else {
        // Unknown type — emit zero payload
    }

    size_t pkt_len = data - packet;
    crc16_append(packet, pkt_len);
    pkt_len += AXON_CRC_SIZE;

    ctx->net->send_udp(packet, pkt_len);
    return pkt_len;
}

static void tx_task_func(void *arg) {
    TxContext *ctx = (TxContext *)arg;
    uint8_t packet[AXON_MAX_PACKET_SIZE];

    const size_t theoretical_max = AXON_MAX_SAMPLES_PER_PACKET(
        ADS1293_NUM_CHANNELS, AXON_ADS1293_SAMPLE_SIZE);
    const size_t max_samples = (theoretical_max < TX_MAX_SAMPLES_PER_PKT)
                             ? theoretical_max : TX_MAX_SAMPLES_PER_PKT;

    while (ctx->running) {
        // Fixed batch window: collect ~30ms of samples before sending.
        // The semaphore is still "given" by ACQ but we ignore it here and poll
        // on a timer — this is simpler and gives predictable batch sizes.
        vTaskDelay(pdMS_TO_TICKS(TX_BATCH_WINDOW_MS));
        if (!ctx->running) break;

        // Drain everything accumulated. May take multiple packets if the
        // ring buffer wrapped (i.e., previous send was delayed).
        while (ctx->running) {
            size_t count = 0;
            const SampleFrame *frames = ctx->ring->peek_contiguous(count);
            if (!frames || count == 0) break;

            size_t batch = (count > max_samples) ? max_samples : count;
            send_one_packet(ctx, frames, batch, packet);
            ctx->ring->consume(batch);

            // If we took everything in this contiguous chunk, stop — next pass
            // through the outer delay loop will pick up anything new.
            if (batch == count) break;
        }

        // Drain the semaphore to prevent it piling up (binary sem so at most 1)
        xSemaphoreTake(ctx->sem, 0);
    }

    ESP_LOGI(TAG, "TX task exiting");
    vTaskDelete(NULL);
}

void tx_task_start(MonomodRingBuffer *ring, NetworkManager *net,
                   SemaphoreHandle_t sem, TaskHandle_t *handle) {
    s_tx_ctx.ring = ring;
    s_tx_ctx.net = net;
    s_tx_ctx.sem = sem;
    s_tx_ctx.seq = 0;
    s_tx_ctx.running = true;

    xTaskCreate(tx_task_func, "tx", STACK_TX, &s_tx_ctx, PRIO_TX, handle);
    ESP_LOGI(TAG, "TX task started");
}

void tx_task_stop(TaskHandle_t handle) {
    s_tx_ctx.running = false;
    if (s_tx_ctx.sem) xSemaphoreGive(s_tx_ctx.sem);  // unblock
    if (handle) {
        vTaskDelay(pdMS_TO_TICKS(50));  // let it exit
    }
    ESP_LOGI(TAG, "TX task stopped");
}
