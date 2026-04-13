/**
 * @file packet_types.h
 * @brief MONOMOD Protocol Packet Definitions
 *
 * Based on AxonCtrl Protocol v1.3, extended for ADS1293 + BNO085.
 * Shared between firmware and Python host library.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Protocol Constants
// =============================================================================

#define AXON_PROTOCOL_VERSION   0x0103  // v1.3
#define AXON_MAGIC              0xAE01
#define AXON_MAGIC_BYTE0        0x01
#define AXON_MAGIC_BYTE1        0xAE

#define AXON_HEADER_SIZE        9       // MAGIC(2) + TYPE(1) + SEQ(2) + TIMESTAMP(4)
#define AXON_CRC_SIZE           2
#define AXON_MAX_PACKET_SIZE    1400
#define AXON_MAX_PAYLOAD_SIZE   (AXON_MAX_PACKET_SIZE - AXON_HEADER_SIZE - AXON_CRC_SIZE)

// Network ports
#define AXON_UDP_DATA_PORT      5000
#define AXON_TCP_CTRL_PORT      5001

// Timing
#define AXON_STATUS_INTERVAL_MS     1000
#define AXON_KEEPALIVE_TIMEOUT_MS   5000
#define AXON_IDLE_TIMEOUT_MS        30000

// =============================================================================
// Message Types
// =============================================================================

typedef enum {
    // Data packets (D→H)
    AXON_TYPE_DATA_RAW      = 0x01,
    AXON_TYPE_DATA_DELTA    = 0x02,
    AXON_TYPE_IMU           = 0x03,
    AXON_TYPE_IMPEDANCE     = 0x04,

    // Status packets (D→H)
    AXON_TYPE_STATUS        = 0x10,
    AXON_TYPE_EVENT         = 0x11,

    // Command packets (H→D)
    AXON_TYPE_CMD           = 0x20,
    AXON_TYPE_ACK           = 0x21,
    AXON_TYPE_NACK          = 0x22,

    // Discovery (H↔D)
    AXON_TYPE_DISCOVER      = 0xFE,

    // Error (D→H)
    AXON_TYPE_ERROR         = 0xFF,
} axon_msg_type_t;

// =============================================================================
// Command IDs
// =============================================================================

typedef enum {
    // Basic commands
    AXON_CMD_PING               = 0x00,
    AXON_CMD_GET_INFO           = 0x01,
    AXON_CMD_GET_STATUS         = 0x02,

    // Streaming control
    AXON_CMD_START              = 0x10,
    AXON_CMD_STOP               = 0x11,
    AXON_CMD_SET_CHANNELS       = 0x12,
    AXON_CMD_SET_SAMPLE_RATE    = 0x13,
    AXON_CMD_SET_IMU            = 0x15,
    AXON_CMD_READ_REGISTER      = 0x16,
    AXON_CMD_WRITE_REGISTER     = 0x17,

    // Markers
    AXON_CMD_SEND_MARKER        = 0x20,

    // WiFi commands
    AXON_CMD_SET_WIFI           = 0x40,
    AXON_CMD_WIFI_SCAN          = 0x41,
    AXON_CMD_WIFI_STATUS        = 0x43,

    // System commands
    AXON_CMD_REBOOT             = 0xF0,
} axon_cmd_id_t;

// =============================================================================
// ADC Types
// =============================================================================

typedef enum {
    AXON_ADC_TYPE_ADS1299       = 0x01,     // 8ch, 24-bit, 3 bytes/sample
    AXON_ADC_TYPE_RHD2132       = 0x02,     // 32ch, 16-bit, 2 bytes/sample
    AXON_ADC_TYPE_ADS1293       = 0x03,     // 3ch, 24-bit, 3 bytes/sample
    AXON_ADC_TYPE_INA           = 0x04,     // 1ch, 16-bit, 2 bytes/sample (analog on GPIO)
} axon_adc_type_t;

#define AXON_ADS1293_SAMPLE_SIZE    3       // 24-bit packed
#define AXON_ADS1293_NUM_CHANNELS   3

#define AXON_INA_SAMPLE_SIZE        2       // 16-bit (padded from 12-bit ADC)
#define AXON_INA_NUM_CHANNELS       1

// =============================================================================
// Status Flags
// =============================================================================

typedef enum {
    AXON_FLAG_WIFI_CONNECTED    = (1 << 2),
    AXON_FLAG_STREAMING         = (1 << 5),
    AXON_FLAG_ERROR             = (1 << 6),
    AXON_FLAG_IMU_ENABLED       = (1 << 7),
} axon_status_flags_t;

// =============================================================================
// Device States
// =============================================================================

typedef enum {
    AXON_STATE_BOOT             = 0,
    AXON_STATE_IDLE             = 1,
    AXON_STATE_READY            = 2,
    AXON_STATE_STREAMING        = 3,
    AXON_STATE_ERROR            = 5,
} axon_device_state_t;

// =============================================================================
// ACK Status Codes
// =============================================================================

typedef enum {
    AXON_ACK_SUCCESS            = 0x00,
    AXON_ACK_INVALID_PARAM      = 0x01,
    AXON_ACK_NOT_SUPPORTED      = 0x02,
    AXON_ACK_BUSY               = 0x03,
    AXON_ACK_HW_ERROR           = 0x04,
} axon_ack_status_t;

// =============================================================================
// Feature Flags
// =============================================================================

typedef enum {
    AXON_FEATURE_IMU            = (1 << 0),
    AXON_FEATURE_USB_CDC        = (1 << 4),
} axon_feature_flags_t;

// =============================================================================
// Packet Structures (packed, little-endian)
// =============================================================================

#pragma pack(push, 1)

/** Common packet header (9 bytes) */
typedef struct {
    uint16_t magic;             // 0xAE01
    uint8_t  type;              // axon_msg_type_t
    uint16_t seq;               // Sequence number
    uint32_t timestamp;         // Microseconds since boot
} axon_header_t;

/** Data packet payload header (6 bytes) */
typedef struct {
    uint32_t ch_mask;           // Channel enable bitmask
    uint8_t  n_samples;         // Samples per channel
    uint8_t  adc_type;          // axon_adc_type_t
} axon_data_header_t;

/** Status packet payload (23 bytes) */
typedef struct {
    uint8_t  battery_pct;
    uint16_t battery_mv;
    uint8_t  flags;             // axon_status_flags_t
    uint8_t  temp_c;
    uint32_t packets_sent;
    uint32_t packets_dropped;
    int8_t   rssi;
    uint8_t  state;             // axon_device_state_t
    uint32_t sample_rate_hz;
    uint32_t channel_mask;
} axon_status_payload_t;

/** IMU packet payload — ICM-42670 style (12 bytes) */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} axon_imu_payload_t;

/** IMU packet payload — BNO085 with quaternions (22 bytes) */
typedef struct {
    int16_t quat_i;             // Q14 fixed-point (divide by 16384.0 for float)
    int16_t quat_j;
    int16_t quat_k;
    int16_t quat_real;
    int16_t accel_x;            // mg (divide by 1000.0 for g)
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;             // 0.1 deg/s (divide by 10.0 for dps)
    int16_t gyro_y;
    int16_t gyro_z;
    uint8_t accuracy;           // 0-3
    uint8_t imu_type;           // 0=ICM42670, 1=BNO085
} axon_imu_bno085_payload_t;

/** Event packet payload */
typedef struct {
    uint8_t event_id;
    uint8_t marker;
} axon_event_payload_t;

/** Command packet payload */
typedef struct {
    uint8_t cmd_id;             // axon_cmd_id_t
} axon_cmd_payload_t;

/** ACK packet payload */
typedef struct {
    uint8_t cmd_id;
    uint8_t status;             // axon_ack_status_t
} axon_ack_payload_t;

/** Device info (28 bytes) — response to GET_INFO */
typedef struct {
    uint8_t  fw_major;
    uint8_t  fw_minor;
    uint8_t  fw_patch;
    uint8_t  fw_build;
    uint16_t hw_rev;
    char     serial[12];
    uint8_t  adc_type;          // axon_adc_type_t
    uint8_t  num_channels;
    uint32_t features;
    uint16_t sample_size;
    uint32_t max_sample_rate;
} axon_device_info_t;

/** START command parameters (8 bytes) */
typedef struct {
    uint32_t sample_rate;
    uint32_t ch_mask;
} axon_cmd_start_params_t;

/** SET_IMU command parameters (2 bytes) */
typedef struct {
    uint8_t enable;
    uint8_t rate_div;
} axon_cmd_set_imu_params_t;

/** READ_REGISTER command parameters (1 byte) */
typedef struct {
    uint8_t reg_addr;
} axon_cmd_read_reg_params_t;

/** READ_REGISTER response (2 bytes) */
typedef struct {
    uint8_t reg_addr;
    uint8_t value;
} axon_read_reg_response_t;

/** WRITE_REGISTER command parameters (2 bytes) */
typedef struct {
    uint8_t reg_addr;
    uint8_t value;
} axon_cmd_write_reg_params_t;

/** SEND_MARKER command parameters (1 byte) */
typedef struct {
    uint8_t marker_id;
} axon_cmd_send_marker_params_t;

#pragma pack(pop)

// =============================================================================
// Helper Macros
// =============================================================================

#define AXON_DATA_PAYLOAD_SIZE(ch_count, n_samples, sample_size) \
    (sizeof(axon_data_header_t) + ((ch_count) * (n_samples) * (sample_size)))

#define AXON_MAX_SAMPLES_PER_PACKET(ch_count, sample_size) \
    ((AXON_MAX_PAYLOAD_SIZE - sizeof(axon_data_header_t)) / ((ch_count) * (sample_size)))

static inline uint8_t axon_popcount32(uint32_t x) {
    x = x - ((x >> 1) & 0x55555555);
    x = (x & 0x33333333) + ((x >> 2) & 0x33333333);
    x = (x + (x >> 4)) & 0x0F0F0F0F;
    x = x + (x >> 8);
    x = x + (x >> 16);
    return (uint8_t)(x & 0x3F);
}

#ifdef __cplusplus
}
#endif
