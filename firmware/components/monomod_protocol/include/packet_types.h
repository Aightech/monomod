/**
 * @file packet_types.h
 * @brief MONOMOD Protocol Packet Definitions
 *
 * Based on MONOMOD Protocol v1.3, extended for ADS1293 + BNO085.
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

#define MONOMOD_PROTOCOL_VERSION   0x0103  // v1.3
#define MONOMOD_MAGIC              0xAE01
#define MONOMOD_MAGIC_BYTE0        0x01
#define MONOMOD_MAGIC_BYTE1        0xAE

#define MONOMOD_HEADER_SIZE        9       // MAGIC(2) + TYPE(1) + SEQ(2) + TIMESTAMP(4)
#define MONOMOD_CRC_SIZE           2
#define MONOMOD_MAX_PACKET_SIZE    1400
#define MONOMOD_MAX_PAYLOAD_SIZE   (MONOMOD_MAX_PACKET_SIZE - MONOMOD_HEADER_SIZE - MONOMOD_CRC_SIZE)

// Network ports
#define MONOMOD_UDP_DATA_PORT      5000
#define MONOMOD_TCP_CTRL_PORT      5001

// Timing
#define MONOMOD_STATUS_INTERVAL_MS     1000
#define MONOMOD_KEEPALIVE_TIMEOUT_MS   5000
#define MONOMOD_IDLE_TIMEOUT_MS        30000

// =============================================================================
// Message Types
// =============================================================================

typedef enum {
    // Data packets (D→H)
    MONOMOD_TYPE_DATA_RAW      = 0x01,
    MONOMOD_TYPE_DATA_DELTA    = 0x02,
    MONOMOD_TYPE_IMU           = 0x03,
    MONOMOD_TYPE_IMPEDANCE     = 0x04,

    // Status packets (D→H)
    MONOMOD_TYPE_STATUS        = 0x10,
    MONOMOD_TYPE_EVENT         = 0x11,

    // Command packets (H→D)
    MONOMOD_TYPE_CMD           = 0x20,
    MONOMOD_TYPE_ACK           = 0x21,
    MONOMOD_TYPE_NACK          = 0x22,

    // Discovery (H↔D)
    MONOMOD_TYPE_DISCOVER      = 0xFE,

    // Error (D→H)
    MONOMOD_TYPE_ERROR         = 0xFF,
} monomod_msg_type_t;

// =============================================================================
// Command IDs
// =============================================================================

typedef enum {
    // Basic commands
    MONOMOD_CMD_PING               = 0x00,
    MONOMOD_CMD_GET_INFO           = 0x01,
    MONOMOD_CMD_GET_STATUS         = 0x02,

    // Streaming control
    MONOMOD_CMD_START              = 0x10,
    MONOMOD_CMD_STOP               = 0x11,
    MONOMOD_CMD_SET_CHANNELS       = 0x12,
    MONOMOD_CMD_SET_SAMPLE_RATE    = 0x13,
    MONOMOD_CMD_SET_IMU            = 0x15,
    MONOMOD_CMD_READ_REGISTER      = 0x16,
    MONOMOD_CMD_WRITE_REGISTER     = 0x17,

    // Markers
    MONOMOD_CMD_SEND_MARKER        = 0x20,

    // WiFi commands
    MONOMOD_CMD_SET_WIFI           = 0x40,
    MONOMOD_CMD_WIFI_SCAN          = 0x41,
    MONOMOD_CMD_WIFI_STATUS        = 0x43,

    // System commands
    MONOMOD_CMD_REBOOT             = 0xF0,
} monomod_cmd_id_t;

// =============================================================================
// ADC Types
// =============================================================================

typedef enum {
    MONOMOD_ADC_TYPE_ADS1299       = 0x01,     // 8ch, 24-bit, 3 bytes/sample
    MONOMOD_ADC_TYPE_RHD2132       = 0x02,     // 32ch, 16-bit, 2 bytes/sample
    MONOMOD_ADC_TYPE_ADS1293       = 0x03,     // 3ch, 24-bit, 3 bytes/sample
    MONOMOD_ADC_TYPE_INA           = 0x04,     // 1ch, 16-bit, 2 bytes/sample (analog on GPIO)
} monomod_adc_type_t;

#define MONOMOD_ADS1293_SAMPLE_SIZE    3       // 24-bit packed
#define MONOMOD_ADS1293_NUM_CHANNELS   3

#define MONOMOD_INA_SAMPLE_SIZE        2       // 16-bit (padded from 12-bit ADC)
#define MONOMOD_INA_NUM_CHANNELS       1

// =============================================================================
// Status Flags
// =============================================================================

typedef enum {
    MONOMOD_FLAG_WIFI_CONNECTED    = (1 << 2),
    MONOMOD_FLAG_STREAMING         = (1 << 5),
    MONOMOD_FLAG_ERROR             = (1 << 6),
    MONOMOD_FLAG_IMU_ENABLED       = (1 << 7),
} monomod_status_flags_t;

// =============================================================================
// Device States
// =============================================================================

typedef enum {
    MONOMOD_STATE_BOOT             = 0,
    MONOMOD_STATE_IDLE             = 1,
    MONOMOD_STATE_READY            = 2,
    MONOMOD_STATE_STREAMING        = 3,
    MONOMOD_STATE_ERROR            = 5,
} monomod_device_state_t;

// =============================================================================
// ACK Status Codes
// =============================================================================

typedef enum {
    MONOMOD_ACK_SUCCESS            = 0x00,
    MONOMOD_ACK_INVALID_PARAM      = 0x01,
    MONOMOD_ACK_NOT_SUPPORTED      = 0x02,
    MONOMOD_ACK_BUSY               = 0x03,
    MONOMOD_ACK_HW_ERROR           = 0x04,
} monomod_ack_status_t;

// =============================================================================
// Feature Flags
// =============================================================================

typedef enum {
    MONOMOD_FEATURE_IMU            = (1 << 0),
    MONOMOD_FEATURE_USB_CDC        = (1 << 4),
} monomod_feature_flags_t;

// =============================================================================
// Packet Structures (packed, little-endian)
// =============================================================================

#pragma pack(push, 1)

/** Common packet header (9 bytes) */
typedef struct {
    uint16_t magic;             // 0xAE01
    uint8_t  type;              // monomod_msg_type_t
    uint16_t seq;               // Sequence number
    uint32_t timestamp;         // Microseconds since boot
} monomod_header_t;

/** Data packet payload header (6 bytes) */
typedef struct {
    uint32_t ch_mask;           // Channel enable bitmask
    uint8_t  n_samples;         // Samples per channel
    uint8_t  adc_type;          // monomod_adc_type_t
} monomod_data_header_t;

/** Status packet payload (23 bytes) */
typedef struct {
    uint8_t  battery_pct;
    uint16_t battery_mv;
    uint8_t  flags;             // monomod_status_flags_t
    uint8_t  temp_c;
    uint32_t packets_sent;
    uint32_t packets_dropped;
    int8_t   rssi;
    uint8_t  state;             // monomod_device_state_t
    uint32_t sample_rate_hz;
    uint32_t channel_mask;
} monomod_status_payload_t;

/** IMU packet payload — ICM-42670 style (12 bytes) */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} monomod_imu_payload_t;

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
} monomod_imu_bno085_payload_t;

/** Event packet payload */
typedef struct {
    uint8_t event_id;
    uint8_t marker;
} monomod_event_payload_t;

/** Command packet payload */
typedef struct {
    uint8_t cmd_id;             // monomod_cmd_id_t
} monomod_cmd_payload_t;

/** ACK packet payload */
typedef struct {
    uint8_t cmd_id;
    uint8_t status;             // monomod_ack_status_t
} monomod_ack_payload_t;

/** Device info (28 bytes) — response to GET_INFO */
typedef struct {
    uint8_t  fw_major;
    uint8_t  fw_minor;
    uint8_t  fw_patch;
    uint8_t  fw_build;
    uint16_t hw_rev;
    char     serial[12];
    uint8_t  adc_type;          // monomod_adc_type_t
    uint8_t  num_channels;
    uint32_t features;
    uint16_t sample_size;
    uint32_t max_sample_rate;
} monomod_device_info_t;

/** START command parameters (8 bytes) */
typedef struct {
    uint32_t sample_rate;
    uint32_t ch_mask;
} monomod_cmd_start_params_t;

/** SET_IMU command parameters (2 bytes) */
typedef struct {
    uint8_t enable;
    uint8_t rate_div;
} monomod_cmd_set_imu_params_t;

/** READ_REGISTER command parameters (1 byte) */
typedef struct {
    uint8_t reg_addr;
} monomod_cmd_read_reg_params_t;

/** READ_REGISTER response (2 bytes) */
typedef struct {
    uint8_t reg_addr;
    uint8_t value;
} monomod_read_reg_response_t;

/** WRITE_REGISTER command parameters (2 bytes) */
typedef struct {
    uint8_t reg_addr;
    uint8_t value;
} monomod_cmd_write_reg_params_t;

/** SEND_MARKER command parameters (1 byte) */
typedef struct {
    uint8_t marker_id;
} monomod_cmd_send_marker_params_t;

#pragma pack(pop)

// =============================================================================
// Wire-layout guards — these sizes are mirrored in driver/src/monomod/protocol.py
// (struct format strings). If you change a struct, update the Python side AND the
// size here; test_protocol.py asserts the Python calcsize matches the same values.
// =============================================================================
#ifdef __cplusplus
#define MONOMOD_SASSERT(c, m) static_assert(c, m)
#else
#define MONOMOD_SASSERT(c, m) _Static_assert(c, m)
#endif

MONOMOD_SASSERT(sizeof(monomod_header_t)        == 9,  "header must be 9 bytes");
MONOMOD_SASSERT(sizeof(monomod_data_header_t)   == 6,  "data header must be 6 bytes");
MONOMOD_SASSERT(sizeof(monomod_imu_payload_t)   == 12, "imu payload must be 12 bytes");
MONOMOD_SASSERT(sizeof(monomod_status_payload_t) == 23, "status payload must be 23 bytes");
MONOMOD_SASSERT(sizeof(monomod_device_info_t)   == 30, "device info must be 30 bytes");
MONOMOD_SASSERT(sizeof(monomod_ack_payload_t)   == 2,  "ack payload must be 2 bytes");

// =============================================================================
// Helper Macros
// =============================================================================

#define MONOMOD_DATA_PAYLOAD_SIZE(ch_count, n_samples, sample_size) \
    (sizeof(monomod_data_header_t) + ((ch_count) * (n_samples) * (sample_size)))

#define MONOMOD_MAX_SAMPLES_PER_PACKET(ch_count, sample_size) \
    ((MONOMOD_MAX_PAYLOAD_SIZE - sizeof(monomod_data_header_t)) / ((ch_count) * (sample_size)))

static inline uint8_t monomod_popcount32(uint32_t x) {
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
