/**
 * @file crc16.h
 * @brief CRC-16-CCITT Implementation
 *
 * Polynomial: 0x1021
 * Initial value: 0xFFFF
 * No final XOR
 *
 * This is the standard CRC-16-CCITT used by AxonCtrl protocol.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Calculate CRC-16-CCITT checksum
 *
 * @param data Pointer to data buffer
 * @param len Length of data in bytes
 * @return uint16_t CRC-16 checksum
 */
static inline uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Update CRC-16-CCITT with a single byte (for streaming)
 *
 * @param crc Current CRC value
 * @param byte Byte to add
 * @return uint16_t Updated CRC value
 */
static inline uint16_t crc16_ccitt_update(uint16_t crc, uint8_t byte) {
    crc ^= (uint16_t)byte << 8;
    for (int j = 0; j < 8; j++) {
        if (crc & 0x8000) {
            crc = (crc << 1) ^ 0x1021;
        } else {
            crc <<= 1;
        }
    }
    return crc;
}

/**
 * @brief Verify CRC-16 of a packet
 *
 * @param data Pointer to packet data (including CRC at end)
 * @param len Total length including CRC bytes
 * @return true if CRC is valid, false otherwise
 */
static inline bool crc16_verify(const uint8_t *data, size_t len) {
    if (len < 2) return false;
    uint16_t calculated = crc16_ccitt(data, len - 2);
    uint16_t received = (uint16_t)data[len - 2] | ((uint16_t)data[len - 1] << 8);
    return calculated == received;
}

/**
 * @brief Append CRC-16 to a packet buffer
 *
 * @param data Pointer to packet data
 * @param len Length of data (CRC will be written at data[len] and data[len+1])
 */
static inline void crc16_append(uint8_t *data, size_t len) {
    uint16_t crc = crc16_ccitt(data, len);
    data[len] = (uint8_t)(crc & 0xFF);          // LSB first (little-endian)
    data[len + 1] = (uint8_t)(crc >> 8);
}

#ifdef __cplusplus
}
#endif
