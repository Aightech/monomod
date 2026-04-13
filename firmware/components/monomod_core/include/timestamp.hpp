/**
 * @file timestamp.hpp
 * @brief Microsecond Timestamp Utilities
 *
 * Provides high-resolution timestamps for sample timing and synchronization.
 * Uses ESP32's hardware timer for microsecond precision.
 */

#pragma once

#include <stdint.h>
#include "esp_timer.h"
#include "esp_attr.h"

namespace axon {

/**
 * @brief Get current timestamp in microseconds since boot
 *
 * Uses esp_timer_get_time() which provides monotonic µs timestamps.
 * Wraps at ~71.6 minutes (2^32 µs) when stored as uint32_t.
 *
 * @return uint32_t Timestamp in microseconds (wrapping)
 */
IRAM_ATTR static inline uint32_t get_timestamp_us() {
    return (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
}

/**
 * @brief Get current timestamp in microseconds (64-bit, no wrap)
 *
 * @return uint64_t Full 64-bit timestamp
 */
IRAM_ATTR static inline uint64_t get_timestamp_us64() {
    return (uint64_t)esp_timer_get_time();
}

/**
 * @brief Get current timestamp in milliseconds since boot
 *
 * @return uint32_t Timestamp in milliseconds
 */
static inline uint32_t get_timestamp_ms() {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/**
 * @brief Calculate elapsed time in microseconds
 *
 * Handles 32-bit wraparound correctly.
 *
 * @param start Start timestamp
 * @param end End timestamp (default: current time)
 * @return uint32_t Elapsed microseconds
 */
static inline uint32_t elapsed_us(uint32_t start, uint32_t end = 0) {
    if (end == 0) {
        end = get_timestamp_us();
    }
    return end - start;  // Works correctly with wraparound due to unsigned math
}

/**
 * @brief Check if a timeout has elapsed
 *
 * @param start Start timestamp in µs
 * @param timeout_us Timeout duration in µs
 * @return true if timeout has elapsed
 */
static inline bool timeout_elapsed(uint32_t start, uint32_t timeout_us) {
    return elapsed_us(start) >= timeout_us;
}

/**
 * @brief Timestamp wraparound period
 *
 * 32-bit µs timestamp wraps every ~71.6 minutes
 */
constexpr uint64_t TIMESTAMP_WRAP_PERIOD_US = 0x100000000ULL;  // 2^32
constexpr uint32_t TIMESTAMP_WRAP_PERIOD_MIN = 71;             // ~71.6 minutes

/**
 * @brief Simple timestamp tracker for detecting wraparound
 */
class TimestampTracker {
public:
    TimestampTracker() : last_ts_(0), epoch_(0) {}

    /**
     * @brief Update with new timestamp and detect wraparound
     *
     * @param ts New 32-bit timestamp
     * @return uint64_t Full 64-bit timestamp accounting for wraps
     */
    uint64_t update(uint32_t ts) {
        // Detect wraparound: if new ts is much smaller than last
        // (more than half the range), assume wrap occurred
        if (ts < last_ts_ && (last_ts_ - ts) > 0x80000000) {
            epoch_++;
        }
        last_ts_ = ts;
        return ((uint64_t)epoch_ << 32) | ts;
    }

    /**
     * @brief Get current epoch (number of wraparounds)
     */
    uint32_t epoch() const { return epoch_; }

    /**
     * @brief Reset tracker
     */
    void reset() {
        last_ts_ = 0;
        epoch_ = 0;
    }

private:
    uint32_t last_ts_;
    uint32_t epoch_;
};

}  // namespace axon
