/**
 * @file ring_buffer.hpp
 * @brief Lock-free SPSC Ring Buffer for MONOMOD
 *
 * Adapted from AxonCtrl ring_buffer.hpp with smaller SampleFrame
 * for the 3-channel ADS1293.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <atomic>
#include "esp_attr.h"

/**
 * @brief Sample frame — one sample from all enabled channels
 *
 * Sized for ADS1293 (3 channels max) + headroom.
 * sizeof(SampleFrame) ≈ 48 bytes (vs 140 in axonCtrl's 32-channel version)
 */
struct SampleFrame {
    uint32_t timestamp_us;
    uint32_t sequence;
    uint8_t  num_channels;
    uint8_t  adc_type;
    uint8_t  reserved[2];
    int32_t  samples[8];           // Max 8 channels (ADS1293 = 3)
};

/**
 * @brief Lock-free SPSC Ring Buffer
 *
 * @tparam T Element type
 * @tparam SIZE Buffer size (must be power of 2)
 */
template<typename T = SampleFrame, size_t SIZE = 4096>
class RingBuffer {
public:
    static_assert((SIZE & (SIZE - 1)) == 0, "SIZE must be a power of 2");
    static_assert(SIZE >= 16, "SIZE must be at least 16");

    RingBuffer() : head_(0), tail_(0), overflow_count_(0), total_written_(0) {}

    void reset() {
        head_.store(0, std::memory_order_relaxed);
        tail_.store(0, std::memory_order_relaxed);
        overflow_count_.store(0, std::memory_order_relaxed);
        total_written_.store(0, std::memory_order_relaxed);
    }

    bool empty() const {
        return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire);
    }

    bool full() const {
        size_t next_head = (head_.load(std::memory_order_relaxed) + 1) & MASK;
        return next_head == tail_.load(std::memory_order_acquire);
    }

    size_t size() const {
        size_t h = head_.load(std::memory_order_acquire);
        size_t t = tail_.load(std::memory_order_acquire);
        return (h - t) & MASK;
    }

    size_t available() const {
        return SIZE - size() - 1;
    }

    constexpr size_t capacity() const {
        return SIZE - 1;
    }

    uint32_t overflow_count() const {
        return overflow_count_.load(std::memory_order_relaxed);
    }

    uint64_t total_written() const {
        return total_written_.load(std::memory_order_relaxed);
    }

    // ---- Producer ----

    bool push(const T& item) {
        size_t h = head_.load(std::memory_order_relaxed);
        size_t next_h = (h + 1) & MASK;
        if (next_h == tail_.load(std::memory_order_acquire)) {
            overflow_count_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        buffer_[h] = item;
        head_.store(next_h, std::memory_order_release);
        total_written_.fetch_add(1, std::memory_order_relaxed);
        return true;
    }

    bool push_overwrite(const T& item) {
        size_t h = head_.load(std::memory_order_relaxed);
        size_t next_h = (h + 1) & MASK;
        size_t t = tail_.load(std::memory_order_acquire);
        bool overflow = (next_h == t);
        if (overflow) {
            tail_.store((t + 1) & MASK, std::memory_order_release);
            overflow_count_.fetch_add(1, std::memory_order_relaxed);
        }
        buffer_[h] = item;
        head_.store(next_h, std::memory_order_release);
        total_written_.fetch_add(1, std::memory_order_relaxed);
        return !overflow;
    }

    // ---- Consumer ----

    bool pop(T& item) {
        size_t t = tail_.load(std::memory_order_relaxed);
        if (t == head_.load(std::memory_order_acquire)) return false;
        item = buffer_[t];
        tail_.store((t + 1) & MASK, std::memory_order_release);
        return true;
    }

    bool peek(T& item) const {
        size_t t = tail_.load(std::memory_order_relaxed);
        if (t == head_.load(std::memory_order_acquire)) return false;
        item = buffer_[t];
        return true;
    }

    const T* peek_contiguous(size_t& count) const {
        size_t t = tail_.load(std::memory_order_relaxed);
        size_t h = head_.load(std::memory_order_acquire);
        if (t == h) { count = 0; return nullptr; }
        count = (h > t) ? (h - t) : (SIZE - t);
        return &buffer_[t];
    }

    void consume(size_t count) {
        size_t t = tail_.load(std::memory_order_relaxed);
        tail_.store((t + count) & MASK, std::memory_order_release);
    }

    void flush() {
        tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release);
    }

private:
    static constexpr size_t MASK = SIZE - 1;
    T buffer_[SIZE];
    alignas(64) std::atomic<size_t> head_;
    alignas(64) std::atomic<size_t> tail_;
    std::atomic<uint32_t> overflow_count_;
    std::atomic<uint64_t> total_written_;
};

// Pre-configured types for MONOMOD
// 512 frames × 48 bytes ≈ 24 KB — fits comfortably in ESP32-C3 SRAM
using MonomodRingBuffer = RingBuffer<SampleFrame, 512>;
