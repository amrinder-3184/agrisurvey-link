// Bounded, thread-safe FIFO between the telemetry callback (producer) and the geotag
// logger thread (consumer).
//
// Design (phase 3):
//  - Fixed-size ring buffer allocated once in the constructor: push() never allocates and
//    never blocks, so it is safe to call from a MAVSDK callback.
//  - When full, push() OVERWRITES THE OLDEST item and counts it in dropped(). If the logger
//    falls behind, the newest geotag is the most valuable; blocking the producer instead
//    would stall MAVSDK's callback thread and delay all telemetry.
//  - pop_wait() sleeps on a condition variable until an item arrives, the timeout expires,
//    or close() is called. After close(), remaining items can still be popped (drained),
//    so shutdown does not lose captures that were already queued.
#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

namespace asl {

/// One payload capture, geotagged at the moment the trigger fired.
struct Capture {
    std::uint64_t id = 0;
    std::int64_t t_mono_ns = 0;  ///< steady_clock time of the trigger
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    double rel_alt_m = 0.0;
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;
    double ground_speed_mps = 0.0;
    int line_index = -1;
    double ndvi_synthetic = 0.0;  ///< SYNTHETIC value from synthetic_field.h, not real data
};

class CaptureQueue {
public:
    /// @throws std::invalid_argument if capacity == 0.
    explicit CaptureQueue(std::size_t capacity);

    /// Never blocks. Returns false if an old item had to be dropped (or the queue is closed,
    /// in which case the item is ignored).
    bool push(const Capture& item);

    /// Wait up to `timeout` for an item. Returns false on timeout, or when the queue is
    /// closed AND empty (the consumer's signal to exit).
    bool pop_wait(Capture& out, std::chrono::milliseconds timeout);

    /// Stop accepting items and wake any waiting consumer.
    void close();

    std::size_t size() const;
    std::size_t capacity() const { return buffer_.size(); }
    std::uint64_t dropped() const;
    bool closed() const;

private:
    mutable std::mutex mutex_;
    std::condition_variable not_empty_;
    std::vector<Capture> buffer_;  // fixed size == capacity
    std::size_t head_ = 0;         // index of the oldest item
    std::size_t count_ = 0;        // number of items stored
    std::uint64_t dropped_ = 0;
    bool closed_ = false;
};

}  // namespace asl
