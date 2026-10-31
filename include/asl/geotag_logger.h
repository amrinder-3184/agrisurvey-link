// Consumer side of the payload pipeline: a thread that pops Captures from a CaptureQueue
// and writes them to geotags.csv.
//
// RAII: the constructor opens the file and starts the thread; stop() (also called by the
// destructor) closes the queue, lets the thread drain what is left, and joins it. So every
// capture that reached the queue is written, even on Ctrl-C.
#pragma once

#include <atomic>
#include <cstdint>
#include <fstream>
#include <string>
#include <thread>

#include "asl/capture_queue.h"

namespace asl {

class GeotagLogger {
public:
    /// @throws std::runtime_error if the file cannot be opened.
    GeotagLogger(CaptureQueue& queue, const std::string& csv_path);
    ~GeotagLogger();

    GeotagLogger(const GeotagLogger&) = delete;  // owns a thread and a file: not copyable
    GeotagLogger& operator=(const GeotagLogger&) = delete;

    /// Close the queue, write the remaining captures, join the thread. Idempotent.
    void stop();

    /// Rows written so far (safe to read from any thread).
    std::uint64_t written() const { return written_; }

    /// Exact CSV header (column order of every row).
    static const char* header();

private:
    void run();

    CaptureQueue& queue_;
    std::ofstream out_;
    std::atomic<std::uint64_t> written_{0};
    std::thread thread_;  // declared last: starts only after the members above exist
};

}  // namespace asl
