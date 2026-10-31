// Spec: include/asl/geotag_logger.h. Tests: tests/test_geotag_logger.cpp.
#include "asl/geotag_logger.h"

#include <chrono>
#include <iomanip>
#include <stdexcept>

namespace asl {

const char* GeotagLogger::header() {
    return "capture_id,t_mono_ns,lat_deg,lon_deg,rel_alt_m,roll_deg,pitch_deg,yaw_deg,"
           "ground_speed_mps,line_index,ndvi_synthetic";
}

GeotagLogger::GeotagLogger(CaptureQueue& queue, const std::string& csv_path)
    : queue_(queue), out_(csv_path) {
    if (!out_) {
        throw std::runtime_error(csv_path + ": cannot open for writing");
    }
    out_ << header() << '\n';
    out_.flush();
    thread_ = std::thread(&GeotagLogger::run, this);
}

GeotagLogger::~GeotagLogger() { stop(); }

void GeotagLogger::stop() {
    queue_.close();  // wakes run() if it is waiting
    if (thread_.joinable()) {
        thread_.join();
    }
}

void GeotagLogger::run() {
    using namespace std::chrono_literals;
    Capture c;
    // pop_wait returns false only on timeout (keep waiting) or when closed and empty (exit).
    while (true) {
        if (!queue_.pop_wait(c, 200ms)) {
            if (queue_.closed()) {
                break;
            }
            continue;
        }
        out_ << c.id << ',' << c.t_mono_ns << ',' << std::fixed << std::setprecision(8)
             << c.lat_deg << ',' << c.lon_deg << ',' << std::setprecision(2) << c.rel_alt_m
             << ',' << c.roll_deg << ',' << c.pitch_deg << ',' << c.yaw_deg << ','
             << c.ground_speed_mps << ',' << c.line_index << ',' << std::setprecision(4)
             << c.ndvi_synthetic << '\n';
        ++written_;
    }
    out_.flush();
}

}  // namespace asl
