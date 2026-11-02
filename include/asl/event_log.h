// Mission event log (phase 5): events.csv with one row per state change.
//
//   t_mono_ns,t_s,event,detail
//   12345678901,41.5,mode,Mission
//   ...,62.0,gps_fix,NoGps
//
// Used from the main thread only: mission_node polls MAVSDK's latest values twice a second
// and logs what changed. No MAVSDK callback ever touches this (callbacks do no I/O).
#pragma once

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>

namespace asl {

// Remembers the last accepted value. changed() is true when a new value is accepted: the
// first value, or a different value that has now been seen `confirm` polls IN A ROW.
//   confirm = 1: every transition is reported (no filtering).
//   confirm = 2: a value must persist for 2 consecutive polls. This filters a signal that
//                flips back and forth every poll (seen in SITL: MAVSDK's local-position
//                health alternated ok/LOST at ~1 Hz during a GPS-loss blind landing).
// T needs operator== and to be copyable.
template <typename T>
class Watch {
public:
    explicit Watch(int confirm = 1) : confirm_(confirm < 1 ? 1 : confirm) {}

    bool changed(const T& value) {
        if (has_value_ && value == last_) {
            pending_count_ = 0;  // back to the accepted value: drop any candidate
            return false;
        }
        if (pending_count_ > 0 && value == pending_) {
            ++pending_count_;
        } else {
            pending_ = value;  // a new candidate value
            pending_count_ = 1;
        }
        if (pending_count_ < confirm_) {
            return false;
        }
        has_value_ = true;
        last_ = value;
        pending_count_ = 0;
        return true;
    }

private:
    int confirm_;
    bool has_value_ = false;
    T last_{};
    T pending_{};
    int pending_count_ = 0;
};

class EventLog {
public:
    // Creates (truncates) the file and writes the header. Throws std::runtime_error if the
    // file cannot be opened.
    explicit EventLog(const std::string& path);

    // Appends one row and flushes it, so the log survives a crash or a kill. Events are rare
    // (a few per minute), so a flush per row costs nothing.
    void log(std::int64_t t_mono_ns, double t_s, const std::string& event,
             const std::string& detail);

    std::size_t count() const { return count_; }

private:
    std::ofstream out_;
    std::size_t count_ = 0;
};

// CSV field quoting: wraps in quotes (doubling inner quotes) if the text has , " or newline.
std::string csv_field(const std::string& text);

}  // namespace asl
