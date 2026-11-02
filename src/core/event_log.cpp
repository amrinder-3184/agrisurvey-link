// Phase 5. Spec: include/asl/event_log.h. Tests: tests/test_event_log.cpp.
#include "asl/event_log.h"

#include <iomanip>
#include <stdexcept>

namespace asl {

EventLog::EventLog(const std::string& path) : out_(path) {
    if (!out_) {
        throw std::runtime_error(path + ": cannot write");
    }
    out_ << "t_mono_ns,t_s,event,detail\n";
    out_.flush();
}

void EventLog::log(std::int64_t t_mono_ns, double t_s, const std::string& event,
                   const std::string& detail) {
    out_ << t_mono_ns << ',' << std::fixed << std::setprecision(1) << t_s << ','
         << csv_field(event) << ',' << csv_field(detail) << '\n';
    out_.flush();
    ++count_;
}

std::string csv_field(const std::string& text) {
    if (text.find_first_of(",\"\n") == std::string::npos) {
        return text;
    }
    std::string quoted = "\"";
    for (char c : text) {
        if (c == '"') {
            quoted += '"';  // CSV escapes a quote by doubling it
        }
        quoted += c;
    }
    return quoted + "\"";
}

}  // namespace asl
