// Phase 3. Spec: include/asl/trigger.h. Tests: tests/test_trigger.cpp.
#include "asl/trigger.h"

#include <stdexcept>

namespace asl {

DistanceTrigger::DistanceTrigger(double distance_m) : distance_m_(distance_m) {
    if (!(distance_m > 0.0)) {  // also rejects NaN, which "distance_m <= 0" would let through
        throw std::invalid_argument("DistanceTrigger: distance_m must be > 0");
    }
}

bool DistanceTrigger::update(Vec2 position) {
    // Measure from the last FIRED position, not the last update: telemetry samples are
    // irregular, and summing small steps would accumulate rounding and path wiggle.
    if (has_fired_ && distance_m(last_fired_, position) < distance_m_) {
        return false;
    }
    has_fired_ = true;
    last_fired_ = position;
    ++count_;
    return true;
}

void DistanceTrigger::reset() {
    has_fired_ = false;
    count_ = 0;
}

int DistanceTrigger::count() const {
    return count_;
}

}  // namespace asl
