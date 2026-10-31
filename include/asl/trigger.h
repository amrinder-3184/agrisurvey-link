// Distance-based payload trigger: decides WHEN the camera should capture.
//
// Phase 3. YOU implement this in src/core/trigger.cpp. Tests: tests/test_trigger.cpp.
//
// Interview angle: why trigger by distance instead of by time? (Ground speed changes with
// wind and acceleration; distance keeps image overlap constant.) And what limits accuracy?
// (You only see position at the telemetry rate, so you always fire slightly late. Measure it.)
#pragma once

#include "asl/geo.h"

namespace asl {

class DistanceTrigger {
public:
    /// @throws std::invalid_argument if distance_m <= 0.
    explicit DistanceTrigger(double distance_m);

    /// Feed the latest vehicle position. Returns true if a capture should fire now.
    ///  - The first update() after construction or reset() always fires.
    ///  - After that, it fires when the straight-line distance from the position of the
    ///    LAST FIRED capture is >= distance_m.
    bool update(Vec2 position);

    /// Forget all history (e.g. at the start of a new survey line or mission).
    void reset();

    /// Number of captures fired since construction or the last reset().
    int count() const;

private:
    double distance_m_;
    bool has_fired_ = false;  // false until the first capture after construction/reset
    Vec2 last_fired_{};       // position of the last capture (valid when has_fired_)
    int count_ = 0;
};

}  // namespace asl
