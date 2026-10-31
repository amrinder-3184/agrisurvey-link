// Phase 1 - hello_drone: connect to PX4 SITL with MAVSDK, print telemetry,
// then arm -> take off -> hover -> land.
//
// Usage:   ./build/hello_drone udpin://0.0.0.0:14540
//
// Threading model: MAVSDK runs our subscription callbacks on its own internal thread.
// The callbacks only copy the newest values into `Latest` (mutex-protected); main()
// does all printing. So std::cout is used from one thread only, and callbacks never
// block on I/O.
//
// Exit codes: 0 ok, 1 usage/connection, 2 no autopilot, 3 not healthy,
//             4 command failed, 5 did not reach altitude or land in time.

#include <mavsdk/mavsdk.hpp>
#include <mavsdk/plugins/action/action.hpp>
#include <mavsdk/plugins/telemetry/telemetry.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

using namespace mavsdk;
using namespace std::chrono_literals;

namespace {

constexpr float kTakeoffAltitudeM = 10.0f;
constexpr double kAutopilotTimeoutS = 10.0;
constexpr auto kHealthTimeout = 60s;
constexpr auto kClimbTimeout = 30s;
constexpr auto kHoverTime = 10s;
constexpr auto kLandTimeout = 60s;
constexpr auto kPrintPeriod = 500ms;  // 2 Hz status line

// Newest telemetry values. Written on MAVSDK's callback thread, read by main():
// every access must hold `m`.
struct Latest {
    std::mutex m;
    bool has_position = false;
    Telemetry::Position position{};
    Telemetry::EulerAngle attitude{};
    float battery_percent = NAN;  // MAVSDK reports 0..100, NAN until known
};

void usage(const char* bin) {
    std::cerr << "Usage: " << bin << " <connection_url>\n"
              << "  PX4 SITL offboard/API link: udpin://0.0.0.0:14540\n";
}

std::string result_text(Action::Result r) {
    std::ostringstream s;
    s << r;
    return s.str();
}

void print_status(Latest& latest) {
    std::lock_guard<std::mutex> lock(latest.m);
    if (!latest.has_position) {
        std::cout << "waiting for position...\n";
        return;
    }
    const auto& p = latest.position;
    const auto& a = latest.attitude;
    std::cout << std::fixed << std::setprecision(1) << "alt " << p.relative_altitude_m << " m | "
              << std::setprecision(7) << "lat " << p.latitude_deg << " lon " << p.longitude_deg
              << " | " << std::setprecision(1) << "roll " << a.roll_deg << " pitch " << a.pitch_deg
              << " yaw " << a.yaw_deg << " | batt " << std::setprecision(0)
              << latest.battery_percent << "%\n";
}

// Print the status line at 2 Hz until done() returns true or the timeout expires.
// Returns true if done() became true, false on timeout.
bool wait_until(Latest& latest, std::chrono::milliseconds timeout,
                const std::function<bool()>& done) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (done()) {
            return true;
        }
        print_status(latest);
        std::this_thread::sleep_for(kPrintPeriod);
    }
    return done();
}

void print_health(const Telemetry::Health& h) {
    std::cerr << "  gyro_cal=" << h.is_gyrometer_calibration_ok
              << " accel_cal=" << h.is_accelerometer_calibration_ok
              << " mag_cal=" << h.is_magnetometer_calibration_ok
              << " local_pos=" << h.is_local_position_ok
              << " global_pos=" << h.is_global_position_ok
              << " home=" << h.is_home_position_ok << " armable=" << h.is_armable << "\n"
              << "  (armable=0 with everything else 1: is a GCS connected? PX4 needs one"
                 " to arm while NAV_DLL_ACT is enabled)\n";
}

// Something failed while the vehicle may be armed or flying: ask it to land, then exit.
int fail_and_land(Action& action, const std::string& what, int exit_code) {
    std::cerr << "ERROR: " << what << " -> sending land\n";
    const Action::Result r = action.land();
    if (r != Action::Result::Success) {
        std::cerr << "ERROR: land also failed: " << r << "\n";
    }
    return exit_code;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }
    const std::string url = argv[1];

    // We identify ourselves to PX4 as a ground station.
    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};

    // 1. Open the connection (for udpin this binds the local UDP port).
    const ConnectionResult conn = mavsdk.add_any_connection(url);
    if (conn != ConnectionResult::Success) {
        std::cerr << "ERROR: connection '" << url << "' failed: " << conn << "\n";
        return 1;
    }

    // 2. Wait for PX4's heartbeat.
    std::cout << "Waiting up to " << kAutopilotTimeoutS << " s for an autopilot on " << url
              << "...\n";
    const auto system = mavsdk.first_autopilot(kAutopilotTimeoutS);
    if (!system) {
        std::cerr << "ERROR: no autopilot found (is PX4 SITL running?)\n";
        return 2;
    }
    std::cout << "Autopilot found.\n";

    // 3. Plugins. `latest` is declared before `telemetry`, so it is destroyed after it:
    //    the subscriptions (which reference `latest`) are gone before `latest` is.
    Latest latest;
    Telemetry telemetry{system.value()};
    Action action{system.value()};

    // 4. Telemetry subscriptions: callbacks only store values.
    const Telemetry::Result rate = telemetry.set_rate_position(2.0);
    if (rate != Telemetry::Result::Success) {
        std::cerr << "WARN: set_rate_position(2 Hz) failed: " << rate << "\n";
    }
    telemetry.subscribe_position([&latest](Telemetry::Position p) {
        std::lock_guard<std::mutex> lock(latest.m);
        latest.position = p;
        latest.has_position = true;
    });
    telemetry.subscribe_attitude_euler([&latest](Telemetry::EulerAngle a) {
        std::lock_guard<std::mutex> lock(latest.m);
        latest.attitude = a;
    });
    telemetry.subscribe_battery([&latest](Telemetry::Battery b) {
        std::lock_guard<std::mutex> lock(latest.m);
        latest.battery_percent = b.remaining_percent;
    });

    // 5. Wait until PX4 reports it is ready to arm. health_all_ok() covers sensors and
    //    position only; PX4's arming checks (e.g. "GCS connected") are in is_armable.
    //    Measured: health_all_ok() alone was true while PX4 still denied arming.
    std::cout << "Waiting up to " << kHealthTimeout.count() << " s for vehicle health...\n";
    const auto ready = [&telemetry] {
        return telemetry.health_all_ok() && telemetry.health().is_armable;
    };
    if (!wait_until(latest, kHealthTimeout, ready)) {
        std::cerr << "ERROR: vehicle not healthy after " << kHealthTimeout.count() << " s:\n";
        print_health(telemetry.health());
        return 3;
    }

    // 6. Arm, take off, hover, land. Every Action::Result is checked.
    Action::Result r = action.set_takeoff_altitude(kTakeoffAltitudeM);
    if (r != Action::Result::Success) {
        std::cerr << "ERROR: set_takeoff_altitude failed: " << r << "\n";
        return 4;
    }
    std::cout << "Arming...\n";
    r = action.arm();
    if (r != Action::Result::Success) {
        std::cerr << "ERROR: arm failed: " << r << "\n";
        return 4;
    }
    std::cout << "Taking off to " << kTakeoffAltitudeM << " m...\n";
    r = action.takeoff();
    if (r != Action::Result::Success) {
        return fail_and_land(action, "takeoff failed: " + result_text(r), 4);
    }
    const bool reached = wait_until(latest, kClimbTimeout, [&latest] {
        std::lock_guard<std::mutex> lock(latest.m);
        return latest.position.relative_altitude_m >= 0.95f * kTakeoffAltitudeM;
    });
    if (!reached) {
        return fail_and_land(action, "did not reach takeoff altitude in time", 5);
    }

    std::cout << "Hovering for " << kHoverTime.count() << " s...\n";
    wait_until(latest, kHoverTime, [] { return false; });  // only prints while hovering

    std::cout << "Landing...\n";
    r = action.land();
    if (r != Action::Result::Success) {
        std::cerr << "ERROR: land failed: " << r << "\n";
        return 4;
    }

    // 7. Wait until PX4's land detector says we are on the ground.
    if (!wait_until(latest, kLandTimeout, [&telemetry] { return !telemetry.in_air(); })) {
        std::cerr << "ERROR: still in air " << kLandTimeout.count() << " s after land\n";
        return 5;
    }
    print_status(latest);
    std::cout << "Landed. Done.\n";
    return 0;
}
