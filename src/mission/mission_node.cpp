// mission_node: plan a lawnmower survey from config, upload it to PX4 with MAVSDK, fly it,
// trigger the (simulated) payload by distance, geotag every capture, and return home.
//
// Usage:
//   mission_node --url udpin://0.0.0.0:14540 --field config/field_local.csv
//                --camera config/camera.ini --out runs/ --role gcs
//
//   --role gcs        (default) mission_node is the ground station: its heartbeat is the
//                     link PX4 monitors for NAV_DLL_ACT.
//   --role companion  mission_node runs as an onboard computer; a separate GCS must provide
//                     the ground-station heartbeat (used for the data-link-loss test).
//
// Output folder runs/<YYYYmmdd-HHMMSS>/:
//   mission_plan.csv  planned waypoints (phase 2)
//   geotags.csv       one row per capture (phase 3)
//   telemetry.csv     2 Hz flown track (phase 3)
//   events.csv        mode / armed / landed / GPS / position / battery / mission changes,
//                     and our own commands (phase 5)
//   run_meta.json     config, versions, counts, outcome
//
// Threading:
//   MAVSDK callback thread: position -> DistanceTrigger -> CaptureQueue::push (no I/O,
//                           no allocation); attitude/velocity/mode/progress store "latest".
//   GeotagLogger thread:    CaptureQueue::pop_wait -> geotags.csv
//   main thread:            commands, waits, status line, telemetry.csv, events.csv
//
// Failsafes: if PX4 leaves Mission mode before the last item (GPS loss, link loss, low
// battery...), the autopilot is in charge. We log it, send no commands, and wait to land.
//
// Exit codes: 0 ok, 1 usage/config/files, 2 no autopilot, 3 not ready (health/home),
//             4 command failed, 5 timeout, 6 mission aborted by an autopilot failsafe,
//             130 interrupted (Ctrl-C).

#include <mavsdk/mavsdk.hpp>
#include <mavsdk/plugins/action/action.hpp>
#include <mavsdk/plugins/info/info.hpp>
#include <mavsdk/plugins/mission/mission.hpp>
#include <mavsdk/plugins/telemetry/telemetry.hpp>

#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "asl/capture_queue.h"
#include "asl/config.h"
#include "asl/event_log.h"
#include "asl/geo.h"
#include "asl/geotag_logger.h"
#include "asl/survey_planner.h"
#include "asl/synthetic_field.h"
#include "asl/trigger.h"

using namespace mavsdk;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace {

constexpr double kAutopilotTimeoutS = 10.0;
constexpr auto kReadyTimeout = 60s;
constexpr auto kClimbTimeout = 45s;
constexpr auto kLandTimeout = 180s;          // RTL: climb, fly home, descend
constexpr auto kTick = 500ms;                // telemetry.csv rate (2 Hz)
constexpr double kTelemetryRateHz = 10.0;    // position/attitude/velocity stream rate
constexpr std::size_t kQueueCapacity = 256;  // captures buffered for the logger

// Set by the SIGINT handler; polled by every wait loop. A signal handler may only touch
// lock-free atomics (no I/O, no locks), so this is all it does.
std::atomic<bool> g_stop{false};

void on_sigint(int) { g_stop = true; }

std::int64_t mono_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// State shared between MAVSDK callbacks and main(). main() declares it BEFORE the MAVSDK
// objects, so it is destroyed after them: no callback can outlive the data it touches.
struct Shared {
    std::atomic<int> progress_current{0};
    std::atomic<int> progress_total{0};
    std::atomic<Telemetry::FlightMode> mode{Telemetry::FlightMode::Unknown};

    std::mutex m;  // guards attitude and velocity
    Telemetry::EulerAngle attitude{};
    Telemetry::VelocityNed velocity{};

    // Used only inside the position callback (a single MAVSDK thread), so no lock.
    int current_line = -1;
    std::uint64_t next_capture_id = 1;
};

struct Args {
    std::string url = "udpin://0.0.0.0:14540";
    std::string field = "config/field_local.csv";
    std::string camera = "config/camera.ini";
    std::string out = "runs";
    std::string role = "gcs";  // gcs | companion
};

void usage(const char* bin) {
    std::cerr << "Usage: " << bin
              << " [--url URL] [--field CSV] [--camera INI] [--out DIR] [--role gcs|companion]\n"
                 "  defaults: --url udpin://0.0.0.0:14540 --field config/field_local.csv\n"
                 "            --camera config/camera.ini --out runs --role gcs\n";
}

bool parse_args(int argc, char** argv, Args& args) {
    for (int i = 1; i < argc; ++i) {
        const std::string flag = argv[i];
        if (i + 1 >= argc) {
            return false;  // every flag needs a value
        }
        const std::string value = argv[++i];
        if (flag == "--url") {
            args.url = value;
        } else if (flag == "--field") {
            args.field = value;
        } else if (flag == "--camera") {
            args.camera = value;
        } else if (flag == "--out") {
            args.out = value;
        } else if (flag == "--role" && (value == "gcs" || value == "companion")) {
            args.role = value;
        } else {
            return false;
        }
    }
    return true;
}

template <typename T>
std::string text(const T& value) {  // anything MAVSDK can print with <<
    std::ostringstream s;
    s << value;
    return s.str();
}

std::string local_time(const char* format) {
    const std::time_t now = std::time(nullptr);
    std::tm tm{};
    localtime_r(&now, &tm);  // thread-safe variant of localtime
    std::ostringstream s;
    s << std::put_time(&tm, format);
    return s.str();
}

enum class Wait { Done, Timeout, Interrupted };

// Poll done() once per tick until it is true, the timeout expires or Ctrl-C is pressed.
// on_tick() runs every tick (status line + telemetry.csv).
Wait wait_until(std::chrono::seconds timeout, const std::function<bool()>& done,
                const std::function<void()>& on_tick) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (g_stop) {
            return Wait::Interrupted;
        }
        if (done()) {
            return Wait::Done;
        }
        on_tick();
        std::this_thread::sleep_for(kTick);
    }
    return done() ? Wait::Done : Wait::Timeout;
}

double path_length_m(const std::vector<asl::Vec2>& wps) {
    double total = 0.0;
    asl::Vec2 prev{0.0, 0.0};  // start and end at home (the local origin)
    for (const asl::Vec2& p : wps) {
        total += asl::distance_m(prev, p);
        prev = p;
    }
    return total + asl::distance_m(prev, {0.0, 0.0});
}

void write_plan_csv(const fs::path& path, const std::vector<asl::Vec2>& local,
                    const asl::LocalFrame& frame, double alt_m) {
    std::ofstream f(path);
    if (!f) {
        throw std::runtime_error(path.string() + ": cannot write");
    }
    f << "index,east_m,north_m,lat_deg,lon_deg,alt_m,line_index\n" << std::fixed;
    for (std::size_t i = 0; i < local.size(); ++i) {
        const asl::LatLon g = frame.to_global(local[i]);
        f << i << ',' << std::setprecision(3) << local[i].east_m << ',' << local[i].north_m << ','
          << std::setprecision(8) << g.lat_deg << ',' << g.lon_deg << ',' << std::setprecision(1)
          << alt_m << ',' << i / 2 << '\n';  // 2 waypoints (entry, exit) per line
    }
}

// Everything that goes into run_meta.json. Filled in as the run progresses.
struct RunMeta {
    Args args;
    asl::SurveyConfig cfg;
    double line_spacing_m = NAN;
    double trigger_distance_m = NAN;
    std::size_t waypoints = 0;
    double planned_path_m = NAN;
    asl::LatLon home{NAN, NAN};
    std::string mavsdk_version;
    std::string px4_version = "unknown";
    std::string start_time;
    std::string end_time;
    std::string outcome = "not started";
    std::uint64_t captures = 0;
    std::uint64_t queue_drops = 0;
    std::size_t events = 0;
};

std::string json_string(const std::string& s) {
    std::string out = "\"";
    for (char c : s) {
        if (c == '"' || c == '\\') {
            out += '\\';
        }
        out += c;
    }
    return out + "\"";
}

std::string json_number(double v) {
    if (!std::isfinite(v)) {
        return "null";  // JSON has no NaN
    }
    std::ostringstream s;
    s << std::setprecision(10) << v;
    return s.str();
}

void write_meta_json(const fs::path& path, const RunMeta& m) {
    std::ofstream f(path);
    if (!f) {
        std::cerr << "ERROR: cannot write " << path << "\n";
        return;
    }
    f << "{\n"
      << "  \"outcome\": " << json_string(m.outcome) << ",\n"
      << "  \"start_time\": " << json_string(m.start_time) << ",\n"
      << "  \"end_time\": " << json_string(m.end_time) << ",\n"
      << "  \"px4_version\": " << json_string(m.px4_version) << ",\n"
      << "  \"mavsdk_version\": " << json_string(m.mavsdk_version) << ",\n"
      << "  \"connection_url\": " << json_string(m.args.url) << ",\n"
      << "  \"role\": " << json_string(m.args.role) << ",\n"
      << "  \"field_file\": " << json_string(m.args.field) << ",\n"
      << "  \"camera_file\": " << json_string(m.args.camera) << ",\n"
      << "  \"home_lat_deg\": " << json_number(m.home.lat_deg) << ",\n"
      << "  \"home_lon_deg\": " << json_number(m.home.lon_deg) << ",\n"
      << "  \"hfov_deg\": " << json_number(m.cfg.camera.hfov_deg) << ",\n"
      << "  \"vfov_deg\": " << json_number(m.cfg.camera.vfov_deg) << ",\n"
      << "  \"altitude_m\": " << json_number(m.cfg.survey.altitude_m) << ",\n"
      << "  \"sidelap\": " << json_number(m.cfg.survey.sidelap) << ",\n"
      << "  \"frontlap\": " << json_number(m.cfg.survey.frontlap) << ",\n"
      << "  \"speed_mps\": " << json_number(m.cfg.speed_mps) << ",\n"
      << "  \"line_spacing_m\": " << json_number(m.line_spacing_m) << ",\n"
      << "  \"trigger_distance_m\": " << json_number(m.trigger_distance_m) << ",\n"
      << "  \"telemetry_rate_hz\": " << json_number(kTelemetryRateHz) << ",\n"
      << "  \"waypoints\": " << m.waypoints << ",\n"
      << "  \"planned_path_m\": " << json_number(m.planned_path_m) << ",\n"
      << "  \"captures\": " << m.captures << ",\n"
      << "  \"queue_capacity\": " << kQueueCapacity << ",\n"
      << "  \"queue_drops\": " << m.queue_drops << ",\n"
      << "  \"events\": " << m.events << ",\n"
      << "  \"ndvi_note\": \"ndvi_synthetic values are SYNTHETIC, not real crop data\"\n"
      << "}\n";
}

std::string clean_git_hash(const std::string& raw) {
    // PX4 sends the git hash as a fixed-size byte array; keep printable chars only
    // (the raw string contained NUL padding, which corrupted the JSON file).
    std::string hash;
    for (char c : raw) {
        if (std::isprint(static_cast<unsigned char>(c)) && c != ' ') {
            hash += c;
        }
    }
    return hash;
}

}  // namespace

int main(int argc, char** argv) {
    Args args;
    if (!parse_args(argc, argv, args)) {
        usage(argv[0]);
        return 1;
    }
    std::signal(SIGINT, on_sigint);

    // ---- Config and run folder (before touching the vehicle) -------------------------
    RunMeta meta;
    meta.args = args;
    meta.start_time = local_time("%Y-%m-%dT%H:%M:%S%z");
    std::vector<asl::Vec2> field;
    try {
        meta.cfg = asl::load_camera_ini(args.camera);
        field = asl::load_field_csv(args.field);
    } catch (const std::exception& e) {
        std::cerr << "ERROR: config: " << e.what() << "\n";
        return 1;
    }
    meta.line_spacing_m = asl::line_spacing_m(meta.cfg.camera, meta.cfg.survey);
    meta.trigger_distance_m = asl::trigger_distance_m(meta.cfg.camera, meta.cfg.survey);

    const fs::path run_dir = fs::path(args.out) / local_time("%Y%m%d-%H%M%S");
    std::error_code ec;
    fs::create_directories(run_dir, ec);
    if (ec) {
        std::cerr << "ERROR: cannot create " << run_dir << ": " << ec.message() << "\n";
        return 1;
    }
    std::cout << "Run folder: " << run_dir.string() << "\n";

    // ---- Payload pipeline (declared before MAVSDK objects: outlives all callbacks) ----
    Shared shared;
    asl::DistanceTrigger trigger(meta.trigger_distance_m);  // position callback only
    asl::CaptureQueue queue(kQueueCapacity);
    std::ofstream telemetry_csv(run_dir / "telemetry.csv");
    if (!telemetry_csv) {
        std::cerr << "ERROR: cannot write telemetry.csv\n";
        return 1;
    }
    telemetry_csv << "t_mono_ns,lat_deg,lon_deg,rel_alt_m,flight_mode\n";
    std::unique_ptr<asl::GeotagLogger> logger;
    try {
        logger = std::make_unique<asl::GeotagLogger>(queue, (run_dir / "geotags.csv").string());
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }
    std::unique_ptr<asl::EventLog> events;
    try {
        events = std::make_unique<asl::EventLog>((run_dir / "events.csv").string());
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }
    const std::int64_t t0_ns = mono_ns();
    const auto event = [&](const std::string& name, const std::string& detail) {
        const std::int64_t now = mono_ns();
        events->log(now, static_cast<double>(now - t0_ns) * 1e-9, name, detail);
    };

    // Every exit from here on stops the logger (flushing geotags.csv) and records why.
    const auto finish = [&](const std::string& outcome, int code) {
        event("outcome", outcome + " (exit " + std::to_string(code) + ")");
        meta.events = events->count();
        logger->stop();
        telemetry_csv.flush();
        meta.captures = logger->written();
        meta.queue_drops = queue.dropped();
        meta.outcome = outcome;
        meta.end_time = local_time("%Y-%m-%dT%H:%M:%S%z");
        write_meta_json(run_dir / "run_meta.json", meta);
        std::cout << "Captures: " << meta.captures << ", queue drops: " << meta.queue_drops
                  << "\nOutcome: " << outcome << " (exit " << code << ")\n";
        return code;
    };

    // ---- Connect -----------------------------------------------------------------------
    // The role decides the MAVLink heartbeat type we send: MAV_TYPE_GCS for a ground
    // station, MAV_TYPE_ONBOARD_CONTROLLER for a companion. PX4's "GCS connection" (and its
    // NAV_DLL_ACT failsafe) only counts GCS heartbeats.
    Mavsdk mavsdk{Mavsdk::Configuration{args.role == "companion" ? ComponentType::CompanionComputer
                                                                  : ComponentType::GroundStation}};
    meta.mavsdk_version = mavsdk.version();
    const ConnectionResult conn = mavsdk.add_any_connection(args.url);
    if (conn != ConnectionResult::Success) {
        std::cerr << "ERROR: connection '" << args.url << "' failed: " << conn << "\n";
        return finish("connection failed", 1);
    }
    const auto system = mavsdk.first_autopilot(kAutopilotTimeoutS);
    if (!system) {
        std::cerr << "ERROR: no autopilot within " << kAutopilotTimeoutS << " s\n";
        return finish("no autopilot", 2);
    }
    Telemetry telemetry{system.value()};
    Action action{system.value()};
    Mission mission{system.value()};
    Info info{system.value()};

    // ---- Event detection: poll MAVSDK's cached values, log transitions ----------------
    // The getters return the latest value MAVSDK received (no MAVLink round trip), so this
    // is cheap. Resolution is one tick (500 ms): enough for mode and health changes.
    // Health flags must hold for 2 polls (1 s) before they are logged: during a GPS-loss
    // blind landing MAVSDK's local-position health flipped every poll (run 20261102-045711).
    asl::Watch<std::string> w_mode, w_landed, w_fix;
    asl::Watch<bool> w_armed;
    asl::Watch<bool> w_global_ok(2), w_local_ok(2), w_home_ok(2);
    asl::Watch<int> w_item, w_battery_10pct;
    // Log `detail` under `name` if `value` differs from the previous poll.
    const auto track = [&](auto& watch, const char* name, const auto& value,
                           const std::string& detail) {
        if (watch.changed(value)) {
            event(name, detail);
        }
    };
    const auto ok_text = [](bool ok) { return std::string(ok ? "ok" : "LOST"); };
    const auto poll_events = [&] {
        const std::string mode = text(telemetry.flight_mode());
        const std::string landed = text(telemetry.landed_state());
        const std::string fix = text(telemetry.gps_info().fix_type);
        const bool armed = telemetry.armed();
        const Telemetry::Health h = telemetry.health();
        // MAVSDK has no name for some PX4 modes (e.g. the failsafe Descend / "blind land").
        track(w_mode, "mode", mode,
              mode == "Unknown" ? "Unknown (no MAVSDK name; e.g. PX4 failsafe Descend)" : mode);
        track(w_armed, "armed", armed, armed ? "yes" : "no");
        track(w_landed, "landed_state", landed, landed);
        track(w_fix, "gps_fix", fix, fix);
        track(w_global_ok, "global_position", h.is_global_position_ok,
              ok_text(h.is_global_position_ok));
        track(w_local_ok, "local_position", h.is_local_position_ok,
              ok_text(h.is_local_position_ok));
        track(w_home_ok, "home_position", h.is_home_position_ok, ok_text(h.is_home_position_ok));

        const float battery = telemetry.battery().remaining_percent;  // 0..100, NaN if unknown
        if (std::isfinite(battery)) {
            const int bucket = static_cast<int>(battery / 10.0f) * 10;  // log every 10 % step
            track(w_battery_10pct, "battery", bucket, ">= " + std::to_string(bucket) + " %");
        }
        const int item = shared.progress_current;
        const int total = shared.progress_total;
        if (total > 0) {
            track(w_item, "mission_item", item, std::to_string(item) + "/" + std::to_string(total));
        }
    };

    int tick = 0;
    const auto on_tick = [&] {  // main thread: events, 2 Hz track row, 1 Hz status line
        poll_events();
        const Telemetry::Position p = telemetry.position();
        const std::string mode = text(telemetry.flight_mode());
        telemetry_csv << mono_ns() << ',' << std::fixed << std::setprecision(8) << p.latitude_deg
                      << ',' << p.longitude_deg << ',' << std::setprecision(2)
                      << p.relative_altitude_m << ',' << mode << '\n';
        if (tick++ % 2 == 0) {
            std::cout << std::fixed << std::setprecision(1) << "mode " << mode << " | item "
                      << shared.progress_current << "/" << shared.progress_total << " | alt "
                      << p.relative_altitude_m << " m | captures " << logger->written() << "\n";
        }
    };

    // ---- Ready to fly: armable and a valid home position ------------------------------
    std::cout << "Waiting up to " << kReadyTimeout.count() << " s for health and home...\n";
    const Wait ready = wait_until(
        kReadyTimeout,
        [&] {
            const Telemetry::Health h = telemetry.health();
            return telemetry.health_all_ok() && h.is_armable && h.is_home_position_ok;
        },
        poll_events);
    if (ready == Wait::Interrupted) {
        return finish("interrupted before takeoff", 130);
    }
    if (ready == Wait::Timeout) {
        std::cerr << "ERROR: vehicle not armable or no home position\n";
        return finish("not ready", 3);
    }
    const Telemetry::HomePosition home = telemetry.home();
    if (!std::isfinite(home.latitude_deg) || !std::isfinite(home.longitude_deg)) {
        std::cerr << "ERROR: home position not valid\n";
        return finish("no home position", 3);
    }
    meta.home = {home.latitude_deg, home.longitude_deg};
    const auto [ver_result, ver] = info.get_version();
    if (ver_result == Info::Result::Success) {
        meta.px4_version = std::to_string(ver.flight_sw_major) + "." +
                           std::to_string(ver.flight_sw_minor) + "." +
                           std::to_string(ver.flight_sw_patch) + " (" +
                           clean_git_hash(ver.flight_sw_git_hash) + ")";
    }

    // ---- Plan in the local frame, convert to GPS ---------------------------------------
    const asl::LocalFrame frame(meta.home);
    const std::vector<asl::Vec2> wps = asl::plan_lawnmower(field, meta.line_spacing_m);
    if (wps.empty()) {
        std::cerr << "ERROR: planner produced no waypoints (field smaller than line spacing?)\n";
        return finish("empty plan", 1);
    }
    meta.waypoints = wps.size();
    meta.planned_path_m = path_length_m(wps);
    try {
        write_plan_csv(run_dir / "mission_plan.csv", wps, frame, meta.cfg.survey.altitude_m);
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return finish("cannot write plan", 1);
    }
    std::cout << "Plan: " << wps.size() << " waypoints (" << wps.size() / 2
              << " lines), spacing " << std::fixed << std::setprecision(2) << meta.line_spacing_m
              << " m, trigger distance " << meta.trigger_distance_m << " m, path "
              << std::setprecision(0) << meta.planned_path_m << " m\n";

    Mission::MissionPlan plan;
    for (const asl::Vec2& p : wps) {
        const asl::LatLon g = frame.to_global(p);
        Mission::MissionItem item;
        item.latitude_deg = g.lat_deg;
        item.longitude_deg = g.lon_deg;
        item.relative_altitude_m = static_cast<float>(meta.cfg.survey.altitude_m);
        item.speed_m_s = static_cast<float>(meta.cfg.speed_mps);
        item.is_fly_through = true;  // don't stop at each waypoint
        plan.mission_items.push_back(item);
    }

    // ---- Telemetry streams and callbacks -----------------------------------------------
    for (const auto& [name, result] :
         {std::pair{"position", telemetry.set_rate_position(kTelemetryRateHz)},
          std::pair{"attitude", telemetry.set_rate_attitude_euler(kTelemetryRateHz)},
          std::pair{"velocity", telemetry.set_rate_velocity_ned(kTelemetryRateHz)}}) {
        if (result != Telemetry::Result::Success) {
            std::cerr << "WARN: set_rate_" << name << " failed: " << result << "\n";
        }
    }
    telemetry.subscribe_flight_mode([&](Telemetry::FlightMode m) { shared.mode = m; });
    telemetry.subscribe_attitude_euler([&](Telemetry::EulerAngle a) {
        std::lock_guard<std::mutex> lock(shared.m);
        shared.attitude = a;
    });
    telemetry.subscribe_velocity_ned([&](Telemetry::VelocityNed v) {
        std::lock_guard<std::mutex> lock(shared.m);
        shared.velocity = v;
    });
    mission.subscribe_mission_progress([&](Mission::MissionProgress p) {
        shared.progress_current = p.current;
        shared.progress_total = p.total;
    });
    // The producer. Runs on MAVSDK's callback thread for every position sample: decide,
    // build a Capture, push. Nothing here blocks, allocates or does I/O.
    telemetry.subscribe_position([&shared, &trigger, &queue, frame](Telemetry::Position p) {
        // On a survey leg = Mission mode and flying TOWARDS a line's exit waypoint, which
        // is an odd mission item (planner output: 2k = entry, 2k + 1 = exit of line k).
        const int current = shared.progress_current;
        if (shared.mode != Telemetry::FlightMode::Mission || current % 2 == 0 ||
            current >= shared.progress_total) {
            return;
        }
        const int line = current / 2;
        if (line != shared.current_line) {
            trigger.reset();  // each line starts with a capture at its entry point
            shared.current_line = line;
        }
        const asl::Vec2 local = frame.to_local({p.latitude_deg, p.longitude_deg});
        if (!trigger.update(local)) {
            return;
        }
        asl::Capture c;
        c.id = shared.next_capture_id++;
        c.t_mono_ns = mono_ns();
        c.lat_deg = p.latitude_deg;
        c.lon_deg = p.longitude_deg;
        c.rel_alt_m = p.relative_altitude_m;
        {
            std::lock_guard<std::mutex> lock(shared.m);
            c.roll_deg = shared.attitude.roll_deg;
            c.pitch_deg = shared.attitude.pitch_deg;
            c.yaw_deg = shared.attitude.yaw_deg;
            c.ground_speed_mps = std::hypot(shared.velocity.north_m_s, shared.velocity.east_m_s);
        }
        c.line_index = line;
        c.ndvi_synthetic = asl::ndvi_synthetic(local.east_m, local.north_m);
        queue.push(c);
    });

    // ---- Upload ------------------------------------------------------------------------
    Mission::Result mr = mission.set_return_to_launch_after_mission(true);
    if (mr != Mission::Result::Success) {
        std::cerr << "ERROR: set_return_to_launch_after_mission: " << mr << "\n";
        return finish("rtl setting failed", 4);
    }
    std::cout << "Uploading mission...\n";
    mr = mission.upload_mission(plan);
    if (mr != Mission::Result::Success) {
        std::cerr << "ERROR: upload_mission: " << mr << "\n";
        return finish("upload failed", 4);
    }

    // ---- Arm and take off --------------------------------------------------------------
    const auto abort_rtl = [&](const std::string& why, int code) {
        std::cerr << "ERROR: " << why << " -> return to launch\n";
        event("command", "return_to_launch (" + why + ")");
        const Action::Result r = action.return_to_launch();
        if (r != Action::Result::Success) {
            std::cerr << "ERROR: return_to_launch also failed: " << r << "\n";
        }
        return finish(why, code);
    };
    Action::Result ar = action.set_takeoff_altitude(static_cast<float>(meta.cfg.survey.altitude_m));
    if (ar != Action::Result::Success) {
        std::cerr << "ERROR: set_takeoff_altitude: " << ar << "\n";
        return finish("takeoff altitude failed", 4);
    }
    event("command", "arm");
    ar = action.arm();
    if (ar != Action::Result::Success) {
        std::cerr << "ERROR: arm: " << ar << "\n";
        return finish("arm failed", 4);
    }
    std::cout << "Armed. Taking off to " << meta.cfg.survey.altitude_m << " m...\n";
    event("command", "takeoff");
    ar = action.takeoff();
    if (ar != Action::Result::Success) {
        return abort_rtl("takeoff failed: " + text(ar), 4);
    }
    const auto target = static_cast<float>(0.95 * meta.cfg.survey.altitude_m);
    const Wait climbed = wait_until(
        kClimbTimeout, [&] { return telemetry.position().relative_altitude_m >= target; },
        on_tick);
    if (climbed == Wait::Interrupted) {
        return abort_rtl("interrupted during climb", 130);
    }
    if (climbed == Wait::Timeout) {
        return abort_rtl("did not reach survey altitude", 5);
    }

    // ---- Fly the survey ----------------------------------------------------------------
    event("command", "start_mission");
    mr = mission.start_mission();
    if (mr != Mission::Result::Success) {
        return abort_rtl("start_mission failed: " + text(mr), 4);
    }
    // Generous limit from the plan itself: twice the nominal flight time plus 2 minutes.
    const auto mission_timeout = std::chrono::seconds(
        static_cast<long>(2.0 * meta.planned_path_m / meta.cfg.speed_mps) + 120);
    std::cout << "Mission started (timeout " << mission_timeout.count() << " s)\n";
    const auto finished = [&] {
        return shared.progress_total > 0 && shared.progress_current == shared.progress_total;
    };
    // Mode switches are not instant, so "left Mission" only counts after we have seen it.
    bool seen_mission = false;
    const Wait flown = wait_until(
        mission_timeout,
        [&] {
            const bool in_mission = shared.mode == Telemetry::FlightMode::Mission;
            seen_mission = seen_mission || in_mission;
            return finished() || (seen_mission && !in_mission);
        },
        on_tick);
    if (flown == Wait::Interrupted) {
        return abort_rtl("interrupted during mission", 130);
    }
    if (flown == Wait::Timeout) {
        return abort_rtl("mission did not finish in time", 5);
    }
    if (!finished()) {
        // After the last item PX4 switches to Return itself, and the final progress message
        // may arrive just after the mode change: give it a moment before calling a failsafe.
        wait_until(3s, finished, on_tick);
    }
    if (!finished()) {
        // PX4 took over (failsafe or a pilot/GCS mode change). Commanding anything now could
        // fight the autopilot's decision, so we only record what happens until touchdown.
        const std::string why = "aborted by autopilot (mode " + text(shared.mode.load()) +
                                " at item " + std::to_string(shared.progress_current.load()) +
                                "/" + std::to_string(shared.progress_total.load()) + ")";
        std::cerr << "WARNING: mission " << why << "; waiting for the vehicle to land\n";
        event("failsafe", why);
        const Wait down = wait_until(kLandTimeout, [&] { return !telemetry.in_air(); }, on_tick);
        if (down == Wait::Interrupted) {
            return finish(why + ", interrupted before landing", 130);
        }
        if (down == Wait::Timeout) {
            return finish(why + ", did not land in time", 5);
        }
        on_tick();
        return finish(why + ", landed", 6);
    }

    // ---- Return to launch (automatic after the last item) and land --------------------
    std::cout << "Survey complete; returning to launch...\n";
    const Wait landed = wait_until(kLandTimeout, [&] { return !telemetry.in_air(); }, on_tick);
    if (landed == Wait::Interrupted) {
        return finish("interrupted during return (vehicle continues RTL)", 130);
    }
    if (landed == Wait::Timeout) {
        return finish("did not land in time", 5);
    }
    on_tick();
    return finish("success", 0);
}
