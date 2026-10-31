# Phase 1: hello_drone

## What was built

`src/mission/hello_drone.cpp`: connects to PX4 SITL over MAVSDK v4.1.0, prints position,
attitude and battery at 2 Hz, then arms, takes off to 10 m, hovers 10 s, lands and exits.

Acceptance (measured 2026-10-31, PX4 `main` 789389fd46, headless Gazebo):
- 5 consecutive runs against one PX4 instance: all exit 0, 38-39 s each, max altitude 10.0 m.
  PX4 log: 5x `Armed by external command`, `Takeoff detected`, `Landing detected`,
  `Disarmed by landing`.
- Failure paths: no arguments -> exit 1; invalid URL -> `Invalid connection URL`, exit 1;
  PX4 not running -> `no autopilot found`, exit 2 after 10.0 s (no hang).
- 0 compiler warnings; ASan + UBSan build flown once: exit 0, 0 sanitizer reports.

## Key decisions

- **Callbacks only store, main() prints.** MAVSDK runs subscription callbacks on its own
  internal thread (stated in `mavsdk.hpp`). The callbacks copy the newest value into a
  `Latest` struct under a `std::mutex`; main() prints a status line at 2 Hz. One thread
  owns `std::cout`, and callbacks never block on I/O. This is the pattern Phase 3 needs.
- **Every wait has a timeout** (autopilot 10 s, health 60 s, climb 30 s, land 60 s), and every
  `Action::Result` is checked. Distinct exit codes (1-5) make failures scriptable.
- **Failure after arming -> land first** (`fail_and_land`), so the program never leaves an
  armed vehicle without a command.
- **Declaration order is a lifetime guarantee.** `Latest` is declared before `Telemetry`;
  locals are destroyed in reverse order, so the subscriptions (lambdas holding a reference
  to `latest`) are destroyed first.

## Edge cases found by testing

- `health_all_ok()` was true about 1 s after connecting, but `arm()` returned
  `Command Denied` (PX4: `Arming denied: Resolve system health failures first`).
  `health_all_ok()` covers sensors and position; PX4's arming checks are in
  `health().is_armable`. Fix: wait for both. After the fix, the GCS requirement was also
  satisfied by MAVSDK itself (it identifies as a GroundStation) once PX4 had received its
  heartbeats; no QGC was needed.
- MAVSDK provides `to_string(Action::Result)`, so a local helper with the same name was
  ambiguous (found by argument-dependent lookup). Renamed to `result_text`.
- SITL battery drains from about 97% to 50% in one 40 s flight (simulated drain); relevant
  to the optional low-battery test in Phase 5.

## What an interviewer may ask

1. **Which thread runs MAVSDK callbacks, and what does that mean for your code?**
   A MAVSDK-internal thread, not main(). Any data shared between a callback and main() (or
   between two callbacks) needs a mutex or atomics, and callbacks must be quick: no file
   I/O or blocking calls, otherwise they delay every other callback and message.

2. **Why check every `Action::Result`?**
   Commands can be denied (arming checks), time out (lost link) or be unsupported. If
   `arm()` fails and you call `takeoff()` anyway, the program's model of the vehicle is
   wrong. Checking each result lets the program stop or recover (land) deliberately.

3. **Your first run failed to arm even though health was OK. Why?**
   `health_all_ok()` does not include PX4's arming checks; one of them (GCS connected) was
   not yet satisfied about 1 s after connecting. Waiting on `health().is_armable` fixed it.
   I found it from PX4's log line, not by guessing.

4. **How does the program avoid hanging forever?**
   Every blocking step has a deadline based on `std::chrono::steady_clock` (monotonic, not
   affected by wall-clock changes). On timeout it prints what was missing and exits with a
   specific non-zero code; if the vehicle might be armed, it sends `land()` first.

5. **Why is `Latest` declared before `Telemetry` in main()?**
   C++ destroys local objects in reverse order of declaration. The telemetry subscriptions
   hold lambdas that reference `latest`; destroying `Telemetry` first removes them, so no
   callback can run against a destroyed `latest` (a use-after-free).
