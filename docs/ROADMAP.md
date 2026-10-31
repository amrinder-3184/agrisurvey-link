# AgriSurvey-Link: 14-day build roadmap

Plan: **Day 1 = Sat 31 Oct 2026 → Day 14 = Fri 13 Nov 2026.** About 2 hours a day on the
project; the other ~4 hours go to C++/DSA/OS/protocols (see the preparation doc).

Each day has a "done when" check and a list of things you must be able to **explain
without notes**. The explanation matters as much as the code: interviewers test it.

| Day | Date | Phase | Work | Done when | Be able to explain |
| --- | --- | --- | --- | --- | --- |
| 1 | Sat 31 Oct | 0 | Install Ubuntu 24.04 tools, PX4, QGC, MAVSDK (docs/SETUP.md) | Drone takes off and lands from QGC; repo builds; tests run (red) | What SITL is; why ports 14540 and 14550 |
| 2 | Sun 1 Nov | 1 | hello_drone TODOs 1–7 | Takeoff, hover and land, 5 runs in a row; clean timeouts | Which thread runs MAVSDK callbacks; why every Action::Result is checked |
| 3 | Mon 2 Nov | 2 | geo.cpp + footprint functions | `Geo.*` and `Footprint.*` green | Flat-Earth projection and its error; footprint and GSD formulas |
| 4 | Tue 3 Nov | 2 | plan_lawnmower + mission_node (upload, fly, RTL) | `Lawnmower.*` green; full grid flown in Gazebo | Edge intersection; horizontal edges; why lines alternate direction |
| 5 | Wed 4 Nov | 3 | DistanceTrigger + bounded capture queue + its tests | `Trigger.*` and your queue tests green | Distance vs time triggering; mutex and condition variable; drop-oldest policy |
| 6 | Thu 5 Nov | 3 | GeotagLogger thread + synthetic NDVI; first real geotags.csv | CSV produced; spacing error **measured** | Why no file I/O in callbacks; where spacing error comes from |
| 7 | Fri 6 Nov | 4 | crc.cpp + link_stats.cpp | `Crc.*` and `LinkStats.*` green | The CRC algorithm and CRC_EXTRA; 8-bit wrap in loss counting |
| 8 | Sat 7 Nov | 4 | mavlink_parser.cpp | All `Parser.*` green (42/42 total) | Frame layout; resync after a bad frame; signed frames; trimmed payloads |
| 9 | Sun 8 Nov | 4 | monitor_main.cpp (UDP socket, 1 Hz stats, CSV) + `tc netem` loss test | Monitor reports injected loss | UDP vs TCP for telemetry; recvfrom timeouts; heartbeat timeout |
| 10 | Mon 9 Nov | 5 | GPS-failure injection (+ link loss / low battery if time) | Faults in events.csv; vehicle reacts | PX4 failsafe actions; who is in control during a failsafe |
| 11 | Tue 10 Nov | 6 | analysis/report.py | One command → report from a run folder | How each plot is computed |
| 12 | Wed 11 Nov | 7 | README with measured results; CI; ASan run | CI green; ASan clean | What changes on a real Jetson + Pixhawk |
| 13 | Thu 12 Nov | 7 | Demo video; resume bullets with real numbers | Video saved offline; resume updated | 60-second and 5-minute project pitch |
| 14 | Fri 13 Nov | — | Rehearse; `git tag v1.0`; mock project interview | You can explain every file without notes | Everything above |

## If you fall behind

Cut in this order: low-battery and link-loss failsafes → NDVI map → CI → demo GIF.
**Never cut:** the survey flight, the trigger and geotag log, the parser with its tests, and GPS-loss handling.

## Fallback if PX4 + Gazebo won't run by the end of Day 2

Use ArduPilot SITL (lighter; it runs without a 3D view) and its `sim_vehicle.py`. The core
libraries, tests and parser are unchanged. Only the mission node's connection code changes.

## Checking AI-written work

- `git diff tests/` must be empty: tests are never changed to pass.
- Run with sanitizers: `cmake -S . -B build-asan -DASL_SANITIZE=ON && cmake --build build-asan -j && ctest --test-dir build-asan`
- Run the full survey 3 times. A result you can't reproduce is a result you can't claim.
- Every README number must trace to a file in a run folder.
