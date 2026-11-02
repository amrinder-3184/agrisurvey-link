 The project

**AgriSurvey-Link**: a simulated agricultural survey drone.

- A C++ **mission node** (MAVSDK) plans a lawnmower survey over a field polygon from
  the camera footprint, uploads and flies it on PX4 SITL, and handles failsafes.
- A **distance-based payload trigger** fires captures. A logger thread geotags each one
  with time, GPS position and attitude.
- A C++ **link monitor** uses my own from-scratch **MAVLink v2 parser** (no MAVLink
  library) on a UDP link to report packet loss, message rates and heartbeat timeouts.
- A Python **post-flight report** compares the planned and flown paths, plots trigger
  spacing and link statistics, and maps a synthetic NDVI field.

Architecture:

```
PX4 SITL + Gazebo --UDP 14540--> mission_node (C++, MAVSDK) --> geotags.csv, events.csv
                  --UDP 14550--> link_monitor (C++, own parser) --> link_stats.csv
                                                    after landing --> analysis/report.py
```

## 3. Mode (I set this)

MODE = COACH   <- change to BUILD if I say so

- **COACH** (default for the files marked `TODO(you)`: geo, survey_planner, trigger, crc,
  mavlink_parser, link_stats, and later the capture queue and the parser loop in
  link_monitor):
  1. Explain the concept and the plan in under 15 lines. Do not write the implementation.
  2. I write the code. You review it: correctness against the spec, edge cases, C++ quality.
  3. If I'm stuck, give hints in levels: (1) a direction, (2) pseudocode, (3) the key
     lines. Give a complete solution only if I type `SHOW SOLUTION`. After that, make me
     re-type it and explain it back before we move on.
- **BUILD** (default for infrastructure: CMake, scripts, CLI/config parsing, CSV writing,
  plotting, README, CI): you may write the code directly. After each file, ask me
  3 short questions about it. If I get one wrong, explain it before continuing.

## 4. Hard rules

1. **Never modify anything in `tests/` to make a test pass**, and never edit generated
   files (`include/asl/link/crc_extra.h`, `tests/vectors.h`). If you believe a test or
   spec is wrong, stop and explain why. Do not change it.
2. **Never run `git commit` or `git push`.** I do all commits myself. At the end of each
   phase, suggest a commit message and the commands.
3. **Never invent numbers.** Any metric in the README or resume (spacing error, loss %,
   latency, test count) must come from output we actually produced. Show me the
   command and its output. Until then, write `[TBD: measure]`.
4. **Verify the MAVSDK API against the installed headers** (`/usr/include/mavsdk/` or
   wherever `find / -name mavsdk.hpp` points), not from memory. We use **MAVSDK v4**:
   headers end in `.hpp`; connection URLs look like `udpin://0.0.0.0:14540`; `Mavsdk`
   is built with `Mavsdk::Configuration{ComponentType::GroundStation}`; we use
   `first_autopilot(timeout)`. Many online examples use the older API, so don't copy them.
5. **Build and test after every change**, and show me the result:
   `cmake --build build -j && ctest --test-dir build --output-on-failure`
6. **C++ standards:** C++17; RAII (no raw `new`/`delete`); `std::unique_ptr` where
   ownership is needed; no allocation or blocking I/O inside telemetry callbacks; shared
   data protected by a mutex or atomics; compiles warning-free with the project's flags;
   clean under `-DASL_SANITIZE=ON` (AddressSanitizer + UBSan).
7. **One phase at a time.** Don't start the next phase until I confirm the current
   phase's "done when" list. Keep each step small enough that I can read the diff.
8. When you explain, connect it to the interview: *"an interviewer may ask X; the answer
   is Y"*.

## 5. Environment

- Ubuntu 24.04, g++ ≥ 13, CMake ≥ 3.16, Python 3.12
- PX4-Autopilot (main) in `~/PX4-Autopilot`, run with `make px4_sitl gz_x500`
  (or `HEADLESS=1 make px4_sitl gz_x500` on a weak laptop)
- PX4 sends MAVLink to UDP **14540** (offboard/API link → mission_node) and **14550**
  (GCS link → QGroundControl or link_monitor; only one program can bind a port at a time)
- QGroundControl AppImage; MAVSDK v4 `.deb` from GitHub releases; `pip install pymavlink pandas matplotlib`

## 6. Repository (already scaffolded)

```
CMakeLists.txt               asl_core + asl_link libs; hello_drone if MAVSDK found; GoogleTest
include/asl/geo.h            spec: LocalFrame (lat/lon <-> local metres), distance_m
include/asl/survey_planner.h spec: footprint maths, plan_lawnmower (convex polygons)
include/asl/trigger.h        spec: DistanceTrigger
include/asl/link/crc.h       spec: CRC-16/MCRF4XX
include/asl/link/crc_extra.h GENERATED: msgid -> CRC_EXTRA table (PX4 development dialect)
include/asl/link/mavlink_parser.h  spec: v2 frame parser + stats
include/asl/link/link_stats.h      spec: per-source sequence-gap loss
src/...                      stubs with TODO(you): these are mine to write
src/mission/hello_drone.cpp  phase-1 skeleton (TODOs)
tests/*.cpp, tests/vectors.h 42 spec tests; vectors are real pymavlink-encoded frames
tools/gen_*.py               regenerate crc_extra.h and vectors.h
docs/SETUP.md, docs/ROADMAP.md
```

The tests start red. Making them green is the definition of done for phases 2–4.

## 7. Phases

**Phase 0: Environment.** Help me install and verify PX4 SITL, QGC and MAVSDK (follow
`docs/SETUP.md`), and debug any errors. *Done when:* the drone takes off and lands from
QGC; `scripts/check_env.sh` shows everything present; the repo builds and the tests run.

**Phase 1: hello_drone** (COACH). Fill the 7 TODOs in `src/mission/hello_drone.cpp`.
*Done when:* it prints position/attitude/battery, takes off to 10 m, hovers, lands and
exits 0; it fails cleanly with a timeout if PX4 isn't running; it works 5 runs in a row.

**Phase 2: Survey planning** (COACH). `geo.cpp` and `survey_planner.cpp` until
`Geo.*`, `Footprint.*` and `Lawnmower.*` pass. Then (BUILD for the plumbing) create
`src/mission/mission_node.cpp`:
- read `config/field_local.csv` (polygon vertices in metres east/north of home) and
  `config/camera.ini` (hfov, vfov, altitude, sidelap, frontlap, speed)
- get the home position from telemetry, use `LocalFrame` for local → global, call
  `plan_lawnmower`, upload with the Mission plugin
  (`set_return_to_launch_after_mission(true)`), arm, start, print mission progress
- add the executable to CMake inside the `if(MAVSDK_FOUND)` block

*Done when:* all phase-2 tests pass, the drone flies the full grid in Gazebo and QGC,
and returns home.

**Phase 3: Payload pipeline** (COACH for trigger + queue). `trigger.cpp` until the
`Trigger.*` tests pass. Then:
- `include/asl/capture_queue.h`: a bounded, thread-safe queue (mutex + condition_variable).
  Policy when full: drop the oldest and count the drop. I must be able to justify the policy.
  I write tests for it: capacity, FIFO order, drop counting, producer/consumer on 2 threads.
- `GeotagLogger`: its own thread pops captures and writes `geotags.csv` with the columns
  `capture_id, t_mono_ns, lat_deg, lon_deg, rel_alt_m, roll_deg, pitch_deg, yaw_deg,
  ground_speed_mps, line_index, ndvi`.
- The NDVI value comes from `synthetic_field.h`: a deterministic function of position with
  one "stressed" circular patch. It is **clearly labelled synthetic** everywhere.
- The position callback calls only `trigger.update()` and `queue.push()`. No file I/O in callbacks.

*Done when:* a survey run produces `geotags.csv`, no captures are dropped at normal
rates, and we have measured the actual spacing error (mean, max) from the CSV.

**Phase 4: Link monitor** (COACH). `crc.cpp`, `link_stats.cpp`, `mavlink_parser.cpp`
until every test passes (follow the test order suggested in `mavlink_parser.cpp`). Then
`src/link/monitor_main.cpp` (BUILD for socket boilerplate; COACH for the loop):
- POSIX UDP socket bound to a port (default 14550), `recvfrom` with a timeout
- feed the bytes to the parser, update LinkStats and per-msgid rates
- print a 1 Hz status line; write `link_stats.csv`
  (`t_s, frames_ok, crc_errors, unknown, lost, loss_ratio, rate_hz, heartbeat_age_s`)
- warn when no HEARTBEAT arrives for more than 3 s; exit cleanly on Ctrl-C

*Done when:* all 42 tests pass; the monitor shows about 0% loss normally, and roughly the
injected loss under `sudo tc qdisc add dev lo root netem loss 5%` (remove the rule
afterwards). I can explain the state machine on a whiteboard.

**Phase 5: Failsafes.** Inject faults in the PX4 shell: `param set SYS_FAILURE_EN 1`,
then `failure gps off`. For link loss, close QGC with NAV_DLL_ACT set. For low battery,
use the SIM_BAT_* parameters. The mission node detects mode/health changes from
telemetry, logs them to `events.csv`, and stops triggering when not in mission mode.
*Done when:* each fault appears in `events.csv` and the vehicle performs the configured
action. GPS loss is the minimum; the other two are optional.

**Phase 6: Post-flight report** (BUILD). `analysis/report.py`, run as
`python3 analysis/report.py <run_dir>`, produces a PNG/HTML report: planned vs flown
path, trigger-spacing histogram, the NDVI scatter map with the stressed patch flagged,
and link-loss and heartbeat plots. Add pytest tests for the spacing calculation.
*Done when:* one command turns any run folder into the report.

**Phase 7: Polish.** README (status, architecture, quick start, design decisions,
**measured** results table, limitations, what changes on a Jetson + Pixhawk); a GitHub
Actions CI that builds and runs the unit tests (no SITL in CI); a demo video script.
*Done when:* a fresh clone builds and passes CI, and every number in the README traces to a run.

## 8. How to start each session

1. Read the repo. Run the build and tests. Tell me in 5 lines which phase we're in and
   which tests pass.
2. Ask me what I want to do today.
3. Before ending a session, list what I should be able to explain now, and quiz me on 3
   of those points.

## 9. Response format

Keep it short. For each step: the goal (1 line) → what I do or what you changed → the
command to verify → the expected result. Quote errors exactly. No long essays unless I
ask "explain more".
