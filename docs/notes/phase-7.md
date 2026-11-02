# Phase 7: polish, CI, demo, final resume bullets

## What was built
- `scripts/run_demo.sh`: one command runs PX4 headless, link_monitor during the flight, mission_node and
  the report → `runs/demo/` (committed; `.gitignore` now has `runs/*` + `!runs/demo/`).
- `.github/workflows/ci.yml`:
  - release build with warnings as errors, and an ASan/UBSan build; both run ctest
  - pytest, plus a report smoke test on `runs/demo`
  - No SITL in CI; CMake skips the MAVSDK targets.
- README: measured results with evidence paths, design decisions, limitations, the hardware path.
- `docs/demo-video-script.md` (3-minute recording plan).

## Verified
- CI simulated locally: fresh `build-ci` with `-DCMAKE_DISABLE_FIND_PACKAGE_MAVSDK=ON
  -DCMAKE_COMPILE_WARNING_AS_ERROR=ON` → MAVSDK skipped, 0 warnings, 78/78 pass.
  The real GitHub run shows after your push: `[TBD: check Actions tab]`.
- Demo `runs/demo`:
  - success, 152 captures, 0 drops
  - spacing mean 0.259 / p95 0.397 / max 0.508 m
  - link 0 lost of 117,808 frames, 375.2 frames/s, during the same flight
- [K] = 33: `grep -c '^TEST'` over test_crc (5), test_mavlink_parser (11), test_link_stats (7),
  test_parser_robustness (6) and test_link_reorder (4). ctest total 78.

## Final resume bullets (values from the files above)
- **B1.** Built a C++17 companion-computer service with MAVSDK that generates lawnmower survey missions
  from a field polygon and camera footprint, then uploads, executes and monitors them on PX4 SITL (Gazebo).
- **B2.** Implemented distance-based payload triggering through a bounded producer-consumer queue; each
  capture is geotagged with time, GPS position and attitude (**152** captures per survey, **0.26 m**
  mean spacing error).
- **B3.** Wrote a MAVLink v2 parser from scratch (frame state machine, CRC-16 with CRC_EXTRA,
  sequence-gap loss tracking) for a UDP link-health monitor, covered by **33** GoogleTest cases.
- **B4.** Validated GPS-loss and data-link-loss failsafes with PX4 fault injection; a Python report
  compares planned vs. flown paths, trigger spacing and link statistics.

Optional stronger B3 ending, also measured: "...; measured 4.88 % loss under 5 % injected packet loss."

## 60-second pitch
"I built a simulated crop-survey drone pipeline in C++17 on PX4. From a field outline and a camera model
it plans a lawnmower survey, uploads it with MAVSDK and flies it in Gazebo. A distance trigger fires the
payload every 6.2 m. Captures go through a bounded queue to a logger thread, so the flight callback
never blocks: 152 geotagged captures per survey, 0.26 m mean spacing error. I also wrote my own MAVLink
parser for a link monitor. Testing it under 5 % injected loss showed 70 % at first, because reordered
packets were counted as huge gaps. I fixed that with the RTP half-window rule and it now reads 4.88 %.
Finally I injected GPS and data-link failures: PX4 lands or returns, and my node detects the takeover
without fighting the autopilot. Everything is in CI with sanitizers, and every number traces to a run
folder."

## Interview Q&A
1. **How would this change on a real Jetson + Pixhawk?** The connection URL becomes serial or radio UDP,
   the trigger becomes GPIO or DIGICAM_CONTROL, geotags use exposure timestamps, and the node runs as a
   systemd service with role companion.
2. **Why no SITL in CI?** It needs Gazebo, a GPU-less render and minutes per flight. CI checks the code;
   flight results are committed run folders that anyone can reproduce with `run_demo.sh`.
3. **Where does the 0.26 m spacing bias come from?** Triggering happens on 10 Hz position updates, so on
   average a capture is half an update late: v/(2f) = 5/(2·10) = 0.25 m. Measured +0.259 m.
4. **What was the hardest bug?** The reordering overcount (70 % vs 5 %). It was found only because I
   tested under injected loss and cross-checked against the frame rate.
5. **What would you do next?** Trigger on interpolated position at exact distance crossings to remove the
   bias, take ground truth from the Gazebo pose, and test the battery failsafe with a drain-enabled model.
