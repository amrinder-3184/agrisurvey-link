# AgriSurvey-Link test report

All numbers come from runs in `runs/` produced by the commands shown. Simulation only:
PX4 SITL (main @789389fd4, v1.18.0-beta) + Gazebo Harmonic, x500 quad, MAVSDK v4.1.0, WSL2 Ubuntu 24.04.

## Unit tests
`ctest` in `build/` and in `build-asan/` (AddressSanitizer + UBSan): **78/78 pass**, 0 compiler warnings
(`-Wall -Wextra -Wpedantic -Wshadow -Wconversion`). 42 of these are the original spec tests.

## Mission failure injection (phase 5): `scripts/failure_test.sh <kind>`
Survey: 16 waypoints, 8 lines, 30 m altitude. Failure injected when mission item 5 is reached (line 2).
Times come from `events.csv` and are relative to the injection (`injection.csv`), both on the monotonic clock.
Event resolution is 0.5 s (main-thread poll).

| test | run | PX4 reaction | mission_node | exit |
|---|---|---|---|---|
| none (baseline) | `runs/20261102-050446` | Mission → Return after the last item (normal) | success, 152 captures, 0 drops | 0 |
| GPS loss: `failure gps off` | `runs/20261102-045711` | GPS fix lost at +1.0 s; "Failsafe: blind land" mode switch at +5.5 s; global position LOST at +7.5 s; landed at +52.5 s (from 30 m) | logged the failsafe at +8.5 s, sent no commands, captures stopped at 42 | 6 |
| GCS data-link loss: fake GCS killed | `runs/20261102-050131` | Hold at +12.9 s (COM_DL_LOSS_T = 10 s); Return at +17.9 s ("entering Hold for 5 seconds"); landed at home at +65.9 s | logged the failsafe at +15.9 s, sent no commands, 47 captures | 6 |

Notes:
- With data-link loss PX4 keeps flying the mission until the 10 s timeout, so captures continue during
  that time (47 vs 42).
- MAVSDK has no flight-mode name for PX4's failsafe Descend, so it is logged as `Unknown`.
- During the blind landing MAVSDK's local-position health alternated ok/LOST every poll. Health events
  are now debounced (they must hold for 2 polls). The GPS run above was recorded before the debounce.
- Not tested: battery failsafe. The SITL battery stops draining at 50 % (see the `battery` events).
  `[TBD: measure]`

## Link monitor (phase 4): `build/link_monitor`, `scripts/netem_loss_test.sh 5`
| test | run | result |
|---|---|---|
| live baseline | `runs/link-baseline` | 375.5 frames/s, 0 lost of 14,996, 0 CRC errors |
| heartbeat watchdog | same | warning 3.8 s after PX4 was killed |
| 5 % netem loss, before the reorder fix | `runs/link-netem-5pct-20261031-142058` | measured 70.64 % (bug: reordered frames counted as 254-frame gaps) |
| 5 % netem loss, after the fix | `runs/link-netem-5pct-20261031-142544` | baseline 0.00 %, **netem 4.88 %**, recovery 0.00 %, 0 CRC errors |

## Survey and payload (phases 2–3)
| metric | result | runs |
|---|---|---|
| survey flights completed | 3/3 in phase 3, plus the phase-5 baseline | `runs/20261031-112229`, `-112908` (ASan), `-113443`, `runs/20261102-050446` |
| captures per flight | 152 (19 per line), 0 queue drops | same |
| trigger spacing abs error (planned 6.213 m) | mean 0.245–0.261 m, p95 0.342–0.423 m, max 0.479–0.524 m | `analysis/spacing.py <run>` |
