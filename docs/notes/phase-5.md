# Phase 5: event log and failure injection

## What was built
- `include/asl/event_log.h`, `src/core/event_log.cpp`:
  - `asl::Watch<T>` is a change detector with an optional confirm count (debounce).
  - `asl::EventLog` writes `events.csv` (`t_mono_ns,t_s,event,detail`) and flushes every row. CSV quoting
    is handled.
- `mission_node`:
  - Polls MAVSDK's cached values every 500 ms in the main thread and logs changes: mode, armed, landed
    state, GPS fix, global/local/home position health, battery (10 % steps), mission item, plus our own
    commands and the final outcome.
  - New `--role gcs|companion` option.
  - **Failsafe handling:** if PX4 leaves Mission mode before the last item, mission_node logs it, sends
    no commands, waits for touchdown and exits with code 6.
- `scripts/failure_test.sh gps|datalink|none` and `scripts/fake_gcs.py`.
- `docs/test-report.md`: every measured result so far, with its run folder.
- Tests: `tests/test_event_log.cpp` (7). Total ctest **78/78**, 0 warnings, ASan/UBSan clean.

## Measured results (see docs/test-report.md)
- **Baseline** `runs/20261102-050446`: success, exit 0, 152 captures, 0 drops, mean spacing error
  0.261 m. The normal Mission → Return switch was not misread as a failsafe.
- **GPS loss** `runs/20261102-045711`:
  - GPS fix lost at +1.0 s; PX4 "blind land" at +5.5 s.
  - mission_node logged the failsafe at +8.5 s; landed at +52.5 s; exit 6, 42 captures.
- **Data-link loss** `runs/20261102-050131`:
  - Hold at +12.9 s, Return at +17.9 s; landed at home at +65.9 s; exit 6.
  - Matches NAV_DLL_ACT = 2, COM_DL_LOSS_T = 10 s, and PX4's 5 s Hold before the action.

## Verified, not from memory
- From the PX4 source:
  - `failure` units include `gps`; GZBridge applies them via `process_gnss`.
  - SITL rcS sets `SYS_FAILURE_EN 1`; airframe 4001_gz_x500 sets `NAV_DLL_ACT 2`.
  - `COM_DL_LOSS_T` defaults to 10 s.
  - `failure mavlink_signal` exists in the command table but nothing consumes it, so it was not used.
- From the MAVSDK headers: `ComponentType::CompanionComputer`; the `Health`, `GpsInfo` and
  `LandedState` fields; and `FlightMode` has no Descend value.

## Decisions
- **Poll in the main thread** instead of an event queue fed by callbacks. MAVSDK getters return cached
  values, so polling is cheap. Callbacks stay I/O-free and there is no new threading. Cost: 0.5 s
  resolution.
- **Do not fight the autopilot.** After a failsafe, mission_node sends no commands. PX4 decided with more
  information (EKF state, link state) than we have.
- **3 s grace before declaring a failsafe.** At the normal end PX4 switches to Return itself, and the last
  progress message can arrive just after that mode change.
- **Companion role for the link test.** On a real BharatRohan-style drone the mission runs on an onboard
  computer while the GCS is on a radio link. That GCS heartbeat is what NAV_DLL_ACT watches.
- **Debounce health only** (confirm = 2). Mode/armed/landed transitions are reported immediately because
  they are real state changes and their timing matters.

## Edge cases
- First poll logs the initial state.
- "Left Mission" only counts after Mission was seen.
- SIGINT during failsafe → exit 130.
- No landing within 180 s after failsafe → exit 5.
- Battery NaN is skipped.
- The event log is flushed per row, so it survives a kill.
- Triggering stops automatically outside Mission (42 captures in the GPS run).

## Interview Q&A
1. **What does your node do when PX4 triggers a failsafe?** It detects that the mode left Mission before
   the last item, logs the mode and item, and stops commanding: no RTL, no restart. The autopilot owns
   the vehicle. Then it waits for touchdown and exits with a distinct code, so a supervisor can tell a
   failsafe from a crash or a timeout.
2. **How long does PX4 take to react to GCS link loss, and why?** Measured: Hold at 12.9 s, Return at
   17.9 s. That is COM_DL_LOSS_T = 10 s, plus up to 1 s between GCS heartbeats, plus reporting and
   polling delay, then PX4's 5 s Hold before executing NAV_DLL_ACT = Return.
3. **What happens on GPS loss in a multicopter mission?** The EKF dead-reckons briefly. Once horizontal
   position is invalid, PX4 can't navigate, so it enters "blind land" (Descend, COM_POS_FS_ACT = 0). It
   lands where it is, not at home. Measured: mode switch 5.5 s after injection.
4. **Why did you add a debounce, and why only for health?** MAVSDK's local-position health flipped every
   poll during the blind landing, which flooded the log. A 2-poll confirm filters that, but mode changes
   must be timed exactly, so they are not debounced.
5. **Why poll instead of using MAVSDK callbacks for events?** Callbacks run on MAVSDK's thread, where we
   must not do file I/O. Polling cached values from the main thread keeps all I/O in one thread, needs no
   extra locking, and 0.5 s resolution is plenty for these events.

## Quiz
1. Why must "left Mission" only count after Mission mode has been seen once?
2. In the data-link test, why did captures continue for about 13 s after the GCS was killed?
3. A health flag reads ok, LOST, ok, LOST on consecutive polls. With `Watch<bool>(2)`, how many events are logged?
