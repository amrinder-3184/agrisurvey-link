# Phase 3: payload pipeline

## What was built

- `src/core/trigger.cpp`: `DistanceTrigger` (fires when the straight-line distance from the
  last FIRED capture is >= the trigger distance; first update after reset always fires).
- `include/asl/capture_queue.h`, `src/core/capture_queue.cpp`: bounded thread-safe FIFO
  (fixed ring buffer, `std::mutex` + `std::condition_variable`), drop-oldest when full.
- `include/asl/geotag_logger.h`, `src/core/geotag_logger.cpp`: consumer thread writing
  `geotags.csv`; RAII stop drains the queue and joins the thread.
- `include/asl/synthetic_field.h`: deterministic SYNTHETIC NDVI with one stressed patch.
- `mission_node`: position callback -> trigger -> queue; attitude/velocity/mode callbacks
  store the latest value; main thread writes `telemetry.csv` at 2 Hz.
- `analysis/spacing.py`: spacing error between consecutive captures on the same line.
- New tests: CaptureQueue (7), GeotagLogger (3), SyntheticField (3).

## Results (measured 2026-10-31, 3 runs, 5 m/s, 30 m altitude, 10 Hz position stream)

| Run | Build | Captures | Queue drops | Abs error mean | max | p95 |
| --- | --- | --- | --- | --- | --- | --- |
| runs/20261031-112229 | normal | 152 | 0 | 0.250 m | 0.501 m | 0.423 m |
| runs/20261031-112908 | ASan + UBSan | 152 | 0 | 0.245 m | 0.479 m | 0.396 m |
| runs/20261031-113443 | normal | 152 | 0 | 0.252 m | 0.524 m | 0.405 m |

Planned trigger distance 6.213 m; 19 captures on each of 8 lines; 144 spacing pairs per run.
Signed error is always positive (captures late): +0.245 to +0.252 m.
Sanitizers: ASan/UBSan flight with 0 reports; ThreadSanitizer on the queue and logger tests
with 0 warnings. All 42 spec tests pass at the end of this phase (with phase 4).

## Why the error is what it is

The trigger can only fire when a position sample arrives. Samples are v/f apart
(5 m/s / 10 Hz = 0.5 m), and the threshold is crossed somewhere between two samples, so each
capture is 0 to about v/f late, roughly uniformly: expected mean v/(2f) = 0.25 m. Measured
means 0.245-0.252 m. The max can slightly exceed 0.5 m because ground speed reached 5.15 m/s
and samples are not perfectly evenly spaced. To reduce it: raise the position rate,
interpolate the exact crossing point between samples, or trigger inside the autopilot.

## Key decisions

- **Distance, not time, triggering.** Ground speed changes (wind, accelerating into a line);
  distance keeps image overlap constant. Measured from the last FIRED position, not summed
  per sample, so errors don't accumulate along a line.
- **Bounded queue, drop oldest.** push() never blocks and never allocates (ring buffer
  allocated once), so it is safe in a MAVSDK callback. If the logger falls behind, the newest
  geotag is the most valuable; blocking would stall MAVSDK's callback thread and all telemetry.
- **"On a survey leg" = Mission mode AND the current mission item is odd** (flying towards a
  line's exit waypoint; planner output is entry 2k, exit 2k+1). The trigger is reset at each
  new line, so every line starts with a capture at its entry point.
- **No file I/O in callbacks.** Only the logger thread writes geotags.csv; only main writes
  telemetry.csv.
- **Lifetime by declaration order.** Shared state, trigger, queue and logger are declared
  before the MAVSDK objects, so they outlive every callback.

## Limitations found in the data

- The first capture of each line is taken while the drone is still turning onto the line
  (e.g. capture 1: roll 14.8 deg, 3.0 m/s). A real survey adds a run-in leg before each line.
- NDVI is SYNTHETIC (a formula of position); there is no camera in SITL.

## What an interviewer may ask

1. **Why distance-based triggering instead of time-based?**
   Image overlap depends on distance travelled, not time. Ground speed changes with wind and
   when accelerating out of a turn, so a fixed time interval gives uneven overlap.

2. **Where does the spacing error come from, and how would you reduce it?**
   Position arrives at 10 Hz, so the trigger fires on the first sample past the threshold:
   0 to v/f late, mean v/(2f) = 0.25 m at 5 m/s (measured 0.245-0.252 m over 3 runs). Reduce it
   with a higher telemetry rate, interpolating between samples, or triggering in the autopilot.

3. **Explain the queue: why a mutex and a condition variable?**
   The mutex protects the ring buffer indices shared by two threads. The condition variable
   lets the consumer sleep until data arrives instead of busy-polling; wait_for with a
   predicate handles spurious wake-ups and gives a timeout so shutdown is never blocked.

4. **Why drop the oldest item instead of blocking or dropping the newest?**
   Blocking the producer would block MAVSDK's callback thread (all telemetry stalls). If we
   must lose data, the newest capture is the most relevant. Drops are counted, and measured
   as 0 at normal rates (capacity 256).

5. **Why must there be no file I/O in the position callback?**
   File writes can block for milliseconds (disk, page cache flushes). The callback runs on
   MAVSDK's thread, which also delivers every other message; blocking it delays all telemetry
   and can make the trigger fire late. The callback only does arithmetic and a non-blocking push.
