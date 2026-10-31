# Phase 2: survey planning and mission node

## What was built

- `src/core/geo.cpp`: `LocalFrame` (equirectangular projection around home) and `distance_m`.
- `src/core/survey_planner.cpp`: camera footprint maths and `plan_lawnmower` for convex fields.
- `include/asl/config.h`, `src/core/config.cpp`: parse `config/camera.ini` and
  `config/field_local.csv` with validation (`tests/test_config.cpp`, 6 tests).
- `src/mission/mission_node.cpp` (target `mission_node`): load config, wait for an armable
  vehicle with a home position, plan, upload with RTL-after-mission, arm, take off to
  survey altitude, start the mission, monitor progress, wait for landing. Writes
  `runs/<timestamp>/mission_plan.csv` and `run_meta.json`.

Acceptance (measured 2026-10-31):
- All 14 `Geo.*`, `Footprint.*`, `Lawnmower.*` spec tests pass; 6 config tests pass; 0 warnings.
- Survey flown on PX4 SITL: run `runs/20261031-111308`, exit 0 after 313 s.
  16 waypoints = 8 lines, line spacing 10.39 m, planned path 1191 m.
  PX4 log: `Executing Mission` -> `Mission finished, loitering` -> `Switching to Return`
  -> `Landing detected`.
- Not yet done: visual comparison of `mission_plan.csv` with the mission shown in QGC.

## Key decisions

- **Plan in metres, fly in degrees.** The planner works in a flat local frame (east, north)
  around the home position; only the final waypoints are converted to lat/lon. At field
  scale (hundreds of metres) the flat-Earth error is far below GPS accuracy.
- **Edge crossing test `(a.n <= y) != (b.n <= y)`.** Skips horizontal edges automatically
  (no division by zero) and counts a vertex lying exactly on a line once.
- **Line heights computed as `min + s/2 + k*s`**, not by repeated `y += s`, so rounding
  errors do not accumulate. Starting at s/2 centres the first image footprint half a
  spacing inside the field edge.
- **Config errors throw with `file:line`**; the field must be convex because the planner
  silently produces wrong lines for concave fields. Unknown INI keys are errors (typos).
- **Explicit takeoff, then mission.** The survey starts at a known altitude, and takeoff
  failures are reported separately from mission failures.
- **Mission timeout derived from the plan** (2 x path / speed + 120 s), not a constant.
- **SIGINT handler only sets an atomic flag**; the wait loops react (RTL if airborne,
  write `run_meta.json`, exit 130). Signal handlers must not do I/O or take locks.

## Bugs found and fixed

- `px4_version` contained NUL bytes: PX4 sends the git hash as a fixed-size byte array.
  Fixed by keeping printable characters only.
- Numbers printed as `1e+03`: `setprecision` without `std::fixed` uses scientific notation.
- Lifetime bug (found in review before it crashed): the mission-progress atomics were
  declared after the `Mission` plugin, so they would be destroyed first and a late callback
  could write freed memory. All callback-shared state now lives in one `Shared` struct
  declared before the MAVSDK objects.

## What an interviewer may ask

1. **How does the flat-Earth projection work, and what is its error?**
   north = Δlat·R, east = Δlon·R·cos(lat0) (radians). A degree of longitude shrinks with
   cos(latitude); using cos(lat0) for the whole field ignores how it changes across the
   field and ignores Earth's curvature. For a field a few hundred metres across the error
   is centimetres, much smaller than GPS error.

2. **How do you find where a survey line enters and leaves the field?**
   For each line height y, intersect it with every polygon edge whose endpoints are on
   opposite sides of y, interpolating east linearly. For a convex polygon the smallest and
   largest crossing are the entry and exit points.

3. **What happens with horizontal edges?**
   Both endpoints have the same north value, so the crossing test is false and the edge is
   skipped. That avoids a division by zero in the interpolation, and the edge never
   contributes a crossing anyway.

4. **Why do the lines alternate direction?**
   Boustrophedon ("ox-turning"): each line starts where the previous one ended, so the
   drone never flies back across the field empty. It minimises path length and flight time.

5. **How are the footprint and line spacing computed?**
   Footprint width = 2·h·tan(hfov/2); spacing = width·(1 − sidelap). With hfov 60°, h 30 m and
   sidelap 0.7: 34.64 m × 0.3 = 10.39 m, which is the spacing the mission flew.
