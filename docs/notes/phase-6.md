# Phase 6: post-flight report

## What was built
- `analysis/report.py runs/<run> [--link link_stats.csv]` → `runs/<run>/report/`:
  - path.png, spacing_hist.png, ndvi_map.png (titled and labelled **SYNTHETIC**), link.png
  - summary.json and report.html (summary table, plots, events table)
- It reuses `spacing.py` (`local_xy`, `spacing_errors`, `percentile`), so there is one implementation of
  the projection and the spacing math.
- It tolerates runs from before phase 5 (no events.csv).
- `analysis/tests/test_report.py`: 4 pytest tests on a synthetic run with known answers (spacing error 0,
  flown path 20 m, link loss 10/300, interval loss 10 %). **4/4 pass.**

## Measured (reports generated for runs/20261102-050446, -045711, -050131)
- Baseline:
  - flown path 1173.3 m vs planned 1191.0 m (fly-through waypoints cut the corners); airborne 301.6 s
  - 152 captures on 8 lines, 0 drops
  - spacing abs error mean 0.261 / p95 0.342 / max 0.508 m (same as `spacing.py`)
  - ndvi_synthetic 0.35–0.80 (the synthetic low patch is visible on the map)
- Link plot (netem run 142544, shown as a separate run in the report):
  - steady ~375 frames/s outside the loss window
  - interval loss about 3–8 % inside the 5 % netem window, 0 outside
- GPS-loss run: 42 captures, flown path 397.2 m, airborne 130.0 s; the failsafe is shown in summary and events.

## Decisions
- **Static PNG + HTML instead of an interactive dashboard.** PNGs work in a README, a PR and a slide; no
  JS to explain. Every plotted number is also in the tables (summary.json / HTML), which gives a table view.
- **Colours from a validated palette:** blue planned, orange flown, aqua captures. The validator passes;
  aqua is below 3:1 contrast, so captures also use a diamond marker and appear in the legend and tables.
  NDVI uses a single-hue light→dark blue ramp (magnitude), never a rainbow.
- **Two panels on one time axis for the link plot.** Frame rate and loss % have different units, and two
  y-scales on one chart mislead.
- **Per-interval loss** from differences of the cumulative counters: it shows *when* loss happened, which a
  cumulative ratio hides.
- **Honest sourcing:** a link CSV from another run is captioned as such, and its path is stored in
  summary.json.

## Caveats
- The "flown" track is PX4's own estimate (GLOBAL_POSITION_INT), not ground truth. After GPS loss the
  estimate drifts during dead-reckoning (the curl in the GPS run's path.png).
- The rendered plots were checked by eye. Two layout bugs were fixed: the NDVI title under the colorbar,
  and the mean label on the bars.

## Interview Q&A
1. **How do you know the spacing number is right?** The same function computes it in `spacing.py` and in
   the report, and a unit test feeds captures exactly 5 m apart and expects 0 error.
2. **Why is the flown path shorter than the planned path?** Fly-through waypoints: PX4 starts turning
   before reaching a corner, so it cuts each corner (1173 vs 1191 m).
3. **Why is NDVI labelled SYNTHETIC everywhere?** There is no multispectral camera in SITL. The values
   come from a formula and must never be mistaken for crop data: they are labelled in the title, colorbar,
   footer, HTML and JSON.
4. **Why two panels instead of one dual-axis chart?** With two y-scales the visual crossing points are
   arbitrary (they depend on axis limits), so readers infer correlations that aren't there.
5. **What does the GPS-loss path plot show, and what doesn't it?** It shows the position estimate. After
   GPS loss the estimate drifts, so the plot is not where the drone really went. For ground truth we would
   need the Gazebo model pose.

## Quiz
1. Why compute per-interval loss from counter differences rather than plotting `loss_ratio`?
2. What breaks if report.py had its own copy of the projection formula?
3. Why does the test place captures exactly 5 m apart?
