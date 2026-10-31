#!/usr/bin/env python3
"""Trigger spacing error of one run: python3 analysis/spacing.py runs/<run>

For consecutive captures on the SAME survey line, compare the ground distance between them
with the planned trigger distance (run_meta.json). Prints count, mean/max/p95 absolute error
and the mean signed error (bias). Standard library only.
"""
import csv
import json
import math
import sys
from pathlib import Path

EARTH_RADIUS_M = 6371000.0  # same flat-Earth model as src/core/geo.cpp


def local_xy(lat, lon, lat0, lon0):
    """Equirectangular projection around (lat0, lon0), metres (east, north)."""
    k = math.pi / 180.0
    return ((lon - lon0) * k * EARTH_RADIUS_M * math.cos(lat0 * k),
            (lat - lat0) * k * EARTH_RADIUS_M)


def percentile(values, p):
    """Nearest-rank percentile of a non-empty list (p in 0..100)."""
    ordered = sorted(values)
    rank = max(1, math.ceil(p / 100.0 * len(ordered)))
    return ordered[rank - 1]


def spacing_errors(rows, trigger_distance_m, lat0, lon0):
    """Signed errors (actual - planned) between consecutive captures on the same line."""
    errors = []
    for prev, cur in zip(rows, rows[1:]):
        if prev["line_index"] != cur["line_index"]:
            continue  # the first capture of a line is not spaced from the previous line
        x0, y0 = local_xy(prev["lat_deg"], prev["lon_deg"], lat0, lon0)
        x1, y1 = local_xy(cur["lat_deg"], cur["lon_deg"], lat0, lon0)
        errors.append(math.hypot(x1 - x0, y1 - y0) - trigger_distance_m)
    return errors


def load(run_dir):
    run_dir = Path(run_dir)
    meta = json.loads((run_dir / "run_meta.json").read_text())
    with open(run_dir / "geotags.csv", newline="") as f:
        rows = [{"line_index": int(r["line_index"]),
                 "lat_deg": float(r["lat_deg"]),
                 "lon_deg": float(r["lon_deg"])} for r in csv.DictReader(f)]
    return meta, rows


def main(argv):
    if len(argv) != 2:
        print(__doc__.strip().splitlines()[0], file=sys.stderr)
        return 1
    meta, rows = load(argv[1])
    d = meta["trigger_distance_m"]
    errors = spacing_errors(rows, d, meta["home_lat_deg"], meta["home_lon_deg"])
    if not errors:
        print("no consecutive captures on the same line")
        return 1
    abs_err = [abs(e) for e in errors]
    lines = len({r["line_index"] for r in rows})
    print(f"captures: {len(rows)} on {lines} lines, queue drops: {meta.get('queue_drops')}")
    print(f"planned trigger distance: {d:.3f} m, telemetry rate: {meta.get('telemetry_rate_hz')} Hz")
    print(f"spacing pairs: {len(errors)}")
    print(f"abs error  mean {sum(abs_err) / len(abs_err):.3f} m  max {max(abs_err):.3f} m  "
          f"p95 {percentile(abs_err, 95):.3f} m")
    print(f"signed error mean {sum(errors) / len(errors):+.3f} m (positive = captures late)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
