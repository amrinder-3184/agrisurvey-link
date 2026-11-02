"""Tests for analysis/report.py on a tiny synthetic run with known answers.

    python3 -m pytest analysis/tests -q
"""
import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import report  # noqa: E402

LAT0, LON0 = 47.0, 8.0
R = 6371000.0
K = math.pi / 180.0


def to_latlon(east, north):
    """Inverse of spacing.local_xy (same equirectangular model)."""
    return LAT0 + north / (K * R), LON0 + east / (K * R * math.cos(LAT0 * K))


def write(path, text):
    path.write_text(text)


@pytest.fixture
def run_dir(tmp_path):
    d = tmp_path / "20260101-000000"
    d.mkdir()
    write(d / "run_meta.json", json.dumps({
        "outcome": "success", "home_lat_deg": LAT0, "home_lon_deg": LON0,
        "trigger_distance_m": 5.0, "line_spacing_m": 10.0, "waypoints": 4,
        "planned_path_m": 60.0, "queue_drops": 0}))
    write(d / "mission_plan.csv", "index,east_m,north_m,lat_deg,lon_deg,alt_m,line_index\n"
          "0,0,10,0,0,30,0\n1,10,10,0,0,30,0\n2,10,20,0,0,30,1\n3,0,20,0,0,30,1\n")
    # Telemetry: one ground row (ignored), then 20 m flown at altitude: (0,0)->(10,0)->(10,10).
    rows = ["t_mono_ns,lat_deg,lon_deg,rel_alt_m,flight_mode"]
    for i, (e, n, alt) in enumerate([(0, 0, 0.0), (0, 0, 30.0), (10, 0, 30.0), (10, 10, 30.0)]):
        lat, lon = to_latlon(e, n)
        rows.append(f"{i * 500_000_000},{lat:.10f},{lon:.10f},{alt},Mission")
    write(d / "telemetry.csv", "\n".join(rows) + "\n")
    # Captures exactly 5 m apart on two lines: spacing error 0 for all 4 same-line pairs.
    rows = ["capture_id,t_mono_ns,lat_deg,lon_deg,rel_alt_m,roll_deg,pitch_deg,yaw_deg,"
            "ground_speed_mps,line_index,ndvi_synthetic"]
    pts = [(0, 10, 0), (5, 10, 0), (10, 10, 0), (10, 20, 1), (5, 20, 1), (0, 20, 1)]
    for i, (e, n, line) in enumerate(pts, start=1):
        lat, lon = to_latlon(e, n)
        rows.append(f"{i},{i},{lat:.10f},{lon:.10f},30,0,0,0,5,{line},{0.5 + i / 100:.4f}")
    write(d / "geotags.csv", "\n".join(rows) + "\n")
    write(d / "events.csv", "t_mono_ns,t_s,event,detail\n1,0.0,mode,Mission\n"
          "2,9.5,failsafe,aborted by autopilot (mode Hold at item 1/4)\n")
    return d


@pytest.fixture
def link_csv(tmp_path):
    p = tmp_path / "link_stats.csv"
    write(p, "t_s,frames_ok,crc_errors,unknown_msgid,bad_header,lost,loss_ratio,rate_hz,"
          "heartbeat_age_s,reordered\n"
          "1.0,100,0,0,0,0,0,100,0.5,0\n2.0,200,0,0,0,0,0,100,0.5,0\n3.0,290,1,0,0,10,0.0333,90,0.5,2\n")
    return p


def test_summary_numbers(run_dir):
    s = report.make_report(run_dir)
    assert s["captures"] == 6
    assert s["capture_lines"] == 2
    assert s["spacing_pairs"] == 4  # the first capture of each line has no same-line predecessor
    assert s["spacing_abs_max_m"] == pytest.approx(0.0, abs=1e-6)
    assert s["flown_path_m"] == pytest.approx(20.0, abs=0.05)  # ground row excluded
    assert s["airborne_s"] == pytest.approx(1.0)
    assert s["survey_lines"] == 2
    assert s["failsafe"].startswith("aborted by autopilot")
    assert s["ndvi_synthetic_min"] == pytest.approx(0.51)


def test_outputs_written_and_labelled_synthetic(run_dir):
    report.make_report(run_dir)
    out = run_dir / "report"
    for name in ("path.png", "spacing_hist.png", "ndvi_map.png", "summary.json", "report.html"):
        assert (out / name).stat().st_size > 0, name
    assert not (out / "link.png").exists()  # no --link given
    page = (out / "report.html").read_text()
    assert "SYNTHETIC" in page
    assert "failsafe" in page  # events table
    assert json.loads((out / "summary.json").read_text())["ndvi_note"].startswith("SYNTHETIC")


def test_link_summary_and_interval_loss(run_dir, link_csv):
    s = report.make_report(run_dir, link_csv)
    assert s["link"]["frames_ok"] == 290
    assert s["link"]["lost"] == 10
    assert s["link"]["loss_pct"] == pytest.approx(100 * 10 / 300, abs=1e-3)
    assert s["link"]["reordered"] == 2
    rows = report.load_link(link_csv)
    assert [r["loss_pct_interval"] for r in rows] == pytest.approx([0.0, 0.0, 10.0])  # 10/(90+10)
    assert (run_dir / "report" / "link.png").exists()


def test_run_without_events_file(run_dir):
    (run_dir / "events.csv").unlink()  # runs from before phase 5
    s = report.make_report(run_dir)
    assert s["events"] == 0
    assert s["failsafe"] is None
