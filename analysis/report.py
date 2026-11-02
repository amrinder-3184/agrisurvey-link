#!/usr/bin/env python3
"""Post-flight report for one mission run.

    python3 analysis/report.py runs/<run> [--link runs/<link-run>/link_stats.csv]

Writes runs/<run>/report/:
    path.png          planned survey path vs flown track, with capture points
    spacing_hist.png  trigger-spacing error histogram (consecutive captures on one line)
    ndvi_map.png      capture points coloured by ndvi_synthetic -- SYNTHETIC, not crop data
    link.png          frame rate and per-second loss from link_monitor (only with --link)
    summary.json      every number shown in the report
    report.html       the plots plus summary and event tables

Inputs are the files mission_node writes (run_meta.json, mission_plan.csv, telemetry.csv,
geotags.csv, events.csv). events.csv is optional (runs from before phase 5 have none).
"""
import argparse
import csv
import html
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # render to files; no display needed (headless WSL, CI)
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from spacing import local_xy, percentile, spacing_errors  # noqa: E402  (one implementation)

SYNTHETIC_NOTE = "SYNTHETIC NDVI: simulated values from asl::ndvi_synthetic, not real crop data"

# Colours: validated categorical palette (scripts/validate_palette.js, light mode, all checks
# pass). Aqua is below 3:1 contrast, so captures also differ by marker shape and are in a table.
PLANNED = "#2a78d6"   # blue
FLOWN = "#eb6834"     # orange
CAPTURE = "#1baf7a"   # aqua
INK, INK_2, MUTED = "#0b0b0b", "#52514e", "#898781"
GRID, AXIS, SURFACE = "#e1e0d9", "#c3c2b7", "#fcfcfb"
# Sequential (magnitude): one hue, light -> dark (blue ramp steps 250..700).
NDVI_CMAP = LinearSegmentedColormap.from_list(
    "ndvi_blue", ["#86b6ef", "#5598e7", "#2a78d6", "#1c5cab", "#104281", "#0d366b"])


# ---------------------------------------------------------------- loading
def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def load_run(run_dir):
    run_dir = Path(run_dir)
    meta = json.loads((run_dir / "run_meta.json").read_text())
    lat0, lon0 = meta["home_lat_deg"], meta["home_lon_deg"]
    plan = [(float(r["east_m"]), float(r["north_m"])) for r in read_csv(run_dir / "mission_plan.csv")]
    track = []
    for r in read_csv(run_dir / "telemetry.csv"):
        e, n = local_xy(float(r["lat_deg"]), float(r["lon_deg"]), lat0, lon0)
        track.append({"t_ns": int(r["t_mono_ns"]), "east": e, "north": n,
                      "alt": float(r["rel_alt_m"]), "mode": r["flight_mode"]})
    captures = []
    for r in read_csv(run_dir / "geotags.csv"):
        e, n = local_xy(float(r["lat_deg"]), float(r["lon_deg"]), lat0, lon0)
        captures.append({"line_index": int(r["line_index"]), "lat_deg": float(r["lat_deg"]),
                         "lon_deg": float(r["lon_deg"]), "east": e, "north": n,
                         "ndvi": float(r["ndvi_synthetic"])})
    events_path = run_dir / "events.csv"
    events = read_csv(events_path) if events_path.exists() else []
    return meta, plan, track, captures, events


def load_link(path):
    rows = [{k: float(v) for k, v in r.items()} for r in read_csv(path)]
    # Per-second loss from the cumulative counters: lost / (received + lost) in each interval.
    prev_frames = prev_lost = 0.0
    for r in rows:
        d_frames, d_lost = r["frames_ok"] - prev_frames, r["lost"] - prev_lost
        r["loss_pct_interval"] = 100.0 * d_lost / (d_frames + d_lost) if d_frames + d_lost > 0 else 0.0
        prev_frames, prev_lost = r["frames_ok"], r["lost"]
    return rows


# ---------------------------------------------------------------- numbers
def summarize(meta, plan, track, captures, events, link=None):
    errors = spacing_errors(captures, meta["trigger_distance_m"], meta["home_lat_deg"],
                            meta["home_lon_deg"])
    abs_err = [abs(e) for e in errors]
    airborne = [p for p in track if p["alt"] > 1.0]  # ignore ground time before/after flight
    flown_m = sum(math.hypot(b["east"] - a["east"], b["north"] - a["north"])
                  for a, b in zip(airborne, airborne[1:]))
    s = {
        "outcome": meta.get("outcome"),
        "px4_version": meta.get("px4_version"),
        "mavsdk_version": meta.get("mavsdk_version"),
        "waypoints": meta.get("waypoints"),
        "survey_lines": len(plan) // 2,
        "line_spacing_m": meta.get("line_spacing_m"),
        "trigger_distance_m": meta.get("trigger_distance_m"),
        "planned_path_m": meta.get("planned_path_m"),
        "flown_path_m": round(flown_m, 1),
        "airborne_s": round((airborne[-1]["t_ns"] - airborne[0]["t_ns"]) * 1e-9, 1) if airborne else 0.0,
        "captures": len(captures),
        "capture_lines": len({c["line_index"] for c in captures}),
        "queue_drops": meta.get("queue_drops"),
        "spacing_pairs": len(errors),
        "spacing_abs_mean_m": round(sum(abs_err) / len(abs_err), 3) if abs_err else None,
        "spacing_abs_p95_m": round(percentile(abs_err, 95), 3) if abs_err else None,
        "spacing_abs_max_m": round(max(abs_err), 3) if abs_err else None,
        "spacing_signed_mean_m": round(sum(errors) / len(errors), 3) if errors else None,
        "events": len(events),
        "failsafe": next((e["detail"] for e in events if e["event"] == "failsafe"), None),
        "ndvi_synthetic_min": round(min(c["ndvi"] for c in captures), 4) if captures else None,
        "ndvi_synthetic_max": round(max(c["ndvi"] for c in captures), 4) if captures else None,
        "ndvi_note": SYNTHETIC_NOTE,
    }
    if link:
        last = link[-1]
        total = last["frames_ok"] + last["lost"]
        s["link"] = {
            "duration_s": last["t_s"],
            "frames_ok": int(last["frames_ok"]),
            "lost": int(last["lost"]),
            "loss_pct": round(100.0 * last["lost"] / total, 3) if total else 0.0,
            "crc_errors": int(last["crc_errors"]),
            "mean_rate_hz": round(sum(r["rate_hz"] for r in link) / len(link), 1),
            "reordered": int(last.get("reordered", 0)),
        }
    return s, errors


# ---------------------------------------------------------------- plots
def style():
    plt.rcParams.update({
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
        "axes.edgecolor": AXIS, "axes.labelcolor": INK_2, "text.color": INK,
        "xtick.color": MUTED, "ytick.color": MUTED, "axes.grid": True, "grid.color": GRID,
        "grid.linewidth": 0.6, "axes.spines.top": False, "axes.spines.right": False,
        "axes.titlesize": 12, "axes.titleweight": "bold", "axes.titlelocation": "left",
        "font.size": 10, "legend.frameon": False, "lines.linewidth": 2,
    })


def plot_path(plan, track, captures, out):
    fig, ax = plt.subplots(figsize=(7, 5.5))
    airborne = [p for p in track if p["alt"] > 1.0]
    ax.plot([0] + [p[0] for p in plan] + [0], [0] + [p[1] for p in plan] + [0],
            color=PLANNED, linestyle="--", label="planned (home → lines → home)")
    ax.plot([p["east"] for p in airborne], [p["north"] for p in airborne], color=FLOWN,
            label="flown (telemetry, 2 Hz)")
    ax.scatter([c["east"] for c in captures], [c["north"] for c in captures], s=22, marker="D",
               color=CAPTURE, edgecolors=SURFACE, linewidths=0.8, zorder=3,
               label=f"captures ({len(captures)})")
    ax.scatter([0], [0], s=70, marker="^", color=INK, zorder=4, label="home")
    ax.set_aspect("equal")
    ax.set_xlabel("east of home (m)")
    ax.set_ylabel("north of home (m)")
    ax.set_title("Survey path: planned vs flown")
    ax.legend(loc="upper left", bbox_to_anchor=(1.0, 1.0))
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)


def plot_spacing(errors, trigger_m, out):
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.hist(errors, bins=20, color=PLANNED, edgecolor=SURFACE, linewidth=2)
    ax.axvline(0.0, color=INK_2, linewidth=1)
    mean = sum(errors) / len(errors)
    ax.axvline(mean, color=INK, linewidth=1, linestyle=":")
    # Label in a free corner (not on the bars): what the two vertical lines mean.
    ax.text(0.99, 0.97, f"dotted: mean {mean:+.3f} m\nsolid: zero error", transform=ax.transAxes,
            ha="right", va="top", color=INK_2)
    ax.set_xlabel(f"spacing error (m) vs planned {trigger_m:.3f} m;  positive = capture late")
    ax.set_ylabel("capture pairs")
    ax.set_title(f"Trigger spacing error ({len(errors)} same-line capture pairs)")
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)


def plot_ndvi(plan, captures, out):
    fig, ax = plt.subplots(figsize=(7, 5.5))
    ax.plot([p[0] for p in plan], [p[1] for p in plan], color=AXIS, linewidth=1)
    sc = ax.scatter([c["east"] for c in captures], [c["north"] for c in captures],
                    c=[c["ndvi"] for c in captures], cmap=NDVI_CMAP, s=40, edgecolors=SURFACE,
                    linewidths=0.8, zorder=3)
    bar = fig.colorbar(sc, ax=ax, shrink=0.8)
    bar.set_label("ndvi_synthetic (SYNTHETIC)")
    ax.set_aspect("equal")
    ax.set_xlabel("east of home (m)")
    ax.set_ylabel("north of home (m)")
    # Figure-level title so the colorbar can't overlap it; the footer repeats the caveat.
    fig.suptitle("SYNTHETIC NDVI at capture points (not real crop data)", x=0.02, ha="left",
                 fontsize=12, fontweight="bold")
    fig.text(0.02, 0.01, SYNTHETIC_NOTE, fontsize=8, color=INK_2)
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)


def plot_link(link, out, markers=None):
    # Two panels on one shared time axis: two measures with different units are never put on
    # two y-scales of one chart.
    fig, (top, bottom) = plt.subplots(2, 1, figsize=(7, 5), sharex=True)
    t = [r["t_s"] for r in link]
    top.plot(t, [r["rate_hz"] for r in link], color=PLANNED)
    top.set_ylabel("frames / s")
    top.set_title("MAVLink link: received frame rate and loss (link_monitor)")
    bottom.plot(t, [r["loss_pct_interval"] for r in link], color=FLOWN)
    bottom.set_ylabel("loss in interval (%)")
    bottom.set_xlabel("time since monitor start (s)")
    if markers and "loss_start" in markers and "loss_end" in markers:
        for ax in (top, bottom):
            ax.axvspan(markers["loss_start"], markers["loss_end"], color=GRID, alpha=0.6, lw=0)
        bottom.annotate("netem loss injected", xy=(markers["loss_start"], 0), xytext=(4, 4),
                        textcoords="offset points", color=INK_2, fontsize=9)
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)


# ---------------------------------------------------------------- html
def table(rows):
    body = "".join(f"<tr><th>{html.escape(str(k))}</th><td>{html.escape(str(v))}</td></tr>"
                   for k, v in rows)
    return f"<table>{body}</table>"


def write_html(out, run_name, summary, events, images):
    flat = [(k, v) for k, v in summary.items() if k != "link"]
    flat += [(f"link.{k}", v) for k, v in summary.get("link", {}).items()]
    ev_rows = "".join(
        f"<tr><td>{html.escape(e['t_s'])}</td><td>{html.escape(e['event'])}</td>"
        f"<td>{html.escape(e['detail'])}</td></tr>" for e in events)
    imgs = "".join(
        f'<figure><img src="{name}" alt="{html.escape(alt)}"><figcaption>{html.escape(alt)}'
        f"</figcaption></figure>" for name, alt in images)
    out.write_text(f"""<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><title>AgriSurvey-Link report: {html.escape(run_name)}</title>
<style>
 body {{ font-family: system-ui, sans-serif; background:#f9f9f7; color:#0b0b0b; max-width:960px; margin:2em auto; padding:0 1em; }}
 h1 {{ font-size:1.4em; }} h2 {{ font-size:1.1em; margin-top:2em; }}
 .note {{ background:#fff4d6; border-left:4px solid #eda100; padding:.6em 1em; }}
 table {{ border-collapse:collapse; font-size:.9em; }} th, td {{ text-align:left; padding:3px 12px 3px 0; border-bottom:1px solid #e1e0d9; }}
 th {{ color:#52514e; font-weight:600; }} img {{ max-width:100%; }} figcaption {{ color:#52514e; font-size:.85em; }}
</style></head><body>
<h1>AgriSurvey-Link post-flight report: {html.escape(run_name)}</h1>
<p>Outcome: <b>{html.escape(str(summary.get("outcome")))}</b>. Simulation (PX4 SITL + Gazebo).</p>
<p class="note">{html.escape(SYNTHETIC_NOTE)}</p>
<h2>Summary</h2>{table(flat)}
<h2>Plots</h2>{imgs}
<h2>Events ({len(events)})</h2>
<table><tr><th>t (s)</th><th>event</th><th>detail</th></tr>{ev_rows}</table>
</body></html>
""")


# ---------------------------------------------------------------- main
def make_report(run_dir, link_csv=None):
    run_dir = Path(run_dir)
    meta, plan, track, captures, events = load_run(run_dir)
    link = load_link(link_csv) if link_csv else None
    summary, errors = summarize(meta, plan, track, captures, events, link)
    out = run_dir / "report"
    out.mkdir(exist_ok=True)
    style()
    images = []
    plot_path(plan, track, captures, out / "path.png")
    images.append(("path.png", "Planned survey path vs flown track, with capture points"))
    if errors:
        plot_spacing(errors, meta["trigger_distance_m"], out / "spacing_hist.png")
        images.append(("spacing_hist.png", "Trigger spacing error histogram"))
    if captures:
        plot_ndvi(plan, captures, out / "ndvi_map.png")
        images.append(("ndvi_map.png", "SYNTHETIC NDVI at capture points (not real crop data)"))
    if link:
        markers_path = Path(link_csv).parent / "markers.csv"
        markers = ({r["phase"]: float(r["t_s"]) for r in read_csv(markers_path)}
                   if markers_path.exists() else None)
        plot_link(link, out / "link.png", markers)
        same_run = Path(link_csv).resolve().parent == run_dir.resolve()
        origin = ("recorded by link_monitor during this flight" if same_run else
                  "a separate link_monitor run, not recorded during this flight")
        images.append(("link.png", f"MAVLink frame rate and per-interval loss, from {link_csv} "
                                   f"({origin})"))
        summary["link"]["source"] = str(link_csv)
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    write_html(out / "report.html", run_dir.name, summary, events, images)
    return summary


def main(argv=None):
    p = argparse.ArgumentParser(description="Post-flight report for one mission run.")
    p.add_argument("run_dir")
    p.add_argument("--link", help="link_stats.csv from link_monitor (optional)")
    args = p.parse_args(argv)
    summary = make_report(args.run_dir, args.link)
    print(json.dumps(summary, indent=2))
    print(f"Report: {Path(args.run_dir) / 'report' / 'report.html'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
