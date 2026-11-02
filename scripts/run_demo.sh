#!/usr/bin/env bash
# One-command demo: fly the survey in PX4 SITL with the link monitor running, then build the report.
#
#   scripts/run_demo.sh            (about 6 minutes; no sudo; close QGroundControl first)
#
# Result: runs/demo/  mission_plan.csv, telemetry.csv, geotags.csv, events.csv, run_meta.json,
#                     link_stats.csv (same flight), px4.log, report/ (plots, summary.json, report.html)
# An existing runs/demo is kept as runs/demo.prev-<timestamp>, never deleted.
set -euo pipefail
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO"
[[ -x build/mission_node && -x build/link_monitor ]] || {
    echo "Build first: cmake -S . -B build && cmake --build build -j2" >&2; exit 1; }
if ss -ulpn | grep -q ":14550 "; then
    echo "UDP 14550 is in use (QGroundControl?). Close it and retry." >&2; exit 1
fi

STAGE="runs/demo-staging-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$STAGE"
PX4_LOG="$STAGE/px4.log"

cleanup() {
    [[ -n "${MON:-}" ]] && kill -INT "$MON" 2>/dev/null && wait "$MON" 2>/dev/null || true
    (cd "$PX4_DIR/build/px4_sitl_default" && timeout 5 ./bin/px4-shutdown) >/dev/null 2>&1 || true
    sleep 2
    pkill -x px4 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "sleep 1500" 2>/dev/null || true
}
trap cleanup EXIT

echo "[1/4] Starting PX4 SITL (headless)..."
( sleep 1500 | HEADLESS=1 make -C "$PX4_DIR" px4_sitl gz_x500 > "$PX4_LOG" 2>&1 ) &
for _ in $(seq 240); do grep -q "Startup script returned successfully" "$PX4_LOG" && break; sleep 1; done
grep -q "Startup script returned successfully" "$PX4_LOG" || { echo "PX4 did not start"; exit 1; }
sleep 5

echo "[2/4] Link monitor on UDP 14550, mission_node on UDP 14540..."
./build/link_monitor --port 14550 --out "$STAGE/link_stats.csv" > "$STAGE/link_monitor.log" 2>&1 &
MON=$!
set +e
./build/mission_node --out "$STAGE" | tee "$STAGE/mission_node.log" | grep -vE "^mode "
RC=${PIPESTATUS[0]}
set -e
kill -INT "$MON"; wait "$MON" || true; MON=""
echo "mission_node exit code: $RC"

# mission_node made one timestamped folder inside the staging folder: flatten it into runs/demo.
RUN=$(ls -d "$STAGE"/2*/ | head -1)
mv "$RUN"* "$STAGE"/ && rmdir "$RUN"
if [[ -e runs/demo ]]; then
    mv runs/demo "runs/demo.prev-$(date +%Y%m%d-%H%M%S)"
fi
mv "$STAGE" runs/demo

echo "[3/4] Report..."
python3 analysis/report.py runs/demo --link runs/demo/link_stats.csv > /dev/null
echo "[4/4] Done. Open runs/demo/report/report.html"
grep -oE '"(outcome|captures|queue_drops|spacing_abs_mean_m|spacing_abs_max_m|loss_pct|mean_rate_hz)": [^,]*' \
    runs/demo/report/summary.json
exit "$RC"
