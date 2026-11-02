#!/usr/bin/env bash
# Failure-injection test (phase 5): fly the survey in PX4 SITL and inject a failure mid-survey.
#
#   scripts/failure_test.sh gps        GPS loss:  PX4 shell `failure gps off` (SYS_FAILURE_EN=1
#                                      is the SITL default)
#   scripts/failure_test.sh datalink   GCS link loss: mission_node runs as a companion computer,
#                                      scripts/fake_gcs.py is the GCS; we kill it.
#                                      gz_x500 sets NAV_DLL_ACT=2 (Return), COM_DL_LOSS_T=10 s.
#   scripts/failure_test.sh none       baseline: same flight, no failure
#
# The failure is injected when mission_node logs mission item >= INJECT_ITEM (default 5,
# i.e. on survey line 2). The injection time is written to <run>/injection.csv on the
# monotonic clock, the same clock as events.csv, so the reaction latency is exact.
# No sudo needed. Close QGroundControl first for the datalink test (UDP 14550).
set -euo pipefail
KIND=${1:?usage: failure_test.sh gps|datalink|none}
INJECT_ITEM=${INJECT_ITEM:-5}
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin"
LOG_DIR="$REPO/runs/failure-logs"; mkdir -p "$LOG_DIR"
STAMP=$(date +%Y%m%d-%H%M%S)
PX4_LOG="$LOG_DIR/px4-$KIND-$STAMP.log"

case "$KIND" in gps|datalink|none) ;; *) echo "unknown failure kind: $KIND" >&2; exit 1 ;; esac

mono_ns() { python3 -c 'import time; print(time.monotonic_ns())'; }

cleanup() {
    [[ -n "${GCS:-}" ]] && kill "$GCS" 2>/dev/null || true
    [[ -n "${NODE:-}" ]] && kill -INT "$NODE" 2>/dev/null || true
    (cd "$PX4_DIR/build/px4_sitl_default" && timeout 5 ./bin/px4-shutdown) >/dev/null 2>&1 || true
    sleep 2
    pkill -x px4 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "sleep 1800" 2>/dev/null || true
}
trap cleanup EXIT

echo "Starting PX4 SITL (headless)..."
( sleep 1800 | HEADLESS=1 make -C "$PX4_DIR" px4_sitl gz_x500 > "$PX4_LOG" 2>&1 ) &
for _ in $(seq 240); do grep -q "Startup script returned successfully" "$PX4_LOG" && break; sleep 1; done
grep -q "Startup script returned successfully" "$PX4_LOG" || { echo "PX4 did not start"; exit 1; }
sleep 5

ROLE=gcs
if [[ "$KIND" == datalink ]]; then
    ROLE=companion
    python3 "$REPO/scripts/fake_gcs.py" > "$LOG_DIR/fake_gcs-$STAMP.log" 2>&1 &
    GCS=$!
fi

cd "$REPO"
BEFORE=$(ls -d runs/2*/ 2>/dev/null | tail -1 || true)
./build/mission_node --role "$ROLE" --out runs > "$LOG_DIR/mission_node-$KIND-$STAMP.log" 2>&1 &
NODE=$!

# The run folder appears within a second of start.
RUN=""
for _ in $(seq 20); do
    RUN=$(ls -d runs/2*/ 2>/dev/null | tail -1 || true)
    [[ -n "$RUN" && "$RUN" != "$BEFORE" ]] && break
    sleep 0.5
done
[[ -n "$RUN" && "$RUN" != "$BEFORE" ]] || { echo "mission_node made no run folder"; exit 1; }
echo "Run folder: $RUN  (role $ROLE)"

if [[ "$KIND" != none ]]; then
    echo "Waiting for mission item >= $INJECT_ITEM..."
    until awk -F, -v k="$INJECT_ITEM" '$3=="mission_item" { split($4, a, "/"); if (a[1] + 0 >= k) f=1 } END { exit !f }' \
            "$RUN/events.csv" 2>/dev/null; do
        kill -0 "$NODE" 2>/dev/null || { echo "mission_node exited before injection"; break; }
        sleep 0.5
    done
    T_INJ=$(mono_ns)
    if [[ "$KIND" == gps ]]; then
        (cd "$PX4_DIR/build/px4_sitl_default" && ./bin/px4-failure gps off) | sed 's/\x1b\[[0-9;]*m//g'
    else
        kill "$GCS"; wait "$GCS" 2>/dev/null || true; GCS=""
    fi
    printf 't_mono_ns,failure\n%s,%s\n' "$T_INJ" "$KIND" > "$RUN/injection.csv"
    echo "Injected '$KIND' failure."
fi

set +e
wait "$NODE"; RC=$?; NODE=""
set -e
echo "mission_node exit code: $RC"
cp "$PX4_LOG" "$RUN/px4.log"

echo "--- events.csv (time relative to the injection, if any)"
python3 - "$RUN" <<'EOF'
import csv, os, sys
run = sys.argv[1]
inj = None
if os.path.exists(f"{run}/injection.csv"):
    inj = int(next(csv.DictReader(open(f"{run}/injection.csv")))["t_mono_ns"])
for r in csv.DictReader(open(f"{run}/events.csv")):
    if r["event"] == "mission_item":
        continue  # one per waypoint: too noisy for this summary
    rel = f"{(int(r['t_mono_ns']) - inj) / 1e9:+7.1f} s" if inj else f"{float(r['t_s']):7.1f} s"
    print(f"  {rel}  {r['event']:16s} {r['detail']}")
EOF
echo "--- PX4 failsafe messages"
sed 's/\x1b\[[0-9;]*m//g' "$PX4_LOG" | grep -iE "failsafe|failure|connection to the GCS|GCS connection|data link|navigation|position|Landing detected|Disarmed" \
    | grep -v "Preflight Fail" | head -20
echo "Results in $RUN"
