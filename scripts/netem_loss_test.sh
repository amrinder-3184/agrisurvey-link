#!/usr/bin/env bash
# Injected-packet-loss test for link_monitor (phase 4).
#
#   scripts/netem_loss_test.sh [loss_percent]      (default 5)
#
# Starts PX4 SITL headless, runs link_monitor on UDP 14550, and adds `tc netem` loss on the
# loopback interface for 60 s between a 20 s baseline and a 20 s recovery period. Then it
# prints the loss link_monitor measured in each period.
#
# Needs sudo for `tc`. The loss rule is ALWAYS removed on exit (trap), even on Ctrl-C.
# Close QGroundControl first: only one program can bind UDP 14550.
set -euo pipefail
LOSS=${1:-5}
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$REPO/runs/link-netem-${LOSS}pct-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT"

sudo -v  # ask for the password now, not in the middle of the test

cleanup() {
    sudo tc qdisc del dev lo root 2>/dev/null || true   # never leave loss on loopback
    [[ -n "${MON:-}" ]] && kill -INT "$MON" 2>/dev/null && wait "$MON" 2>/dev/null || true
    pkill -x px4 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "sleep 900" 2>/dev/null || true
    echo "cleanup done: $(tc qdisc show dev lo | head -1)"
}
trap cleanup EXIT

if ss -ulpn | grep -q ":14550 "; then
    echo "UDP 14550 is already in use (QGroundControl?). Close it and retry." >&2
    exit 1
fi

echo "Starting PX4 SITL (headless)..."
( sleep 900 | HEADLESS=1 make -C "$PX4_DIR" px4_sitl gz_x500 > "$OUT/px4.log" 2>&1 ) &
for _ in $(seq 240); do grep -q "Startup script returned successfully" "$OUT/px4.log" && break; sleep 1; done
grep -q "Startup script returned successfully" "$OUT/px4.log" || { echo "PX4 did not start"; exit 1; }

"$REPO/build/link_monitor" --port 14550 --out "$OUT/link_stats.csv" > "$OUT/monitor.log" 2>&1 &
MON=$!
T0=$(date +%s.%N)
mark() { echo "$1,$(echo "$(date +%s.%N) - $T0" | bc)" >> "$OUT/markers.csv"; }
echo "phase,t_s" > "$OUT/markers.csv"

echo "Baseline 20 s..."; mark baseline_start; sleep 20
echo "Adding ${LOSS}% loss on lo for 60 s..."
sudo tc qdisc add dev lo root netem loss "${LOSS}%"; mark loss_start; sleep 60
sudo tc qdisc del dev lo root; mark loss_end
echo "Recovery 20 s..."; sleep 20; mark recovery_end

kill -INT "$MON"; wait "$MON" || true; MON=""

# Loss inside each period, from the cumulative counters at its boundaries.
python3 - "$OUT" "$LOSS" <<'EOF'
import csv, sys
out, loss = sys.argv[1], sys.argv[2]
rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(open(f"{out}/link_stats.csv"))]
marks = {r["phase"]: float(r["t_s"]) for r in csv.DictReader(open(f"{out}/markers.csv"))}
def at(t):  # last row at or before time t
    return [r for r in rows if r["t_s"] <= t][-1]
def period(name, a, b):
    ra, rb = at(a + 2), at(b)  # skip 2 s after each change: let in-flight packets settle
    frames = rb["frames_ok"] - ra["frames_ok"]
    lost = rb["lost"] - ra["lost"]
    pct = 100.0 * lost / (frames + lost) if frames + lost else 0.0
    print(f"{name:10s} {rb['t_s'] - ra['t_s']:5.0f} s  frames {frames:7.0f}  lost {lost:6.0f}  loss {pct:5.2f}%"
          f"  reordered {rb.get('reordered', 0) - ra.get('reordered', 0):5.0f}  crc_errors +{rb['crc_errors'] - ra['crc_errors']:.0f}")
print(f"Injected loss: {loss}%   (results in {out})")
period("baseline", marks["baseline_start"], marks["loss_start"])
period("netem", marks["loss_start"], marks["loss_end"])
period("recovery", marks["loss_end"], marks["recovery_end"])
EOF
