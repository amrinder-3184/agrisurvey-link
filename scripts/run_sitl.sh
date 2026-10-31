#!/usr/bin/env bash
# Start PX4 SITL with the Gazebo x500 quadcopter.
# Usage: scripts/run_sitl.sh [--headless]      (set PX4_DIR if PX4 is not in ~/PX4-Autopilot)
set -euo pipefail
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
if [[ ! -d "$PX4_DIR" ]]; then
  echo "PX4 not found at $PX4_DIR (set PX4_DIR=/path/to/PX4-Autopilot)" >&2
  exit 1
fi
if [[ "${1:-}" == "--headless" ]]; then
  export HEADLESS=1
fi
cd "$PX4_DIR"
exec make px4_sitl gz_x500
