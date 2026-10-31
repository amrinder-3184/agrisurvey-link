#!/usr/bin/env bash
# Check that the phase-0 toolchain is installed. Prints [ok] / [MISSING] per item.
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
status=0
check() {  # check "<label>" <command...>
  local label="$1"; shift
  if "$@" >/dev/null 2>&1; then echo "[ok]      $label"; else echo "[MISSING] $label"; status=1; fi
}
check "g++"                         command -v g++
check "cmake >= 3.16"               bash -c 'v=$(cmake --version | head -1 | grep -oE "[0-9]+\.[0-9]+"); [[ $(printf "%s\n3.16\n" "$v" | sort -V | head -1) == 3.16 ]]'
check "git"                         command -v git
check "python3"                     command -v python3
check "pymavlink (python)"          python3 -c "import pymavlink"
check "pandas + matplotlib"         python3 -c "import pandas, matplotlib"
check "PX4-Autopilot at $PX4_DIR"   test -d "$PX4_DIR"
check "Gazebo (gz)"                 command -v gz
check "MAVSDK CMake package"        bash -c 'find /usr /usr/local /opt -name MAVSDKConfig.cmake 2>/dev/null | grep -q .'
check "GoogleTest (else fetched)"   bash -c 'find /usr -name GTestConfig.cmake 2>/dev/null | grep -q .'
check "socat (serial tests)"        command -v socat
echo "RAM: $(free -h | awk '/Mem:/ {print $2}')  (8 GB minimum, 16 GB comfortable)"
exit $status
