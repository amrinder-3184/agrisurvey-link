# Phase 0: environment

## What was built

A working simulation environment on a Windows laptop (8 GB RAM):

| Component | Version / detail |
| --- | --- |
| Linux | Ubuntu 24.04.5 LTS under WSL2 (5 GB RAM, 8 GB swap, 4 CPUs) |
| Toolchain | g++ 13.3.0, CMake 3.28.3, Ninja 1.11.1, Python 3.12.3 |
| PX4-Autopilot | `main` at commit `789389fd46` (`v1.18.0-beta1-1392`), 45 submodules |
| Simulator | Gazebo Sim 8.15.0 (Harmonic), run headless (`HEADLESS=1`) |
| MAVSDK | `libmavsdk-dev` 4.1.0 (C++), SHA-256 checked against the GitHub release |
| Ground station | QGroundControl 5.1.5 AppImage via WSLg, SHA-256 checked |
| Project | builds with 0 warnings; `ctest` runs (6/42 pass only because stubs return zeros) |

Acceptance (all observed on 2026-10-31):
- `make px4_sitl -j2`: 1201/1201 targets in 232 s, peak RAM about 3.7 GB, no swap.
- SITL takeoff and landing from the `pxh>` shell, with QGC connected
  (`Ready for takeoff!` -> `Takeoff detected` -> `Landing detected` -> `Disarmed by landing`).
- `scripts/check_env.sh`: every item `[ok]`.
- CMake prints `MAVSDK found: building mission executables`.

## Key decisions

- **WSL2 instead of dual boot.** No repartitioning; PX4 SITL runs fine headless. Repo lives
  in the Linux filesystem (`~/`), not under `/mnt/c`, because cross-filesystem builds are slow.
- **Headless Gazebo and `-j2` builds.** With 4.8 GiB available to Linux, a default ninja
  build (about 6 parallel jobs) risks the out-of-memory killer. PX4's Makefile forwards
  `-j N` to ninja (Makefile lines 77-107).
- **`ubuntu.sh --no-nuttx`.** The NuttX toolchain (about 3.6 GB installed) is only for
  flashing real flight controllers; SITL compiles with the host g++.
- **PX4 `main`, pinned to a commit.** Matches the dialect used for `crc_extra.h`; the exact
  commit is recorded so runs are reproducible. Fallback: `v1.17.0`.

## Problems hit and how they were diagnosed

1. **HTTPS from WSL timed out after the TLS Client Hello.** First guess (MTU black hole)
   was disproved: lowering the MTU did not help. Evidence: Windows connected through the
   network's HTTP proxy (`curl -v` showed `CONNECT github.com:443`), while WSL had no proxy
   set. Fix: proxy variables in `~/.proxy.sh`, enabled only when the proxy is reachable.
2. **`sudo` dropped the proxy variables**, so the Gazebo key download in `ubuntu.sh` would
   fail. Fix: one `/etc/sudoers.d/` rule keeping the six proxy variables (validated with
   `visudo -c` before install).
3. **GitHub release downloads returned 504 through the proxy** (also from Windows). Used
   QGC's official CDN instead and verified the SHA-256 matched the GitHub digest.
4. **PX4 refused to arm: `Preflight Fail: No connection to the GCS`.** Source:
   `src/modules/commander/HealthAndArmingChecks/checks/rcAndDataLinkCheck.cpp`; the check
   is active because the data-link-loss failsafe `NAV_DLL_ACT` is enabled. Fixed by
   connecting a GCS (QGC, or any program sending a GCS HEARTBEAT on UDP 14550), not by
   disabling the failsafe, which Phase 5 tests.

Open observation: a few `vehicle_imu ... timestamp error` lines appeared when QGC started
(high CPU load). Flight was unaffected. Watch for it in long survey runs.

## What an interviewer may ask

1. **What is SITL, and how is it different from HITL?**
   SITL (software-in-the-loop) compiles the real PX4 flight stack for the PC; a simulator
   (Gazebo) provides sensor data and physics. HITL runs the firmware on the real flight
   controller board, with the simulator feeding it sensor data over USB/serial. SITL tests
   the logic; HITL also tests the real board's timing and drivers.

2. **Why does PX4 SITL use UDP ports 14550 and 14540?**
   14550 is the ground-station link (QGC, or the link monitor); 14540 is the offboard/API
   link for companion programs (MAVSDK, the mission node). Separate links mean the
   operator's GCS and the onboard program don't compete for one socket, and each link can
   have its own message set and rate. A real drone does the same: GCS over a telemetry
   radio, companion computer over UART/Ethernet.

3. **PX4 would not arm in simulation. How did you debug it?**
   Read the exact warning (`No connection to the GCS`), grepped the PX4 source for it, and
   found it is enabled by the data-link-loss failsafe `NAV_DLL_ACT`. The fix was to connect
   a GCS that sends a HEARTBEAT, not to turn off a safety check.

4. **What is a MAVLink HEARTBEAT for?**
   Every MAVLink node sends it (usually at 1 Hz) to announce it exists: system/component
   ID, vehicle type, autopilot, mode and state. Receivers use it to discover systems and to
   detect link loss when heartbeats stop arriving (the link monitor warns after 3 s).

5. **Why did you build PX4 with `-j2`?**
   Constraint: 4.8 GiB of RAM in WSL. Each parallel compiler process holds its own memory,
   so ninja's default of about 6 jobs risks the out-of-memory killer. At `-j2` the build
   peaked at about 3.7 GB used with no swap and took 232 s (measured), so -j2 was a safe
   choice; -j3 or -j4 is untested.
