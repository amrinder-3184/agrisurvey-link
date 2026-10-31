# Phase 0: environment setup (Ubuntu 24.04)

Target: a quadcopter that takes off in the Gazebo simulator and that QGroundControl can see,
plus this repository building on your machine.

**Requirements:** Ubuntu 24.04 LTS installed natively or dual-boot (best), 8 GB RAM minimum
(16 GB comfortable), about 40 GB free disk. WSL2 on Windows works with headless mode but is
slower. Current QGroundControl builds target Ubuntu 24.04, so use 24.04.

## 1. Base tools

```bash
sudo apt update
sudo apt install -y git build-essential cmake python3-pip libgtest-dev socat
```

## 2. PX4 SITL with Gazebo

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh      # installs toolchain + Gazebo; reboot afterwards
cd ~/PX4-Autopilot
make px4_sitl gz_x500                            # first build: 10-30 min
```

Weak laptop: run `HEADLESS=1 make px4_sitl gz_x500` (no 3D window; QGC still shows the map).

Check: you get a `pxh>` prompt. Type `commander takeoff`, wait, then `commander land`.

What PX4 does on the network: it **sends** MAVLink over UDP to port **14550** (for a ground
station) and port **14540** (for offboard/API programs such as MAVSDK). Your programs
listen on those ports.

## 3. QGroundControl

```bash
sudo usermod -aG dialout "$(id -un)"                  # log out and back in afterwards
sudo systemctl mask --now ModemManager.service        # stops it grabbing serial ports
sudo apt install -y gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl libfuse2 \
                    libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev
```

Download `QGroundControl-x86_64.AppImage` from the QGroundControl docs ("Download and
Install"), then run:

```bash
chmod +x QGroundControl-x86_64.AppImage && ./QGroundControl-x86_64.AppImage
```

With PX4 SITL running, QGC connects automatically on UDP 14550 and shows the drone on the map.

## 4. MAVSDK (C++ library, v4)

From https://github.com/mavlink/MAVSDK/releases download the newest
`libmavsdk-dev_<version>_ubuntu24.04_amd64.deb`, then:

```bash
sudo apt remove mavsdk 2>/dev/null; sudo dpkg -i libmavsdk-dev_*_ubuntu24.04_amd64.deb
find / -name MAVSDKConfig.cmake 2>/dev/null    # should print a path: CMake will find it
```

**Note:** tutorials written for MAVSDK v1/v2 use a different API. This project uses v4
(headers `mavsdk/....hpp`, URLs like `udpin://0.0.0.0:14540`).

## 5. Python tools

```bash
pip install --user pymavlink pandas matplotlib pytest     # add --break-system-packages if pip refuses
```

## 6. Build this repository

```bash
cd agrisurvey-link
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

At this stage most tests **fail on purpose**: the implementations are yours to write
(phases 2–4). CMake prints `MAVSDK found: building mission executables` when step 4 worked.

Run `scripts/check_env.sh` to verify everything at once.

## Troubleshooting

| Symptom | Fix |
| --- | --- |
| `make px4_sitl gz_x500` can't find `gz` | Re-run `Tools/setup/ubuntu.sh`, then reboot |
| Gazebo very slow or crashes | `HEADLESS=1`; close the browser; check `free -h` |
| QGC shows "Disconnected" | Is PX4 running? Is another program bound to 14550? Check with `ss -ulpn \| grep 14550` |
| `hello_drone` times out waiting for the system | Use `udpin://0.0.0.0:14540`, not 14550; is PX4 running? |
| CMake: MAVSDK not found | The `.deb` wasn't installed, or there's a version mismatch; see step 4 |

## Phase 0 done when

- [ ] PX4 SITL starts, and the drone takes off and lands from the `pxh>` shell
- [ ] QGC connects and shows the vehicle
- [ ] CMake reports MAVSDK found; the repo builds; `ctest` runs (red is fine)
- [ ] You can explain what SITL is and why there are two UDP ports
