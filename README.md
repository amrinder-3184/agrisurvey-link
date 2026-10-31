# AgriSurvey-Link

A simulated agricultural survey drone: a C++ companion-computer service plans and flies a
lawnmower survey on **PX4 SITL**, triggers a payload by distance, and geotags every capture.
A link monitor uses a **from-scratch MAVLink v2 parser** to report packet loss and heartbeat
health, and a Python report analyses each flight afterwards.

> **Status: work in progress (phase 0 of 7).** See [docs/ROADMAP.md](docs/ROADMAP.md).
> Results below will be filled in only from real runs.

## Architecture

```
PX4 SITL + Gazebo --UDP 14540--> mission_node (C++, MAVSDK) --> geotags.csv, events.csv
                  --UDP 14550--> link_monitor (C++, own parser) --> link_stats.csv
                                                    after landing --> analysis/report.py
```

## Quick start

See [docs/SETUP.md](docs/SETUP.md) for PX4, QGroundControl and MAVSDK. Then:

```bash
cmake -S . -B build && cmake --build build -j
ctest --test-dir build --output-on-failure
scripts/run_sitl.sh            # terminal 1: PX4 SITL + Gazebo
./build/hello_drone udpin://0.0.0.0:14540   # terminal 2
```

## Layout

| Path | What |
| --- | --- |
| `include/asl/` | Specs (headers) for geometry, survey planning, trigger |
| `include/asl/link/` | CRC, MAVLink parser, link statistics; `crc_extra.h` is generated |
| `src/` | Implementations |
| `tests/` | GoogleTest specs; `vectors.h` = real frames encoded by pymavlink |
| `tools/` | Generators for `crc_extra.h` and `vectors.h` |

## Results

| Metric | Value | How measured |
| --- | --- | --- |
| Captures per survey | TBD | `geotags.csv` row count |
| Trigger spacing error (mean / max) | TBD | `analysis/report.py` |
| Parser unit tests | TBD | `ctest` |
| Loss reported under 5% injected loss | TBD | `tc netem` + `link_stats.csv` |

## Limitations

Simulation only (PX4 SITL); the NDVI values are synthetic; no real camera or hardware.

## License

MIT
