# Demo video script (about 3 minutes, screen recording)

Record with QGroundControl open for the map view. Run mission_node by hand (not run_demo.sh, which
needs UDP 14550 that QGC uses):
`scripts/run_sitl.sh --headless` in terminal 1, `./build/mission_node` in terminal 2.

| time | show | say |
|---|---|---|
| 0:00 | README results table | "AgriSurvey-Link: a simulated crop-survey drone. C++17 on PX4 SITL. Every number here comes from a committed run." |
| 0:15 | `config/field_local.csv`, `camera.ini` | "Input is a field polygon and a camera model. Footprint and overlap give 10.4 m line spacing and a 6.2 m trigger distance." |
| 0:35 | terminal 2: mission_node start | "mission_node plans the lawnmower, uploads it with MAVSDK, waits for health and home, arms and takes off." |
| 0:55 | QGC map: vehicle on the lines | "On each survey line the position callback runs a distance trigger and pushes a capture into a bounded queue. A separate thread writes geotags, so the callback never blocks." |
| 1:25 | `tail -f runs/<run>/events.csv` | "Every mode, health and GPS change goes to events.csv." |
| 1:40 | `runs/demo/report/report.html` | "After landing, one command builds the report: planned vs flown path, spacing error, a SYNTHETIC NDVI map. It's labelled synthetic because SITL has no multispectral camera." |
| 2:10 | link.png from runs/link-netem-5pct-20261031-142544 | "The link monitor uses my own MAVLink v2 parser. With 5 % injected loss the first version reported 70 %. It counted reordered packets as 254-frame gaps. Fixed with the RTP half-window rule; it now measures 4.88 %." |
| 2:40 | docs/test-report.md failure table | "Fault injection: GPS loss gives blind land in 5.5 s; GCS link loss gives Hold at 12.9 s, then Return. My node detects the takeover and never fights the autopilot." |
| 2:55 | GitHub Actions green tick | "CI builds with warnings as errors and runs all tests under AddressSanitizer." |
