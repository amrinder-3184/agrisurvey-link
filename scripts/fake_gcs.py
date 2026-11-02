"""Minimal stand-in ground station for SITL tests.

Binds UDP 14550 (PX4's GCS link), waits for the autopilot heartbeat, then sends a
MAV_TYPE_GCS heartbeat at 1 Hz. Killing this process is how scripts/failure_test.sh
simulates losing the radio link to the ground station.
"""
import time

from pymavlink import mavutil

m = mavutil.mavlink_connection("udpin:0.0.0.0:14550", source_system=255)
m.wait_heartbeat(timeout=60)
print(f"fake_gcs: PX4 heartbeat from sys {m.target_system}", flush=True)
while True:
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    t_end = time.time() + 1.0
    while time.time() < t_end:
        m.recv_match(blocking=True, timeout=0.2)  # drain incoming telemetry
