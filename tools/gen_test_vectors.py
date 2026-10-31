#!/usr/bin/env python3
"""Generate tests/vectors.h: real MAVLink v2 frames encoded by pymavlink.

The parser tests use these bytes, so they check your parser against an independent,
widely used implementation rather than against bytes typed by hand.

Usage (on your machine):
    pip install pymavlink
    python3 tools/gen_test_vectors.py > tests/vectors.h
"""
import argparse
import importlib
import sys


class Sink:
    def __init__(self):
        self.buf = b""

    def write(self, b):
        self.buf += b


def load_dialect(name, module_path):
    if module_path:
        sys.path.insert(0, module_path)
        return importlib.import_module(name)
    return importlib.import_module(f"pymavlink.dialects.v20.{name}")


def c_array(name, data, comment):
    body = ", ".join(f"0x{b:02X}" for b in data)
    return f"// {comment}\ninline const std::vector<std::uint8_t> {name} = {{{body}}};\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dialect", default="development")
    ap.add_argument("--module-path")
    args = ap.parse_args()
    m = load_dialect(args.dialect, args.module_path)

    def encoder(sysid=1, compid=1, seq=0):
        mav = m.MAVLink(Sink(), srcSystem=sysid, srcComponent=compid)
        mav.seq = seq
        return mav

    # 1. HEARTBEAT from a PX4 quadcopter (sysid 1, compid 1, seq 0)
    mav = encoder(seq=0)
    heartbeat = mav.heartbeat_encode(m.MAV_TYPE_QUADROTOR, m.MAV_AUTOPILOT_PX4,
                                     157, 0x40000, m.MAV_STATE_STANDBY).pack(mav)

    # 2. ATTITUDE with all fields non-zero (seq 1): full 28-byte payload
    mav = encoder(seq=1)
    attitude_full = mav.attitude_encode(123456, 0.1, -0.2, 1.5, 0.01, 0.02, 0.03).pack(mav)

    # 3. ATTITUDE whose last fields are zero (seq 2): v2 trims trailing zero bytes
    mav = encoder(seq=2)
    attitude_trimmed = mav.attitude_encode(123456, 0.1, -0.2, 1.5, 0.0, 0.0, 0.0).pack(mav)

    # 4. GLOBAL_POSITION_INT from sysid 1, compid 1, seq 3
    mav = encoder(seq=3)
    gpos = mav.global_position_int_encode(500000, int(47.397742e7), int(8.545594e7),
                                          488000, 20000, 150, -30, 0, 9000).pack(mav)

    # 5. Signed HEARTBEAT (incompat_flags bit 0 set, 13-byte signature appended)
    mav = encoder(seq=4)
    mav.signing.secret_key = bytes(range(32))
    mav.signing.link_id = 0
    mav.signing.timestamp = 1
    mav.signing.sign_outgoing = True
    heartbeat_signed = mav.heartbeat_encode(m.MAV_TYPE_QUADROTOR, m.MAV_AUTOPILOT_PX4,
                                            157, 0x40000, m.MAV_STATE_STANDBY).pack(mav)

    assert heartbeat[0] == 0xFD and heartbeat_signed[2] & 0x01
    assert attitude_trimmed[1] < attitude_full[1], "expected trailing-zero trimming"

    print("// GENERATED FILE - do not edit by hand. Regenerate with tools/gen_test_vectors.py")
    print(f"// Encoded by pymavlink, dialect '{args.dialect}'.")
    print("#pragma once")
    print()
    print("#include <cstdint>")
    print("#include <vector>")
    print()
    print("namespace vectors {")
    print()
    print(c_array("kHeartbeat", heartbeat,
                  "HEARTBEAT, sysid 1, compid 1, seq 0, payload 9 bytes (type=QUADROTOR, autopilot=PX4)"))
    print(c_array("kAttitudeFull", attitude_full,
                  f"ATTITUDE (msgid 30), seq 1, payload {attitude_full[1]} bytes, no trimming"))
    print(c_array("kAttitudeTrimmed", attitude_trimmed,
                  f"ATTITUDE (msgid 30), seq 2, payload trimmed to {attitude_trimmed[1]} bytes"))
    print(c_array("kGlobalPositionInt", gpos,
                  f"GLOBAL_POSITION_INT (msgid 33), seq 3, payload {gpos[1]} bytes"))
    print(c_array("kHeartbeatSigned", heartbeat_signed,
                  "HEARTBEAT, seq 4, SIGNED: incompat_flags = 0x01, 13 signature bytes after the CRC"))
    print("}  // namespace vectors")


if __name__ == "__main__":
    main()
