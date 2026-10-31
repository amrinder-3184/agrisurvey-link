#!/usr/bin/env python3
"""Generate include/asl/link/crc_extra.h: msgid -> (CRC_EXTRA, max payload length, name).

Every MAVLink message has a CRC_EXTRA byte, computed from its XML definition (field
names and types). Sender and receiver both fold it into the checksum, so if the two
sides were built from different message definitions the CRC fails instead of the
bytes being silently misread.

Usage (on your machine):
    pip install pymavlink
    python3 tools/gen_crc_extra.py > include/asl/link/crc_extra.h

PX4 is built with the MAVLink "development" dialect (a superset of "common"), so that
is the default here.
"""
import argparse
import importlib
import sys


def load_dialect(name, module_path):
    if module_path:
        sys.path.insert(0, module_path)
        return importlib.import_module(name)
    return importlib.import_module(f"pymavlink.dialects.v20.{name}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dialect", default="development")
    ap.add_argument("--module-path", help="directory holding a mavgen-generated dialect module")
    ap.add_argument("--source-note", default="pymavlink", help="provenance note for the header")
    args = ap.parse_args()

    m = load_dialect(args.dialect, args.module_path)
    rows = []
    for msgid in sorted(m.mavlink_map):
        cls = m.mavlink_map[msgid]
        rows.append((msgid, cls.crc_extra, cls.unpacker.size, cls.msgname))

    out = []
    out.append("// GENERATED FILE - do not edit by hand. Regenerate with tools/gen_crc_extra.py")
    out.append(f"// Dialect: {args.dialect} ({len(rows)} messages). Source: {args.source_note}")
    out.append("#pragma once")
    out.append("")
    out.append("#include <algorithm>")
    out.append("#include <cstdint>")
    out.append("#include <iterator>")
    out.append("")
    out.append("namespace asl::link {")
    out.append("")
    out.append("struct MsgInfo {")
    out.append("    std::uint32_t msgid;")
    out.append("    std::uint8_t crc_extra;")
    out.append("    std::uint8_t max_payload_len;  // full length incl. extensions, before trimming")
    out.append("    const char* name;")
    out.append("};")
    out.append("")
    out.append("// Sorted by msgid so lookups can use binary search.")
    out.append("inline constexpr MsgInfo kMsgTable[] = {")
    for msgid, extra, size, name in rows:
        out.append(f"    {{{msgid}, {extra}, {size}, \"{name}\"}},")
    out.append("};")
    out.append("")
    out.append("/// Returns nullptr if msgid is not in the dialect.")
    out.append("inline const MsgInfo* find_msg_info(std::uint32_t msgid) {")
    out.append("    const auto* first = std::begin(kMsgTable);")
    out.append("    const auto* last = std::end(kMsgTable);")
    out.append("    const auto* it = std::lower_bound(first, last, msgid,")
    out.append("        [](const MsgInfo& info, std::uint32_t id) { return info.msgid < id; });")
    out.append("    return (it != last && it->msgid == msgid) ? it : nullptr;")
    out.append("}")
    out.append("")
    out.append("}  // namespace asl::link")
    print("\n".join(out))


if __name__ == "__main__":
    main()
