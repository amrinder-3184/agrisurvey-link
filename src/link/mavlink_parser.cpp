// Phase 4. Spec: include/asl/link/mavlink_parser.h. Tests: tests/test_mavlink_parser.cpp.
#include "asl/link/mavlink_parser.h"

#include <algorithm>

#include "asl/link/crc.h"
#include "asl/link/crc_extra.h"

namespace asl::link {

std::vector<Frame> MavlinkParser::push(const std::uint8_t* data, std::size_t len) {
    buf_.insert(buf_.end(), data, data + len);
    std::vector<Frame> frames;
    std::size_t pos = 0;  // start of the unconsumed data in buf_

    while (pos < buf_.size()) {
        // 1. Hunt for the v2 magic byte. Everything before it is garbage (incl. v1 frames).
        const auto magic = std::find(buf_.begin() + static_cast<std::ptrdiff_t>(pos), buf_.end(),
                                     kMagicV2);
        const auto magic_pos = static_cast<std::size_t>(magic - buf_.begin());
        stats_.bytes_skipped += magic_pos - pos;
        pos = magic_pos;
        if (pos == buf_.size()) {
            break;  // no magic in the buffer
        }

        // 2. Need the whole 10-byte header before deciding anything.
        if (buf_.size() - pos < kHeaderLen) {
            break;  // wait for more bytes
        }
        const std::uint8_t* h = buf_.data() + pos;
        const std::uint8_t payload_len = h[1];
        const std::uint8_t incompat = h[2];
        if ((incompat & ~kIncompatSigned) != 0) {
            ++stats_.bad_header;
            ++pos;  // resume right after this magic: it may have been a fake 0xFD
            continue;
        }
        const std::uint32_t msgid = static_cast<std::uint32_t>(h[7]) |
                                    (static_cast<std::uint32_t>(h[8]) << 8) |
                                    (static_cast<std::uint32_t>(h[9]) << 16);
        const MsgInfo* info = find_msg_info(msgid);
        if (info == nullptr) {
            ++stats_.unknown_msgid;  // without CRC_EXTRA we cannot validate it
            ++pos;
            continue;
        }

        // 3. Wait for payload + checksum (+ signature when the signed flag is set).
        const bool is_signed = (incompat & kIncompatSigned) != 0;
        const std::size_t frame_len =
            kHeaderLen + payload_len + kChecksumLen + (is_signed ? kSignatureLen : 0);
        if (buf_.size() - pos < frame_len) {
            break;  // wait for more bytes
        }
        h = buf_.data() + pos;  // (no reallocation since above, but keep it obviously valid)

        // 4. Checksum: every byte after the magic up to the end of the payload, then the
        //    message's CRC_EXTRA (catches sender/receiver disagreeing on the message layout).
        std::uint16_t crc = crc_calculate(h + 1, kHeaderLen - 1 + payload_len);
        crc_accumulate(info->crc_extra, crc);
        const auto on_wire = static_cast<std::uint16_t>(h[kHeaderLen + payload_len] |
                                                        (h[kHeaderLen + payload_len + 1] << 8));
        if (crc != on_wire) {
            ++stats_.crc_errors;
            ++pos;
            continue;
        }

        // 5. Valid frame. The payload is kept as received (possibly trimmed: MAVLink v2
        //    drops trailing zero bytes; the receiver would zero-fill when decoding fields).
        Frame f;
        f.seq = h[4];
        f.sysid = h[5];
        f.compid = h[6];
        f.msgid = msgid;
        f.is_signed = is_signed;
        f.payload.assign(h + kHeaderLen, h + kHeaderLen + payload_len);
        frames.push_back(std::move(f));
        ++stats_.frames_ok;
        pos += frame_len;  // the signature (if any) is consumed, not verified
    }

    // Drop everything consumed in this call in one erase (O(n) once, not per frame).
    buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(pos));
    return frames;
}

}  // namespace asl::link
