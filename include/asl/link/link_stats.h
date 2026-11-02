// Link-quality statistics from MAVLink sequence numbers.
//
// Phase 4. YOU implement this in src/link/link_stats.cpp. Tests: tests/test_link_stats.cpp.
//
// Every MAVLink sender increments an 8-bit `seq` for each frame it sends (255 wraps to 0).
// A receiver tracks the last seq it saw from each source; a jump means frames were lost.
//
// Specification:
//  - Track each source (sysid, compid) separately. PX4 can send from several components.
//  - The first frame from a source never counts as loss.
//  - After that, missing = (seq - last_seq - 1) mod 256, added to lost().
//  - seq == last_seq is a duplicate: count it as received, add no loss.
//  - loss_ratio() = lost / (received + lost), or 0.0 before anything is received.
//
// Reordering (added after the phase-4 netem test showed 70 % "loss" for 5 % injected loss):
//  - If (seq - last_seq) mod 256 >= 128 the frame is BEHIND the newest one: a late frame that
//    was already counted as missing when its gap was skipped. Count it as received and in
//    reordered(), take back that one lost frame, and do not move last_seq backwards.
//    (Half-window rule, as in RTP / RFC 3550.) Limitation: a burst of >= 128 consecutive
//    lost frames is indistinguishable from reordering; the heartbeat watchdog covers outages.
#pragma once

#include <cstdint>
#include <unordered_map>

namespace asl::link {

class LinkStats {
public:
    void on_frame(std::uint8_t sysid, std::uint8_t compid, std::uint8_t seq);

    std::uint64_t received() const;
    std::uint64_t lost() const;
    double loss_ratio() const;
    std::uint64_t reordered() const;  // late frames (arrived after a newer seq)

private:
    // Key = (sysid << 8) | compid. A link carries only a handful of sources, so a hash map
    // gives O(1) average lookup per frame with tiny memory.
    std::unordered_map<std::uint16_t, std::uint8_t> last_seq_;
    std::uint64_t received_ = 0;
    std::uint64_t lost_ = 0;
    std::uint64_t reordered_ = 0;
};

}  // namespace asl::link
