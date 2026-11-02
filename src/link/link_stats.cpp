// Phase 4. Spec: include/asl/link/link_stats.h. Tests: tests/test_link_stats.cpp.
#include "asl/link/link_stats.h"

namespace asl::link {

void LinkStats::on_frame(std::uint8_t sysid, std::uint8_t compid, std::uint8_t seq) {
    ++received_;
    const auto key = static_cast<std::uint16_t>((sysid << 8) | compid);
    const auto it = last_seq_.find(key);
    if (it == last_seq_.end()) {
        last_seq_.emplace(key, seq);  // first frame from this source: no history, no loss
        return;
    }
    if (seq == it->second) {
        return;  // duplicate: received, no loss
    }
    // Distance forward from the newest seq, in 8-bit arithmetic: 0 - 255 == 1 across the wrap.
    const auto ahead = static_cast<std::uint8_t>(seq - it->second);
    if (ahead >= 128) {
        // Behind the newest frame: a late (reordered) frame. It was counted as missing when
        // the newer frame skipped over it, so take that back. last_seq stays at the newest.
        ++reordered_;
        if (lost_ > 0) {
            --lost_;
        }
        return;
    }
    lost_ += static_cast<std::uint8_t>(ahead - 1);  // frames skipped between last and seq
    it->second = seq;
}

std::uint64_t LinkStats::received() const { return received_; }

std::uint64_t LinkStats::lost() const { return lost_; }

std::uint64_t LinkStats::reordered() const { return reordered_; }

double LinkStats::loss_ratio() const {
    const std::uint64_t total = received_ + lost_;
    return total == 0 ? 0.0 : static_cast<double>(lost_) / static_cast<double>(total);
}

}  // namespace asl::link
