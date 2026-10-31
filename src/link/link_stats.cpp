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
    if (seq != it->second) {
        // 8-bit wrap: computing in uint8_t makes 0 - 255 - 1 == 0, i.e. no gap across
        // 255 -> 0. A duplicate (seq == last) would give 255 here, so it is excluded above.
        const auto missing = static_cast<std::uint8_t>(seq - it->second - 1);
        lost_ += missing;
    }
    it->second = seq;
}

std::uint64_t LinkStats::received() const { return received_; }

std::uint64_t LinkStats::lost() const { return lost_; }

double LinkStats::loss_ratio() const {
    const std::uint64_t total = received_ + lost_;
    return total == 0 ? 0.0 : static_cast<double>(lost_) / static_cast<double>(total);
}

}  // namespace asl::link
