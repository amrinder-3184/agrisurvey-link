// Phase 4. Spec: include/asl/link/crc.h. Tests: tests/test_crc.cpp.
#include "asl/link/crc.h"

namespace asl::link {

// CRC-16/MCRF4XX, one byte at a time (the MAVLink "X.25" routine). Integer promotion
// turns uint8_t arithmetic into int, so every intermediate is cast back to its width.
void crc_accumulate(std::uint8_t byte, std::uint16_t& crc) {
    std::uint8_t tmp = static_cast<std::uint8_t>(byte ^ (crc & 0xFF));
    tmp = static_cast<std::uint8_t>(tmp ^ (tmp << 4));  // keep 8 bits
    crc = static_cast<std::uint16_t>((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4));
}

std::uint16_t crc_calculate(const std::uint8_t* data, std::size_t len) {
    std::uint16_t crc = kCrcInit;
    for (std::size_t i = 0; i < len; ++i) {
        crc_accumulate(data[i], crc);
    }
    return crc;
}

}  // namespace asl::link
