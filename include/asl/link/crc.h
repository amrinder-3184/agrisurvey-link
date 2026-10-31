// MAVLink checksum: CRC-16/MCRF4XX (often called "X.25" in MAVLink sources).
//
// Phase 4. YOU implement these in src/link/crc.cpp. Tests: tests/test_crc.cpp.
//
// How MAVLink uses it (v2 frame):
//   crc = 0xFFFF
//   accumulate every byte AFTER the 0xFD magic byte, up to the end of the payload
//   accumulate one more byte: CRC_EXTRA for that message id (see crc_extra.h)
//   the result is sent little-endian (low byte first) right after the payload
//
// Algorithm for one byte (this is the whole thing; make sure you can explain it):
//   tmp = byte XOR (crc & 0xFF)
//   tmp = tmp XOR (tmp << 4)                       -- kept to 8 bits
//   crc = (crc >> 8) XOR (tmp << 8) XOR (tmp << 3) XOR (tmp >> 4)
#pragma once

#include <cstddef>
#include <cstdint>

namespace asl::link {

constexpr std::uint16_t kCrcInit = 0xFFFF;

/// Fold one byte into a running CRC.
void crc_accumulate(std::uint8_t byte, std::uint16_t& crc);

/// CRC of a whole buffer, starting from kCrcInit.
std::uint16_t crc_calculate(const std::uint8_t* data, std::size_t len);

}  // namespace asl::link
