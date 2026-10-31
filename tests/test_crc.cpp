// Spec for include/asl/link/crc.h (phase 4).
#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "asl/link/crc.h"
#include "asl/link/crc_extra.h"
#include "vectors.h"

using namespace asl::link;

TEST(Crc, EmptyBufferIsInitValue) {
    EXPECT_EQ(crc_calculate(nullptr, 0), 0xFFFF);
}

TEST(Crc, StandardCheckValue) {
    // Every CRC variant publishes a "check" value: the CRC of ASCII "123456789".
    // For CRC-16/MCRF4XX it is 0x6F91.
    const std::string s = "123456789";
    EXPECT_EQ(crc_calculate(reinterpret_cast<const std::uint8_t*>(s.data()), s.size()), 0x6F91);
}

TEST(Crc, AccumulateMatchesCalculate) {
    const std::uint8_t data[] = {0x01, 0x02, 0xFD, 0xFF, 0x00};
    std::uint16_t crc = kCrcInit;
    for (std::uint8_t b : data) crc_accumulate(b, crc);
    EXPECT_EQ(crc, crc_calculate(data, sizeof(data)));
}

TEST(Crc, RealHeartbeatChecksum) {
    // Recompute the checksum of a pymavlink-encoded HEARTBEAT and compare to the bytes on the wire.
    const auto& f = vectors::kHeartbeat;
    const std::size_t len = f[1];
    std::uint16_t crc = crc_calculate(f.data() + 1, 9 + len);  // 9 header bytes after magic + payload
    const MsgInfo* info = find_msg_info(0);
    ASSERT_NE(info, nullptr);
    EXPECT_EQ(info->crc_extra, 50);
    crc_accumulate(info->crc_extra, crc);
    const std::uint16_t on_wire = static_cast<std::uint16_t>(f[10 + len] | (f[11 + len] << 8));
    EXPECT_EQ(crc, on_wire);
}

TEST(CrcExtraTable, LookupWorks) {
    ASSERT_NE(find_msg_info(30), nullptr);
    EXPECT_STREQ(find_msg_info(30)->name, "ATTITUDE");
    EXPECT_EQ(find_msg_info(33)->crc_extra, 104);  // GLOBAL_POSITION_INT
    EXPECT_EQ(find_msg_info(0xFFFFFF), nullptr);   // not a real message
}
