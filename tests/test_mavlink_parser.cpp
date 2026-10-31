// Spec for include/asl/link/mavlink_parser.h (phase 4).
// All frames come from tests/vectors.h, encoded by pymavlink (an independent implementation).
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "asl/link/mavlink_parser.h"
#include "vectors.h"

using asl::link::Frame;
using asl::link::MavlinkParser;
using Bytes = std::vector<std::uint8_t>;

namespace {
Bytes concat(std::initializer_list<Bytes> parts) {
    Bytes out;
    for (const Bytes& p : parts) out.insert(out.end(), p.begin(), p.end());
    return out;
}

std::vector<Frame> push_all(MavlinkParser& p, const Bytes& b) { return p.push(b.data(), b.size()); }
}  // namespace

TEST(Parser, SingleHeartbeat) {
    MavlinkParser p;
    auto frames = push_all(p, vectors::kHeartbeat);
    ASSERT_EQ(frames.size(), 1u);
    const Frame& f = frames[0];
    EXPECT_EQ(f.msgid, 0u);
    EXPECT_EQ(f.sysid, 1);
    EXPECT_EQ(f.compid, 1);
    EXPECT_EQ(f.seq, 0);
    EXPECT_FALSE(f.is_signed);
    ASSERT_EQ(f.payload.size(), 9u);
    EXPECT_EQ(f.payload[4], 2);   // HEARTBEAT.type = MAV_TYPE_QUADROTOR (after 4-byte custom_mode)
    EXPECT_EQ(f.payload[5], 12);  // HEARTBEAT.autopilot = MAV_AUTOPILOT_PX4
    EXPECT_EQ(p.stats().frames_ok, 1u);
    EXPECT_EQ(p.stats().bytes_skipped, 0u);
}

TEST(Parser, OneByteAtATime) {
    MavlinkParser p;
    std::vector<Frame> frames;
    for (std::uint8_t b : vectors::kGlobalPositionInt) {
        auto got = p.push(&b, 1);
        frames.insert(frames.end(), got.begin(), got.end());
    }
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0].msgid, 33u);
    EXPECT_EQ(frames[0].seq, 3);
}

TEST(Parser, SplitAtEveryPossibleOffset) {
    const Bytes& f = vectors::kAttitudeFull;
    for (std::size_t k = 1; k < f.size(); ++k) {
        MavlinkParser p;
        auto a = p.push(f.data(), k);
        auto b = p.push(f.data() + k, f.size() - k);
        EXPECT_EQ(a.size() + b.size(), 1u) << "split at byte " << k;
    }
}

TEST(Parser, BackToBackFramesInOrder) {
    MavlinkParser p;
    auto frames = push_all(p, concat({vectors::kHeartbeat, vectors::kAttitudeFull, vectors::kGlobalPositionInt}));
    ASSERT_EQ(frames.size(), 3u);
    EXPECT_EQ(frames[0].msgid, 0u);
    EXPECT_EQ(frames[1].msgid, 30u);
    EXPECT_EQ(frames[2].msgid, 33u);
    EXPECT_EQ(p.stats().frames_ok, 3u);
}

TEST(Parser, TrimmedPayloadIsValid) {
    MavlinkParser p;
    auto frames = push_all(p, vectors::kAttitudeTrimmed);
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0].msgid, 30u);
    EXPECT_LT(frames[0].payload.size(), 28u);  // ATTITUDE is 28 bytes untrimmed
    EXPECT_EQ(frames[0].payload.size(), vectors::kAttitudeTrimmed[1]);
}

TEST(Parser, LeadingGarbageIsSkipped) {
    MavlinkParser p;
    auto frames = push_all(p, concat({{0x00, 0x11, 0x22, 0x33, 0x44}, vectors::kHeartbeat}));
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(p.stats().bytes_skipped, 5u);
}

TEST(Parser, MavlinkV1MagicIsTreatedAsGarbage) {
    MavlinkParser p;
    auto frames = push_all(p, concat({{0xFE, 0x01, 0x02}, vectors::kHeartbeat}));
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(p.stats().bytes_skipped, 3u);
}

TEST(Parser, CorruptFrameDroppedAndNextFrameRecovered) {
    Bytes bad = vectors::kHeartbeat;
    bad[12] ^= 0xFF;  // flip bits in the payload; checksum no longer matches
    MavlinkParser p;
    auto frames = push_all(p, concat({bad, vectors::kAttitudeFull}));
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0].msgid, 30u);
    EXPECT_EQ(p.stats().crc_errors, 1u);
    EXPECT_EQ(p.stats().frames_ok, 1u);
}

TEST(Parser, FakeMagicInsideGarbage) {
    // 0xFD 0x02 looks like a frame start. Its header then reads incompat_flags = 0xFD (the real
    // frame's magic), which is unsupported. The parser must drop it and find the real frame
    // that starts two bytes later.
    MavlinkParser p;
    auto frames = push_all(p, concat({{0xFD, 0x02}, vectors::kHeartbeat}));
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0].msgid, 0u);
    EXPECT_EQ(p.stats().bad_header, 1u);
    EXPECT_EQ(p.stats().bytes_skipped, 1u);  // the 0x02
}

TEST(Parser, UnknownMessageIdDropped) {
    // Hand-built frame with msgid 0x123456, which no dialect defines.
    const Bytes unknown{0xFD, 0x03, 0x00, 0x00, 0x07, 0x01, 0x01, 0x56, 0x34, 0x12,
                        0x01, 0x02, 0x03, 0xAA, 0xBB};
    MavlinkParser p;
    auto frames = push_all(p, concat({unknown, vectors::kHeartbeat}));
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0].msgid, 0u);
    EXPECT_EQ(p.stats().unknown_msgid, 1u);
}

TEST(Parser, SignedFrameConsumesSignature) {
    MavlinkParser p;
    // If the 13 signature bytes are not consumed, the following heartbeat is lost or garbled.
    auto frames = push_all(p, concat({vectors::kHeartbeatSigned, vectors::kHeartbeat}));
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_TRUE(frames[0].is_signed);
    EXPECT_EQ(frames[0].seq, 4);
    EXPECT_FALSE(frames[1].is_signed);
    EXPECT_EQ(p.stats().bytes_skipped, 0u);
}
