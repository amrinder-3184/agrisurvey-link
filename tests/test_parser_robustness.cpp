// Extra parser and link-stats tests (added in phase 4, beyond the 42 spec tests):
// fuzzing, random chunking, interleaved garbage, maximum payload, long sequence wraps.
#include <gtest/gtest.h>

#include <cstdint>
#include <random>
#include <vector>

#include "asl/link/crc.h"
#include "asl/link/crc_extra.h"
#include "asl/link/link_stats.h"
#include "asl/link/mavlink_parser.h"
#include "vectors.h"

using asl::link::Frame;
using asl::link::MavlinkParser;
using Bytes = std::vector<std::uint8_t>;

namespace {

// Encode a v2 frame with our own CRC code (lets us build frames the vectors don't cover).
Bytes encode(std::uint32_t msgid, std::uint8_t seq, const Bytes& payload) {
    Bytes f{asl::link::kMagicV2,
            static_cast<std::uint8_t>(payload.size()),
            0x00,
            0x00,
            seq,
            1,
            1,
            static_cast<std::uint8_t>(msgid & 0xFF),
            static_cast<std::uint8_t>((msgid >> 8) & 0xFF),
            static_cast<std::uint8_t>((msgid >> 16) & 0xFF)};
    f.insert(f.end(), payload.begin(), payload.end());
    std::uint16_t crc = asl::link::crc_calculate(f.data() + 1, f.size() - 1);
    asl::link::crc_accumulate(asl::link::find_msg_info(msgid)->crc_extra, crc);
    f.push_back(static_cast<std::uint8_t>(crc & 0xFF));
    f.push_back(static_cast<std::uint8_t>(crc >> 8));
    return f;
}

const std::vector<const Bytes*> kValid = {&vectors::kHeartbeat, &vectors::kAttitudeFull,
                                          &vectors::kAttitudeTrimmed, &vectors::kGlobalPositionInt,
                                          &vectors::kHeartbeatSigned};

}  // namespace

// 200 kB of random bytes: no crash, no false frames (fixed seed: reproducible).
TEST(ParserRobustness, RandomGarbageProducesNoFrames) {
    std::mt19937 rng(12345);
    std::uniform_int_distribution<int> byte(0, 255);
    Bytes junk(200000);
    for (auto& b : junk) b = static_cast<std::uint8_t>(byte(rng));
    MavlinkParser p;
    std::size_t frames = 0;
    for (std::size_t i = 0; i < junk.size(); i += 997) {  // odd chunk size
        const std::size_t n = std::min<std::size_t>(997, junk.size() - i);
        frames += p.push(junk.data() + i, n).size();
    }
    EXPECT_EQ(frames, 0u);
    EXPECT_EQ(p.stats().frames_ok, 0u);
    EXPECT_GT(p.stats().bytes_skipped, 190000u);  // almost everything was hunted past
}

// 1000 real frames back to back, delivered in random chunk sizes 1..64.
TEST(ParserRobustness, ThousandFramesRandomChunks) {
    Bytes stream;
    std::vector<std::uint32_t> expected_msgids;
    for (int i = 0; i < 1000; ++i) {
        const Bytes& f = *kValid[static_cast<std::size_t>(i) % kValid.size()];
        stream.insert(stream.end(), f.begin(), f.end());
        expected_msgids.push_back(f[7]);  // all test msgids fit in the low byte
    }
    std::mt19937 rng(7);
    std::uniform_int_distribution<std::size_t> chunk(1, 64);
    MavlinkParser p;
    std::vector<std::uint32_t> got;
    for (std::size_t i = 0; i < stream.size();) {
        const std::size_t n = std::min(chunk(rng), stream.size() - i);
        for (const Frame& f : p.push(stream.data() + i, n)) got.push_back(f.msgid);
        i += n;
    }
    EXPECT_EQ(got, expected_msgids);
    EXPECT_EQ(p.stats().bytes_skipped, 0u);
}

// Random non-magic garbage between frames: every frame recovered, garbage counted exactly.
TEST(ParserRobustness, GarbageBetweenFramesIsCountedExactly) {
    std::mt19937 rng(99);
    std::uniform_int_distribution<int> len(0, 20);
    std::uniform_int_distribution<int> byte(0, 0xFC);  // never 0xFD
    Bytes stream;
    std::size_t garbage = 0;
    for (int i = 0; i < 200; ++i) {
        const int n = len(rng);
        for (int k = 0; k < n; ++k) stream.push_back(static_cast<std::uint8_t>(byte(rng)));
        garbage += static_cast<std::size_t>(n);
        const Bytes& f = *kValid[static_cast<std::size_t>(i) % kValid.size()];
        stream.insert(stream.end(), f.begin(), f.end());
    }
    MavlinkParser p;
    EXPECT_EQ(p.push(stream.data(), stream.size()).size(), 200u);
    EXPECT_EQ(p.stats().bytes_skipped, garbage);
}

// Largest payload a v2 frame can carry (len byte = 255), split across two pushes.
TEST(ParserRobustness, MaxPayloadFrame) {
    Bytes payload(255);
    for (std::size_t i = 0; i < payload.size(); ++i) payload[i] = static_cast<std::uint8_t>(i);
    const Bytes f = encode(131, 9, payload);  // ENCAPSULATED_DATA: max_payload_len 255
    ASSERT_EQ(f.size(), 10u + 255u + 2u);
    MavlinkParser p;
    EXPECT_TRUE(p.push(f.data(), 200).empty());  // header + part of the payload
    const auto frames = p.push(f.data() + 200, f.size() - 200);
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0].payload, payload);
    EXPECT_EQ(frames[0].seq, 9);
}

// A corrupted frame inside a long stream costs exactly that frame, nothing else.
TEST(ParserRobustness, OneCorruptFrameInStreamLosesOnlyThatFrame) {
    Bytes stream;
    for (int i = 0; i < 50; ++i) {
        Bytes f = vectors::kGlobalPositionInt;
        if (i == 25) f[20] ^= 0x01;  // single bit flip in the payload
        stream.insert(stream.end(), f.begin(), f.end());
    }
    MavlinkParser p;
    EXPECT_EQ(p.push(stream.data(), stream.size()).size(), 49u);
    EXPECT_EQ(p.stats().crc_errors, 1u);
}

// Ten full 0..255 wraps with no gaps: no loss. Then drop every 10th frame: exact loss count.
TEST(LinkStatsRobustness, LongWrapsAndPeriodicLoss) {
    asl::link::LinkStats clean;
    for (int i = 0; i < 2560; ++i) clean.on_frame(1, 1, static_cast<std::uint8_t>(i));
    EXPECT_EQ(clean.lost(), 0u);

    asl::link::LinkStats lossy;
    std::uint64_t dropped = 0;
    for (int i = 0; i < 2560; ++i) {
        if (i % 10 == 5) {
            ++dropped;  // never sent
            continue;
        }
        lossy.on_frame(1, 1, static_cast<std::uint8_t>(i));
    }
    EXPECT_EQ(lossy.lost(), dropped);
    EXPECT_DOUBLE_EQ(lossy.loss_ratio(), 0.1);
}
