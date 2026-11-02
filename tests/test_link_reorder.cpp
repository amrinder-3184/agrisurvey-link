// Reordering tests for LinkStats (added in phase 4 after the netem test: `tc netem` on the
// loopback interface delivers some datagrams out of order).
#include <gtest/gtest.h>

#include <cstdint>

#include "asl/link/link_stats.h"

using asl::link::LinkStats;

// 10, 12, 11, 13: one frame late, none lost.
TEST(LinkStatsReorder, SwappedPairIsNotLoss) {
    LinkStats s;
    for (int seq : {10, 12, 11, 13}) s.on_frame(1, 1, static_cast<std::uint8_t>(seq));
    EXPECT_EQ(s.received(), 4u);
    EXPECT_EQ(s.lost(), 0u);
    EXPECT_EQ(s.reordered(), 1u);
}

// A late frame across the 255 -> 0 wrap.
TEST(LinkStatsReorder, SwapAcrossWrap) {
    LinkStats s;
    for (int seq : {254, 0, 255, 1}) s.on_frame(1, 1, static_cast<std::uint8_t>(seq));
    EXPECT_EQ(s.lost(), 0u);
    EXPECT_EQ(s.reordered(), 1u);
}

// Real loss and reordering together.
TEST(LinkStatsReorder, LossAndReorderTogether) {
    LinkStats s;
    for (int seq : {0, 2, 1, 4, 5, 9, 10, 8, 14}) s.on_frame(1, 1, static_cast<std::uint8_t>(seq));
    // Never arrived: 3, 6, 7, 11, 12, 13 -> 6 lost. Arrived late: 1 and 8 -> 2 reordered.
    EXPECT_EQ(s.reordered(), 2u);
    EXPECT_EQ(s.lost(), 6u);
    EXPECT_EQ(s.received(), 9u);
}

// 100 swapped pairs over several wraps: zero loss (the old estimator reported ~25 000).
TEST(LinkStatsReorder, ManySwapsNoPhantomLoss) {
    LinkStats s;
    for (int i = 0; i < 1000; i += 2) {
        if (i % 20 == 0) {
            s.on_frame(1, 1, static_cast<std::uint8_t>(i + 1));
            s.on_frame(1, 1, static_cast<std::uint8_t>(i));
        } else {
            s.on_frame(1, 1, static_cast<std::uint8_t>(i));
            s.on_frame(1, 1, static_cast<std::uint8_t>(i + 1));
        }
    }
    EXPECT_EQ(s.received(), 1000u);
    EXPECT_EQ(s.lost(), 0u);
    EXPECT_EQ(s.reordered(), 50u);
}
