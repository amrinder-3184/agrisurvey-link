// Spec for include/asl/link/link_stats.h (phase 4).
#include <gtest/gtest.h>

#include "asl/link/link_stats.h"

using asl::link::LinkStats;

TEST(LinkStats, EmptyHasZeroLoss) {
    LinkStats s;
    EXPECT_EQ(s.received(), 0u);
    EXPECT_EQ(s.lost(), 0u);
    EXPECT_DOUBLE_EQ(s.loss_ratio(), 0.0);
}

TEST(LinkStats, ContiguousSequenceHasNoLoss) {
    LinkStats s;
    for (int seq = 10; seq < 20; ++seq) s.on_frame(1, 1, static_cast<std::uint8_t>(seq));
    EXPECT_EQ(s.received(), 10u);
    EXPECT_EQ(s.lost(), 0u);
}

TEST(LinkStats, GapCountsMissingFrames) {
    LinkStats s;
    for (int seq : {0, 1, 2, 5}) s.on_frame(1, 1, static_cast<std::uint8_t>(seq));  // 3 and 4 missing
    EXPECT_EQ(s.received(), 4u);
    EXPECT_EQ(s.lost(), 2u);
    EXPECT_DOUBLE_EQ(s.loss_ratio(), 2.0 / 6.0);
}

TEST(LinkStats, WrapAroundIsNotLoss) {
    LinkStats s;
    for (int seq : {253, 254, 255, 0, 1}) s.on_frame(1, 1, static_cast<std::uint8_t>(seq));
    EXPECT_EQ(s.lost(), 0u);
}

TEST(LinkStats, GapAcrossWrapAround) {
    LinkStats s;
    s.on_frame(1, 1, 254);
    s.on_frame(1, 1, 2);  // 255, 0, 1 missing
    EXPECT_EQ(s.lost(), 3u);
}

TEST(LinkStats, SourcesAreTrackedSeparately) {
    LinkStats s;
    // Autopilot (1,1) and a camera component (1,100) interleaved, each contiguous on its own.
    s.on_frame(1, 1, 40);
    s.on_frame(1, 100, 7);
    s.on_frame(1, 1, 41);
    s.on_frame(1, 100, 8);
    s.on_frame(2, 1, 0);  // a second vehicle
    EXPECT_EQ(s.received(), 5u);
    EXPECT_EQ(s.lost(), 0u);
}

TEST(LinkStats, DuplicateIsNotLoss) {
    LinkStats s;
    s.on_frame(1, 1, 9);
    s.on_frame(1, 1, 9);
    EXPECT_EQ(s.received(), 2u);
    EXPECT_EQ(s.lost(), 0u);
}
