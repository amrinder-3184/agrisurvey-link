// Spec for include/asl/trigger.h (phase 3).
#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "asl/trigger.h"

using asl::DistanceTrigger;
using asl::Vec2;

TEST(Trigger, RejectsNonPositiveDistance) {
    EXPECT_THROW(DistanceTrigger(0.0), std::invalid_argument);
    EXPECT_THROW(DistanceTrigger(-1.0), std::invalid_argument);
}

TEST(Trigger, FirstUpdateFires) {
    DistanceTrigger t(10.0);
    EXPECT_TRUE(t.update({5, 5}));
    EXPECT_EQ(t.count(), 1);
}

TEST(Trigger, FiresAtDistanceFromLastCapture) {
    DistanceTrigger t(10.0);
    // Positions arrive at telemetry rate, so they rarely land exactly on a multiple of 10.
    const std::vector<double> xs{0, 3, 6, 9, 10.5, 12, 19, 21};
    std::vector<bool> fired;
    for (double x : xs) fired.push_back(t.update({x, 0}));
    const std::vector<bool> expected{true, false, false, false, true, false, false, true};
    EXPECT_EQ(fired, expected);  // 21 - 10.5 = 10.5 >= 10 fires; 19 - 10.5 = 8.5 does not
    EXPECT_EQ(t.count(), 3);
}

TEST(Trigger, UsesStraightLineDistance) {
    DistanceTrigger t(5.0);
    EXPECT_TRUE(t.update({0, 0}));
    EXPECT_FALSE(t.update({3, 3.9}));  // 4.92 m
    EXPECT_TRUE(t.update({3, 4}));     // exactly 5 m: >= fires
}

TEST(Trigger, ResetStartsOver) {
    DistanceTrigger t(10.0);
    t.update({0, 0});
    t.update({10, 0});
    ASSERT_EQ(t.count(), 2);
    t.reset();
    EXPECT_EQ(t.count(), 0);
    EXPECT_TRUE(t.update({10.5, 0}));  // first update after reset always fires
}
