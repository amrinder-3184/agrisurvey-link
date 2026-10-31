// Spec for include/asl/survey_planner.h (phase 2).
#include <gtest/gtest.h>

#include <vector>

#include "asl/survey_planner.h"

using asl::CameraModel;
using asl::SurveySpec;
using asl::Vec2;

namespace {
void expect_point(const Vec2& got, double east, double north) {
    EXPECT_NEAR(got.east_m, east, 1e-6);
    EXPECT_NEAR(got.north_m, north, 1e-6);
}
}  // namespace

// ---- footprint maths --------------------------------------------------------

TEST(Footprint, WidthAt90DegreesEqualsTwiceAltitude) {
    CameraModel cam{90.0, 60.0};
    EXPECT_NEAR(asl::footprint_width_m(cam, 50.0), 100.0, 1e-9);  // 2 * 50 * tan(45)
}

TEST(Footprint, LengthUsesVerticalFov) {
    CameraModel cam{90.0, 60.0};
    EXPECT_NEAR(asl::footprint_length_m(cam, 50.0), 57.735026919, 1e-6);  // 2 * 50 * tan(30)
}

TEST(Footprint, LineSpacingAndTriggerDistance) {
    CameraModel cam{90.0, 60.0};
    SurveySpec spec{50.0, 0.7, 0.8};
    EXPECT_NEAR(asl::line_spacing_m(cam, spec), 30.0, 1e-9);              // 100 * (1 - 0.7)
    EXPECT_NEAR(asl::trigger_distance_m(cam, spec), 11.547005384, 1e-6);  // 57.735 * (1 - 0.8)
}

// ---- lawnmower pattern ------------------------------------------------------

TEST(Lawnmower, Rectangle) {
    // 100 m east x 60 m north, spacing 20 m -> lines at north = 10, 30, 50
    std::vector<Vec2> field{{0, 0}, {100, 0}, {100, 60}, {0, 60}};
    auto wps = asl::plan_lawnmower(field, 20.0);
    ASSERT_EQ(wps.size(), 6u);
    expect_point(wps[0], 0, 10);    // line 0: west -> east
    expect_point(wps[1], 100, 10);
    expect_point(wps[2], 100, 30);  // line 1: east -> west
    expect_point(wps[3], 0, 30);
    expect_point(wps[4], 0, 50);    // line 2: west -> east
    expect_point(wps[5], 100, 50);
}

TEST(Lawnmower, Triangle) {
    // Right triangle; the hypotenuse runs from (100, 0) to (0, 100): east = 100 - north.
    std::vector<Vec2> field{{0, 0}, {100, 0}, {0, 100}};
    auto wps = asl::plan_lawnmower(field, 20.0);
    ASSERT_EQ(wps.size(), 10u);  // lines at north = 10, 30, 50, 70, 90
    expect_point(wps[0], 0, 10);
    expect_point(wps[1], 90, 10);
    expect_point(wps[2], 70, 30);  // line 1 starts at the east end
    expect_point(wps[3], 0, 30);
    expect_point(wps[8], 0, 90);
    expect_point(wps[9], 10, 90);
}

TEST(Lawnmower, WindingOrderDoesNotMatter) {
    std::vector<Vec2> ccw{{0, 0}, {100, 0}, {100, 60}, {0, 60}};
    std::vector<Vec2> cw{{0, 0}, {0, 60}, {100, 60}, {100, 0}};
    auto a = asl::plan_lawnmower(ccw, 20.0);
    auto b = asl::plan_lawnmower(cw, 20.0);
    ASSERT_EQ(a.size(), b.size());
    for (std::size_t i = 0; i < a.size(); ++i) expect_point(b[i], a[i].east_m, a[i].north_m);
}

TEST(Lawnmower, OffsetField) {
    // Same rectangle shifted: checks you use min_north / min_east, not 0.
    std::vector<Vec2> field{{-50, 200}, {50, 200}, {50, 260}, {-50, 260}};
    auto wps = asl::plan_lawnmower(field, 20.0);
    ASSERT_EQ(wps.size(), 6u);
    expect_point(wps[0], -50, 210);
    expect_point(wps[5], 50, 250);
}

TEST(Lawnmower, DegenerateInputsReturnEmpty) {
    EXPECT_TRUE(asl::plan_lawnmower({}, 20.0).empty());
    EXPECT_TRUE(asl::plan_lawnmower({{0, 0}, {10, 0}}, 20.0).empty());
    std::vector<Vec2> field{{0, 0}, {100, 0}, {100, 60}, {0, 60}};
    EXPECT_TRUE(asl::plan_lawnmower(field, 0.0).empty());
    EXPECT_TRUE(asl::plan_lawnmower(field, -5.0).empty());
}

TEST(Lawnmower, SpacingWiderThanField) {
    // First line would be at north = 100, not < 60 -> no lines.
    std::vector<Vec2> field{{0, 0}, {100, 0}, {100, 60}, {0, 60}};
    EXPECT_TRUE(asl::plan_lawnmower(field, 200.0).empty());
}
