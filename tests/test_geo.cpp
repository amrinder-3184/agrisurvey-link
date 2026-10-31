// Spec for include/asl/geo.h (phase 2). Make these pass; then add at least one test of your own.
#include <gtest/gtest.h>

#include <cmath>

#include "asl/geo.h"

using asl::LatLon;
using asl::LocalFrame;
using asl::Vec2;

namespace {
// PX4 SITL's default home position (Zurich).
const LatLon kHome{47.397742, 8.545594};
constexpr double kPi = 3.14159265358979323846;
}  // namespace

TEST(Geo, OriginMapsToZero) {
    LocalFrame f(kHome);
    Vec2 v = f.to_local(kHome);
    EXPECT_NEAR(v.east_m, 0.0, 1e-9);
    EXPECT_NEAR(v.north_m, 0.0, 1e-9);
}

TEST(Geo, NorthOffset) {
    LocalFrame f(kHome);
    // 0.001 deg of latitude = R * 0.001 * pi / 180 metres north, no east component.
    Vec2 v = f.to_local({kHome.lat_deg + 0.001, kHome.lon_deg});
    EXPECT_NEAR(v.north_m, asl::kEarthRadiusM * 0.001 * kPi / 180.0, 1e-6);
    EXPECT_NEAR(v.east_m, 0.0, 1e-9);
}

TEST(Geo, EastOffsetShrinksWithLatitude) {
    LocalFrame f(kHome);
    Vec2 v = f.to_local({kHome.lat_deg, kHome.lon_deg + 0.001});
    const double expected = asl::kEarthRadiusM * 0.001 * kPi / 180.0 * std::cos(kHome.lat_deg * kPi / 180.0);
    EXPECT_NEAR(v.east_m, expected, 1e-6);   // about 75.3 m at 47.4 deg N
    EXPECT_NEAR(v.north_m, 0.0, 1e-9);
}

TEST(Geo, RoundTrip) {
    LocalFrame f(kHome);
    const Vec2 points[] = {{0, 0}, {120.5, -40.25}, {-300, 800}, {1500, 1500}};
    for (const Vec2& p : points) {
        Vec2 back = f.to_local(f.to_global(p));
        EXPECT_NEAR(back.east_m, p.east_m, 1e-6);
        EXPECT_NEAR(back.north_m, p.north_m, 1e-6);
    }
}

TEST(Geo, Distance) {
    EXPECT_DOUBLE_EQ(asl::distance_m({0, 0}, {3, 4}), 5.0);
    EXPECT_DOUBLE_EQ(asl::distance_m({1, 1}, {1, 1}), 0.0);
}
