// Phase 2. Spec: include/asl/geo.h. Tests: tests/test_geo.cpp.
#include "asl/geo.h"

#include <cmath>

namespace asl {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
}  // namespace

LocalFrame::LocalFrame(LatLon origin)
    : origin_(origin), cos_lat0_(std::cos(origin.lat_deg * kDegToRad)) {}

// Equirectangular projection: a degree of latitude is always the same length; a degree of
// longitude shrinks with cos(latitude). Using cos(lat0) for the whole field is the
// flat-Earth approximation (error grows with distance from the origin).
Vec2 LocalFrame::to_local(LatLon point) const {
    const double north = (point.lat_deg - origin_.lat_deg) * kDegToRad * kEarthRadiusM;
    const double east = (point.lon_deg - origin_.lon_deg) * kDegToRad * kEarthRadiusM * cos_lat0_;
    return {east, north};
}

// The same two formulas solved for lat and lon, so to_local(to_global(p)) == p.
LatLon LocalFrame::to_global(Vec2 point) const {
    const double lat = origin_.lat_deg + (point.north_m / kEarthRadiusM) * kRadToDeg;
    const double lon = origin_.lon_deg + (point.east_m / (kEarthRadiusM * cos_lat0_)) * kRadToDeg;
    return {lat, lon};
}

double distance_m(Vec2 a, Vec2 b) {
    // hypot avoids overflow/underflow in the intermediate squares.
    return std::hypot(b.east_m - a.east_m, b.north_m - a.north_m);
}

}  // namespace asl
