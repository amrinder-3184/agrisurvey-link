// Geometry helpers: converting GPS coordinates to a flat local frame and back.
//
// Phase 2. YOU implement these in src/core/geo.cpp. Tests: tests/test_geo.cpp.
//
// Why a local frame? Survey planning is easy in metres on a flat plane, painful in
// degrees on a sphere. For a field a few kilometres across, treating the Earth as
// flat around a reference point (the "origin", e.g. the home position) is accurate
// to well under a metre.
#pragma once

namespace asl {

/// A WGS-84 position in degrees.
struct LatLon {
    double lat_deg{0.0};
    double lon_deg{0.0};
};

/// A point in the local East-North-Up plane, in metres from the origin.
/// (ENU: x = east, y = north. Note PX4 itself uses NED internally: north, east, down.)
struct Vec2 {
    double east_m{0.0};
    double north_m{0.0};
};

/// Mean Earth radius used for the flat-Earth approximation.
constexpr double kEarthRadiusM = 6371000.0;

/// Equirectangular (flat-Earth) projection around a fixed origin.
///
///   north_m = (lat - lat0) [rad] * R
///   east_m  = (lon - lon0) [rad] * R * cos(lat0)
///
/// to_global() must be the exact inverse of to_local().
class LocalFrame {
public:
    explicit LocalFrame(LatLon origin);

    Vec2 to_local(LatLon point) const;
    LatLon to_global(Vec2 point) const;

    LatLon origin() const { return origin_; }

private:
    LatLon origin_;
    double cos_lat0_{1.0};   // cache cos(origin latitude); it never changes
};

/// Straight-line distance between two local points, in metres.
double distance_m(Vec2 a, Vec2 b);

}  // namespace asl
