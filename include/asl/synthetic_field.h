// SYNTHETIC NDVI field for the simulated payload. NOT real crop data.
//
// There is no multispectral camera in simulation, so each capture is tagged with a
// deterministic, made-up NDVI value computed from its position. This lets the post-flight
// report demonstrate a stress map end to end. Every output that uses it is labelled
// SYNTHETIC.
//
// Model: healthy crop around 0.75 with mild spatial variation, plus one circular
// "stressed" patch (NDVI about 0.35) centred at (90 m E, 70 m N) of home with a 15 m radius,
// i.e. inside the default field in config/field_local.csv.
#pragma once

#include <cmath>

namespace asl {

constexpr double kStressCentreEastM = 90.0;
constexpr double kStressCentreNorthM = 70.0;
constexpr double kStressRadiusM = 15.0;
constexpr double kStressNdvi = 0.35;

/// SYNTHETIC NDVI in [0, 1] at a local position (metres east/north of home).
inline double ndvi_synthetic(double east_m, double north_m) {
    // Smooth, deterministic variation of +/- 0.05 so the map is not flat.
    const double healthy = 0.75 + 0.05 * std::sin(east_m / 25.0) * std::cos(north_m / 30.0);
    const double d = std::hypot(east_m - kStressCentreEastM, north_m - kStressCentreNorthM);
    if (d >= kStressRadiusM) {
        return healthy;
    }
    // Inside the patch: blend towards the stressed value, fully stressed in the centre half.
    const double w = d <= kStressRadiusM / 2.0 ? 1.0 : 2.0 * (1.0 - d / kStressRadiusM);
    return w * kStressNdvi + (1.0 - w) * healthy;
}

}  // namespace asl
