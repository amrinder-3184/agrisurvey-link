// Phase 2. Spec: include/asl/survey_planner.h. Tests: tests/test_survey_planner.cpp.
#include "asl/survey_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace asl {

namespace {

constexpr double kPi = 3.14159265358979323846;

double half_angle_tan(double fov_deg) {
    return std::tan(fov_deg * kPi / 180.0 / 2.0);
}

}  // namespace

// Pinhole camera looking straight down: the image half-width on the ground is
// h * tan(fov / 2), so the full width is twice that.
double footprint_width_m(const CameraModel& cam, double altitude_m) {
    return 2.0 * altitude_m * half_angle_tan(cam.hfov_deg);
}

double footprint_length_m(const CameraModel& cam, double altitude_m) {
    return 2.0 * altitude_m * half_angle_tan(cam.vfov_deg);
}

// With sidelap s, neighbouring lines share a fraction s of the footprint width,
// so each new line only adds (1 - s) of new ground.
double line_spacing_m(const CameraModel& cam, const SurveySpec& spec) {
    return footprint_width_m(cam, spec.altitude_m) * (1.0 - spec.sidelap);
}

// Same idea along the line: consecutive images overlap by the frontlap fraction.
double trigger_distance_m(const CameraModel& cam, const SurveySpec& spec) {
    return footprint_length_m(cam, spec.altitude_m) * (1.0 - spec.frontlap);
}

std::vector<Vec2> plan_lawnmower(const std::vector<Vec2>& polygon, double spacing_m) {
    std::vector<Vec2> waypoints;
    if (polygon.size() < 3 || !(spacing_m > 0.0)) {  // !(x > 0) also rejects NaN
        return waypoints;
    }

    double min_north = polygon[0].north_m;
    double max_north = polygon[0].north_m;
    for (const Vec2& v : polygon) {
        min_north = std::min(min_north, v.north_m);
        max_north = std::max(max_north, v.north_m);
    }

    std::size_t lines_emitted = 0;
    for (int k = 0;; ++k) {
        // Computed from k (not y += spacing) so rounding errors do not accumulate.
        const double y = min_north + spacing_m / 2.0 + k * spacing_m;
        if (y >= max_north) {
            break;
        }

        // Intersect the horizontal line north = y with every edge (a -> b).
        double west = std::numeric_limits<double>::infinity();
        double east = -std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < polygon.size(); ++i) {
            const Vec2& a = polygon[i];
            const Vec2& b = polygon[(i + 1) % polygon.size()];  // wrap: last vertex -> first
            // True only if a and b are on different sides of y. A horizontal edge
            // (a.north == b.north) is never true, so the division below is safe, and a
            // vertex exactly on y is counted for one of its two edges, not both.
            if ((a.north_m <= y) != (b.north_m <= y)) {
                const double t = (y - a.north_m) / (b.north_m - a.north_m);
                const double x = a.east_m + t * (b.east_m - a.east_m);
                west = std::min(west, x);
                east = std::max(east, x);
            }
        }
        if (!(west < east)) {
            continue;  // no proper crossing (cannot happen for a valid convex polygon)
        }

        // Boustrophedon: alternate direction so each line starts where the last one ended.
        if (lines_emitted % 2 == 0) {
            waypoints.push_back({west, y});
            waypoints.push_back({east, y});
        } else {
            waypoints.push_back({east, y});
            waypoints.push_back({west, y});
        }
        ++lines_emitted;
    }
    return waypoints;
}

}  // namespace asl
