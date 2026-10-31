// Survey planning: camera footprint maths and the lawnmower (boustrophedon) pattern.
//
// Phase 2. YOU implement these in src/core/survey_planner.cpp.
// Tests: tests/test_survey_planner.cpp.
#pragma once

#include <vector>

#include "asl/geo.h"

namespace asl {

/// A simple pinhole camera: field of view across track (horizontal) and along track (vertical).
struct CameraModel {
    double hfov_deg{0.0};
    double vfov_deg{0.0};
};

/// How the survey should be flown.
struct SurveySpec {
    double altitude_m{0.0};  ///< height above ground
    double sidelap{0.0};     ///< overlap between neighbouring lines, in [0, 1)
    double frontlap{0.0};    ///< overlap between consecutive images on one line, in [0, 1)
};

/// Ground width covered by one image across the flight direction: 2 * h * tan(hfov / 2).
double footprint_width_m(const CameraModel& cam, double altitude_m);

/// Ground length covered by one image along the flight direction: 2 * h * tan(vfov / 2).
double footprint_length_m(const CameraModel& cam, double altitude_m);

/// Distance between adjacent survey lines: footprint width * (1 - sidelap).
double line_spacing_m(const CameraModel& cam, const SurveySpec& spec);

/// Distance flown between two captures: footprint length * (1 - frontlap).
double trigger_distance_m(const CameraModel& cam, const SurveySpec& spec);

/// Plan a lawnmower pattern over a CONVEX polygon given in the local frame.
///
/// Specification (the tests check exactly this):
///  - Lines run east-west (constant north_m).
///  - Line k sits at  north = min_north + spacing/2 + k * spacing,
///    for every k >= 0 where that value is strictly less than max_north.
///  - Each line contributes two waypoints: where it enters and where it leaves the polygon.
///  - Line 0 is flown west -> east, line 1 east -> west, line 2 west -> east, and so on.
///  - Vertices may be given clockwise or counter-clockwise; the result must be the same.
///  - Return an empty vector if the polygon has fewer than 3 vertices or spacing_m <= 0.
///
/// Concave (e.g. L-shaped) fields are a stretch goal; not required for v1.
std::vector<Vec2> plan_lawnmower(const std::vector<Vec2>& polygon, double spacing_m);

}  // namespace asl
