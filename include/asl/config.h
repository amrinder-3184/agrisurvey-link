// Loading the survey configuration: field polygon (CSV) and camera/survey settings (INI).
//
// Every function throws std::runtime_error("<source>:<line>: <what is wrong>") on bad
// input. Config is read once at startup, and nothing useful can happen with a bad config,
// so failing loudly with the exact line is the right behaviour.
//
// The parse_* functions take a stream (easy to unit-test with strings); the load_*
// functions open a file and call them.
#pragma once

#include <istream>
#include <string>
#include <vector>

#include "asl/geo.h"
#include "asl/survey_planner.h"

namespace asl {

/// Everything in config/camera.ini.
struct SurveyConfig {
    CameraModel camera;
    SurveySpec survey;
    double speed_mps{0.0};
};

/// Field polygon CSV: "east_m,north_m" per line, metres relative to home.
/// Blank lines and lines starting with '#' are ignored; a first line that is not numeric
/// is treated as the header. Requires >= 3 vertices forming a CONVEX polygon
/// (plan_lawnmower() only supports convex fields).
std::vector<Vec2> parse_field_csv(std::istream& in, const std::string& source);
std::vector<Vec2> load_field_csv(const std::string& path);

/// Camera INI: "key=value" lines; '#' starts a comment line. Required keys, all exactly
/// once: hfov_deg, vfov_deg (0, 180), altitude_m (0, 120], sidelap, frontlap [0, 1),
/// speed_mps (0, 20]. Unknown keys are errors (catches typos).
SurveyConfig parse_camera_ini(std::istream& in, const std::string& source);
SurveyConfig load_camera_ini(const std::string& path);

/// True if the polygon is convex (no turn changes direction; collinear points allowed).
bool is_convex(const std::vector<Vec2>& polygon);

}  // namespace asl
