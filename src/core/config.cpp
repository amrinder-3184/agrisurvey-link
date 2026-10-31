// Spec: include/asl/config.h. Tests: tests/test_config.cpp.
#include "asl/config.h"

#include <charconv>
#include <cmath>
#include <fstream>
#include <map>
#include <stdexcept>
#include <system_error>

namespace asl {

namespace {

[[noreturn]] void fail(const std::string& source, int line, const std::string& what) {
    throw std::runtime_error(source + ":" + std::to_string(line) + ": " + what);
}

std::string trim(const std::string& s) {
    const auto first = s.find_first_not_of(" \t\r");
    if (first == std::string::npos) {
        return "";
    }
    const auto last = s.find_last_not_of(" \t\r");
    return s.substr(first, last - first + 1);
}

// Parse the WHOLE string as a finite double. std::from_chars is locale-independent and,
// unlike std::stod, lets us reject trailing junk such as "12abc".
bool parse_double(const std::string& text, double& out) {
    const std::string t = trim(text);
    if (t.empty()) {
        return false;
    }
    const char* begin = t.data();
    const char* end = t.data() + t.size();
    const auto [ptr, ec] = std::from_chars(begin, end, out);
    return ec == std::errc() && ptr == end && std::isfinite(out);
}

bool is_comment_or_blank(const std::string& trimmed) {
    return trimmed.empty() || trimmed[0] == '#';
}

std::ifstream open_or_throw(const std::string& path) {
    std::ifstream f(path);
    if (!f) {
        throw std::runtime_error(path + ": cannot open file");
    }
    return f;
}

}  // namespace

bool is_convex(const std::vector<Vec2>& polygon) {
    // Walk the edges; the z-component of the cross product of consecutive edges tells
    // whether we turn left (+) or right (-). Convex = never both.
    bool has_left = false;
    bool has_right = false;
    const std::size_t n = polygon.size();
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2& a = polygon[i];
        const Vec2& b = polygon[(i + 1) % n];
        const Vec2& c = polygon[(i + 2) % n];
        const double cross = (b.east_m - a.east_m) * (c.north_m - b.north_m) -
                             (b.north_m - a.north_m) * (c.east_m - b.east_m);
        if (cross > 1e-9) {
            has_left = true;
        } else if (cross < -1e-9) {
            has_right = true;
        }
    }
    return !(has_left && has_right);
}

std::vector<Vec2> parse_field_csv(std::istream& in, const std::string& source) {
    std::vector<Vec2> polygon;
    std::string raw;
    int line_no = 0;
    bool seen_data_line = false;
    while (std::getline(in, raw)) {
        ++line_no;
        const std::string line = trim(raw);
        if (is_comment_or_blank(line)) {
            continue;
        }
        const auto comma = line.find(',');
        double east = 0.0;
        double north = 0.0;
        const bool numeric = comma != std::string::npos &&
                             parse_double(line.substr(0, comma), east) &&
                             parse_double(line.substr(comma + 1), north);
        if (!numeric) {
            if (!seen_data_line && polygon.empty()) {
                seen_data_line = true;  // first non-comment line: the header
                continue;
            }
            fail(source, line_no, "expected 'east_m,north_m' (two numbers), got '" + line + "'");
        }
        seen_data_line = true;
        polygon.push_back({east, north});
    }
    if (polygon.size() < 3) {
        fail(source, line_no, "need at least 3 vertices, got " + std::to_string(polygon.size()));
    }
    if (!is_convex(polygon)) {
        fail(source, line_no, "field polygon is not convex (only convex fields are supported)");
    }
    return polygon;
}

std::vector<Vec2> load_field_csv(const std::string& path) {
    std::ifstream f = open_or_throw(path);
    return parse_field_csv(f, path);
}

SurveyConfig parse_camera_ini(std::istream& in, const std::string& source) {
    struct Rule {
        double min;
        double max;
        bool min_inclusive;
    };
    // Allowed range per key: (min, max], or [min, max) where noted.
    const std::map<std::string, Rule> rules = {
        {"hfov_deg", {0.0, 179.999, false}},  {"vfov_deg", {0.0, 179.999, false}},
        {"altitude_m", {0.0, 120.0, false}},  // 120 m: common legal limit for small drones
        {"sidelap", {0.0, 0.999, true}},      {"frontlap", {0.0, 0.999, true}},
        {"speed_mps", {0.0, 20.0, false}},
    };

    std::map<std::string, double> values;
    std::string raw;
    int line_no = 0;
    while (std::getline(in, raw)) {
        ++line_no;
        const std::string line = trim(raw);
        if (is_comment_or_blank(line)) {
            continue;
        }
        const auto eq = line.find('=');
        if (eq == std::string::npos) {
            fail(source, line_no, "expected 'key=value', got '" + line + "'");
        }
        const std::string key = trim(line.substr(0, eq));
        const auto rule = rules.find(key);
        if (rule == rules.end()) {
            fail(source, line_no, "unknown key '" + key + "'");
        }
        if (values.count(key) != 0) {
            fail(source, line_no, "duplicate key '" + key + "'");
        }
        double v = 0.0;
        if (!parse_double(line.substr(eq + 1), v)) {
            fail(source, line_no, "value of '" + key + "' is not a number");
        }
        const Rule& r = rule->second;
        const bool above_min = r.min_inclusive ? v >= r.min : v > r.min;
        if (!above_min || v > r.max) {
            fail(source, line_no,
                 "'" + key + "' = " + trim(line.substr(eq + 1)) + " is out of range " +
                     (r.min_inclusive ? "[" : "(") + std::to_string(r.min) + ", " +
                     std::to_string(r.max) + "]");
        }
        values[key] = v;
    }
    for (const auto& entry : rules) {
        if (values.count(entry.first) == 0) {
            fail(source, line_no, "missing required key '" + entry.first + "'");
        }
    }

    SurveyConfig cfg;
    cfg.camera = {values["hfov_deg"], values["vfov_deg"]};
    cfg.survey = {values["altitude_m"], values["sidelap"], values["frontlap"]};
    cfg.speed_mps = values["speed_mps"];
    return cfg;
}

SurveyConfig load_camera_ini(const std::string& path) {
    std::ifstream f = open_or_throw(path);
    return parse_camera_ini(f, path);
}

}  // namespace asl
