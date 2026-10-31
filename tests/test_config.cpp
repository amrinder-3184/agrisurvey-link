// Tests for include/asl/config.h (added in phase 2; not part of the original 42 spec tests).
#include <gtest/gtest.h>

#include <sstream>
#include <stdexcept>
#include <string>

#include "asl/config.h"

namespace {

asl::SurveyConfig ini(const std::string& text) {
    std::istringstream in(text);
    return asl::parse_camera_ini(in, "test.ini");
}

std::vector<asl::Vec2> csv(const std::string& text) {
    std::istringstream in(text);
    return asl::parse_field_csv(in, "test.csv");
}

// Expect a runtime_error whose message contains `needle`.
template <typename F>
void expect_error(F f, const std::string& needle) {
    try {
        f();
        FAIL() << "expected an exception containing '" << needle << "'";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find(needle), std::string::npos) << e.what();
    }
}

const char* kGoodIni =
    "# comment\n"
    "hfov_deg=60\n"
    "vfov_deg = 45\n"
    "altitude_m=30\n"
    "sidelap=0.7\n"
    "frontlap=0.75\n"
    "speed_mps=5\n";

}  // namespace

TEST(Config, IniParsesAllKeys) {
    const asl::SurveyConfig c = ini(kGoodIni);
    EXPECT_DOUBLE_EQ(c.camera.hfov_deg, 60.0);
    EXPECT_DOUBLE_EQ(c.camera.vfov_deg, 45.0);
    EXPECT_DOUBLE_EQ(c.survey.altitude_m, 30.0);
    EXPECT_DOUBLE_EQ(c.survey.sidelap, 0.7);
    EXPECT_DOUBLE_EQ(c.survey.frontlap, 0.75);
    EXPECT_DOUBLE_EQ(c.speed_mps, 5.0);
}

TEST(Config, IniRejectsBadInput) {
    expect_error([] { ini("hfov_deg=60\n"); }, "missing required key");
    expect_error([] { ini(std::string(kGoodIni) + "altitude_m=40\n"); }, "duplicate key");
    expect_error([] { ini(std::string(kGoodIni) + "hfov=60\n"); }, "unknown key 'hfov'");
    expect_error([] { ini("hfov_deg=60abc\n"); }, "not a number");
    expect_error([] { ini("sidelap=1.0\n"); }, "out of range");
    expect_error([] { ini("altitude_m=0\n"); }, "out of range");
    expect_error([] { ini("just text\n"); }, "test.ini:1:");
}

TEST(Config, CsvSkipsCommentsAndHeader) {
    const auto poly = csv("# field\neast_m,north_m\n30,30\n150, 30\n\n150,110\n30,110\n");
    ASSERT_EQ(poly.size(), 4u);
    EXPECT_DOUBLE_EQ(poly[1].east_m, 150.0);
    EXPECT_DOUBLE_EQ(poly[1].north_m, 30.0);
}

TEST(Config, CsvRejectsBadInput) {
    expect_error([] { csv("east_m,north_m\n0,0\n10,0\n"); }, "at least 3 vertices");
    expect_error([] { csv("east_m,north_m\n0,0\n10,x\n0,10\n"); }, "test.csv:3:");
    // L-shape (concave): plan_lawnmower would silently produce wrong lines.
    expect_error([] { csv("0,0\n20,0\n20,10\n10,10\n10,20\n0,20\n"); }, "not convex");
}

TEST(Config, ConvexityIgnoresCollinearPointsAndWinding) {
    EXPECT_TRUE(asl::is_convex({{0, 0}, {50, 0}, {100, 0}, {100, 60}, {0, 60}}));
    EXPECT_TRUE(asl::is_convex({{0, 0}, {0, 60}, {100, 60}, {100, 0}}));
}

TEST(Config, ShippedConfigMatchesPlanner) {
    // The real config files must load, and give the numbers the mission will use.
    const asl::SurveyConfig c = asl::load_camera_ini(ASL_SOURCE_DIR "/config/camera.ini");
    const auto field = asl::load_field_csv(ASL_SOURCE_DIR "/config/field_local.csv");
    EXPECT_EQ(field.size(), 4u);
    EXPECT_GT(asl::line_spacing_m(c.camera, c.survey), 0.0);
    EXPECT_FALSE(asl::plan_lawnmower(field, asl::line_spacing_m(c.camera, c.survey)).empty());
}
