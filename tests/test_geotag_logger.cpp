// Tests for include/asl/geotag_logger.h (added in phase 3).
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "asl/geotag_logger.h"

namespace {
std::vector<std::string> read_lines(const std::string& path) {
    std::ifstream f(path);
    std::vector<std::string> lines;
    for (std::string l; std::getline(f, l);) lines.push_back(l);
    return lines;
}
}  // namespace

TEST(GeotagLogger, WritesHeaderAndEveryQueuedCaptureOnStop) {
    const std::string path =
        (std::filesystem::temp_directory_path() / "asl_test_geotags.csv").string();
    asl::CaptureQueue q(16);
    {
        asl::GeotagLogger logger(q, path);
        for (std::uint64_t i = 1; i <= 5; ++i) {
            asl::Capture c;
            c.id = i;
            c.line_index = 2;
            q.push(c);
        }
        logger.stop();  // must drain all 5 before returning
        EXPECT_EQ(logger.written(), 5u);
    }
    const auto lines = read_lines(path);
    ASSERT_EQ(lines.size(), 6u);
    EXPECT_EQ(lines[0], asl::GeotagLogger::header());
    EXPECT_EQ(lines[1].substr(0, 2), "1,");
    EXPECT_EQ(lines[5].substr(0, 2), "5,");
    std::filesystem::remove(path);
}

TEST(GeotagLogger, DestructorStopsCleanlyWithEmptyQueue) {
    const std::string path =
        (std::filesystem::temp_directory_path() / "asl_test_geotags_empty.csv").string();
    asl::CaptureQueue q(4);
    { asl::GeotagLogger logger(q, path); }  // must not hang
    EXPECT_EQ(read_lines(path).size(), 1u);
    std::filesystem::remove(path);
}

TEST(GeotagLogger, ThrowsIfFileCannotBeOpened) {
    asl::CaptureQueue q(4);
    EXPECT_THROW(asl::GeotagLogger(q, "/nonexistent_dir/x.csv"), std::runtime_error);
}
