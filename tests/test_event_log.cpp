// Tests for include/asl/event_log.h (added in phase 5).
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "asl/event_log.h"

namespace fs = std::filesystem;

namespace {

std::string read_file(const fs::path& p) {
    std::ifstream f(p);
    std::stringstream s;
    s << f.rdbuf();
    return s.str();
}

fs::path temp_path(const std::string& name) {
    return fs::temp_directory_path() / ("asl_test_" + name);
}

}  // namespace

TEST(Watch, FirstValueCountsAsChange) {
    asl::Watch<int> w;
    EXPECT_TRUE(w.changed(0));  // even the default value: we have never seen anything
}

TEST(Watch, ReportsOnlyTransitions) {
    asl::Watch<std::string> w;
    int changes = 0;
    for (const char* mode : {"Hold", "Hold", "Mission", "Mission", "Mission", "Return", "Hold"}) {
        changes += w.changed(mode) ? 1 : 0;
    }
    EXPECT_EQ(changes, 4);  // Hold, Mission, Return, Hold
}

// The SITL GPS-loss run: health alternated every poll. confirm=2 reports only real changes.
TEST(Watch, ConfirmFiltersFlapping) {
    asl::Watch<bool> w(2);
    EXPECT_FALSE(w.changed(true));  // first sighting: not yet confirmed
    EXPECT_TRUE(w.changed(true));   // second in a row: accepted
    int changes = 0;
    for (int i = 1; i <= 20; ++i) changes += w.changed(i % 2 == 0) ? 1 : 0;  // F,T,...,F,T
    EXPECT_EQ(changes, 0);
    EXPECT_FALSE(w.changed(false));
    EXPECT_TRUE(w.changed(false));  // a real, lasting change is still reported
}

TEST(Watch, ConfirmOneReportsEveryTransition) {
    asl::Watch<bool> w(1);
    EXPECT_TRUE(w.changed(true));
    EXPECT_TRUE(w.changed(false));
    EXPECT_TRUE(w.changed(true));
}

TEST(EventLog, WritesHeaderAndRows) {
    const fs::path p = temp_path("events.csv");
    {
        asl::EventLog log(p.string());
        log.log(1000, 0.0, "mode", "Hold");
        log.log(2000, 41.5, "gps_fix", "NoGps");
        EXPECT_EQ(log.count(), 2u);
        // Flushed per row: readable while the log is still open (crash safety).
        EXPECT_NE(read_file(p).find("gps_fix,NoGps"), std::string::npos);
    }
    EXPECT_EQ(read_file(p),
              "t_mono_ns,t_s,event,detail\n"
              "1000,0.0,mode,Hold\n"
              "2000,41.5,gps_fix,NoGps\n");
    fs::remove(p);
}

TEST(EventLog, UnwritablePathThrows) {
    EXPECT_THROW(asl::EventLog("/nonexistent_dir_asl/events.csv"), std::runtime_error);
}

TEST(CsvField, QuotesOnlyWhenNeeded) {
    EXPECT_EQ(asl::csv_field("Mission"), "Mission");
    EXPECT_EQ(asl::csv_field("a,b"), "\"a,b\"");
    EXPECT_EQ(asl::csv_field("say \"hi\""), "\"say \"\"hi\"\"\"");
}
