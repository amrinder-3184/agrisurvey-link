// Tests for include/asl/capture_queue.h (added in phase 3).
#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <thread>
#include <vector>

#include "asl/capture_queue.h"

using asl::Capture;
using asl::CaptureQueue;
using namespace std::chrono_literals;

namespace {
Capture make(std::uint64_t id) {
    Capture c;
    c.id = id;
    return c;
}
}  // namespace

TEST(CaptureQueue, RejectsZeroCapacity) {
    EXPECT_THROW(CaptureQueue(0), std::invalid_argument);
}

TEST(CaptureQueue, FifoOrder) {
    CaptureQueue q(4);
    for (std::uint64_t i = 1; i <= 3; ++i) EXPECT_TRUE(q.push(make(i)));
    EXPECT_EQ(q.size(), 3u);
    Capture c;
    for (std::uint64_t i = 1; i <= 3; ++i) {
        ASSERT_TRUE(q.pop_wait(c, 0ms));
        EXPECT_EQ(c.id, i);
    }
    EXPECT_FALSE(q.pop_wait(c, 0ms));  // empty -> timeout
}

TEST(CaptureQueue, FullQueueDropsOldestAndCounts) {
    CaptureQueue q(3);
    for (std::uint64_t i = 1; i <= 3; ++i) EXPECT_TRUE(q.push(make(i)));
    EXPECT_FALSE(q.push(make(4)));  // drops 1
    EXPECT_FALSE(q.push(make(5)));  // drops 2
    EXPECT_EQ(q.size(), 3u);
    EXPECT_EQ(q.dropped(), 2u);
    Capture c;
    for (std::uint64_t expected : {3u, 4u, 5u}) {  // newest three survive, still in order
        ASSERT_TRUE(q.pop_wait(c, 0ms));
        EXPECT_EQ(c.id, expected);
    }
}

TEST(CaptureQueue, PopTimesOut) {
    CaptureQueue q(2);
    Capture c;
    const auto t0 = std::chrono::steady_clock::now();
    EXPECT_FALSE(q.pop_wait(c, 50ms));
    EXPECT_GE(std::chrono::steady_clock::now() - t0, 45ms);
}

TEST(CaptureQueue, CloseWakesWaitingConsumerAndDrains) {
    CaptureQueue q(4);
    q.push(make(7));
    bool got_item = false;
    bool second_pop = true;
    std::thread consumer([&] {
        Capture c;
        got_item = q.pop_wait(c, 10s) && c.id == 7;  // item already queued
        second_pop = q.pop_wait(c, 10s);             // blocks until close()
    });
    std::this_thread::sleep_for(50ms);
    const auto t0 = std::chrono::steady_clock::now();
    q.close();
    consumer.join();
    EXPECT_TRUE(got_item);
    EXPECT_FALSE(second_pop);
    EXPECT_LT(std::chrono::steady_clock::now() - t0, 1s);  // woken, not timed out
    EXPECT_FALSE(q.push(make(8)));                         // closed: ignored
}

// Producer and consumer on two threads, capacity large enough: nothing may be lost or
// reordered. Run under ASan/TSan builds as well.
TEST(CaptureQueue, TwoThreadsNoLossWhenCapacitySufficient) {
    constexpr std::uint64_t kItems = 100000;
    CaptureQueue q(kItems);
    std::vector<std::uint64_t> received;
    received.reserve(kItems);
    std::thread consumer([&] {
        Capture c;
        while (q.pop_wait(c, 1s)) received.push_back(c.id);
    });
    for (std::uint64_t i = 0; i < kItems; ++i) q.push(make(i));
    q.close();
    consumer.join();
    EXPECT_EQ(q.dropped(), 0u);
    ASSERT_EQ(received.size(), kItems);
    for (std::uint64_t i = 0; i < kItems; ++i) ASSERT_EQ(received[i], i);
}

// Small capacity under contention: drops may happen, but every item is either received or
// counted as dropped, and the received ones stay in order.
TEST(CaptureQueue, TwoThreadsSmallCapacityAccountsForEveryItem) {
    constexpr std::uint64_t kItems = 100000;
    CaptureQueue q(64);
    std::vector<std::uint64_t> received;
    std::thread consumer([&] {
        Capture c;
        while (q.pop_wait(c, 1s)) received.push_back(c.id);
    });
    for (std::uint64_t i = 0; i < kItems; ++i) q.push(make(i));
    q.close();
    consumer.join();
    EXPECT_EQ(received.size() + q.dropped(), kItems);
    for (std::size_t i = 1; i < received.size(); ++i) ASSERT_LT(received[i - 1], received[i]);
}
