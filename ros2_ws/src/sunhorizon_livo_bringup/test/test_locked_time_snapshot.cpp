#include <atomic>
#include <chrono>
#include <future>
#include <mutex>

#include <gtest/gtest.h>

#include "sunhorizon_livo_bringup/locked_time_snapshot.hpp"
#include "sunhorizon_livo_bringup/measurement_freshness.hpp"

using sunhorizon_livo_bringup::evaluate_measurement_freshness;
using sunhorizon_livo_bringup::with_locked_time;

namespace
{
struct ObservedMutex
{
  std::mutex state;
  std::promise<void> attempting;
  void lock() {attempting.set_value(); state.lock();}
  void unlock() {state.unlock();}
};
}  // namespace

TEST(LockedTimeSnapshot, InputUpdatedWhileStatusWaitsIsNotFalselyInFuture)
{
  ObservedMutex mutex;
  std::atomic<std::int64_t> clock{1'000'000'000LL};
  std::int64_t received = clock.load();
  std::promise<void> sampled;
  auto sampled_future = sampled.get_future();
  auto attempting = mutex.attempting.get_future();
  std::unique_lock<std::mutex> writer(mutex.state);
  auto status = std::async(std::launch::async, [&]() {
      return with_locked_time(mutex, [&]() {
          sampled.set_value();
          return clock.load();
        }, [&](auto now) {
          return evaluate_measurement_freshness(received, now, 0.5);
        });
    });
  // The reader has reached lock(), so this tests a forced interleaving rather
  // than relying on sleeps or the scheduler happening to reproduce the race.
  attempting.wait();
  EXPECT_EQ(sampled_future.wait_for(std::chrono::seconds(0)), std::future_status::timeout);
  received = 2'000'000'000LL;
  clock.store(2'001'000'000LL);
  writer.unlock();
  const auto decision = status.get();
  EXPECT_TRUE(decision.accepted);
  EXPECT_NEAR(decision.age_sec, 0.001, 1e-9);
}

TEST(LockedTimeSnapshot, StillRejectsActuallyStaleOrFutureInput)
{
  std::mutex mutex;
  const auto check = [&](std::int64_t received) {
      return with_locked_time(mutex, []() {return 2'000'000'000LL;}, [&](auto now) {
          return evaluate_measurement_freshness(received, now, 0.5);
        });
    };
  EXPECT_EQ(check(1'000'000'000LL).reason, "measurement_stale");
  EXPECT_EQ(check(2'000'000'001LL).reason, "measurement_stamp_in_future");
}
