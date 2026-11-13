#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <vector>

#include "sunhorizon_navigation_core/latest_observer.hpp"

TEST(LatestObserver, BlockedEgressCannotBlockAdmissionAndOnlyNewestPendingOwnerSurvives) {
  std::mutex mutex;
  std::condition_variable condition;
  bool first_started = false, release_first = false;
  std::vector<int> observed;
  sunhorizon_navigation_core::LatestObserver<int> observer([&](auto owner) {
    std::unique_lock<std::mutex> lock(mutex);
    observed.push_back(*owner);
    if (*owner == 1) {
      first_started = true;
      condition.notify_all();
      condition.wait(lock, [&]() {return release_first;});
    }
    condition.notify_all();
  }, [](std::exception_ptr) {ADD_FAILURE() << "unexpected observer failure";});
  ASSERT_TRUE(observer.enqueue(std::make_shared<const int>(1)));
  {
    std::unique_lock<std::mutex> lock(mutex);
    const bool started = condition.wait_for(lock, std::chrono::seconds(2), [&]() {return first_started;});
    if (!started) {release_first = true; condition.notify_all();}
    ASSERT_TRUE(started);
  }
  auto second = std::make_shared<const int>(2);
  std::weak_ptr<const int> retired = second;
  ASSERT_TRUE(observer.enqueue(std::move(second)));
  ASSERT_TRUE(observer.enqueue(std::make_shared<const int>(3)));
  EXPECT_TRUE(retired.expired());
  {
    std::unique_lock<std::mutex> lock(mutex);
    release_first = true;
    condition.notify_all();
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(2), [&]() {return observed.size() == 2;}));
    EXPECT_EQ(observed, (std::vector<int>{1, 3}));
  }
}

TEST(LatestObserver, FailureClosesAdmissionAndIsReportedOutsideObserverLock) {
  std::mutex mutex;
  std::condition_variable condition;
  bool failed = false;
  sunhorizon_navigation_core::LatestObserver<int> observer([](auto) {
    throw std::runtime_error("publisher failure");
  }, [&](std::exception_ptr error) {
    EXPECT_THROW(std::rethrow_exception(error), std::runtime_error);
    {
      std::lock_guard<std::mutex> lock(mutex);
      failed = true;
    }
    condition.notify_one();
  });
  ASSERT_TRUE(observer.enqueue(std::make_shared<const int>(1)));
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(2), [&]() {return failed;}));
  }
  EXPECT_FALSE(observer.enqueue(std::make_shared<const int>(2)));
}

TEST(LatestObserver, ClosedObserverNeverRetainsNewOwners) {
  sunhorizon_navigation_core::LatestObserver<int> observer([](auto) {}, [](auto) {});
  observer.close();
  auto owner = std::make_shared<const int>(5);
  std::weak_ptr<const int> weak = owner;
  EXPECT_FALSE(observer.enqueue(std::move(owner)));
  EXPECT_TRUE(weak.expired());
}
