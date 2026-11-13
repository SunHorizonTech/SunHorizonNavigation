#include <gtest/gtest.h>

#include <limits>

#include "sunhorizon_sensor_rate_probe/rate_tracker.hpp"

using sunhorizon_sensor_rate_probe::RateTracker;

TEST(RateTracker, UsesIntervalsOverObservedSpan)
{
  RateTracker tracker;
  tracker.observe(10.0, 100.0);
  tracker.observe(10.5, 100.4);
  tracker.observe(11.0, 100.8);

  const auto snapshot = tracker.snapshot();
  EXPECT_EQ(snapshot.samples, 3U);
  EXPECT_EQ(snapshot.receive_interval_count, 2U);
  EXPECT_DOUBLE_EQ(snapshot.receive_span_sec, 1.0);
  EXPECT_DOUBLE_EQ(snapshot.receive_rate_hz, 2.0);
  EXPECT_EQ(snapshot.header_interval_count, 2U);
  EXPECT_NEAR(snapshot.header_span_sec, 0.8, 1.0e-12);
  EXPECT_NEAR(snapshot.header_rate_hz, 2.5, 1.0e-12);
}

TEST(RateTracker, ReportsHeaderDuplicatesAndRegressions)
{
  RateTracker tracker;
  tracker.observe(1.0, 5.0);
  tracker.observe(2.0, 5.0);
  tracker.observe(3.0, 4.0);
  tracker.observe(4.0, std::numeric_limits<double>::quiet_NaN());

  const auto snapshot = tracker.snapshot();
  EXPECT_EQ(snapshot.samples, 4U);
  EXPECT_EQ(snapshot.header_samples, 3U);
  EXPECT_EQ(snapshot.header_duplicates, 1U);
  EXPECT_EQ(snapshot.header_regressions, 1U);
}
