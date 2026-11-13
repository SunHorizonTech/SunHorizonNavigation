#include <gtest/gtest.h>

#include "sunhorizon_livo_bringup/measurement_freshness.hpp"

using sunhorizon_livo_bringup::evaluate_measurement_freshness;

TEST(MeasurementFreshnessTest, AcceptsRecentMeasurement)
{
  const auto decision = evaluate_measurement_freshness(
    1'000'000'000LL, 1'200'000'000LL, 0.5);
  EXPECT_TRUE(decision.accepted);
  EXPECT_NEAR(decision.age_sec, 0.2, 1.0e-9);
  EXPECT_EQ(decision.reason, "ok");
}

TEST(MeasurementFreshnessTest, RejectsStaleMeasurement)
{
  const auto decision = evaluate_measurement_freshness(
    1'000'000'000LL, 1'500'000'001LL, 0.5);
  EXPECT_FALSE(decision.accepted);
  EXPECT_GT(decision.age_sec, 0.5);
  EXPECT_EQ(decision.reason, "measurement_stale");
}

TEST(MeasurementFreshnessTest, RejectsFutureAndInvalidMeasurement)
{
  EXPECT_EQ(
    evaluate_measurement_freshness(
      1'100'000'000LL, 1'000'000'000LL, 0.5).reason,
    "measurement_stamp_in_future");
  EXPECT_EQ(
    evaluate_measurement_freshness(0, 1'000'000'000LL, 0.5).reason,
    "measurement_stamp_invalid");
  EXPECT_EQ(
    evaluate_measurement_freshness(
      1'000'000'000LL, 1'100'000'000LL, 0.0).reason,
    "measurement_stamp_invalid");
}
