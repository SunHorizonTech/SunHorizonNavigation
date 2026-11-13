#include <gtest/gtest.h>

#include "sunhorizon_livo_bringup/measurement_continuity.hpp"

namespace
{

using sunhorizon_livo_bringup::MeasurementContinuityGate;

TEST(MeasurementContinuityGateTest, AcceptsInitialAndBoundedMonotonicMeasurements)
{
  MeasurementContinuityGate gate(0.25);
  EXPECT_TRUE(gate.evaluate(10'000'000'000LL).accepted);
  const auto next = gate.evaluate(10'100'000'000LL);
  EXPECT_TRUE(next.accepted);
  EXPECT_NEAR(next.gap_sec, 0.1, 1e-12);
  EXPECT_FALSE(gate.latched());
}

TEST(MeasurementContinuityGateTest, LatchesGapWithoutAdvancingAcceptedStamp)
{
  MeasurementContinuityGate gate(0.25);
  ASSERT_TRUE(gate.evaluate(10'000'000'000LL).accepted);
  ASSERT_TRUE(gate.evaluate(10'100'000'000LL).accepted);

  const auto gap = gate.evaluate(11'868'000'000LL);
  EXPECT_FALSE(gap.accepted);
  EXPECT_EQ(gap.reason, "vendor_odometry_measurement_gap");
  EXPECT_NEAR(gap.gap_sec, 1.768, 1e-12);
  EXPECT_EQ(gate.accepted_stamp_nanoseconds(), 10'100'000'000LL);

  const auto following = gate.evaluate(11'968'000'000LL);
  EXPECT_FALSE(following.accepted);
  EXPECT_EQ(following.reason, "vendor_odometry_measurement_gap");
  EXPECT_EQ(gate.accepted_stamp_nanoseconds(), 10'100'000'000LL);
}

TEST(MeasurementContinuityGateTest, RejectsInvalidDuplicateAndBackwardStamps)
{
  MeasurementContinuityGate invalid(0.25);
  EXPECT_EQ(
    invalid.evaluate(0).reason, "vendor_odometry_measurement_stamp_invalid");

  MeasurementContinuityGate duplicate(0.25);
  ASSERT_TRUE(duplicate.evaluate(10'000'000'000LL).accepted);
  EXPECT_EQ(
    duplicate.evaluate(10'000'000'000LL).reason,
    "vendor_odometry_measurement_non_monotonic");

  MeasurementContinuityGate backward(0.25);
  ASSERT_TRUE(backward.evaluate(10'000'000'000LL).accepted);
  EXPECT_EQ(
    backward.evaluate(9'900'000'000LL).reason,
    "vendor_odometry_measurement_non_monotonic");
}

TEST(MeasurementContinuityGateTest, ExplicitResetStartsANewContinuityEpoch)
{
  MeasurementContinuityGate gate(0.25);
  ASSERT_TRUE(gate.evaluate(10'000'000'000LL).accepted);
  ASSERT_FALSE(gate.evaluate(11'000'000'000LL).accepted);
  ASSERT_TRUE(gate.latched());
  gate.reset();
  EXPECT_FALSE(gate.latched());
  EXPECT_TRUE(gate.evaluate(11'100'000'000LL).accepted);
}

TEST(MeasurementContinuityGateTest, RejectsInvalidConfiguration)
{
  EXPECT_THROW(MeasurementContinuityGate(0.0), std::invalid_argument);
  EXPECT_THROW(MeasurementContinuityGate(-1.0), std::invalid_argument);
}

}  // namespace
