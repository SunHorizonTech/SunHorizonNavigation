#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "sunhorizon_3d_localizer/ndt_match_rate_gate.hpp"

namespace {

using sunhorizon_3d_localizer::NdtMatchRateDecision;
using sunhorizon_3d_localizer::NdtMatchRateGate;

TEST(NdtMatchRateGateTest, FirstValidTimestampAlwaysSchedulesMatch) {
  NdtMatchRateGate gate(4.0);

  EXPECT_EQ(gate.minimum_match_interval_ns(), 250000000);
  EXPECT_EQ(gate.evaluate(123456789), NdtMatchRateDecision::kMatch);
}

TEST(NdtMatchRateGateTest, ThrottlesAgainstLastScheduledSensorTimestamp) {
  NdtMatchRateGate gate(4.0);

  EXPECT_EQ(gate.evaluate(1000000000), NdtMatchRateDecision::kMatch);
  EXPECT_EQ(gate.evaluate(1100000000), NdtMatchRateDecision::kThrottled);
  EXPECT_EQ(gate.evaluate(1249999999), NdtMatchRateDecision::kThrottled);
  EXPECT_EQ(gate.evaluate(1250000000), NdtMatchRateDecision::kMatch);
  EXPECT_EQ(gate.evaluate(1499999999), NdtMatchRateDecision::kThrottled);
  EXPECT_EQ(gate.evaluate(1500000000), NdtMatchRateDecision::kMatch);
}

TEST(NdtMatchRateGateTest, TimestampRegressionRequiresExplicitEpochReset) {
  NdtMatchRateGate gate(4.0);

  EXPECT_EQ(gate.evaluate(2000000000), NdtMatchRateDecision::kMatch);
  EXPECT_EQ(gate.evaluate(2100000000), NdtMatchRateDecision::kThrottled);
  EXPECT_EQ(gate.evaluate(1900000000), NdtMatchRateDecision::kTimestampRegressed);
  EXPECT_EQ(gate.evaluate(2250000000), NdtMatchRateDecision::kMatch);

  gate.reset();
  EXPECT_EQ(gate.evaluate(1900000000), NdtMatchRateDecision::kMatch);
  EXPECT_EQ(gate.evaluate(2000000000), NdtMatchRateDecision::kThrottled);
}

TEST(NdtMatchRateGateTest, RejectsInvalidConfigurationAndTimestamp) {
  EXPECT_THROW(NdtMatchRateGate(0.0), std::invalid_argument);
  EXPECT_THROW(NdtMatchRateGate(-1.0), std::invalid_argument);
  EXPECT_THROW(
      NdtMatchRateGate(std::numeric_limits<double>::infinity()),
      std::invalid_argument);

  NdtMatchRateGate gate(4.0);
  EXPECT_EQ(gate.evaluate(0), NdtMatchRateDecision::kInvalidTimestamp);
  EXPECT_EQ(gate.evaluate(1), NdtMatchRateDecision::kMatch);
}

TEST(NdtMatchRateGateTest, CompatibilityDefaultPassesEveryIncreasingNanosecond) {
  NdtMatchRateGate gate(1.0e9);

  EXPECT_EQ(gate.minimum_match_interval_ns(), 1);
  EXPECT_EQ(gate.evaluate(1), NdtMatchRateDecision::kMatch);
  EXPECT_EQ(gate.evaluate(2), NdtMatchRateDecision::kMatch);
  EXPECT_EQ(gate.evaluate(3), NdtMatchRateDecision::kMatch);
}

}  // namespace
