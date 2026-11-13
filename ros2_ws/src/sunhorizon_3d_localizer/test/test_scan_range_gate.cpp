#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "sunhorizon_3d_localizer/scan_range_gate.hpp"

namespace {

using sunhorizon_3d_localizer::ScanRangeGate;

TEST(ScanRangeGateTest, AcceptsInclusiveConfiguredRange) {
  ScanRangeGate gate(0.5, 15.0);

  EXPECT_DOUBLE_EQ(gate.minimum_scan_range_m(), 0.5);
  EXPECT_DOUBLE_EQ(gate.maximum_scan_range_m(), 15.0);
  EXPECT_TRUE(gate.accepts(0.5, 0.0, 0.0));
  EXPECT_TRUE(gate.accepts(3.0, 4.0, 0.0));
  EXPECT_TRUE(gate.accepts(9.0, 12.0, 0.0));
}

TEST(ScanRangeGateTest, RejectsNearFarAndNonFinitePointsBeforeVoxelization) {
  ScanRangeGate gate(0.5, 15.0);

  EXPECT_FALSE(gate.accepts(0.499, 0.0, 0.0));
  EXPECT_FALSE(gate.accepts(15.001, 0.0, 0.0));
  EXPECT_FALSE(gate.accepts(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
  EXPECT_FALSE(gate.accepts(
      std::numeric_limits<double>::infinity(), 0.0, 0.0));
}

TEST(ScanRangeGateTest, RejectsInvalidLimits) {
  EXPECT_THROW(ScanRangeGate(0.0, 15.0), std::invalid_argument);
  EXPECT_THROW(ScanRangeGate(-0.5, 15.0), std::invalid_argument);
  EXPECT_THROW(ScanRangeGate(15.0, 15.0), std::invalid_argument);
  EXPECT_THROW(ScanRangeGate(16.0, 15.0), std::invalid_argument);
  EXPECT_THROW(
      ScanRangeGate(0.5, std::numeric_limits<double>::infinity()),
      std::invalid_argument);
}

TEST(ScanRangeGateTest, CompatibilityDefaultsRetainExistingSensorWindow) {
  ScanRangeGate gate(0.5, 30.0);

  EXPECT_TRUE(gate.accepts(30.0, 0.0, 0.0));
  EXPECT_FALSE(gate.accepts(30.001, 0.0, 0.0));
}

}  // namespace
