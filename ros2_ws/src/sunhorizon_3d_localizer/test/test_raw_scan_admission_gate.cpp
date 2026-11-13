#include <gtest/gtest.h>

#include "sunhorizon_3d_localizer/raw_scan_admission_gate.hpp"

namespace sunhorizon_3d_localizer {
namespace {

TEST(RawScanAdmissionGate, GenericZeroMinimumIsDisabled) {
  const auto decision = evaluate_raw_scan_admission(0U, 0U);
  EXPECT_TRUE(decision.accepted);
  EXPECT_EQ(decision.reason, "ok");
}

TEST(RawScanAdmissionGate, RobotAIncompleteFrameIsSkipped) {
  const auto decision = evaluate_raw_scan_admission(82656U, 85000U);
  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.raw_points, 82656U);
  EXPECT_EQ(decision.minimum_raw_points, 85000U);
  EXPECT_EQ(decision.reason, "raw_scan_points_below_minimum");
}

TEST(RawScanAdmissionGate, RobotACompleteFrameIsAccepted) {
  const auto decision = evaluate_raw_scan_admission(86304U, 85000U);
  EXPECT_TRUE(decision.accepted);
  EXPECT_EQ(decision.reason, "ok");
}

TEST(RawScanAdmissionGate, MinimumBoundaryIsInclusive) {
  EXPECT_TRUE(evaluate_raw_scan_admission(85000U, 85000U).accepted);
  EXPECT_FALSE(evaluate_raw_scan_admission(84999U, 85000U).accepted);
}

}  // namespace
}  // namespace sunhorizon_3d_localizer
