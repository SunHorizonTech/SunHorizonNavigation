#include <gtest/gtest.h>

#include <limits>

#include "sunhorizon_3d_localizer/ndt_startup_alignment_gate.hpp"

namespace {

using sunhorizon_3d_localizer::NdtStartupAlignmentGate;
using sunhorizon_3d_localizer::NdtStartupAlignmentLimits;
using sunhorizon_3d_localizer::NdtStartupAlignmentSample;

NdtStartupAlignmentSample sample(
    std::int64_t stamp_ns, double x, double y, double yaw,
    double seed_translation = 0.51, double seed_yaw = 0.063) {
  return {stamp_ns, x, y, yaw, seed_translation, seed_yaw};
}

TEST(NdtStartupAlignmentGate, RequiresAStableConsecutiveWindow) {
  NdtStartupAlignmentGate gate;
  NdtStartupAlignmentLimits limits;
  limits.required_consecutive_samples = 3U;

  auto decision = gate.observe(sample(1000000000LL, 0.10, 0.07, -0.029), limits);
  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.consecutive_samples, 1U);
  EXPECT_EQ(decision.reason, "ndt_startup_alignment_pending");

  decision = gate.observe(sample(1100000000LL, 0.11, 0.065, -0.028), limits);
  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.consecutive_samples, 2U);

  decision = gate.observe(sample(1200000000LL, 0.09, 0.075, -0.030), limits);
  EXPECT_TRUE(decision.accepted);
  EXPECT_EQ(decision.consecutive_samples, 3U);
  EXPECT_EQ(decision.reason, "ndt_startup_alignment_stable");
}

TEST(NdtStartupAlignmentGate, ResetsOnCandidateDisagreement) {
  NdtStartupAlignmentGate gate;
  NdtStartupAlignmentLimits limits;
  limits.required_consecutive_samples = 3U;
  ASSERT_FALSE(gate.observe(
      sample(1000000000LL, 0.10, 0.07, -0.029), limits).accepted);
  const auto decision = gate.observe(
      sample(1100000000LL, 0.30, 0.07, -0.029), limits);
  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.consecutive_samples, 1U);
  EXPECT_EQ(decision.reason, "ndt_startup_alignment_inconsistent");
}

TEST(NdtStartupAlignmentGate, ResetsOnSensorTimeGap) {
  NdtStartupAlignmentGate gate;
  NdtStartupAlignmentLimits limits;
  limits.required_consecutive_samples = 2U;
  ASSERT_FALSE(gate.observe(
      sample(1000000000LL, 0.10, 0.07, -0.029), limits).accepted);
  const auto decision = gate.observe(
      sample(1500000000LL, 0.10, 0.07, -0.029), limits);
  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.consecutive_samples, 1U);
  EXPECT_EQ(decision.reason, "ndt_startup_alignment_inconsistent");
}

TEST(NdtStartupAlignmentGate, RejectsOutsideTrustedStartupRegion) {
  NdtStartupAlignmentGate gate;
  NdtStartupAlignmentLimits limits;
  auto decision = gate.observe(
      sample(1000000000LL, 0.10, 0.07, -0.029, 1.01, 0.063), limits);
  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.reason, "ndt_initial_translation_limit");

  decision = gate.observe(
      sample(1100000000LL, 0.10, 0.07, -0.029, 0.51, 0.30), limits);
  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.reason, "ndt_initial_yaw_limit");
}

TEST(NdtStartupAlignmentGate, RejectsNonFiniteEvidence) {
  NdtStartupAlignmentGate gate;
  NdtStartupAlignmentLimits limits;
  auto invalid = sample(1000000000LL, 0.10, 0.07, -0.029);
  invalid.x_m = std::numeric_limits<double>::quiet_NaN();
  const auto decision = gate.observe(invalid, limits);
  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.reason, "ndt_startup_alignment_invalid");
}

}  // namespace
