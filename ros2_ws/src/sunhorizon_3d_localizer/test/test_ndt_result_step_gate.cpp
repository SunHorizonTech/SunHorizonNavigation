#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "sunhorizon_3d_localizer/ndt_result_step_gate.hpp"
#include "sunhorizon_3d_localizer/tracking_odom_logic.hpp"

namespace {

using sunhorizon_3d_localizer::NdtResultStepGate;
using sunhorizon_3d_localizer::NdtResultStepLimits;

constexpr double kTwoDegrees = 0.03490658503988659;

Eigen::Matrix4d transform(
    double x, double y, double z = 0.0, double yaw = 0.0) {
  Eigen::Matrix4d output = Eigen::Matrix4d::Identity();
  output.block<3, 3>(0, 0) = Eigen::AngleAxisd(
      yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  output.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
  return output;
}

Eigen::Matrix4d transform_rpy(
    double x, double y, double z, double roll, double pitch, double yaw) {
  Eigen::Matrix4d output = Eigen::Matrix4d::Identity();
  output.block<3, 3>(0, 0) = (
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  output.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
  return output;
}

TEST(NdtResultStepGate, CanonicalCorrectionMatchesManagerPlanarBaseDecision) {
  const auto map_to_source =
      transform_rpy(1.2, -0.7, 0.25, 0.12, -0.08, 0.40);
  const auto odom_to_source =
      transform_rpy(0.8, -0.4, 0.10, -0.04, 0.06, 0.32);
  const auto source_to_base =
      transform_rpy(-0.096, 0.02, -0.21926, 0.01, -0.02, 0.03);
  const auto planar_map_to_base =
      sunhorizon_3d_localizer::planarize(map_to_source * source_to_base);
  const auto planar_odom_to_base =
      sunhorizon_3d_localizer::planarize(odom_to_source * source_to_base);

  const auto canonical = sunhorizon_3d_localizer::
      canonical_map_to_odom_from_planar_base_poses(
          planar_map_to_base, planar_odom_to_base);
  const auto manager_equivalent =
      planar_map_to_base * planar_odom_to_base.inverse();
  const auto raw_source_correction = map_to_source * odom_to_source.inverse();

  EXPECT_TRUE(canonical.isApprox(manager_equivalent, 1e-12));
  EXPECT_FALSE(canonical.isApprox(raw_source_correction, 1e-6));
}

TEST(NdtResultStepGate, FirstAcceptedResultEstablishesEpochBaseline) {
  NdtResultStepGate gate;

  const auto decision = gate.evaluate(
      1'788'758'119'604'976'384LL,
      transform(0.511425, 0.157317, -0.22, 0.01), {});

  EXPECT_TRUE(decision.accepted) << decision.reason;
  EXPECT_FALSE(decision.has_baseline);
  EXPECT_TRUE(decision.establishes_baseline);
  EXPECT_FALSE(decision.gap_recovered);
  EXPECT_TRUE(gate.has_baseline());
}

TEST(NdtResultStepGate, AcceptsExactTranslationAndWrappedYawBoundaries) {
  NdtResultStepGate gate;
  ASSERT_TRUE(gate.evaluate(
      1'000'000'000LL, transform(0.0, 0.0, 0.3, 3.13), {}).accepted);

  const auto decision = gate.evaluate(
      1'200'000'000LL,
      transform(0.03, 0.04, -4.0, 3.13 + kTwoDegrees), {});

  EXPECT_TRUE(decision.accepted) << decision.reason;
  EXPECT_TRUE(decision.has_baseline);
  EXPECT_FALSE(decision.establishes_baseline);
  EXPECT_NEAR(decision.dt_sec, 0.2, 1e-12);
  EXPECT_NEAR(decision.translation_step_m, 0.05, 1e-12);
  EXPECT_NEAR(decision.yaw_step_rad, kTwoDegrees, 1e-12);
}

TEST(NdtResultStepGate, RejectsCorrectionTranslationStepWithoutAdvancing) {
  NdtResultStepGate gate;
  const auto baseline = transform(0.511425, 0.157317, -0.22, 0.01);
  ASSERT_TRUE(gate.evaluate(
      1'788'758'119'604'976'384LL, baseline, {}).accepted);

  const auto rejected = gate.evaluate(
      1'788'758'119'804'890'368LL,
      transform(0.450993, 0.149264, -0.22, 0.01), {});

  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.reason, "ndt_correction_translation_step_limit");
  EXPECT_NEAR(rejected.dt_sec, 0.199913984, 1e-12);
  EXPECT_GT(rejected.translation_step_m, 0.06);
  EXPECT_FALSE(rejected.gap_recovered);

  // A rejected outlier does not poison the baseline. A later candidate that
  // returns inside the old 5 cm envelope is admissible.
  const auto recovered = gate.evaluate(
      1'788'758'119'904'000'000LL,
      transform(0.501425, 0.157317, -0.22, 0.01), {});
  EXPECT_TRUE(recovered.accepted) << recovered.reason;
  EXPECT_NEAR(recovered.translation_step_m, 0.01, 1e-12);
}

TEST(NdtResultStepGate, RejectsYawAboveTwoDegrees) {
  NdtResultStepGate gate;
  ASSERT_TRUE(gate.evaluate(
      1'000'000'000LL, transform(0.0, 0.0), {}).accepted);

  const auto decision = gate.evaluate(
      1'200'000'000LL, transform(0.0, 0.0, 0.0, kTwoDegrees + 1e-6), {});

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.reason, "ndt_correction_yaw_step_limit");
}

TEST(NdtResultStepGate, LongGapRecoversOnlyWithBoundedCorrection) {
  NdtResultStepGate gate;
  ASSERT_TRUE(gate.evaluate(
      1'000'000'000LL, transform(0.0, 0.0), {}).accepted);

  const auto gap = gate.evaluate(
      1'500'000'001LL, transform(0.001, 0.0), {});
  EXPECT_TRUE(gap.accepted) << gap.reason;
  EXPECT_TRUE(gap.gap_recovered);
  EXPECT_EQ(
      gap.reason, "ok_after_result_gap_with_bounded_correction");

  const auto jump_after_gap = gate.evaluate(
      2'100'000'000LL, transform(0.20, 0.0), {});
  EXPECT_FALSE(jump_after_gap.accepted);
  EXPECT_FALSE(jump_after_gap.gap_recovered);
  EXPECT_EQ(
      jump_after_gap.reason, "ndt_correction_translation_step_limit");

  const auto recovered = gate.evaluate(
      2'200'000'000LL, transform(0.011, 0.0), {});
  EXPECT_TRUE(recovered.accepted) << recovered.reason;
  EXPECT_TRUE(recovered.gap_recovered);
  EXPECT_EQ(
      recovered.reason, "ok_after_result_gap_with_bounded_correction");
}

TEST(NdtResultStepGate, RejectsInvalidInputAndLimitsFailClosed) {
  NdtResultStepGate gate;
  Eigen::Matrix4d invalid = Eigen::Matrix4d::Identity();
  invalid(0, 3) = std::numeric_limits<double>::quiet_NaN();

  EXPECT_EQ(
      gate.evaluate(1'000'000'000LL, invalid, {}).reason,
      "ndt_correction_transform_non_finite");
  EXPECT_EQ(
      gate.evaluate(0, Eigen::Matrix4d::Identity(), {}).reason,
      "ndt_correction_stamp_not_strictly_increasing");

  NdtResultStepLimits invalid_limits;
  invalid_limits.maximum_translation_step_m = 0.0;
  EXPECT_THROW(
      gate.evaluate(1'000'000'000LL, Eigen::Matrix4d::Identity(), invalid_limits),
      std::invalid_argument);
}

TEST(NdtResultStepGate, TenCentimeterLimitRequiresExplicitAttendedProfile) {
  using sunhorizon_3d_localizer::valid_result_translation_step_limit;
  EXPECT_TRUE(valid_result_translation_step_limit(.05, false));
  EXPECT_FALSE(valid_result_translation_step_limit(.050001, false));
  EXPECT_FALSE(valid_result_translation_step_limit(.10, false));
  EXPECT_TRUE(valid_result_translation_step_limit(.05, true));
  EXPECT_TRUE(valid_result_translation_step_limit(.10, true));
  EXPECT_FALSE(valid_result_translation_step_limit(.100001, true));
  for (bool attended : {false, true}) {
    EXPECT_FALSE(valid_result_translation_step_limit(0., attended));
    EXPECT_FALSE(valid_result_translation_step_limit(-.01, attended));
    EXPECT_FALSE(valid_result_translation_step_limit(
        std::numeric_limits<double>::quiet_NaN(), attended));
    EXPECT_FALSE(valid_result_translation_step_limit(
        std::numeric_limits<double>::infinity(), attended));
  }
}

TEST(NdtResultStepGate, Trial13RecordedCandidateReplayShowsFrozenBaselineEffect) {
  // Robot A 2026-09-10 trial13, 11:39:58.765--11:40:00.266.
  // Canonical corrections reconstructed from the rejected NDT journal poses
  // and cloud-time interpolated /livo/odom_candidate in goal-13-nearbody.
  // Interpolation spans 4--40 ms and journal values have 6 decimal places;
  // this is a gate-only replay, NOT a fresh point-cloud registration replay.
  struct Sample { double x; double y; double yaw; std::int64_t stamp_ns; };
  const std::array<Sample, 15> samples{{
    {.3779927717, -.3339754994, -.0233684307, 1789011598765478144LL},
    {.3546230021, -.3367502850,  .0025067988, 1789011598865506048LL},
    {.3422778890, -.3543263933,  .0146440297, 1789011598965390336LL},
    {.3247593168, -.3505980703, -.0075162106, 1789011599065382656LL},
    {.3202699140, -.3531476916, -.0161223011, 1789011599165557760LL},
    {.3389225209, -.3767193979, -.0031958165, 1789011599265529088LL},
    {.3389662096, -.4144403841,  .0162184383, 1789011599465522176LL},
    {.3260454301, -.4262271358,  .0039867184, 1789011599565494016LL},
    {.3405004090, -.4010789106, -.0071723747, 1789011599665577216LL},
    {.3369639880, -.4139484176, -.0013788417, 1789011599765538816LL},
    {.3408016812, -.4348285711,  .0045402366, 1789011599865594112LL},
    {.3289360202, -.4097842332,  .0133029578, 1789011599965532416LL},
    {.3153412820, -.4054363285,  .0089718068, 1789011600065532416LL},
    {.3123227369, -.3662889610, -.0031808670, 1789011600165563136LL},
    {.3262482124, -.3543441603, -.0038538662, 1789011600265579008LL},
  }};
  const auto baseline = transform(.4028922393, -.3865433054, 0., -.0049235571);
  const auto baseline_stamp = 1789011598665548000LL;
  NdtResultStepGate strict, attended;
  NdtResultStepLimits attended_limits;
  attended_limits.maximum_translation_step_m = .10;
  ASSERT_TRUE(strict.evaluate(baseline_stamp, baseline, {}).accepted);
  ASSERT_TRUE(attended.evaluate(baseline_stamp, baseline, attended_limits).accepted);
  for (std::size_t index = 0; index < samples.size(); ++index) {
    const auto& sample = samples[index];
    const auto candidate = transform(sample.x, sample.y, 0., sample.yaw);
    const auto denied = strict.evaluate(sample.stamp_ns, candidate, {});
    EXPECT_FALSE(denied.accepted);
    EXPECT_EQ(denied.reason, "ndt_correction_translation_step_limit");
    const auto admitted = attended.evaluate(sample.stamp_ns, candidate, attended_limits);
    EXPECT_TRUE(admitted.accepted) << admitted.reason;
    if (index == 0) {
      EXPECT_NEAR(admitted.translation_step_m, .058206, .001);
    } else {
      EXPECT_LT(admitted.translation_step_m, .05);
    }
    EXPECT_LE(admitted.yaw_step_rad, kTwoDegrees);
  }
  const auto& last = samples.back();
  EXPECT_EQ(attended.evaluate(last.stamp_ns + 100000000LL,
      transform(last.x + .100001, last.y, 0., last.yaw), attended_limits).reason,
      "ndt_correction_translation_step_limit");
  EXPECT_EQ(attended.evaluate(last.stamp_ns + 200000000LL,
      transform(last.x, last.y, 0., last.yaw + kTwoDegrees + .000001),
      attended_limits).reason, "ndt_correction_yaw_step_limit");
}

}  // namespace
