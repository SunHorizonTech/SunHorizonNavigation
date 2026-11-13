#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "sunhorizon_3d_localizer/direct_map_tracking_logic.hpp"

namespace {

using sunhorizon_3d_localizer::DirectMapTrackingState;
using sunhorizon_3d_localizer::DirectMapTrackingLimits;

Eigen::Matrix4d transform(
    double x, double y, double z, double roll = 0.0,
    double pitch = 0.0, double yaw = 0.0) {
  Eigen::Matrix4d output = Eigen::Matrix4d::Identity();
  output.block<3, 3>(0, 0) = (
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  output.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
  return output;
}

TEST(DirectMapTrackingState, FirstFrameUsesExplicitlyAcceptedInitialPose) {
  DirectMapTrackingState state;
  const auto initial = transform(1.0, -2.0, 0.4, 0.0, 0.0, 0.3);

  state.reset(initial, Eigen::Matrix4d::Identity());

  EXPECT_TRUE(state.initialized());
  EXPECT_TRUE(state.next_initial_guess().isApprox(initial, 1e-12));
}

TEST(DirectMapTrackingState, AcceptedResultBecomesExactlyNextInitialGuess) {
  DirectMapTrackingState state;
  state.reset(transform(0.0, 0.0, 0.0), transform(-0.096, 0.0, -0.21926));
  const auto accepted = transform(0.25, -0.1, 0.02, 0.01, -0.02, 0.08);

  const auto decision = state.evaluate(1'000'000'000LL, accepted, {});

  ASSERT_TRUE(decision.accepted) << decision.reason;
  EXPECT_TRUE(state.next_initial_guess().isApprox(accepted, 1e-12));
}

TEST(DirectMapTrackingState, UnacceptedResultCannotAdvanceInitialGuess) {
  DirectMapTrackingState state;
  state.reset(transform(0.0, 0.0, 0.0), Eigen::Matrix4d::Identity());
  const auto accepted = transform(0.1, 0.0, 0.0);
  ASSERT_TRUE(state.evaluate(
      1'000'000'000LL, accepted, {}).accepted);

  const auto rejected = state.evaluate(
      1'100'000'000LL, transform(0.5, 0.0, 0.0), {});
  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.reason, "direct_map_translation_step_limit");

  // A rejected result does not advance either the pose or accepted timestamp.
  EXPECT_TRUE(state.next_initial_guess().isApprox(accepted, 1e-12));
  const auto retry = state.evaluate(
      1'200'000'000LL, transform(0.15, 0.0, 0.0), {});
  EXPECT_TRUE(retry.accepted) << retry.reason;
}

TEST(DirectMapTrackingState, OdomCoordinatesEqualPlanarMapCoordinates) {
  DirectMapTrackingState state;
  state.reset(
      transform(2.0, -1.0, 0.3, 0.08, -0.05, 0.4),
      transform(-0.096, 0.0, -0.21926));
  const auto decision = state.evaluate(
      1'000'000'000LL,
      transform(2.05, -1.0, 0.3, 0.08, -0.05, 0.42), {});

  ASSERT_TRUE(decision.accepted) << decision.reason;
  EXPECT_TRUE(decision.output.map_equivalent_odom_to_base.isApprox(
      decision.output.planar_map_to_base, 1e-12));
  EXPECT_TRUE(decision.output.map_to_odom.isApprox(
      Eigen::Matrix4d::Identity(), 1e-12));
  EXPECT_NEAR(decision.output.planar_map_to_base(2, 3), 0.0, 1e-12);
  EXPECT_NEAR(decision.output.planar_map_to_base(2, 0), 0.0, 1e-12);
  EXPECT_NEAR(decision.output.planar_map_to_base(2, 1), 0.0, 1e-12);
}

TEST(DirectMapTrackingState, RejectsMissingOrNonFiniteAcceptedPose) {
  DirectMapTrackingState state;
  EXPECT_THROW(state.next_initial_guess(), std::logic_error);

  Eigen::Matrix4d invalid = Eigen::Matrix4d::Identity();
  invalid(0, 3) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
      state.reset(invalid, Eigen::Matrix4d::Identity()), std::invalid_argument);

  state.reset(Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity());
  const auto decision = state.evaluate(
      1'000'000'000LL, invalid, {});
  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.reason, "direct_map_candidate_transform_non_finite");
  EXPECT_TRUE(state.next_initial_guess().isApprox(
      Eigen::Matrix4d::Identity(), 1e-12));
}

TEST(DirectMapTrackingState, AcceptedTimestampsMustStrictlyIncrease) {
  DirectMapTrackingState state;
  state.reset(Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity());
  ASSERT_TRUE(state.evaluate(
      1'000'000'000LL, transform(0.05, 0.0, 0.0), {}).accepted);

  const auto duplicate = state.evaluate(
      1'000'000'000LL, transform(0.06, 0.0, 0.0), {});
  EXPECT_FALSE(duplicate.accepted);
  EXPECT_EQ(duplicate.reason, "direct_map_stamp_not_strictly_increasing");
  for (const auto stamp : {0LL, -1LL, 999'999'999LL}) {
    const auto invalid = state.evaluate(stamp, transform(0.06, 0.0, 0.0), {});
    EXPECT_FALSE(invalid.accepted);
    EXPECT_EQ(invalid.reason, "direct_map_stamp_not_strictly_increasing");
  }
  EXPECT_TRUE(state.next_initial_guess().isApprox(
      transform(0.05, 0.0, 0.0), 1e-12));
}

TEST(DirectMapTrackingState, TranslationStepAndRateAreIndependentGates) {
  DirectMapTrackingState state;
  state.reset(Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity());
  ASSERT_TRUE(state.evaluate(
      1'000'000'000LL, Eigen::Matrix4d::Identity(), {}).accepted);

  const auto rate_failure = state.evaluate(
      1'100'000'000LL, transform(0.25, 0.0, 0.0), {});
  EXPECT_FALSE(rate_failure.accepted);
  EXPECT_EQ(rate_failure.reason, "direct_map_translation_rate_limit");

  DirectMapTrackingLimits loose_rate;
  loose_rate.maximum_translation_rate_mps = 100.0;
  const auto step_failure = state.evaluate(
      1'200'000'000LL, transform(0.31, 0.0, 0.0), loose_rate);
  EXPECT_FALSE(step_failure.accepted);
  EXPECT_EQ(step_failure.reason, "direct_map_translation_step_limit");
}

TEST(DirectMapTrackingState, RotationStepAndRateAreIndependentGates) {
  DirectMapTrackingState state;
  state.reset(Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity());
  ASSERT_TRUE(state.evaluate(
      1'000'000'000LL, Eigen::Matrix4d::Identity(), {}).accepted);

  const auto rate_failure = state.evaluate(
      1'100'000'000LL, transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.25), {});
  EXPECT_FALSE(rate_failure.accepted);
  EXPECT_EQ(rate_failure.reason, "direct_map_rotation_rate_limit");

  DirectMapTrackingLimits loose_rate;
  loose_rate.maximum_rotation_rate_radps = 100.0;
  const auto step_failure = state.evaluate(
      1'200'000'000LL, transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.27),
      loose_rate);
  EXPECT_FALSE(step_failure.accepted);
  EXPECT_EQ(step_failure.reason, "direct_map_rotation_step_limit");
}

TEST(DirectMapTrackingState, QualifiedNearPoseRecoversAfterThreeHundredMilliseconds) {
  DirectMapTrackingState state;
  state.reset(Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity());
  const auto accepted = transform(0.05, 0.0, 0.0);
  ASSERT_TRUE(state.evaluate(1'000'000'000LL, accepted, {}).accepted);

  const auto recovered_pose = transform(0.06, 0.0, 0.0);
  const auto recovered = state.evaluate(1'300'000'000LL, recovered_pose, {});
  ASSERT_TRUE(recovered.accepted) << recovered.reason;
  EXPECT_TRUE(recovered.has_previous_measurement);
  EXPECT_NEAR(recovered.dt_sec, 0.3, 1e-12);
  EXPECT_NEAR(recovered.translation_rate_mps, 0.01 / 0.3, 1e-12);
  EXPECT_TRUE(state.next_initial_guess().isApprox(recovered_pose, 1e-12));

  // The next rate still uses the recovered timestamp, not the older baseline.
  const auto too_fast = state.evaluate(1'400'000'000LL, transform(0.31, 0.0, 0.0), {});
  EXPECT_FALSE(too_fast.accepted);
  EXPECT_EQ(too_fast.reason, "direct_map_translation_rate_limit");
  EXPECT_TRUE(state.next_initial_guess().isApprox(recovered_pose, 1e-12));
}

TEST(DirectMapTrackingState, LongIntervalDoesNotPermitSpatialJumpOrRefreshRejectedState) {
  DirectMapTrackingState state;
  state.reset(Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity());
  ASSERT_TRUE(state.evaluate(1'000'000'000LL, Eigen::Matrix4d::Identity(), {}).accepted);
  const auto jump = state.evaluate(2'000'000'000LL, transform(0.31, 0.0, 0.0), {});
  EXPECT_FALSE(jump.accepted);
  EXPECT_EQ(jump.reason, "direct_map_translation_step_limit");
  const auto rotation = state.evaluate(2'100'000'000LL, transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.27), {});
  EXPECT_FALSE(rotation.accepted);
  EXPECT_EQ(rotation.reason, "direct_map_rotation_step_limit");
  EXPECT_TRUE(state.next_initial_guess().isApprox(Eigen::Matrix4d::Identity(), 1e-12));

  const auto recovered = state.evaluate(3'000'000'000LL, transform(0.02, 0.0, 0.0), {});
  ASSERT_TRUE(recovered.accepted) << recovered.reason;
  EXPECT_NEAR(recovered.dt_sec, 2.0, 1e-12);
}

TEST(DirectMapTrackingState, ContinuityUsesPlanarBasePoseWithLeverArm) {
  DirectMapTrackingState state;
  state.reset(Eigen::Matrix4d::Identity(), transform(1.0, 0.0, 0.0));
  DirectMapTrackingLimits limits;
  limits.maximum_translation_step_m = 0.05;
  limits.maximum_rotation_step_rad = 1.0;

  // Source translation is unchanged, but rotating a 1 m source->base lever arm
  // moves base_footprint by about 0.1 m. A source-only gate would miss this.
  const auto decision = state.evaluate(
      1'000'000'000LL, transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.1), limits);
  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.reason, "direct_map_translation_step_limit");
  EXPECT_GT(decision.translation_step_m, 0.09);
}

}  // namespace
