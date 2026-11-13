#include <limits>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "sunhorizon_3d_localizer/tracking_odom_logic.hpp"

namespace {

using sunhorizon_3d_localizer::TrackingOdomGate;
using sunhorizon_3d_localizer::TrackingOdomLimits;
using sunhorizon_3d_localizer::TrackingOdomSample;

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

Eigen::Matrix4d airy_basis() {
  Eigen::Quaterniond quaternion(0.5, -0.5, 0.5, -0.5);
  quaternion.normalize();
  Eigen::Matrix4d output = Eigen::Matrix4d::Identity();
  output.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
  return output;
}

TrackingOdomSample sample(
    std::int64_t stamp_ns, const Eigen::Matrix4d& pose,
    const std::string& frame = "odom",
    const std::string& child = "rslidar") {
  TrackingOdomSample output;
  output.stamp_ns = stamp_ns;
  output.frame_id = frame;
  output.child_frame_id = child;
  output.raw_odom_to_source = pose;
  return output;
}

TrackingOdomGate gate(TrackingOdomLimits limits = {}) {
  return TrackingOdomGate(
      "odom", "rslidar", airy_basis(),
      transform(-0.096, 0.0, -0.21926), limits);
}

TEST(TrackingOdomGate, AppliesBasisExtrinsicAndPlanarizesOnce) {
  auto tracker = gate();
  const auto decision = tracker.evaluate(sample(
      1'000'000'000LL, transform(1.0, 2.0, 3.0)));

  ASSERT_TRUE(decision.accepted) << decision.reason;
  EXPECT_NEAR(decision.odom_to_base(0, 3), 2.904, 1e-9);
  EXPECT_NEAR(decision.odom_to_base(1, 3), -1.0, 1e-9);
  EXPECT_NEAR(decision.odom_to_base(2, 3), -2.21926, 1e-9);
  EXPECT_NEAR(decision.planar_odom_to_base(0, 3), 2.904, 1e-9);
  EXPECT_NEAR(decision.planar_odom_to_base(1, 3), -1.0, 1e-9);
  EXPECT_NEAR(decision.planar_odom_to_base(2, 3), 0.0, 1e-9);
  EXPECT_NEAR(decision.planar_odom_to_base(2, 0), 0.0, 1e-9);
  EXPECT_NEAR(decision.planar_odom_to_base(2, 1), 0.0, 1e-9);
}

TEST(InitialPoseGeometry, UsesFixedMapBaseHeightForRvizTwoDimensionalSeed) {
  const auto input = transform(0.55, 1.08, 0.0, 0.2, -0.1, 0.03);
  const auto output =
      sunhorizon_3d_localizer::planarize_with_z_reference(input, -0.21926);

  EXPECT_NEAR(output(0, 3), 0.55, 1e-12);
  EXPECT_NEAR(output(1, 3), 1.08, 1e-12);
  EXPECT_NEAR(output(2, 3), -0.21926, 1e-12);
  EXPECT_NEAR(output(2, 0), 0.0, 1e-12);
  EXPECT_NEAR(output(2, 1), 0.0, 1e-12);
  EXPECT_NEAR(std::atan2(output(1, 0), output(0, 0)), 0.03, 1e-12);
}

TEST(InitialPoseGeometry, RejectsInvalidFixedMapBaseHeight) {
  EXPECT_THROW(
      sunhorizon_3d_localizer::planarize_with_z_reference(
          Eigen::Matrix4d::Identity(),
          std::numeric_limits<double>::quiet_NaN()),
      std::invalid_argument);
}

TEST(TrackingOdomGate, AcceptsMonotonicBoundedMotion) {
  auto tracker = gate();
  ASSERT_TRUE(tracker.evaluate(sample(
      1'000'000'000LL, transform(0.0, 0.0, 0.0))).accepted);
  const auto decision = tracker.evaluate(sample(
      1'100'000'000LL, transform(0.10, 0.0, 0.0, 0.0, 0.0, 0.10)));
  EXPECT_TRUE(decision.accepted) << decision.reason;
  EXPECT_TRUE(decision.has_previous);
  EXPECT_NEAR(decision.dt_sec, 0.1, 1e-12);
}

TEST(TrackingOdomGate, FrameFailureLatchesAndStopsFutureSamples) {
  auto tracker = gate();
  const auto invalid = tracker.evaluate(sample(
      1'000'000'000LL, transform(0.0, 0.0, 0.0), "map"));
  EXPECT_FALSE(invalid.accepted);
  EXPECT_TRUE(invalid.latched);
  EXPECT_EQ(invalid.reason, "odometry_prior_frame_mismatch");

  const auto later = tracker.evaluate(sample(
      1'100'000'000LL, transform(0.0, 0.0, 0.0)));
  EXPECT_FALSE(later.accepted);
  EXPECT_EQ(later.reason, "odometry_prior_frame_mismatch");
}

TEST(TrackingOdomGate, DuplicateStampLatches) {
  auto tracker = gate();
  ASSERT_TRUE(tracker.evaluate(sample(
      1'000'000'000LL, transform(0.0, 0.0, 0.0))).accepted);
  const auto decision = tracker.evaluate(sample(
      1'000'000'000LL, transform(0.0, 0.0, 0.0)));
  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.reason, "odometry_prior_stamp_not_strictly_increasing");
}

TEST(TrackingOdomGate, GapLatches) {
  auto tracker = gate();
  ASSERT_TRUE(tracker.evaluate(sample(
      1'000'000'000LL, transform(0.0, 0.0, 0.0))).accepted);
  const auto decision = tracker.evaluate(sample(
      1'300'000'001LL, transform(0.0, 0.0, 0.0)));
  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.reason, "odometry_prior_gap");
}

TEST(TrackingOdomGate, TranslationRateAndHardStepAreIndependentGates) {
  auto rate_tracker = gate();
  ASSERT_TRUE(rate_tracker.evaluate(sample(
      1'000'000'000LL, transform(0.0, 0.0, 0.0))).accepted);
  const auto rate_failure = rate_tracker.evaluate(sample(
      1'100'000'000LL, transform(0.25, 0.0, 0.0)));
  EXPECT_FALSE(rate_failure.accepted);
  EXPECT_EQ(rate_failure.reason, "odometry_prior_translation_rate_limit");

  TrackingOdomLimits hard_limits;
  hard_limits.maximum_translation_rate_mps = 100.0;
  auto hard_tracker = gate(hard_limits);
  ASSERT_TRUE(hard_tracker.evaluate(sample(
      1'000'000'000LL, transform(0.0, 0.0, 0.0))).accepted);
  const auto hard_failure = hard_tracker.evaluate(sample(
      1'200'000'000LL, transform(0.31, 0.0, 0.0)));
  EXPECT_FALSE(hard_failure.accepted);
  EXPECT_EQ(hard_failure.reason, "odometry_prior_translation_step_limit");
}

TEST(TrackingOdomGate, RotationRateAndHardStepAreIndependentGates) {
  auto rate_tracker = gate();
  ASSERT_TRUE(rate_tracker.evaluate(sample(
      1'000'000'000LL, transform(0.0, 0.0, 0.0))).accepted);
  const auto rate_failure = rate_tracker.evaluate(sample(
      1'100'000'000LL, transform(0.0, 0.0, 0.0, 0.0, 0.25)));
  EXPECT_FALSE(rate_failure.accepted);
  EXPECT_EQ(rate_failure.reason, "odometry_prior_rotation_rate_limit");

  TrackingOdomLimits hard_limits;
  hard_limits.maximum_rotation_rate_radps = 100.0;
  auto hard_tracker = gate(hard_limits);
  ASSERT_TRUE(hard_tracker.evaluate(sample(
      1'000'000'000LL, transform(0.0, 0.0, 0.0))).accepted);
  const auto hard_failure = hard_tracker.evaluate(sample(
      1'200'000'000LL, transform(0.0, 0.0, 0.0, 0.0, 0.27)));
  EXPECT_FALSE(hard_failure.accepted);
  EXPECT_EQ(hard_failure.reason, "odometry_prior_rotation_step_limit");
}

TEST(TrackingOdomGate, NonFinitePoseLatches) {
  auto tracker = gate();
  auto pose = transform(0.0, 0.0, 0.0);
  pose(0, 3) = std::numeric_limits<double>::quiet_NaN();
  const auto decision = tracker.evaluate(sample(1'000'000'000LL, pose));
  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.reason, "odometry_prior_pose_non_finite");
}

TEST(CorrectionContinuity, MeasuresMapToOdomInsteadOfRobotTravel) {
  const Eigen::Matrix4d correction = transform(5.0, -2.0, 0.0, 0.0, 0.0, 0.1);
  const Eigen::Matrix4d first_odom = transform(0.0, 0.0, 0.0);
  const Eigen::Matrix4d second_odom = transform(1.0, 0.0, 0.0);
  const Eigen::Matrix4d first_map = correction * first_odom;
  const Eigen::Matrix4d second_map = correction * second_odom;
  const Eigen::Matrix4d first_correction = first_map * first_odom.inverse();
  const Eigen::Matrix4d second_correction = second_map * second_odom.inverse();

  const auto stable = sunhorizon_3d_localizer::evaluate_transform_step(
      first_correction, second_correction, 0.30, 0.26);
  EXPECT_TRUE(stable.accepted) << stable.reason;
  EXPECT_NEAR(stable.translation_step_m, 0.0, 1e-9);

  const auto jump = sunhorizon_3d_localizer::evaluate_transform_step(
      first_correction, transform(5.31, -2.0, 0.0, 0.0, 0.0, 0.1),
      0.30, 0.26);
  EXPECT_FALSE(jump.accepted);
  EXPECT_EQ(jump.reason, "translation_step_limit");
}

}  // namespace
