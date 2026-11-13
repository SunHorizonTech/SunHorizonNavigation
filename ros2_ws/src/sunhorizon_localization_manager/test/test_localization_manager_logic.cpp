#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include <gtest/gtest.h>
#include <builtin_interfaces/msg/time.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "sunhorizon_localization_manager/localization_manager_logic.hpp"
#include "sunhorizon_localization_manager/topic_contract.hpp"

namespace {

using LocalizationStatus = sunhorizon_navigation_interfaces::msg::LocalizationStatus;
using sunhorizon_localization_manager::BackendEvidence;
using sunhorizon_localization_manager::CorrectionGate;
using sunhorizon_localization_manager::EvidenceLimits;
using sunhorizon_localization_manager::HealthyPoseStatusPairCache;
using sunhorizon_localization_manager::OdometryHistory;
using sunhorizon_localization_manager::OdometryHistoryConfig;

builtin_interfaces::msg::Time stamp(double seconds) {
  builtin_interfaces::msg::Time output;
  output.sec = static_cast<int32_t>(std::floor(seconds));
  output.nanosec = static_cast<uint32_t>(
      std::llround((seconds - static_cast<double>(output.sec)) * 1e9));
  return output;
}

geometry_msgs::msg::Quaternion rpy_quaternion(
    double roll, double pitch, double yaw) {
  tf2::Quaternion output;
  output.setRPY(roll, pitch, yaw);
  output.normalize();
  return tf2::toMsg(output);
}

geometry_msgs::msg::Quaternion yaw_quaternion(double yaw) {
  return rpy_quaternion(0.0, 0.0, yaw);
}

tf2::Transform rigid_rpy(
    double x, double y, double z, double roll, double pitch, double yaw) {
  tf2::Quaternion rotation;
  rotation.setRPY(roll, pitch, yaw);
  return tf2::Transform(rotation, tf2::Vector3(x, y, z));
}

geometry_msgs::msg::Pose rigid_pose(const tf2::Transform& value) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = value.getOrigin().x();
  pose.position.y = value.getOrigin().y();
  pose.position.z = value.getOrigin().z();
  pose.orientation = tf2::toMsg(value.getRotation());
  return pose;
}

tf2::Transform rigid_message(const geometry_msgs::msg::Transform& value) {
  tf2::Transform result;
  tf2::fromMsg(value, result);
  return result;
}

void expect_rigid_near(const tf2::Transform& actual, const tf2::Transform& expected) {
  for (int row = 0; row < 3; ++row) {
    EXPECT_NEAR(actual.getOrigin()[row], expected.getOrigin()[row], 1.e-10);
    for (int column = 0; column < 3; ++column) {
      EXPECT_NEAR(actual.getBasis()[row][column], expected.getBasis()[row][column], 1.e-10);
    }
  }
}

BackendEvidence healthy_evidence(
    const std::string& backend, double time,
    double odom_x = 0.0, double odom_y = 0.0, double odom_yaw = 0.0,
    double map_x = 0.0, double map_y = 0.0, double map_yaw = 0.0) {
  BackendEvidence evidence;
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp(time);
  odometry.header.frame_id = "odom";
  odometry.child_frame_id = "base_footprint";
  odometry.pose.pose.position.x = odom_x;
  odometry.pose.pose.position.y = odom_y;
  odometry.pose.pose.orientation = yaw_quaternion(odom_yaw);
  evidence.odometry = odometry;

  geometry_msgs::msg::PoseWithCovarianceStamped map_pose;
  map_pose.header.stamp = stamp(time);
  map_pose.header.frame_id = "map";
  map_pose.pose.pose.position.x = map_x;
  map_pose.pose.pose.position.y = map_y;
  map_pose.pose.pose.orientation = yaw_quaternion(map_yaw);
  evidence.map_pose = map_pose;

  LocalizationStatus status;
  status.header.stamp = stamp(time);
  status.header.frame_id = "map";
  status.state = LocalizationStatus::STATE_HEALTHY;
  status.backend = backend;
  status.measurement_accepted = true;
  status.reason = "ok";
  evidence.status = status;
  evidence.odometry_received_sec = time;
  evidence.map_pose_received_sec = time;
  evidence.status_received_sec = time;
  return evidence;
}

nav_msgs::msg::Odometry tracking_odometry(
    double time, double x = 0.0, double yaw = 0.0) {
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp(time);
  odometry.header.frame_id = "odom";
  odometry.child_frame_id = "base_footprint";
  odometry.pose.pose.position.x = x;
  odometry.pose.pose.orientation = yaw_quaternion(yaw);
  odometry.twist.twist.linear.x = x;
  odometry.pose.covariance[0] = x;
  odometry.twist.covariance[0] = x;
  return odometry;
}

TEST(LocalizationManagerLogic, RegistrationTransportRetainsFullDatumAndSourceLeverArm) {
  // Independent SE(3) reference with nonzero roll/pitch/Z and a rotated,
  // translated source lever arm. Neither public planar TF nor body_tilt enters.
  tf2::Transform q,lower,upper,e;
  tf2::Quaternion rotation;
  rotation.setRPY(.08,-.06,.4);q.setRotation(rotation);q.setOrigin(tf2::Vector3(1.,-2.,-.3));
  rotation.setRPY(.1,-.2,.3);lower.setRotation(rotation);lower.setOrigin(tf2::Vector3(.5,.6,.7));
  rotation.setRPY(-.15,.12,.35);upper.setRotation(rotation);upper.setOrigin(tf2::Vector3(.55,.63,.8));
  rotation.setRPY(.03,-.02,.1);e.setRotation(rotation);e.setOrigin(tf2::Vector3(-.096,.01,-.21926));
  const auto pose=[](const tf2::Transform& value){geometry_msgs::msg::Pose out;
    out.position.x=value.getOrigin().x();out.position.y=value.getOrigin().y();out.position.z=value.getOrigin().z();
    out.orientation=tf2::toMsg(value.getRotation());return out;};
  const auto actual=sunhorizon_localization_manager::transport_pcd_from_airy(
    tf2::toMsg(q),pose(lower),pose(upper),tf2::toMsg(e));
  const auto expected=q*(lower*e.inverse()).inverse()*(upper*e.inverse());
  EXPECT_NEAR(actual.translation.x,expected.getOrigin().x(),1.e-12);
  EXPECT_NEAR(actual.translation.y,expected.getOrigin().y(),1.e-12);
  EXPECT_NEAR(actual.translation.z,expected.getOrigin().z(),1.e-12);
  tf2::Quaternion actual_rotation;tf2::fromMsg(actual.rotation,actual_rotation);
  EXPECT_NEAR(actual_rotation.angleShortestPath(expected.getRotation()),0.,1.e-7);
  EXPECT_GT(std::abs(actual.translation.z),.01);
  EXPECT_GT(std::abs(actual.rotation.x)+std::abs(actual.rotation.y),.01);
  const auto same=sunhorizon_localization_manager::transport_pcd_from_airy(
    tf2::toMsg(q),pose(lower),pose(lower),tf2::toMsg(e));
  EXPECT_NEAR(same.translation.z,q.getOrigin().z(),1.e-12);
  EXPECT_NEAR(same.rotation.x,q.getRotation().x(),1.e-12);
}

TEST(LocalizationManagerLogic, ComputesPlanarMapToOdom) {
  constexpr double kHalfPi = 1.57079632679489661923;
  const auto evidence = healthy_evidence(
      "lidar", 10.0, 2.0, 0.0, 0.0, 10.0, 3.0, kHalfPi);

  const auto decision = sunhorizon_localization_manager::evaluate_backend(
      "lidar", evidence, 10.1);

  ASSERT_TRUE(decision.valid) << decision.reason;
  EXPECT_NEAR(decision.map_to_odom.translation.x, 10.0, 1e-9);
  EXPECT_NEAR(decision.map_to_odom.translation.y, 1.0, 1e-9);
  EXPECT_NEAR(decision.map_to_odom.translation.z, 0.0, 1e-9);
  EXPECT_NEAR(decision.odom_to_base_footprint.translation.x, 2.0, 1e-9);
  tf2::Quaternion rotation;
  tf2::fromMsg(decision.map_to_odom.rotation, rotation);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
  EXPECT_NEAR(yaw, kHalfPi, 1e-9);
}

TEST(LocalizationManagerLogic, CanonicalCorrectionIsPlanarForSixDofEvidence) {
  auto evidence = healthy_evidence(
      "livo", 10.0, 2.0, -1.0, 0.2, 10.0, 3.0, 0.4);
  evidence.odometry->pose.pose.position.z = -0.23;
  evidence.odometry->pose.pose.orientation = rpy_quaternion(0.04, -0.10, 0.2);
  evidence.map_pose->pose.pose.position.z = 0.31;
  evidence.map_pose->pose.pose.orientation = rpy_quaternion(-0.03, 0.06, 0.4);

  const auto decision = sunhorizon_localization_manager::evaluate_backend(
      "livo", evidence, 10.1);

  ASSERT_TRUE(decision.valid) << decision.reason;
  EXPECT_DOUBLE_EQ(decision.odom_to_base_footprint.translation.z, 0.0);
  EXPECT_DOUBLE_EQ(decision.map_to_odom.translation.z, 0.0);
  for (const auto& transform : {
      decision.odom_to_base_footprint, decision.map_to_odom}) {
    tf2::Quaternion rotation;
    tf2::fromMsg(transform.rotation, rotation);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.0, 1e-12);
    EXPECT_NEAR(pitch, 0.0, 1e-12);
    EXPECT_TRUE(std::isfinite(yaw));
  }
  // Projection is confined to the canonical decision. The upstream 6DoF
  // evidence remains available to NDT and any diagnostic consumer.
  EXPECT_DOUBLE_EQ(evidence.odometry->pose.pose.position.z, -0.23);
}

TEST(LocalizationManagerLogic, FullMapAlignmentRestoresRegistrationSensorWithDynamicWaist) {
  sunhorizon_localization_manager::FrameContract frames;
  frames.preserve_6d_map_alignment = true;
  const auto map_from_odom = rigid_rpy(3., -2., 1.4, -.04, .06, .7);
  // Vendor world starts at its IMU; the body origin is below that origin.
  const auto odom_from_body = rigid_rpy(.2, -.1, -.215, .08, -.07, .4);
  const auto base_from_waist_zero = rigid_rpy(0., 0., .15424, 0., 0., 0.);
  const auto source_from_body_zero = rigid_rpy(-.096, .005, -.21926, .006, -.002, .003);
  for (const double waist : {-.08726646259971647, 0., .08726646259971647}) {
    SCOPED_TRACE(waist);
    const auto base_from_waist = rigid_rpy(0., 0., .15424, 0., 0., waist);
    const auto source_from_body = source_from_body_zero * base_from_waist_zero *
        base_from_waist.inverse();
    // NDT supplies map<-source, independently of the vendor's world datum.
    const auto registered_map_from_source = map_from_odom * odom_from_body *
        source_from_body.inverse();
    auto evidence = healthy_evidence("livo", 10.);
    evidence.odometry->pose.pose = rigid_pose(odom_from_body);
    evidence.map_pose->pose.pose = rigid_pose(registered_map_from_source * source_from_body);
    const auto decision = sunhorizon_localization_manager::evaluate_backend(
        "livo", evidence, 10.1, frames);
    ASSERT_TRUE(decision.valid) << decision.reason;
    const auto actual_alignment = rigid_message(decision.map_to_odom);
    expect_rigid_near(actual_alignment, map_from_odom);
    const auto leveled_body = rigid_message(decision.odom_to_base_footprint);
    const auto residual = leveled_body.inverse() * odom_from_body;
    EXPECT_NEAR((leveled_body.getOrigin() - odom_from_body.getOrigin()).length(), 0., 1.e-12);
    EXPECT_NEAR(residual.getOrigin().length(), 0., 1.e-12);
    double roll, pitch, yaw;
    leveled_body.getBasis().getRPY(roll, pitch, yaw);
    EXPECT_NEAR(roll, 0., 1.e-12);
    EXPECT_NEAR(pitch, 0., 1.e-12);
    EXPECT_NEAR(yaw, .4, 1.e-12);
    const auto tracking = sunhorizon_localization_manager::evaluate_tracking_odometry(
        evidence.odometry, evidence.odometry_received_sec, 10.1, frames);
    ASSERT_TRUE(tracking.valid) << tracking.reason;
    EXPECT_DOUBLE_EQ(tracking.canonical_odometry.pose.pose.position.z, -.215);
    expect_rigid_near(rigid_message(tracking.odom_to_base_footprint), leveled_body);
    expect_rigid_near(actual_alignment * leveled_body * residual * source_from_body.inverse(),
        registered_map_from_source);
    EXPECT_DOUBLE_EQ(evidence.odometry->pose.pose.position.z, -.215);
  }
}

TEST(LocalizationManagerLogic, DefaultTrackingStillDropsBodyHeightAndTilt) {
  const auto full = rigid_rpy(.2, -.1, -.215, .08, -.07, .4);
  auto evidence = healthy_evidence("livo", 10.);
  evidence.odometry->pose.pose = rigid_pose(full);
  EXPECT_FALSE(sunhorizon_localization_manager::FrameContract{}.preserve_6d_map_alignment);
  const auto tracking = sunhorizon_localization_manager::evaluate_tracking_odometry(
      evidence.odometry, evidence.odometry_received_sec, 10.1);
  ASSERT_TRUE(tracking.valid) << tracking.reason;
  EXPECT_DOUBLE_EQ(tracking.canonical_odometry.pose.pose.position.z, 0.);
  expect_rigid_near(rigid_message(tracking.odom_to_base_footprint),
      rigid_rpy(.2, -.1, 0., 0., 0., .4));
  EXPECT_DOUBLE_EQ(evidence.odometry->pose.pose.position.z, -.215);
}

TEST(LocalizationManagerLogic, FullMapAlignmentCancelsVendorHeightAndTiltGauge) {
  sunhorizon_localization_manager::FrameContract frames;
  frames.preserve_6d_map_alignment = true;
  const auto nominal_odom_body = rigid_rpy(.2, -.1, -.215, .08, -.07, .4);
  const auto registered_map_source = rigid_rpy(4., -2., 1.4, -.04, .06, .7);
  const auto source_body = rigid_rpy(-.096, .005, -.21926, .006, -.002, .08726646259971647);
  for (const auto& gauge : {
      rigid_rpy(0., 0., 0., 0., 0., 0.),
      rigid_rpy(0., 0., 2.3, 0., 0., 0.),
      rigid_rpy(.5, -.2, -1.7, -.12, .09, -.6)}) {
    const auto odom_body = gauge * nominal_odom_body;
    auto evidence = healthy_evidence("livo", 10.);
    evidence.odometry->pose.pose = rigid_pose(odom_body);
    evidence.map_pose->pose.pose = rigid_pose(registered_map_source * source_body);
    const auto decision = sunhorizon_localization_manager::evaluate_backend(
        "livo", evidence, 10.1, frames);
    ASSERT_TRUE(decision.valid) << decision.reason;
    const auto leveled_body = rigid_message(decision.odom_to_base_footprint);
    const auto residual = leveled_body.inverse() * odom_body;
    expect_rigid_near(rigid_message(decision.map_to_odom) * leveled_body * residual *
        source_body.inverse(), registered_map_source);
  }
}

TEST(LocalizationManagerLogic, FullMapAlignmentZAndTiltJumpsLatchCorrectionGateClosed) {
  sunhorizon_localization_manager::FrameContract frames;
  frames.preserve_6d_map_alignment = true;
  auto initial_evidence = healthy_evidence("livo", 10.);
  initial_evidence.odometry->pose.pose.position.z = -.215;
  initial_evidence.map_pose->pose.pose.position.z = .6;
  const auto initial = sunhorizon_localization_manager::evaluate_backend(
      "livo", initial_evidence, 10.1, frames);
  ASSERT_TRUE(initial.valid) << initial.reason;
  EvidenceLimits limits;
  limits.maximum_correction_translation_step_m = .30;
  limits.maximum_correction_rotation_step_rad = .25;
  for (const bool tilt_jump : {false, true}) {
    SCOPED_TRACE(tilt_jump);
    CorrectionGate gate(limits, 1.0);
    ASSERT_TRUE(gate.consider(10000000000LL, initial.map_to_odom).valid);
    auto changed_evidence = initial_evidence;
    if (tilt_jump) {
      changed_evidence.map_pose->pose.pose.orientation = rpy_quaternion(.4, 0., 0.);
    } else {
      changed_evidence.map_pose->pose.pose.position.z += .31;
    }
    const auto changed = sunhorizon_localization_manager::evaluate_backend(
        "livo", changed_evidence, 10.1, frames);
    ASSERT_TRUE(changed.valid) << changed.reason;
    const auto rejected = gate.consider(10100000000LL, changed.map_to_odom);
    EXPECT_FALSE(rejected.valid);
    EXPECT_TRUE(rejected.requires_reinitialization);
    EXPECT_EQ(rejected.reason, tilt_jump ? "map_to_odom_correction_rotation_jump" :
        "map_to_odom_correction_translation_jump");
    const auto held = gate.consider(10200000000LL, initial.map_to_odom);
    EXPECT_FALSE(held.valid);
    EXPECT_TRUE(held.requires_reinitialization);
    expect_rigid_near(rigid_message(*gate.accepted_correction()),
        rigid_message(initial.map_to_odom));
    gate.reinitialize();
    EXPECT_TRUE(gate.consider(10300000000LL, changed.map_to_odom).valid);
  }
}

TEST(LocalizationManagerLogic, FullMapAlignmentFilterRetainsBoundedZAndTilt) {
  CorrectionGate gate({}, 1.0);
  const auto first = rigid_rpy(0., 0., .8, .03, -.02, .4);
  const auto second = rigid_rpy(.05, 0., .9, .06, -.04, .45);
  ASSERT_TRUE(gate.consider(10000000000LL, tf2::toMsg(first)).valid);
  ASSERT_TRUE(gate.consider(11000000000LL, tf2::toMsg(second)).valid);
  const double alpha = 1. - std::exp(-1.);
  const tf2::Transform expected(first.getRotation().slerp(second.getRotation(), alpha),
      first.getOrigin().lerp(second.getOrigin(), alpha));
  expect_rigid_near(rigid_message(*gate.accepted_correction()), expected);
}

TEST(LocalizationManagerLogic, RejectsStaleEvidence) {
  const auto evidence = healthy_evidence("lidar", 10.0);
  const auto decision = sunhorizon_localization_manager::evaluate_backend(
      "lidar", evidence, 11.0);
  EXPECT_FALSE(decision.valid);
  EXPECT_EQ(decision.reason, "odometry_stream_stale");
}

TEST(LocalizationManagerLogic, KeepsCorrectionAndTrackingFreshnessBudgetsSeparate) {
  const auto evidence = healthy_evidence("livo", 10.0);
  const auto correction = sunhorizon_localization_manager::evaluate_backend(
      "livo", evidence, 10.4);
  ASSERT_TRUE(correction.valid) << correction.reason;

  const auto tracking =
      sunhorizon_localization_manager::evaluate_tracking_odometry(
          evidence.odometry, evidence.odometry_received_sec, 10.4);
  EXPECT_FALSE(tracking.valid);
  EXPECT_EQ(tracking.reason, "tracking_odometry_stream_stale");
}

TEST(LocalizationManagerLogic, AcceptsBoundedPipelineDelayFromLiveStreams) {
  EvidenceLimits limits;
  limits.odometry_timeout_sec = 0.80;
  limits.map_pose_timeout_sec = 0.80;
  limits.status_timeout_sec = 0.80;
  limits.tracking_odometry_timeout_sec = 0.65;
  auto evidence = healthy_evidence("livo", 10.0);
  evidence.odometry_received_sec = 10.49;
  evidence.map_pose_received_sec = 10.49;
  evidence.status_received_sec = 10.49;

  const auto decision = sunhorizon_localization_manager::evaluate_backend(
      "livo", evidence, 11.20, {}, limits);
  ASSERT_TRUE(decision.valid) << decision.reason;

  const auto tracking_sample = tracking_odometry(20.0);
  const auto tracking =
      sunhorizon_localization_manager::evaluate_tracking_odometry(
          tracking_sample, 20.45, 20.55, {}, limits);
  EXPECT_TRUE(tracking.valid) << tracking.reason;
}

TEST(LocalizationManagerLogic, RejectsDropoutByReceiptTime) {
  EvidenceLimits limits;
  limits.odometry_timeout_sec = 0.80;
  limits.map_pose_timeout_sec = 0.80;
  limits.status_timeout_sec = 0.80;
  limits.tracking_odometry_timeout_sec = 0.65;
  auto evidence = healthy_evidence("livo", 9.60);
  evidence.odometry_received_sec = 10.00;
  evidence.map_pose_received_sec = 10.00;
  evidence.status_received_sec = 10.00;

  const auto decision = sunhorizon_localization_manager::evaluate_backend(
      "livo", evidence, 10.81, {}, limits);
  EXPECT_FALSE(decision.valid);
  EXPECT_EQ(decision.reason, "odometry_stream_stale");

  const auto tracking =
      sunhorizon_localization_manager::evaluate_tracking_odometry(
          evidence.odometry, 10.00, 10.81, {}, limits);
  EXPECT_FALSE(tracking.valid);
  EXPECT_EQ(tracking.reason, "tracking_odometry_stream_stale");
}

TEST(LocalizationManagerLogic, AcceptsDelayedMeasurementWithLiveReceipt) {
  auto evidence = healthy_evidence("livo", 10.0);
  evidence.odometry_received_sec = 10.75;
  evidence.map_pose_received_sec = 10.75;
  evidence.status_received_sec = 10.75;

  const auto decision = sunhorizon_localization_manager::evaluate_backend(
      "livo", evidence, 10.80);
  EXPECT_TRUE(decision.valid) << decision.reason;

  const auto tracking =
      sunhorizon_localization_manager::evaluate_tracking_odometry(
          evidence.odometry, 10.75, 10.80);
  EXPECT_TRUE(tracking.valid) << tracking.reason;
}

TEST(LocalizationManagerLogic, MissingReceiptFailsClosed) {
  auto evidence = healthy_evidence("livo", 10.0);
  evidence.odometry_received_sec.reset();

  const auto decision = sunhorizon_localization_manager::evaluate_backend(
      "livo", evidence, 10.10);
  EXPECT_FALSE(decision.valid);
  EXPECT_EQ(decision.reason, "odometry_receipt_missing");

  const auto tracking =
      sunhorizon_localization_manager::evaluate_tracking_odometry(
          evidence.odometry, std::nullopt, 10.10);
  EXPECT_FALSE(tracking.valid);
  EXPECT_EQ(tracking.reason, "tracking_odometry_receipt_missing");
}

TEST(LocalizationManagerLogic, FutureMeasurementStillFailsClosed) {
  auto evidence = healthy_evidence("livo", 10.014);
  // Receipt is within the clock-future tolerance, and the header is within
  // tolerance of receipt, but the header is still too far ahead of now.
  evidence.odometry_received_sec = 10.005;
  evidence.map_pose_received_sec = 10.005;
  evidence.status_received_sec = 10.005;

  const auto decision = sunhorizon_localization_manager::evaluate_backend(
      "livo", evidence, 10.0);
  EXPECT_FALSE(decision.valid);
  EXPECT_EQ(decision.reason, "odometry_from_future");
}

TEST(LocalizationManagerLogic, ReportsUpstreamLostBeforeHistoricalAlignment) {
  auto evidence = healthy_evidence("livo", 10.0);
  evidence.status->header.stamp = stamp(12.1);
  evidence.status->state = LocalizationStatus::STATE_LOST;
  evidence.status->measurement_accepted = false;
  evidence.status->reason =
      "recovering_livo_odometry_stale:waiting_for_new_amcl_correction";

  const auto failure = sunhorizon_localization_manager::reported_backend_failure(
      "livo", evidence);
  ASSERT_TRUE(failure);
  EXPECT_EQ(
      *failure,
      "backend_unhealthy:recovering_livo_odometry_stale:"
      "waiting_for_new_amcl_correction");

  evidence.status->state = LocalizationStatus::STATE_HEALTHY;
  evidence.status->measurement_accepted = true;
  EXPECT_FALSE(sunhorizon_localization_manager::reported_backend_failure(
      "livo", evidence));

  evidence.status->state = LocalizationStatus::STATE_LOST;
  evidence.status->measurement_accepted = false;
  evidence.status->backend = "visual";
  EXPECT_FALSE(sunhorizon_localization_manager::reported_backend_failure(
      "livo", evidence));
}

TEST(LocalizationManagerLogic, RejectsMismatchedEvidenceStamps) {
  auto evidence = healthy_evidence("visual", 10.0);
  evidence.map_pose->header.stamp = stamp(10.2);
  // The newer map pose arrived at its own sensor epoch.  Keep the receipt
  // physically possible so this test isolates cross-evidence synchronization
  // rather than the independent header-versus-receipt future-time gate.
  evidence.map_pose_received_sec = 10.2;
  const auto decision = sunhorizon_localization_manager::evaluate_backend(
      "visual", evidence, 10.21);
  EXPECT_FALSE(decision.valid);
  EXPECT_EQ(decision.reason, "candidate_stamp_mismatch");
}

TEST(LocalizationManagerLogic, RejectsStatusFromDifferentMeasurementEpoch) {
  auto evidence = healthy_evidence("lidar", 10.0);
  evidence.status->header.stamp = stamp(10.2);
  // Model a valid receipt for the newer status so the expected failure is the
  // status/map epoch mismatch, not an impossible future-dated measurement.
  evidence.status_received_sec = 10.2;
  const auto decision = sunhorizon_localization_manager::evaluate_backend(
      "lidar", evidence, 10.21);
  EXPECT_FALSE(decision.valid);
  EXPECT_EQ(decision.reason, "candidate_status_stamp_mismatch");
}

TEST(LocalizationManagerLogic, PropagatesUnhealthyStatusBeforeEpochMismatch) {
  auto evidence = healthy_evidence("livo", 10.0);
  evidence.status->header.stamp = stamp(10.2);
  evidence.status->state = LocalizationStatus::STATE_LOST;
  evidence.status->measurement_accepted = false;
  evidence.status->reason = "ndt_fitness_limit";
  const auto decision = sunhorizon_localization_manager::evaluate_backend(
      "livo", evidence, 10.21);
  EXPECT_FALSE(decision.valid);
  EXPECT_EQ(decision.reason, "backend_unhealthy:ndt_fitness_limit");
}

TEST(LocalizationManagerLogic, CommitsHealthyPoseThenStatusAsOneEpoch) {
  HealthyPoseStatusPairCache cache;
  const auto epoch_one = healthy_evidence("livo", 10.0);
  EXPECT_FALSE(cache.update_map_pose(*epoch_one.map_pose));
  EXPECT_FALSE(cache.committed_map_pose());
  EXPECT_FALSE(cache.effective_status());

  EXPECT_TRUE(cache.update_status(*epoch_one.status));
  ASSERT_TRUE(cache.committed_map_pose());
  ASSERT_TRUE(cache.effective_status());
  EXPECT_EQ(cache.committed_map_pose()->header.stamp, stamp(10.0));
  EXPECT_EQ(cache.effective_status()->header.stamp, stamp(10.0));

  const auto epoch_two = healthy_evidence("livo", 10.1);
  EXPECT_FALSE(cache.update_map_pose(*epoch_two.map_pose));
  EXPECT_EQ(cache.committed_map_pose()->header.stamp, stamp(10.0));
  EXPECT_EQ(cache.effective_status()->header.stamp, stamp(10.0));

  BackendEvidence asynchronous_evidence = epoch_one;
  asynchronous_evidence.map_pose = cache.committed_map_pose();
  asynchronous_evidence.status = cache.effective_status();
  const auto waiting = sunhorizon_localization_manager::evaluate_backend(
      "livo", asynchronous_evidence, 10.11);
  ASSERT_TRUE(waiting.valid) << waiting.reason;

  EXPECT_TRUE(cache.update_status(*epoch_two.status));
  EXPECT_EQ(cache.committed_map_pose()->header.stamp, stamp(10.1));
  EXPECT_EQ(cache.effective_status()->header.stamp, stamp(10.1));
  EXPECT_EQ(cache.committed_stamp_ns(), 10100000000LL);
}

TEST(LocalizationManagerLogic, RepeatedCommittedEpochDoesNotRefreshPair) {
  HealthyPoseStatusPairCache cache;
  const auto epoch = healthy_evidence("livo", 10.0);
  EXPECT_FALSE(cache.update_map_pose(*epoch.map_pose));
  EXPECT_TRUE(cache.update_status(*epoch.status));

  EXPECT_FALSE(cache.update_status(*epoch.status));
  EXPECT_FALSE(cache.update_map_pose(*epoch.map_pose));
  EXPECT_EQ(cache.committed_stamp_ns(), 10000000000LL);

  const auto delayed = healthy_evidence("livo", 9.9);
  EXPECT_FALSE(cache.update_status(*delayed.status));
  EXPECT_FALSE(cache.update_map_pose(*delayed.map_pose));
  EXPECT_EQ(cache.committed_stamp_ns(), 10000000000LL);
}

TEST(LocalizationManagerLogic, CommitsHealthyStatusThenPoseAsOneEpoch) {
  HealthyPoseStatusPairCache cache;
  const auto epoch_one = healthy_evidence("visual", 20.0);
  cache.update_status(*epoch_one.status);
  EXPECT_FALSE(cache.committed_map_pose());
  EXPECT_FALSE(cache.effective_status());

  cache.update_map_pose(*epoch_one.map_pose);
  ASSERT_TRUE(cache.committed_map_pose());
  ASSERT_TRUE(cache.effective_status());
  EXPECT_EQ(cache.committed_map_pose()->header.stamp, stamp(20.0));
  EXPECT_EQ(cache.effective_status()->header.stamp, stamp(20.0));

  const auto epoch_two = healthy_evidence("visual", 20.1);
  cache.update_status(*epoch_two.status);
  EXPECT_EQ(cache.committed_map_pose()->header.stamp, stamp(20.0));
  EXPECT_EQ(cache.effective_status()->header.stamp, stamp(20.0));
  cache.update_map_pose(*epoch_two.map_pose);
  EXPECT_EQ(cache.committed_map_pose()->header.stamp, stamp(20.1));
  EXPECT_EQ(cache.effective_status()->header.stamp, stamp(20.1));
}

TEST(LocalizationManagerLogic, UnhealthyStatusFailsClosedUntilHealthyPairCommits) {
  HealthyPoseStatusPairCache cache;
  const auto initial = healthy_evidence("livo", 30.0);
  cache.update_map_pose(*initial.map_pose);
  cache.update_status(*initial.status);

  auto unhealthy = *initial.status;
  unhealthy.header.stamp = stamp(30.1);
  unhealthy.state = LocalizationStatus::STATE_LOST;
  unhealthy.measurement_accepted = false;
  unhealthy.reason = "sensor_input_stale";
  EXPECT_FALSE(cache.update_status(unhealthy));
  ASSERT_TRUE(cache.effective_status());
  EXPECT_EQ(cache.effective_status()->state, LocalizationStatus::STATE_LOST);

  BackendEvidence failed = initial;
  failed.map_pose = cache.committed_map_pose();
  failed.status = cache.effective_status();
  const auto failed_decision = sunhorizon_localization_manager::evaluate_backend(
      "livo", failed, 30.11);
  EXPECT_FALSE(failed_decision.valid);
  EXPECT_EQ(failed_decision.reason, "backend_unhealthy:sensor_input_stale");

  const auto recovery = healthy_evidence("livo", 30.2);
  EXPECT_FALSE(cache.update_status(*recovery.status));
  EXPECT_EQ(cache.effective_status()->state, LocalizationStatus::STATE_LOST);
  EXPECT_TRUE(cache.update_map_pose(*recovery.map_pose));
  EXPECT_EQ(cache.committed_map_pose()->header.stamp, stamp(30.2));
  EXPECT_EQ(cache.effective_status()->header.stamp, stamp(30.2));
  EXPECT_EQ(cache.effective_status()->state, LocalizationStatus::STATE_HEALTHY);
}

TEST(LocalizationManagerLogic, OldHealthyPairCannotClearSameEpochFailure) {
  HealthyPoseStatusPairCache cache;
  const auto initial = healthy_evidence("livo", 35.0);
  cache.update_map_pose(*initial.map_pose);
  cache.update_status(*initial.status);

  auto unhealthy = *initial.status;
  unhealthy.state = LocalizationStatus::STATE_LOST;
  unhealthy.measurement_accepted = false;
  unhealthy.reason = "sensor_input_stale";
  cache.update_status(unhealthy);

  const auto next_epoch = healthy_evidence("livo", 35.1);
  cache.update_status(*next_epoch.status);
  ASSERT_TRUE(cache.effective_status());
  EXPECT_EQ(cache.effective_status()->state, LocalizationStatus::STATE_LOST);
  EXPECT_EQ(cache.effective_status()->reason, "sensor_input_stale");

  cache.update_map_pose(*next_epoch.map_pose);
  EXPECT_EQ(cache.effective_status()->state, LocalizationStatus::STATE_HEALTHY);
  EXPECT_EQ(cache.effective_status()->header.stamp, stamp(35.1));
}

TEST(LocalizationManagerLogic, NeverRollsCommittedPairBackToOlderEpoch) {
  HealthyPoseStatusPairCache cache;
  const auto current = healthy_evidence("lidar", 40.0);
  cache.update_map_pose(*current.map_pose);
  cache.update_status(*current.status);

  const auto delayed = healthy_evidence("lidar", 39.9);
  cache.update_status(*delayed.status);
  cache.update_map_pose(*delayed.map_pose);
  EXPECT_EQ(cache.committed_map_pose()->header.stamp, stamp(40.0));
  EXPECT_EQ(cache.effective_status()->header.stamp, stamp(40.0));
  EXPECT_EQ(cache.committed_stamp_ns(), 40000000000LL);
}

TEST(LocalizationManagerLogic, RejectsMismatchedFrames) {
  auto evidence = healthy_evidence("visual", 10.0);
  evidence.odometry->child_frame_id = "base_link";
  const auto decision = sunhorizon_localization_manager::evaluate_backend(
      "visual", evidence, 10.1);
  EXPECT_FALSE(decision.valid);
  EXPECT_EQ(decision.reason, "odometry_frame_mismatch");
}

TEST(LocalizationManagerLogic, RejectsNonFiniteEvidence) {
  auto evidence = healthy_evidence("lidar", 10.0);
  evidence.map_pose->pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  const auto decision = sunhorizon_localization_manager::evaluate_backend(
      "lidar", evidence, 10.1);
  EXPECT_FALSE(decision.valid);
  EXPECT_EQ(decision.reason, "map_pose_non_finite");
}

TEST(LocalizationManagerLogic, DoesNotFailOverFromConfiguredBackend) {
  auto lidar = healthy_evidence("lidar", 9.0);
  const auto visual = healthy_evidence("visual", 10.0);
  const BackendEvidence livo;
  const auto lidar_primary = sunhorizon_localization_manager::evaluate_active_backend(
      "lidar", lidar, visual, livo, 10.1);
  const auto visual_primary = sunhorizon_localization_manager::evaluate_active_backend(
      "visual", lidar, visual, livo, 10.1);
  EXPECT_FALSE(lidar_primary.valid);
  EXPECT_EQ(lidar_primary.reason, "odometry_stream_stale");
  EXPECT_TRUE(visual_primary.valid) << visual_primary.reason;
}

TEST(LocalizationManagerLogic, SupportsLivoAsConfiguredPrimary) {
  const BackendEvidence lidar;
  const BackendEvidence visual;
  const auto livo = healthy_evidence(
      "livo", 10.0, 2.0, 0.0, 0.1, 3.0, 1.0, 0.2);
  const auto decision = sunhorizon_localization_manager::evaluate_active_backend(
      "livo", lidar, visual, livo, 10.1);
  ASSERT_TRUE(decision.valid) << decision.reason;
  EXPECT_NEAR(decision.odom_to_base_footprint.translation.x, 2.0, 1e-9);
}

TEST(LocalizationManagerLogic, ValidatesIndependentTrackingOdometry) {
  const auto evidence = healthy_evidence("lidar", 10.0, 1.0, 2.0, 0.2);
  const auto valid = sunhorizon_localization_manager::evaluate_tracking_odometry(
      evidence.odometry, 10.05, 10.1);
  ASSERT_TRUE(valid.valid) << valid.reason;
  EXPECT_NEAR(valid.odom_to_base_footprint.translation.x, 1.0, 1e-9);

  const auto stale = sunhorizon_localization_manager::evaluate_tracking_odometry(
      evidence.odometry, 10.0, 11.0);
  EXPECT_FALSE(stale.valid);
  EXPECT_EQ(stale.reason, "tracking_odometry_stream_stale");
}

TEST(LocalizationManagerLogic, CanonicalTrackingOdometryIsPlanar) {
  auto raw = tracking_odometry(10.0, 1.25, 0.35);
  raw.pose.pose.position.y = -0.75;
  raw.pose.pose.position.z = -0.23;
  raw.pose.pose.orientation = rpy_quaternion(0.04, -0.10, 0.35);
  raw.twist.twist.linear.x = 0.4;
  raw.twist.twist.linear.y = -0.1;
  raw.twist.twist.linear.z = 0.3;
  raw.twist.twist.angular.x = 0.2;
  raw.twist.twist.angular.y = -0.4;
  raw.twist.twist.angular.z = 0.5;

  const auto decision =
      sunhorizon_localization_manager::evaluate_tracking_odometry(
          raw, 10.05, 10.1);

  ASSERT_TRUE(decision.valid) << decision.reason;
  EXPECT_EQ(decision.canonical_odometry.header, raw.header);
  EXPECT_EQ(decision.canonical_odometry.header.frame_id, raw.header.frame_id);
  EXPECT_EQ(decision.canonical_odometry.child_frame_id, "base_footprint");
  EXPECT_DOUBLE_EQ(decision.canonical_odometry.pose.pose.position.x, 1.25);
  EXPECT_DOUBLE_EQ(decision.canonical_odometry.pose.pose.position.y, -0.75);
  EXPECT_DOUBLE_EQ(decision.canonical_odometry.pose.pose.position.z, 0.0);
  EXPECT_DOUBLE_EQ(decision.canonical_odometry.twist.twist.linear.x, 0.4);
  EXPECT_DOUBLE_EQ(decision.canonical_odometry.twist.twist.linear.y, -0.1);
  EXPECT_DOUBLE_EQ(decision.canonical_odometry.twist.twist.linear.z, 0.0);
  EXPECT_DOUBLE_EQ(decision.canonical_odometry.twist.twist.angular.x, 0.0);
  EXPECT_DOUBLE_EQ(decision.canonical_odometry.twist.twist.angular.y, 0.0);
  EXPECT_DOUBLE_EQ(decision.canonical_odometry.twist.twist.angular.z, 0.5);
  EXPECT_DOUBLE_EQ(decision.odom_to_base_footprint.translation.z, 0.0);
  tf2::Quaternion rotation;
  tf2::fromMsg(decision.canonical_odometry.pose.pose.orientation, rotation);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
  EXPECT_NEAR(roll, 0.0, 1e-12);
  EXPECT_NEAR(pitch, 0.0, 1e-12);
  EXPECT_NEAR(yaw, 0.35, 1e-12);
  EXPECT_DOUBLE_EQ(raw.pose.pose.position.z, -0.23);
}

TEST(LocalizationManagerLogic, PublishesEachTrackingStampAtMostOnce) {
  EXPECT_FALSE(sunhorizon_localization_manager::should_publish_tracking_stamp(0, 0));
  EXPECT_TRUE(sunhorizon_localization_manager::should_publish_tracking_stamp(100, 0));
  EXPECT_FALSE(sunhorizon_localization_manager::should_publish_tracking_stamp(100, 100));
  EXPECT_FALSE(sunhorizon_localization_manager::should_publish_tracking_stamp(99, 100));
  EXPECT_TRUE(sunhorizon_localization_manager::should_publish_tracking_stamp(101, 100));
}

TEST(LocalizationManagerLogic, RejectsDiscontinuousMapToOdomCorrection) {
  EvidenceLimits limits;
  limits.maximum_correction_translation_step_m = 0.30;
  limits.maximum_correction_rotation_step_rad = 0.25;

  geometry_msgs::msg::Transform first;
  first.translation.x = 1.0;
  first.rotation = yaw_quaternion(0.0);
  geometry_msgs::msg::Transform smooth = first;
  smooth.translation.x = 1.1;
  smooth.rotation = yaw_quaternion(0.1);
  geometry_msgs::msg::Transform translation_jump = first;
  translation_jump.translation.x = 1.5;
  geometry_msgs::msg::Transform rotation_jump = first;
  rotation_jump.rotation = yaw_quaternion(0.5);

  EXPECT_TRUE(sunhorizon_localization_manager::evaluate_correction_transition(
      std::nullopt, first, limits).valid);
  EXPECT_TRUE(sunhorizon_localization_manager::evaluate_correction_transition(
      first, smooth, limits).valid);
  EXPECT_EQ(sunhorizon_localization_manager::evaluate_correction_transition(
      first, translation_jump, limits).reason,
      "map_to_odom_correction_translation_jump");
  EXPECT_EQ(sunhorizon_localization_manager::evaluate_correction_transition(
      first, rotation_jump, limits).reason,
      "map_to_odom_correction_rotation_jump");
}

TEST(LocalizationManagerLogic, InterpolatesHistoricalTrackingOdometry) {
  OdometryHistory history(OdometryHistoryConfig{2.0, 0.05, 100, 1e-3});
  ASSERT_TRUE(history.insert(
      tracking_odometry(10.0, 0.0, 0.0), 10.02).accepted);
  ASSERT_TRUE(history.insert(
      tracking_odometry(10.1, 2.0, 1.0), 10.12).accepted);

  const auto lookup = history.lookup(10050000000LL);
  ASSERT_TRUE(lookup.valid) << lookup.reason;
  EXPECT_TRUE(lookup.interpolated);
  EXPECT_NEAR(lookup.odometry.pose.pose.position.x, 1.0, 1e-9);
  EXPECT_NEAR(lookup.odometry.twist.twist.linear.x, 1.0, 1e-9);
  EXPECT_NEAR(lookup.odometry.pose.covariance[0], 1.0, 1e-9);
  EXPECT_NEAR(lookup.odometry.twist.covariance[0], 1.0, 1e-9);
  tf2::Quaternion rotation;
  tf2::fromMsg(lookup.odometry.pose.pose.orientation, rotation);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
  EXPECT_NEAR(yaw, 0.5, 1e-9);
  EXPECT_EQ(stamp(10.05), lookup.odometry.header.stamp);
  ASSERT_TRUE(lookup.received_sec);
  EXPECT_DOUBLE_EQ(*lookup.received_sec, 10.12);
}

TEST(LocalizationManagerLogic, HistoricalEpochKeepsItsOwnReceiptTime) {
  OdometryHistory history(OdometryHistoryConfig{2.0, 0.05, 100, 1e-3});
  ASSERT_TRUE(history.insert(tracking_odometry(10.0), 10.20).accepted);
  ASSERT_TRUE(history.insert(tracking_odometry(10.2), 10.70).accepted);
  const auto duplicate = history.insert(tracking_odometry(10.2), 10.90);
  EXPECT_FALSE(duplicate.accepted);
  EXPECT_EQ(duplicate.reason, "odometry_history_stamp_duplicate");

  const auto historical = history.lookup(10000000000LL);
  ASSERT_TRUE(historical.valid) << historical.reason;
  ASSERT_TRUE(historical.received_sec);
  EXPECT_DOUBLE_EQ(*historical.received_sec, 10.20);

  auto evidence = healthy_evidence("livo", 10.0);
  evidence.odometry = historical.odometry;
  evidence.odometry_received_sec = historical.received_sec;
  evidence.map_pose_received_sec = 10.30;
  evidence.status_received_sec = 10.30;
  EvidenceLimits limits;
  limits.odometry_timeout_sec = 0.80;
  limits.map_pose_timeout_sec = 0.80;
  limits.status_timeout_sec = 0.80;
  const auto decision = sunhorizon_localization_manager::evaluate_backend(
      "livo", evidence, 11.01, {}, limits);
  EXPECT_FALSE(decision.valid);
  EXPECT_EQ(decision.reason, "odometry_stream_stale");

  const auto newest = history.lookup(10200000000LL);
  ASSERT_TRUE(newest.received_sec);
  EXPECT_DOUBLE_EQ(*newest.received_sec, 10.70);
}

TEST(LocalizationManagerLogic, ExactNdtPairAvoidsSparseTrackingInterpolation) {
  OdometryHistory tracking_history(
      OdometryHistoryConfig{2.0, 0.05, 100, 1e-3});
  ASSERT_TRUE(tracking_history.insert(tracking_odometry(10.0)).accepted);
  ASSERT_TRUE(tracking_history.insert(tracking_odometry(10.2)).accepted);
  EXPECT_EQ(tracking_history.lookup(10100000000LL).reason,
      "odometry_history_gap_exceeds_tolerance");

  OdometryHistory ndt_alignment_history(
      OdometryHistoryConfig{2.0, 0.05, 100, 1e-3});
  ASSERT_TRUE(ndt_alignment_history.insert(
      tracking_odometry(10.1, 1.0, 0.1), 10.45).accepted);
  const auto exact = ndt_alignment_history.lookup(10100000000LL);
  ASSERT_TRUE(exact.valid) << exact.reason;
  EXPECT_FALSE(exact.interpolated);
  EXPECT_EQ(stamp(10.1), exact.odometry.header.stamp);
  EXPECT_NEAR(exact.odometry.pose.pose.position.x, 1.0, 1e-9);
  ASSERT_TRUE(exact.received_sec);
  EXPECT_DOUBLE_EQ(*exact.received_sec, 10.45);
}

TEST(LocalizationManagerLogic, KeepsHistoryDurationAndSampleCountBounded) {
  OdometryHistory duration_history(
      OdometryHistoryConfig{2.0, 0.05, 100, 1e-3});
  for (int index = 0; index <= 30; ++index) {
    ASSERT_TRUE(duration_history.insert(
        tracking_odometry(10.0 + 0.1 * index, index)).accepted);
  }
  EXPECT_LE(
      duration_history.newest_stamp_ns() - duration_history.oldest_stamp_ns(),
      2100000000LL);
  EXPECT_GE(
      duration_history.newest_stamp_ns() - duration_history.oldest_stamp_ns(),
      1900000000LL);

  OdometryHistory count_history(
      OdometryHistoryConfig{2.0, 0.05, 5, 1e-3});
  for (int index = 0; index < 10; ++index) {
    ASSERT_TRUE(count_history.insert(
        tracking_odometry(20.0 + 0.01 * index, index)).accepted);
  }
  EXPECT_EQ(count_history.size(), 5U);
}

TEST(LocalizationManagerLogic, RejectsExpiredOutOfBoundsAndWideGapLookups) {
  OdometryHistory history(OdometryHistoryConfig{2.0, 0.05, 100, 1e-3});
  ASSERT_TRUE(history.insert(tracking_odometry(10.0)).accepted);
  ASSERT_TRUE(history.insert(tracking_odometry(10.1)).accepted);
  EXPECT_EQ(history.lookup(9999999999LL).reason,
      "odometry_history_target_too_old");
  EXPECT_EQ(history.lookup(10100000001LL).reason,
      "odometry_history_target_after_newest");

  OdometryHistory gap_history(OdometryHistoryConfig{2.0, 0.05, 100, 1e-3});
  ASSERT_TRUE(gap_history.insert(tracking_odometry(20.0)).accepted);
  ASSERT_TRUE(gap_history.insert(tracking_odometry(20.2)).accepted);
  EXPECT_EQ(gap_history.lookup(20100000000LL).reason,
      "odometry_history_gap_exceeds_tolerance");

  OdometryHistory off_center_history(
      OdometryHistoryConfig{2.0, 0.05, 100, 1e-3});
  ASSERT_TRUE(off_center_history.insert(tracking_odometry(21.0)).accepted);
  ASSERT_TRUE(off_center_history.insert(tracking_odometry(21.09)).accepted);
  EXPECT_EQ(off_center_history.lookup(21080000000LL).reason,
      "odometry_history_gap_exceeds_tolerance");

  OdometryHistory expired_history(
      OdometryHistoryConfig{2.0, 0.05, 100, 1e-3});
  for (int index = 0; index <= 30; ++index) {
    ASSERT_TRUE(expired_history.insert(
        tracking_odometry(30.0 + 0.1 * index)).accepted);
  }
  EXPECT_EQ(expired_history.lookup(30000000000LL).reason,
      "odometry_history_target_too_old");
}

TEST(LocalizationManagerLogic, ReverseTimeAndNonFiniteHistoryLatchClosed) {
  OdometryHistory reversed(OdometryHistoryConfig{2.0, 0.05, 100, 1e-3});
  ASSERT_TRUE(reversed.insert(tracking_odometry(10.0)).accepted);
  ASSERT_TRUE(reversed.insert(tracking_odometry(10.1)).accepted);
  const auto reverse = reversed.insert(tracking_odometry(10.05));
  EXPECT_FALSE(reverse.accepted);
  EXPECT_TRUE(reverse.requires_reinitialization);
  EXPECT_EQ(reverse.reason, "odometry_history_stamp_reversed");
  EXPECT_FALSE(reversed.lookup(10050000000LL).valid);
  EXPECT_FALSE(reversed.insert(tracking_odometry(10.2)).accepted);
  reversed.reset();
  EXPECT_TRUE(reversed.insert(tracking_odometry(10.2)).accepted);

  OdometryHistory non_finite(OdometryHistoryConfig{2.0, 0.05, 100, 1e-3});
  auto invalid = tracking_odometry(20.0);
  invalid.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  const auto rejected = non_finite.insert(invalid);
  EXPECT_TRUE(rejected.requires_reinitialization);
  EXPECT_EQ(rejected.reason, "odometry_history_non_finite");
}

TEST(LocalizationManagerLogic, CorrectionJumpRequiresExplicitReinitialization) {
  EvidenceLimits limits;
  limits.maximum_correction_translation_step_m = 0.30;
  limits.maximum_correction_rotation_step_rad = 0.2617993877991494;
  CorrectionGate gate(limits);

  geometry_msgs::msg::Transform initial;
  initial.rotation = yaw_quaternion(0.0);
  ASSERT_TRUE(gate.consider(10000000000LL, initial).valid);
  geometry_msgs::msg::Transform jump = initial;
  jump.translation.x = 0.31;
  const auto rejected = gate.consider(10100000000LL, jump);
  EXPECT_FALSE(rejected.valid);
  EXPECT_TRUE(rejected.requires_reinitialization);
  EXPECT_EQ(rejected.reason, "map_to_odom_correction_translation_jump");

  geometry_msgs::msg::Transform ordinary = initial;
  ordinary.translation.x = 0.10;
  const auto still_rejected = gate.consider(10200000000LL, ordinary);
  EXPECT_FALSE(still_rejected.valid);
  EXPECT_TRUE(still_rejected.requires_reinitialization);
  EXPECT_EQ(still_rejected.reason,
      "map_to_odom_correction_translation_jump");

  gate.reinitialize();
  const auto reinitialized = gate.consider(10300000000LL, jump);
  EXPECT_TRUE(reinitialized.valid);
  EXPECT_TRUE(reinitialized.accepted_new_correction);
}

TEST(LocalizationManagerLogic, ReversedCorrectionStampLatchesClosed) {
  CorrectionGate gate;
  geometry_msgs::msg::Transform correction;
  correction.rotation = yaw_quaternion(0.0);
  ASSERT_TRUE(gate.consider(10000000000LL, correction).valid);
  const auto reversed = gate.consider(9999999999LL, correction);
  EXPECT_FALSE(reversed.valid);
  EXPECT_TRUE(reversed.requires_reinitialization);
  EXPECT_EQ(reversed.reason, "correction_stamp_not_monotonic");
}

TEST(LocalizationManagerLogic, FiltersAcceptedCorrectionsAfterRawJumpGate) {
  EvidenceLimits limits;
  limits.maximum_correction_translation_step_m = 0.30;
  limits.maximum_correction_rotation_step_rad = 0.2617993877991494;
  CorrectionGate gate(limits, 1.0);

  geometry_msgs::msg::Transform initial;
  initial.rotation = yaw_quaternion(0.0);
  ASSERT_TRUE(gate.consider(10000000000LL, initial).valid);
  ASSERT_TRUE(gate.accepted_correction());
  EXPECT_NEAR(gate.accepted_correction()->translation.x, 0.0, 1e-12);

  geometry_msgs::msg::Transform second = initial;
  second.translation.x = 0.29;
  second.rotation = yaw_quaternion(0.20);
  ASSERT_TRUE(gate.consider(11000000000LL, second).valid);
  ASSERT_TRUE(gate.accepted_correction());
  const double alpha = 1.0 - std::exp(-1.0);
  EXPECT_NEAR(gate.accepted_correction()->translation.x, 0.29 * alpha, 1e-9);
  tf2::Quaternion filtered_rotation;
  tf2::fromMsg(gate.accepted_correction()->rotation, filtered_rotation);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(filtered_rotation).getRPY(roll, pitch, yaw);
  EXPECT_NEAR(yaw, 0.20 * alpha, 1e-9);

  // The raw-to-raw step is 0.29 m and must pass even though comparing the
  // lagged filtered output directly to this sample would exceed 0.30 m.
  geometry_msgs::msg::Transform third = second;
  third.translation.x = 0.58;
  ASSERT_TRUE(gate.consider(12000000000LL, third).valid);

  geometry_msgs::msg::Transform raw_jump = third;
  raw_jump.translation.x = 0.90;
  const auto rejected = gate.consider(13000000000LL, raw_jump);
  EXPECT_FALSE(rejected.valid);
  EXPECT_TRUE(rejected.requires_reinitialization);
  EXPECT_EQ(rejected.reason, "map_to_odom_correction_translation_jump");
}

TEST(LocalizationManagerLogic, RejectsInvalidCorrectionFilterTimeConstant) {
  EXPECT_THROW(CorrectionGate({}, -0.1), std::invalid_argument);
  EXPECT_THROW(
      CorrectionGate({}, std::numeric_limits<double>::infinity()),
      std::invalid_argument);
}

TEST(LocalizationManagerLogic, CorrectionFilterResetsWithExplicitReinitialization) {
  CorrectionGate gate({}, 1.0);
  geometry_msgs::msg::Transform first;
  first.rotation = yaw_quaternion(0.0);
  ASSERT_TRUE(gate.consider(10000000000LL, first).valid);
  geometry_msgs::msg::Transform second = first;
  second.translation.x = 0.20;
  ASSERT_TRUE(gate.consider(11000000000LL, second).valid);
  ASSERT_LT(gate.accepted_correction()->translation.x, 0.20);

  gate.reinitialize();
  ASSERT_TRUE(gate.consider(12000000000LL, second).valid);
  EXPECT_NEAR(gate.accepted_correction()->translation.x, 0.20, 1e-12);
}

TEST(LocalizationManagerLogic, PublishesComparisonForTwoAvailableBackends) {
  EvidenceLimits limits;
  limits.maximum_comparison_translation_m = 0.6;
  limits.maximum_comparison_yaw_rad = 0.3;
  const auto lidar = healthy_evidence(
      "lidar", 10.0, 0.0, 0.0, 0.0, 1.0, 2.0, 0.1);
  const auto visual = healthy_evidence(
      "visual", 10.0, 0.0, 0.0, 0.0, 1.3, 2.4, 0.3);
  const auto comparison = sunhorizon_localization_manager::compare_backends(
      lidar, visual, 10.1, {}, limits);
  EXPECT_TRUE(comparison.available);
  EXPECT_TRUE(comparison.lidar_healthy);
  EXPECT_TRUE(comparison.visual_healthy);
  EXPECT_NEAR(comparison.translation_delta_m, 0.5, 1e-9);
  EXPECT_NEAR(comparison.yaw_delta_rad, 0.2, 1e-9);
  EXPECT_TRUE(comparison.within_limits);
  EXPECT_EQ(comparison.reason, "ok");
}

TEST(LocalizationTopicContract, UsesFrozenCandidateAndCanonicalNames) {
  namespace topics = sunhorizon_localization_manager::topics;
  EXPECT_STREQ(topics::kLidarOdometryCandidate,
      "/sunhorizon/localization/lidar/odom_candidate");
  EXPECT_STREQ(topics::kLidarTrackingOdometryCandidate,
      "/sunhorizon/localization/lidar/tracking_odom_candidate");
  EXPECT_STREQ(topics::kLidarMapPoseCandidate,
      "/sunhorizon/localization/lidar/map_pose_candidate");
  EXPECT_STREQ(topics::kLidarStatus, "/sunhorizon/localization/lidar/status");
  EXPECT_STREQ(topics::kVisualOdometryCandidate,
      "/sunhorizon/localization/visual/odom_candidate");
  EXPECT_STREQ(topics::kVisualMapPoseCandidate,
      "/sunhorizon/localization/visual/map_pose_candidate");
  EXPECT_STREQ(topics::kVisualStatus, "/sunhorizon/localization/visual/status");
  EXPECT_STREQ(topics::kLivoOdometryCandidate,
      "/sunhorizon/localization/livo/odom_candidate");
  EXPECT_STREQ(topics::kLivoNdtOdometryCandidate,
      "/sunhorizon/localization/livo/ndt_odom_candidate");
  EXPECT_STREQ(topics::kLivoMapPoseCandidate,
      "/sunhorizon/localization/livo/map_pose_candidate");
  EXPECT_STREQ(topics::kLivoStatus, "/sunhorizon/localization/livo/status");
  EXPECT_STREQ(topics::kCanonicalOdometry, "/sunhorizon/localization/odom");
  EXPECT_STREQ(topics::kCanonicalStatus, "/sunhorizon/localization/status");
  EXPECT_STREQ(topics::kResetService, "/sunhorizon/localization/reset");
}

}  // namespace
