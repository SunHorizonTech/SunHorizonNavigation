#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "sunhorizon_navigation_core/localization_evidence.hpp"
#include "sunhorizon_navigation_core/localization_gate.hpp"
#include "sunhorizon_navigation_core/navigation_state_machine.hpp"
#include "sunhorizon_navigation_core/platform_evidence.hpp"
#include "sunhorizon_navigation_core/projection_grid.hpp"
#include "sunhorizon_navigation_core/rgbd_validation.hpp"
#include "sunhorizon_navigation_core/safety_gate.hpp"
#include "sunhorizon_navigation_core/visual_pose.hpp"

using namespace sunhorizon_navigation_core;

namespace {

SafetyInputs healthy_inputs() {
  SafetyInputs inputs;
  inputs.now_sec = 10.0;
  inputs.candidate_seen = true;
  inputs.candidate_stamp_sec = 9.95;
  inputs.platform_stamp_sec = 9.95;
  inputs.obstacle_coverage_seen = true;
  inputs.obstacle_coverage_stamp_sec = 9.95;
  inputs.localization_stamp_sec = 9.95;
  inputs.tf_stamp_sec = 9.95;
  inputs.navigation_status_seen = true;
  inputs.navigation_status_stamp_sec = 9.95;
  inputs.navigation_task_active = true;
  inputs.mode_allowed = true;
  inputs.ethercat_op = true;
  inputs.wkc_normal = true;
  inputs.joint_feedback_fresh = true;
  inputs.leg_joints_error_free = true;
  inputs.leg_joints_operation_enabled = true;
  inputs.motion_ready = true;
  inputs.navigation_mode_discovered = true;
  inputs.platform_fault = false;
  inputs.obstacle_coverage_healthy = true;
  inputs.forward_coverage = true;
  inputs.near_field_coverage = true;
  inputs.rear_coverage = false;
  inputs.localization_healthy = true;
  inputs.tf_continuous = true;
  return inputs;
}

SafetyLimits geometry_hold_limits() {
  SafetyLimits limits;
  limits.geometry_hold_enabled = true;
  return limits;
}

SafetyInputs geometry_hold_inputs() {
  auto inputs = healthy_inputs();
  inputs.geometry_hold_requested = true;
  // The caller has already validated a complete combined geometry decision.
  // Even all three legacy coverage booleans may be false during this hold.
  inputs.obstacle_coverage_healthy = false;
  inputs.forward_coverage = false;
  inputs.near_field_coverage = false;
  return inputs;
}

void expect_zero_output(const SafetyDecision& decision) {
  EXPECT_DOUBLE_EQ(decision.output.vx, 0.0);
  EXPECT_DOUBLE_EQ(decision.output.vy, 0.0);
  EXPECT_DOUBLE_EQ(decision.output.wz, 0.0);
}

void expect_hold_hard_failure(
    const SafetyInputs& inputs, const Twist2D& candidate,
    const std::string& expected_reason, SafetyLimits limits = geometry_hold_limits()) {
  SafetyGate gate(limits);
  const auto decision = gate.evaluate(inputs, candidate);
  EXPECT_FALSE(decision.allowed);
  EXPECT_TRUE(decision.latched);
  EXPECT_FALSE(decision.geometry_hold);
  EXPECT_EQ(decision.reason, expected_reason);
  expect_zero_output(decision);
  // Recovering the evidence or requesting Hold cannot release a hard latch.
  const auto recovered = gate.evaluate(geometry_hold_inputs(), {});
  EXPECT_TRUE(recovered.latched);
  EXPECT_FALSE(recovered.geometry_hold);
  EXPECT_EQ(recovered.reason, expected_reason);
  expect_zero_output(recovered);
}

TEST(SafetyGate, GeometryHoldRequiresBothOptInAndValidatedRequest) {
  for (bool enabled : {false, true}) {
    for (bool requested : {false, true}) {
      SCOPED_TRACE(::testing::Message() << enabled << "," << requested);
      auto limits = SafetyLimits{};
      limits.geometry_hold_enabled = enabled;
      auto inputs = geometry_hold_inputs();
      inputs.geometry_hold_requested = requested;
      const auto decision = SafetyGate(limits).evaluate(inputs, {0.1, 0.05, 0.1});
      EXPECT_FALSE(decision.allowed);
      EXPECT_EQ(decision.latched, !(enabled && requested));
      EXPECT_EQ(decision.geometry_hold, enabled && requested);
      EXPECT_EQ(decision.reason, enabled && requested ?
          "geometry_hold" : "obstacle_coverage_unhealthy");
      expect_zero_output(decision);
    }
  }
  // A request under default limits preserves the original per-field reasons.
  for (const auto& field : std::vector<std::pair<bool SafetyInputs::*, std::string>>{
           {&SafetyInputs::obstacle_coverage_healthy, "obstacle_coverage_unhealthy"},
           {&SafetyInputs::forward_coverage, "forward_obstacle_coverage_missing"},
           {&SafetyInputs::near_field_coverage, "near_field_obstacle_coverage_missing"}}) {
    auto inputs = healthy_inputs();
    inputs.geometry_hold_requested = true;
    inputs.*field.first = false;
    const auto decision = SafetyGate().evaluate(inputs, {0.1, 0.0, 0.0});
    EXPECT_TRUE(decision.latched);
    EXPECT_FALSE(decision.geometry_hold);
    EXPECT_EQ(decision.reason, field.second);
  }
  auto inputs = healthy_inputs();
  inputs.geometry_hold_requested = true;
  inputs.navigation_status_seen = false;
  EXPECT_TRUE(SafetyGate().evaluate(inputs, {}).allowed);
  inputs.candidate_seen = false;
  const auto waiting = SafetyGate().evaluate(inputs, {});
  EXPECT_FALSE(waiting.latched);
  EXPECT_FALSE(waiting.geometry_hold);
  EXPECT_EQ(waiting.reason, "waiting_for_candidate");
}

TEST(SafetyGate, GeometryHoldCannotMaskPlatformLocalizationTfOrMissingEvidence) {
  struct Failure {
    bool SafetyInputs::*field;
    bool value;
    const char* reason;
  };
  const std::vector<Failure> failures = {
      {&SafetyInputs::mode_allowed, false, "robot_mode_not_allowed"},
      {&SafetyInputs::ethercat_op, false, "ethercat_not_op"},
      {&SafetyInputs::wkc_normal, false, "ethercat_wkc_not_proven"},
      {&SafetyInputs::joint_feedback_fresh, false, "stale_joint_feedback"},
      {&SafetyInputs::leg_joints_error_free, false, "leg_joint_error"},
      {&SafetyInputs::leg_joints_operation_enabled, false, "leg_joint_not_operation_enabled"},
      {&SafetyInputs::motion_ready, false, "motion_not_ready"},
      {&SafetyInputs::navigation_mode_discovered, false, "navigation_mode_not_discovered"},
      {&SafetyInputs::platform_fault, true, "platform_fault"},
      {&SafetyInputs::obstacle_coverage_seen, false, "obstacle_coverage_missing"},
      {&SafetyInputs::localization_healthy, false, "localization_unhealthy"},
      {&SafetyInputs::tf_continuous, false, "tf_unhealthy"},
      {&SafetyInputs::candidate_seen, false, "candidate_missing_during_geometry_hold"},
      {&SafetyInputs::navigation_status_seen, false, "navigation_execution_lease_missing"},
      {&SafetyInputs::navigation_task_active, false, "navigation_execution_lease_inactive"}};
  for (const auto& failure : failures) {
    SCOPED_TRACE(failure.reason);
    auto inputs = geometry_hold_inputs();
    inputs.*failure.field = failure.value;
    // Lease checks must run even when the candidate is exactly zero.
    expect_hold_hard_failure(inputs, {}, failure.reason);
  }
  for (double waist_yaw : {-0.11, 0.11}) {
    auto inputs = geometry_hold_inputs();
    inputs.waist_yaw_rad = waist_yaw;
    expect_hold_hard_failure(inputs, {}, "waist_yaw_limit");
  }
}

TEST(SafetyGate, GeometryHoldRejectsStaleFutureMissingAndNonFiniteTimes) {
  const std::vector<std::pair<double SafetyInputs::*, std::string>> stamps = {
      {&SafetyInputs::platform_stamp_sec, "stale_platform_state"},
      {&SafetyInputs::obstacle_coverage_stamp_sec, "stale_obstacle_coverage"},
      {&SafetyInputs::localization_stamp_sec, "localization_unhealthy"},
      {&SafetyInputs::tf_stamp_sec, "tf_unhealthy"},
      {&SafetyInputs::candidate_stamp_sec, "candidate_timeout"},
      {&SafetyInputs::navigation_status_stamp_sec, "navigation_execution_lease_stale"}};
  const std::vector<double> invalid_times = {
      0.0, 9.0, 10.01, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
  for (const auto& stamp : stamps) {
    for (double time : invalid_times) {
      SCOPED_TRACE(::testing::Message() << stamp.second << "," << time);
      auto inputs = geometry_hold_inputs();
      inputs.*stamp.first = time;
      expect_hold_hard_failure(inputs, {}, stamp.second);
    }
  }
  for (double time : {std::numeric_limits<double>::quiet_NaN(),
           std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
    auto inputs = geometry_hold_inputs();
    inputs.now_sec = time;
    expect_hold_hard_failure(inputs, {}, "stale_platform_state");
  }
}

TEST(SafetyGate, NonFiniteWaistYawLatchesWithAndWithoutGeometryHold) {
  for (bool hold : {false, true}) {
    for (double yaw : {std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
      SCOPED_TRACE(::testing::Message() << hold << "," << yaw);
      SafetyGate gate(hold ? geometry_hold_limits() : SafetyLimits{});
      auto inputs = hold ? geometry_hold_inputs() : healthy_inputs();
      inputs.waist_yaw_rad = yaw;
      const auto decision = gate.evaluate(inputs, {});
      EXPECT_TRUE(decision.latched);
      EXPECT_FALSE(decision.allowed);
      EXPECT_FALSE(decision.geometry_hold);
      EXPECT_EQ(decision.reason, "non_finite_waist_yaw");
      expect_zero_output(decision);
      std::string reason;
      inputs.geometry_hold_requested = false;
      EXPECT_FALSE(gate.reset(inputs, {}, &reason));
      EXPECT_EQ(reason, "non_finite_waist_yaw");
    }
  }
}

TEST(SafetyGate, GeometryHoldRejectsIllegalCommandsInsteadOfHidingThemWithZero) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const std::vector<std::pair<Twist2D, std::string>> invalid_commands = {
      {{-0.01, 0.0, 0.0}, "autonomous_reverse_disabled"},
      {{0.21, 0.0, 0.0}, "forward_velocity_limit"},
      {{0.0, 0.11, 0.0}, "lateral_velocity_limit"},
      {{0.0, -0.11, 0.0}, "lateral_velocity_limit"},
      {{0.0, 0.0, 0.31}, "yaw_velocity_limit"},
      {{0.0, 0.0, -0.31}, "yaw_velocity_limit"},
      {{nan, 0.0, 0.0}, "non_finite_command"},
      {{0.0, nan, 0.0}, "non_finite_command"},
      {{0.0, 0.0, nan}, "non_finite_command"},
      {{inf, 0.0, 0.0}, "non_finite_command"},
      {{-inf, 0.0, 0.0}, "autonomous_reverse_disabled"},
      {{0.0, inf, 0.0}, "non_finite_command"},
      {{0.0, -inf, 0.0}, "non_finite_command"},
      {{0.0, 0.0, inf}, "non_finite_command"},
      {{0.0, 0.0, -inf}, "non_finite_command"}};
  for (const auto& command : invalid_commands) {
    SCOPED_TRACE(command.second);
    expect_hold_hard_failure(geometry_hold_inputs(), command.first, command.second);
  }
  auto limits = geometry_hold_limits();
  limits.max_vy = 0.0;
  limits.max_wz = 0.0;
  expect_hold_hard_failure(
      geometry_hold_inputs(), {0.01, 0.001, 0.0}, "lateral_axis_disabled", limits);
  expect_hold_hard_failure(
      geometry_hold_inputs(), {0.01, 0.0, 0.001}, "yaw_axis_disabled", limits);
}

TEST(SafetyGate, GeometryHoldStopsImmediatelyAndClearSlewsFromZero) {
  SafetyGate gate(geometry_hold_limits());
  auto inputs = healthy_inputs();
  ASSERT_TRUE(gate.reset(inputs, {}, nullptr));
  inputs.now_sec = 10.05;
  const Twist2D candidate{0.1, 0.05, 0.1};
  const auto moving = gate.evaluate(inputs, candidate);
  ASSERT_TRUE(moving.allowed);
  ASSERT_GT(moving.output.vx, 0.0);
  ASSERT_GT(moving.output.vy, 0.0);
  ASSERT_GT(moving.output.wz, 0.0);

  auto hold = geometry_hold_inputs();
  hold.now_sec = 10.10;
  for (double time : {10.10, 10.15}) {
    hold.now_sec = time;
    const auto stopped = gate.evaluate(hold, candidate);
    EXPECT_FALSE(stopped.allowed);
    EXPECT_FALSE(stopped.latched);
    EXPECT_TRUE(stopped.geometry_hold);
    EXPECT_EQ(stopped.reason, "geometry_hold");
    expect_zero_output(stopped);
  }
  // The caller owns Clear qualification; the pure gate has no Clear streak.
  inputs.now_sec = 10.20;
  inputs.candidate_stamp_sec = 10.20;
  const auto resumed = gate.evaluate(inputs, candidate);
  EXPECT_TRUE(resumed.allowed);
  EXPECT_FALSE(resumed.geometry_hold);
  EXPECT_NEAR(resumed.output.vx, 0.01, 1e-12);
  EXPECT_NEAR(resumed.output.vy, 0.01, 1e-12);
  EXPECT_NEAR(resumed.output.wz, 0.02, 1e-12);
}

TEST(SafetyGate, GeometryHoldKeepsZeroDuringClearQualificationAndForceLatchWins) {
  SafetyGate gate(geometry_hold_limits());
  auto inputs = healthy_inputs();
  // Legacy coverage may already be healthy while the caller is qualifying
  // fresh Clear evidence. It must still keep zero and refresh the lease.
  inputs.geometry_hold_requested = true;
  const auto hold = gate.evaluate(inputs, {});
  EXPECT_FALSE(hold.allowed);
  EXPECT_FALSE(hold.latched);
  EXPECT_TRUE(hold.geometry_hold);
  EXPECT_EQ(hold.reason, "geometry_hold");
  expect_zero_output(hold);
  gate.force_latch("collision_stop");
  const auto stopped = gate.evaluate(inputs, {});
  EXPECT_TRUE(stopped.latched);
  EXPECT_FALSE(stopped.geometry_hold);
  EXPECT_EQ(stopped.reason, "collision_stop");
  expect_zero_output(stopped);

  SafetyGate stale_lease_gate(geometry_hold_limits());
  ASSERT_TRUE(stale_lease_gate.evaluate(inputs, {}).geometry_hold);
  inputs.navigation_status_stamp_sec = 9.0;
  const auto expired = stale_lease_gate.evaluate(inputs, {});
  EXPECT_TRUE(expired.latched);
  EXPECT_FALSE(expired.geometry_hold);
  EXPECT_EQ(expired.reason, "navigation_execution_lease_stale");
  expect_zero_output(expired);
}

TEST(SafetyGate, GeometryHoldCannotClearOrReplaceAnExistingLatch) {
  for (bool enabled : {false, true}) {
    auto limits = SafetyLimits{};
    limits.geometry_hold_enabled = enabled;
    SafetyGate gate(limits);
    gate.force_latch("operator_stop");
    auto hold = geometry_hold_inputs();
    const auto decision = gate.evaluate(hold, {});
    EXPECT_TRUE(decision.latched);
    EXPECT_FALSE(decision.geometry_hold);
    EXPECT_EQ(decision.reason, "operator_stop");
    expect_zero_output(decision);
    std::string reason;
    EXPECT_FALSE(gate.reset(hold, {}, &reason));
    EXPECT_EQ(reason, "geometry_hold_must_clear_before_reset");
    EXPECT_TRUE(gate.latched());
    EXPECT_EQ(gate.latch_reason(), "operator_stop");
    // Reject even a request whose legacy coverage is otherwise healthy.
    hold = healthy_inputs();
    hold.geometry_hold_requested = true;
    EXPECT_FALSE(gate.reset(hold, {}, nullptr));
    EXPECT_TRUE(gate.latched());
    hold.geometry_hold_requested = false;
    hold.near_field_coverage = false;
    EXPECT_FALSE(gate.reset(hold, {}, &reason));
    EXPECT_EQ(reason, "near_field_obstacle_coverage_missing");
    hold.near_field_coverage = true;
    EXPECT_TRUE(gate.reset(hold, {}, &reason));
    EXPECT_FALSE(gate.latched());
  }
}

TEST(SafetyGate, LatchesOnMissingObstacleCoverage) {
  SafetyGate gate;
  auto inputs = healthy_inputs();
  inputs.obstacle_coverage_seen = false;
  const auto decision = gate.evaluate(inputs, {0.1, 0.0, 0.0});
  EXPECT_FALSE(decision.allowed);
  EXPECT_TRUE(decision.latched);
  EXPECT_EQ(decision.reason, "obstacle_coverage_missing");
  EXPECT_DOUBLE_EQ(decision.output.vx, 0.0);
}

TEST(SafetyGate, LatchesOnStaleObstacleCoverage) {
  SafetyGate gate;
  auto inputs = healthy_inputs();
  inputs.obstacle_coverage_stamp_sec = 9.0;
  const auto decision = gate.evaluate(inputs, {0.1, 0.0, 0.0});
  EXPECT_FALSE(decision.allowed);
  EXPECT_TRUE(decision.latched);
  EXPECT_EQ(decision.reason, "stale_obstacle_coverage");
  EXPECT_DOUBLE_EQ(decision.output.vx, 0.0);
}

TEST(SafetyGate, RejectsReverse) {
  SafetyGate gate;
  const auto decision = gate.evaluate(healthy_inputs(), {-0.1, 0.0, 0.0});
  EXPECT_FALSE(decision.allowed);
  EXPECT_EQ(decision.reason, "autonomous_reverse_disabled");
}

TEST(SafetyGate, RequiresFreshNavigationExecutionLeaseForMotion) {
  auto inputs = healthy_inputs();
  inputs.navigation_status_seen = false;
  EXPECT_EQ(
      SafetyGate().evaluate(inputs, {0.05, 0.0, 0.0}).reason,
      "navigation_execution_lease_missing");

  inputs = healthy_inputs();
  inputs.navigation_status_stamp_sec = 9.0;
  EXPECT_EQ(
      SafetyGate().evaluate(inputs, {0.05, 0.0, 0.0}).reason,
      "navigation_execution_lease_stale");

  inputs = healthy_inputs();
  inputs.navigation_task_active = false;
  EXPECT_EQ(
      SafetyGate().evaluate(inputs, {0.05, 0.0, 0.0}).reason,
      "navigation_execution_lease_inactive");

  inputs.navigation_status_seen = false;
  EXPECT_TRUE(SafetyGate().evaluate(inputs, {}).allowed);
}

TEST(SafetyGate, RejectsCommandsOutsideConfiguredEnvelopeInsteadOfClampingAxes) {
  SafetyLimits straight_only;
  straight_only.max_vx = 0.05;
  straight_only.max_vy = 0.0;
  straight_only.max_wz = 0.0;

  EXPECT_EQ(
      SafetyGate(straight_only).evaluate(
          healthy_inputs(), {0.051, 0.0, 0.0}).reason,
      "forward_velocity_limit");
  EXPECT_EQ(
      SafetyGate(straight_only).evaluate(
          healthy_inputs(), {0.04, 0.001, 0.0}).reason,
      "lateral_axis_disabled");
  EXPECT_EQ(
      SafetyGate(straight_only).evaluate(
          healthy_inputs(), {0.04, 0.0, 0.001}).reason,
      "yaw_axis_disabled");
}

TEST(SafetyGate, LatchesOnUnhealthyObstacleCoverage) {
  SafetyGate gate;
  auto inputs = healthy_inputs();
  inputs.obstacle_coverage_healthy = false;
  const auto decision = gate.evaluate(inputs, {0.1, 0.0, 0.0});
  EXPECT_FALSE(decision.allowed);
  EXPECT_TRUE(decision.latched);
  EXPECT_EQ(decision.reason, "obstacle_coverage_unhealthy");
  EXPECT_DOUBLE_EQ(decision.output.vx, 0.0);
}

TEST(SafetyGate, RequiresForwardAndNearFieldCoverage) {
  SafetyGate gate;
  auto inputs = healthy_inputs();
  inputs.forward_coverage = false;
  EXPECT_EQ(
      gate.evaluate(inputs, {0.1, 0.0, 0.0}).reason,
      "forward_obstacle_coverage_missing");

  SafetyGate near_field_gate;
  inputs = healthy_inputs();
  inputs.near_field_coverage = false;
  EXPECT_EQ(
      near_field_gate.evaluate(inputs, {0.1, 0.0, 0.0}).reason,
      "near_field_obstacle_coverage_missing");
}

TEST(SafetyGate, ResetStillRejectsStaleOrDiscontinuousTf) {
  std::string reason;
  auto inputs = healthy_inputs();
  inputs.tf_stamp_sec = 9.0;
  EXPECT_FALSE(SafetyGate().reset(inputs, {}, &reason));
  EXPECT_EQ(reason, "tf_unhealthy");

  inputs = healthy_inputs();
  inputs.tf_continuous = false;
  EXPECT_FALSE(SafetyGate().reset(inputs, {}, &reason));
  EXPECT_EQ(reason, "tf_unhealthy");

  inputs = healthy_inputs();
  inputs.tf_stamp_sec = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(SafetyGate().reset(inputs, {}, &reason));
  EXPECT_EQ(reason, "tf_unhealthy");
}

TEST(RgbdValidation, AcceptsSynchronizedAlignedPair) {
  RgbdPairValidator validator;
  const ImageMetadata color{10.000, 1280, 720, "rgb8", "color_optical"};
  const ImageMetadata depth{10.004, 1280, 720, "16UC1", "color_optical"};
  const CameraIntrinsics info{1280, 720, 607.0, 607.0, 639.0, 368.0, "color_optical"};
  const auto decision = validator.evaluate(color, depth, info, info, true);
  EXPECT_TRUE(decision.stream_accepted);
  EXPECT_TRUE(decision.pointcloud_allowed);
  EXPECT_EQ(decision.reason, "ok");
}

TEST(RgbdValidation, BlocksPointcloudUntilAlignmentAccepted) {
  RgbdPairValidator validator;
  const ImageMetadata color{10.000, 1280, 720, "rgb8", "color_optical"};
  const ImageMetadata depth{10.001, 1280, 720, "16UC1", "color_optical"};
  const CameraIntrinsics color_info{
      1280, 720, 607.0, 607.0, 639.0, 368.0, "color_optical"};
  const CameraIntrinsics vendor_depth_info{
      1280, 720, 627.0, 627.0, 637.5, 349.5, "color_optical"};
  const auto decision = validator.evaluate(
      color, depth, color_info, vendor_depth_info, false);
  EXPECT_TRUE(decision.stream_accepted);
  EXPECT_FALSE(decision.pointcloud_allowed);
  EXPECT_EQ(decision.reason, "alignment_calibration_not_accepted");
}

TEST(RgbdValidation, RejectsLateAndNonMonotonicPairs) {
  RgbdPairValidator validator;
  const CameraIntrinsics info{640, 480, 500.0, 500.0, 320.0, 240.0, "optical"};
  EXPECT_EQ(validator.evaluate(
      {10.0, 640, 480, "rgb8", "optical"},
      {10.006, 640, 480, "16UC1", "optical"}, info, info, true).reason,
      "pair_delta_exceeded");
  EXPECT_TRUE(validator.evaluate(
      {11.0, 640, 480, "rgb8", "optical"},
      {11.001, 640, 480, "16UC1", "optical"}, info, info, true).stream_accepted);
  EXPECT_EQ(validator.evaluate(
      {10.5, 640, 480, "rgb8", "optical"},
      {11.5, 640, 480, "16UC1", "optical"}, info, info, true).reason,
      "non_monotonic_timestamp");
}

TEST(RgbdValidation, ChecksDepthScaleTolerance) {
  EXPECT_TRUE(depth_sample_within_tolerance(1000, 1.0));
  EXPECT_TRUE(depth_sample_within_tolerance(2020, 2.0));
  EXPECT_FALSE(depth_sample_within_tolerance(950, 1.0));
  EXPECT_FALSE(depth_sample_within_tolerance(0, 1.0));
}

TEST(PlatformEvidence, ParsesLegJoints) {
  const auto joints = parse_joint_info_json(
      R"({"source_stamp_ns":123456789,"joints":[{"index":0,"error_code":0,"status_word":567},{"index":1,"error_code":0,"status_word":39}]})",
      {0, 1});
  EXPECT_TRUE(joints.parsed);
  EXPECT_TRUE(joints.complete);
  EXPECT_TRUE(joints.error_free);
  EXPECT_TRUE(joints.operation_enabled);
  EXPECT_EQ(joints.source_stamp_ns, 123456789);
}

TEST(PlatformEvidence, RequiresExactWalkMode) {
  EXPECT_TRUE(robot_mode_allowed("WALK", "WALK"));
  EXPECT_FALSE(robot_mode_allowed("STAND", "WALK"));
  EXPECT_FALSE(robot_mode_allowed("walk", "WALK"));
  EXPECT_FALSE(robot_mode_allowed("WALK", ""));
}

TEST(PlatformEvidence, FailsClosedOnMalformedOrUnsafeEvidence) {
  const auto joints = parse_joint_info_json(
      R"({"joints":[{"id":0,"error_code":4,"statusword":567}]})", {0, 1});
  EXPECT_FALSE(joints.complete);
  EXPECT_FALSE(joints.error_free);
  EXPECT_FALSE(joints.operation_enabled);
  const auto ethercat = parse_ethercat_text(
      "Master0 Active: yes\nSlave 0: OP\nDomain0 WKC 2/2", 2);
  EXPECT_TRUE(ethercat.parsed);
  EXPECT_TRUE(ethercat.master_active);
  EXPECT_TRUE(ethercat.slave_op);
  EXPECT_TRUE(ethercat.wkc_normal);
  EXPECT_FALSE(parse_ethercat_text("Master0 Active: yes\nSlave 0: OP\nWKC 1/2", 2).wkc_normal);

  const auto navigation = parse_motion_state(
      "State: STAND, Mode: NAVIGATION, Remote Control State: Exit");
  EXPECT_TRUE(navigation.parsed);
  EXPECT_TRUE(navigation.state_ready);
  EXPECT_TRUE(navigation.stand_state_active);
  EXPECT_FALSE(navigation.walking_state_active);
  EXPECT_EQ(navigation.state_name, "STAND");
  EXPECT_TRUE(navigation.navigation_control_active);
  EXPECT_TRUE(navigation.remote_control_exited);
  EXPECT_EQ(navigation.reason, "ok");
  const auto remote = parse_motion_state(
      "State: STAND, Mode: IDLE_MODE, Remote Control State: Running");
  EXPECT_FALSE(remote.navigation_control_active);
  EXPECT_FALSE(remote.remote_control_exited);
  EXPECT_EQ(remote.reason, "navigation_control_not_active");
}

TEST(PlatformEvidence, AllowsOnlyExplicitNonBlockingLegErrorCodes) {
  const auto json =
      R"({"joints":[{"index":0,"error_code":14340,"statusword":567},{"index":1,"error_code":0,"statusword":567}]})";
  const auto default_policy = parse_joint_info_json(json, {0, 1});
  EXPECT_FALSE(default_policy.error_free);
  EXPECT_EQ(default_policy.reason, "leg_joint_error");

  const auto robot_a_policy = parse_joint_info_json(json, {0, 1}, {0x3804U});
  EXPECT_TRUE(robot_a_policy.complete);
  EXPECT_TRUE(robot_a_policy.error_free);
  EXPECT_TRUE(robot_a_policy.operation_enabled);
  EXPECT_EQ(robot_a_policy.reason, "ok_non_blocking_leg_error_code");

  const auto different_error = parse_joint_info_json(
      R"({"joints":[{"index":0,"error_code":14341,"statusword":567},{"index":1,"error_code":0,"statusword":567}]})",
      {0, 1}, {0x3804U});
  EXPECT_FALSE(different_error.error_free);
  EXPECT_EQ(different_error.reason, "leg_joint_error");
}

TEST(PlatformEvidence, ParsesExactStandAndWalkingStateFields) {
  const auto walking = parse_motion_state(
      "State: WALKING; Mode: NAVIGATION; Remote Control State: Exit");
  EXPECT_TRUE(walking.parsed);
  EXPECT_TRUE(walking.state_ready);
  EXPECT_FALSE(walking.stand_state_active);
  EXPECT_TRUE(walking.walking_state_active);
  EXPECT_EQ(walking.state_name, "WALKING");
  EXPECT_EQ(walking.reason, "ok");

  for (const auto& state : {"STANDING", "STANDX", "NOT_WALKING", "FALLING"}) {
    const auto unsupported = parse_motion_state(
        std::string("State: ") + state +
        ", Mode: NAVIGATION, Remote Control State: Exit");
    EXPECT_TRUE(unsupported.parsed);
    EXPECT_FALSE(unsupported.state_ready);
    EXPECT_EQ(unsupported.reason, std::string("unsupported_motion_state:") + state);
  }
  const auto duplicate = parse_motion_state(
      "State: STAND, State: WALKING, Mode: NAVIGATION, "
      "Remote Control State: Exit");
  EXPECT_FALSE(duplicate.parsed);
  EXPECT_EQ(duplicate.reason, "motion_state_malformed");
}

TEST(PlatformEvidence, RequiresHealthyStandBeforeWalking) {
  MotionStateGate gate;
  const auto stand = parse_motion_state(
      "State: STAND, Mode: NAVIGATION, Remote Control State: Exit");
  const auto walking = parse_motion_state(
      "State: WALKING, Mode: NAVIGATION, Remote Control State: Exit");

  auto decision = gate.evaluate(walking, true, true);
  EXPECT_FALSE(decision.ready);
  EXPECT_FALSE(decision.armed);
  EXPECT_EQ(decision.reason, "walking_without_prestart_stand");

  decision = gate.evaluate(stand, true, true);
  EXPECT_TRUE(decision.ready);
  EXPECT_TRUE(decision.armed);
  EXPECT_FALSE(decision.walking);
  EXPECT_EQ(decision.reason, "stand_ready");

  decision = gate.evaluate(walking, true, true);
  EXPECT_TRUE(decision.ready);
  EXPECT_TRUE(decision.armed);
  EXPECT_TRUE(decision.walking);
  EXPECT_EQ(decision.reason, "walking_after_stand");
  EXPECT_TRUE(gate.evaluate(walking, true, true).ready);

  decision = gate.evaluate(stand, true, true);
  EXPECT_TRUE(decision.ready);
  EXPECT_FALSE(decision.walking);
  EXPECT_TRUE(gate.evaluate(walking, true, true).ready);
}

TEST(PlatformEvidence, ClearsStandArmOnEveryUnsafeTransition) {
  const auto stand = parse_motion_state(
      "State: STAND, Mode: NAVIGATION, Remote Control State: Exit");
  const auto walking = parse_motion_state(
      "State: WALKING, Mode: NAVIGATION, Remote Control State: Exit");
  const auto falling = parse_motion_state(
      "State: FALLING, Mode: NAVIGATION, Remote Control State: Exit");
  const auto remote = parse_motion_state(
      "State: WALKING, Mode: NAVIGATION, Remote Control State: Running");
  const auto idle = parse_motion_state(
      "State: WALKING, Mode: IDLE_MODE, Remote Control State: Exit");

  MotionStateGate gate;
  ASSERT_TRUE(gate.evaluate(stand, true, true).ready);
  EXPECT_EQ(gate.evaluate(walking, false, true).reason, "motion_state_stale");
  EXPECT_EQ(
      gate.evaluate(walking, true, true).reason,
      "walking_without_prestart_stand");

  ASSERT_TRUE(gate.evaluate(stand, true, true).ready);
  EXPECT_EQ(
      gate.evaluate(walking, true, false).reason,
      "platform_prerequisite_not_ready");
  EXPECT_EQ(
      gate.evaluate(walking, true, true).reason,
      "walking_without_prestart_stand");

  ASSERT_TRUE(gate.evaluate(stand, true, true).ready);
  EXPECT_EQ(gate.evaluate(falling, true, true).reason, "unsupported_motion_state:FALLING");
  EXPECT_EQ(
      gate.evaluate(walking, true, true).reason,
      "walking_without_prestart_stand");

  ASSERT_TRUE(gate.evaluate(stand, true, true).ready);
  EXPECT_EQ(gate.evaluate(remote, true, true).reason, "remote_control_not_exited");
  EXPECT_EQ(
      gate.evaluate(walking, true, true).reason,
      "walking_without_prestart_stand");

  ASSERT_TRUE(gate.evaluate(stand, true, true).ready);
  EXPECT_EQ(gate.evaluate(idle, true, true).reason, "navigation_control_not_active");
  EXPECT_EQ(
      gate.evaluate(walking, true, true).reason,
      "walking_without_prestart_stand");
}

TEST(VisualPose, DecomposesPlanarAndResidualTransforms) {
  const auto split = decompose_visual_pose({1.0, 2.0, 0.8, 0.1, -0.2, 0.3});
  EXPECT_DOUBLE_EQ(split.odom_to_base_footprint.x, 1.0);
  EXPECT_DOUBLE_EQ(split.odom_to_base_footprint.yaw, 0.3);
  EXPECT_DOUBLE_EQ(split.odom_to_base_footprint.z, 0.0);
  EXPECT_DOUBLE_EQ(split.base_footprint_to_base_link.z, 0.8);
  EXPECT_DOUBLE_EQ(split.base_footprint_to_base_link.roll, 0.1);
  EXPECT_DOUBLE_EQ(split.base_footprint_to_base_link.pitch, -0.2);
}

TEST(VisualPose, RejectsJumpAndTimeRegression) {
  VisualPoseGate gate;
  EXPECT_TRUE(gate.evaluate(10.1, 10.0, {}).accepted);
  EXPECT_EQ(gate.evaluate(10.2, 10.1, {1.0, 0.0, 0.0, 0.0, 0.0, 0.0}).reason,
            "visual_translation_jump");
  EXPECT_EQ(gate.evaluate(10.1, 9.9, {}).reason, "non_monotonic_visual_pose");
}

TEST(SafetyGate, RequiresHealthyZeroCommandToResetLatch) {
  SafetyGate gate;
  auto failed = healthy_inputs();
  failed.localization_healthy = false;
  EXPECT_TRUE(gate.evaluate(failed, {0.1, 0.0, 0.0}).latched);

  auto recovered = healthy_inputs();
  EXPECT_TRUE(gate.evaluate(recovered, {0.0, 0.0, 0.0}).latched);
  std::string reason;
  EXPECT_FALSE(gate.reset(recovered, {0.1, 0.0, 0.0}, &reason));
  EXPECT_EQ(reason, "candidate_must_be_zero_before_reset");
  EXPECT_TRUE(gate.reset(recovered, {0.0, 0.0, 0.0}, &reason));
  EXPECT_EQ(reason, "reset");
  EXPECT_TRUE(gate.evaluate(recovered, {0.0, 0.0, 0.0}).allowed);
}

TEST(SafetyGate, PlatformFeedbackLossStillStopsAndRequiresExplicitReset) {
  // No PMU telemetry participates in navigation authority. Actual platform
  // faults must still stop immediately and never resume on recovery alone.
  const std::vector<std::pair<bool SafetyInputs::*, std::string>> failures = {
      {&SafetyInputs::ethercat_op, "ethercat_not_op"},
      {&SafetyInputs::wkc_normal, "ethercat_wkc_not_proven"},
      {&SafetyInputs::joint_feedback_fresh, "stale_joint_feedback"},
      {&SafetyInputs::leg_joints_error_free, "leg_joint_error"},
      {&SafetyInputs::leg_joints_operation_enabled, "leg_joint_not_operation_enabled"},
      {&SafetyInputs::motion_ready, "motion_not_ready"}};
  for (const auto & failure : failures) {
    SafetyGate gate;
    auto inputs = healthy_inputs();
    ASSERT_TRUE(gate.reset(inputs, {}, nullptr));
    ASSERT_TRUE(gate.evaluate(inputs, {.05, 0., 0.}).allowed);
    inputs.*failure.first = false;
    const auto stopped = gate.evaluate(inputs, {.05, 0., 0.});
    EXPECT_FALSE(stopped.allowed);
    EXPECT_TRUE(stopped.latched);
    EXPECT_EQ(stopped.reason, failure.second);
    EXPECT_DOUBLE_EQ(stopped.output.vx, 0.);
    EXPECT_FALSE(gate.reset(inputs, {}, nullptr));
    inputs = healthy_inputs();
    EXPECT_TRUE(gate.evaluate(inputs, {.05, 0., 0.}).latched);
    EXPECT_FALSE(gate.reset(inputs, {.05, 0., 0.}, nullptr));
    EXPECT_TRUE(gate.reset(inputs, {}, nullptr));
    EXPECT_TRUE(gate.evaluate(inputs, {.05, 0., 0.}).allowed);
  }
}

TEST(SafetyGate, AppliesZeroWithoutSlewDelay) {
  SafetyGate gate;
  auto inputs = healthy_inputs();
  EXPECT_TRUE(gate.reset(inputs, {}, nullptr));

  inputs.now_sec = 10.1;
  inputs.candidate_stamp_sec = 10.1;
  EXPECT_TRUE(gate.evaluate(inputs, {0.1, 0.0, 0.0}).allowed);
  inputs.now_sec = 10.2;
  inputs.candidate_stamp_sec = 10.2;
  const auto stop = gate.evaluate(inputs, {});
  EXPECT_TRUE(stop.allowed);
  EXPECT_FALSE(stop.latched);
  EXPECT_DOUBLE_EQ(stop.output.vx, 0.0);
  EXPECT_DOUBLE_EQ(stop.output.vy, 0.0);
  EXPECT_DOUBLE_EQ(stop.output.wz, 0.0);
}

TEST(SafetyGate, DetectsCollisionMonitorCommandReduction) {
  EXPECT_TRUE(command_reduction_exceeds(
      {0.10, 0.0, 0.20}, {0.05, 0.0, 0.10}, 1e-4));
  EXPECT_TRUE(command_reduction_exceeds(
      {0.10, 0.0, 0.0}, {0.0, 0.0, 0.0}, 1e-4));
  EXPECT_FALSE(command_reduction_exceeds(
      {0.10, 0.0, 0.20}, {0.10, 0.0, 0.20}, 1e-4));
  EXPECT_FALSE(command_reduction_exceeds(
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 1e-4));
}

TEST(SafetyGate, CollisionAuditSeparatesApproachFromImmediateStopFaults) {
  EXPECT_EQ(
      parse_collision_audit_frame_id("collision_clear"),
      CollisionAuditClass::kClear);
  EXPECT_EQ(
      parse_collision_audit_frame_id("collision_approach_slowdown"),
      CollisionAuditClass::kApproachSlowdown);
  EXPECT_EQ(
      parse_collision_audit_frame_id("collision_stop"),
      CollisionAuditClass::kStop);
  EXPECT_EQ(
      parse_collision_audit_frame_id("collision_source_invalid"),
      CollisionAuditClass::kSourceInvalid);
  EXPECT_EQ(
      parse_collision_audit_frame_id("collision_intervention"),
      CollisionAuditClass::kInvalid);
  EXPECT_FALSE(collision_audit_requires_immediate_latch(
      CollisionAuditClass::kClear));
  EXPECT_FALSE(collision_audit_requires_immediate_latch(
      CollisionAuditClass::kApproachSlowdown));
  EXPECT_TRUE(collision_audit_requires_immediate_latch(
      CollisionAuditClass::kStop));
  EXPECT_TRUE(collision_audit_requires_immediate_latch(
      CollisionAuditClass::kSourceInvalid));
  EXPECT_FALSE(collision_audit_requires_immediate_latch(
      CollisionAuditClass::kInvalid));
}

TEST(SafetyGate, CollisionInterventionRequiresTwoFreshCandidateEvents) {
  CollisionInterventionGate gate;
  EXPECT_FALSE(gate.observe(true, 10.00, 0.25));
  EXPECT_EQ(gate.streak(), 1U);
  EXPECT_TRUE(gate.observe(true, 10.05, 0.25));

  gate.reset();
  EXPECT_FALSE(gate.observe(true, 11.00, 0.25));
  EXPECT_FALSE(gate.observe(false, 11.05, 0.25));
  EXPECT_EQ(gate.streak(), 0U);
  EXPECT_FALSE(gate.observe(true, 11.10, 0.25));

  gate.reset();
  EXPECT_FALSE(gate.observe(true, 12.00, 0.25));
  EXPECT_FALSE(gate.observe(true, 12.30, 0.25));
  EXPECT_EQ(gate.streak(), 1U);
}

TEST(SafetyGate, NonFiniteClockFailsClosed) {
  auto inputs = healthy_inputs();
  inputs.now_sec = std::numeric_limits<double>::quiet_NaN();
  SafetyGate gate;
  const auto decision = gate.evaluate(inputs, {});
  EXPECT_TRUE(decision.latched);
  EXPECT_EQ(decision.reason, "stale_platform_state");
}

TEST(SafetyGate, AllowsDisabledAxesAndRejectsInvalidCopiedLimits) {
  SafetyLimits straight_only;
  straight_only.max_vy = 0.0;
  straight_only.max_ay = 0.0;
  straight_only.max_wz = 0.0;
  straight_only.max_aw = 0.0;
  EXPECT_NO_THROW({ SafetyGate gate(straight_only); });

  SafetyLimits limits;
  limits.platform_timeout_sec = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((void)SafetyGate{limits}, std::invalid_argument);

  limits = {};
  limits.max_vx = 0.05;
  limits.max_ax = 0.0;
  EXPECT_THROW((void)SafetyGate{limits}, std::invalid_argument);

  limits = {};
  limits.max_waist_yaw_rad = -0.01;
  EXPECT_THROW((void)SafetyGate{limits}, std::invalid_argument);
}

TEST(LocalizationGate, RejectsPoseJump) {
  LocalizationGate gate;
  LocalizationMeasurement first;
  first.now_sec = 10.0;
  first.stamp_sec = 9.9;
  first.converged = true;
  first.score = 3.0;
  first.iterations = 10;
  first.position_covariance = 0.01;
  first.yaw_covariance = 0.001;
  EXPECT_TRUE(gate.evaluate(first).accepted);
  auto second = first;
  second.pose.x = 2.0;
  EXPECT_EQ(gate.evaluate(second).reason, "position_jump");
}

TEST(LocalizationEvidenceGate, RequiresSynchronizedGeometricMatch) {
  LocalizationEvidenceGate gate;
  EXPECT_EQ(gate.evaluate({10.0, 10.0, 0, 0, 0}).reason,
            "localization_match_evidence_missing");
  EXPECT_EQ(gate.evaluate({10.0, 10.006, 1, 0, 0}).reason,
            "localization_evidence_stamp_mismatch");
  EXPECT_TRUE(gate.evaluate({10.0, 10.004, 0, 27, 0}).accepted);
  EXPECT_TRUE(gate.evaluate({10.0, 10.0, 0, 0, 8}).accepted);
}

TEST(StateMachine, EnforcesLifecycle) {
  NavigationStateMachine machine;
  EXPECT_TRUE(machine.configure().accepted);
  EXPECT_TRUE(machine.start_mapping("session-1").accepted);
  EXPECT_FALSE(machine.start_navigation("task-1").accepted);
  EXPECT_TRUE(machine.stop_mapping("session-1").accepted);
  EXPECT_TRUE(machine.save_map("map-1").accepted);
  EXPECT_TRUE(machine.load_map("map-1").accepted);
  EXPECT_TRUE(machine.localization_result(true, "").accepted);
  EXPECT_TRUE(machine.start_navigation("task-1").accepted);
  EXPECT_TRUE(machine.cancel("task-1").accepted);
}

TEST(StateMachine, RejectedStartupLocalizationCanBeReseeded) {
  NavigationStateMachine machine;
  ASSERT_TRUE(machine.configure().accepted);
  ASSERT_TRUE(machine.load_map("map-1").accepted);

  const auto rejected = machine.localization_result(false, "localization_lost");
  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(machine.state(), NavigationState::kLocalizing);
  EXPECT_EQ(machine.map_id(), "map-1");

  EXPECT_TRUE(machine.localization_result(true, "").accepted);
  EXPECT_EQ(machine.state(), NavigationState::kReady);
}

TEST(StateMachine, LocalizationLossAfterReadyRemainsFaulted) {
  NavigationStateMachine machine;
  ASSERT_TRUE(machine.configure().accepted);
  ASSERT_TRUE(machine.load_map("map-1").accepted);
  ASSERT_TRUE(machine.localization_result(true, "").accepted);

  const auto lost = machine.localization_result(false, "localization_lost");
  EXPECT_FALSE(lost.accepted);
  EXPECT_EQ(machine.state(), NavigationState::kFault);
}

TEST(StateMachine, TerminalResultIsBoundToTheActiveTask) {
  NavigationStateMachine machine;
  ASSERT_TRUE(machine.configure().accepted);
  ASSERT_TRUE(machine.load_map("map-1").accepted);
  ASSERT_TRUE(machine.localization_result(true, "").accepted);
  ASSERT_TRUE(machine.start_navigation("task-1").accepted);

  const auto wrong_task = machine.finish_navigation(
      "stale-task", true, "stale success");
  EXPECT_FALSE(wrong_task.accepted);
  EXPECT_EQ(machine.state(), NavigationState::kNavigating);
  EXPECT_EQ(machine.task_id(), "task-1");

  const auto failed = machine.finish_navigation(
      "task-1", false, "nav2_navigation_failed");
  EXPECT_TRUE(failed.accepted);
  EXPECT_EQ(failed.reason, "nav2_navigation_failed");
  EXPECT_EQ(machine.state(), NavigationState::kStopped);
  EXPECT_TRUE(machine.task_id().empty());
}

TEST(StateMachine, LateTerminalCannotFinishANewerTask) {
  NavigationStateMachine machine;
  ASSERT_TRUE(machine.configure().accepted);
  ASSERT_TRUE(machine.load_map("map-1").accepted);
  ASSERT_TRUE(machine.localization_result(true, "").accepted);
  ASSERT_TRUE(machine.start_navigation("task-1").accepted);
  ASSERT_TRUE(machine.cancel("task-1").accepted);
  ASSERT_TRUE(machine.acknowledge_stop().accepted);
  ASSERT_TRUE(machine.start_navigation("task-2").accepted);

  EXPECT_FALSE(machine.finish_navigation(
      "task-1", true, "late_success").accepted);
  EXPECT_EQ(machine.state(), NavigationState::kNavigating);
  EXPECT_EQ(machine.task_id(), "task-2");
  EXPECT_TRUE(machine.finish_navigation(
      "task-2", true, "navigation_complete").accepted);
  EXPECT_EQ(machine.state(), NavigationState::kReady);
}

TEST(ProjectionGrid, IsDeterministicAndMarksObstacles) {
  ProjectionConfig config;
  config.resolution = 1.0;
  config.padding_m = 0.0;
  config.minimum_points_per_cell = 2;
  config.wall_closing_radius_cells = 0;
  config.unknown_border_cells = 0;
  const std::vector<Point3D> points{{0, 0, 0.5}, {0.1, 0.1, 0.5}, {2, 2, 0.0}};
  const auto first = project_points(points, config);
  const auto second = project_points(points, config);
  EXPECT_EQ(first.cells, second.cells);
  EXPECT_EQ(first.cells.front(), 100);
}

}  // namespace
