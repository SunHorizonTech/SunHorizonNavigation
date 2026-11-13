#include <cassert>
#include <iostream>

#include "sunhorizon_navigation_core/localization_evidence.hpp"
#include "sunhorizon_navigation_core/localization_gate.hpp"
#include "sunhorizon_navigation_core/navigation_state_machine.hpp"
#include "sunhorizon_navigation_core/platform_evidence.hpp"
#include "sunhorizon_navigation_core/projection_grid.hpp"
#include "sunhorizon_navigation_core/rgbd_validation.hpp"
#include "sunhorizon_navigation_core/safety_gate.hpp"
#include "sunhorizon_navigation_core/visual_pose.hpp"

using namespace sunhorizon_navigation_core;

int main() {
  SafetyInputs inputs;
  inputs.now_sec = 5.0;
  inputs.candidate_seen = true;
  inputs.candidate_stamp_sec = 4.9;
  inputs.platform_stamp_sec = 4.9;
  inputs.obstacle_coverage_seen = true;
  inputs.obstacle_coverage_stamp_sec = 4.9;
  inputs.localization_stamp_sec = 4.9;
  inputs.tf_stamp_sec = 4.9;
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
  inputs.localization_healthy = true;
  inputs.tf_continuous = true;

  SafetyGate safety;
  assert(safety.evaluate(inputs, {-0.1, 0.0, 0.0}).reason ==
         "autonomous_reverse_disabled");

  const CameraIntrinsics intrinsics{640, 480, 500.0, 500.0, 320.0, 240.0, "optical"};
  RgbdPairValidator rgbd;
  const auto pair = rgbd.evaluate(
      {4.8, 640, 480, "rgb8", "optical"},
      {4.804, 640, 480, "16UC1", "optical"},
      intrinsics, intrinsics, true);
  assert(pair.stream_accepted && pair.pointcloud_allowed);
  assert(depth_sample_within_tolerance(1000, 1.0));

  const auto joints = parse_joint_info_json(
      R"({"joints":[{"index":0,"error_code":0,"status_word":567}]})", {0});
  assert(joints.complete && joints.error_free && joints.operation_enabled);
  assert(parse_ethercat_text("Active: yes\nSlave 0 OP\nDomain0 WKC 2/2").wkc_normal);

  VisualPoseGate visual_pose;
  assert(visual_pose.evaluate(5.0, 4.9, {}).accepted);
  const auto split = decompose_visual_pose({1.0, 2.0, 0.8, 0.1, 0.2, 0.3});
  assert(split.odom_to_base_footprint.z == 0.0);
  assert(split.base_footprint_to_base_link.z == 0.8);

  LocalizationGate localization;
  LocalizationMeasurement measurement;
  measurement.now_sec = 5.0;
  measurement.stamp_sec = 4.9;
  measurement.converged = true;
  measurement.score = 3.0;
  measurement.iterations = 10;
  measurement.position_covariance = 0.01;
  measurement.yaw_covariance = 0.001;
  assert(localization.evaluate(measurement).accepted);

  LocalizationEvidenceGate localization_evidence;
  assert(!localization_evidence.evaluate({4.9, 4.9, 0, 0, 0}).accepted);
  assert(localization_evidence.evaluate({4.9, 4.9, 42, 0, 0}).accepted);

  NavigationStateMachine state;
  assert(state.configure().accepted);
  assert(state.start_mapping("s").accepted);
  assert(!state.start_navigation("t").accepted);

  ProjectionConfig projection_config;
  projection_config.resolution = 0.5;
  projection_config.padding_m = 0.0;
  projection_config.minimum_points_per_cell = 1;
  projection_config.wall_closing_radius_cells = 0;
  projection_config.unknown_border_cells = 0;
  const auto grid = project_points({{0.0, 0.0, 0.5}}, projection_config);
  assert(!grid.cells.empty() && grid.cells.front() == 100);

  std::cout << "sunhorizon_navigation_core tests passed\n";
  return 0;
}
