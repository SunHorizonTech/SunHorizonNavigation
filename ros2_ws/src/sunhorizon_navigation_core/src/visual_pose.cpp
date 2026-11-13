#include "sunhorizon_navigation_core/visual_pose.hpp"

#include <algorithm>
#include <cmath>

namespace sunhorizon_navigation_core {
namespace {

constexpr double kPi = 3.14159265358979323846;

bool finite_pose(const Pose3D& pose) {
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.z) &&
         std::isfinite(pose.roll) && std::isfinite(pose.pitch) && std::isfinite(pose.yaw);
}

double angle_delta(double first, double second) {
  return std::abs(std::remainder(first - second, 2.0 * kPi));
}

}  // namespace

VisualPoseGate::VisualPoseGate(VisualPoseLimits limits) : limits_(limits) {}

VisualPoseDecision VisualPoseGate::evaluate(
    double now_sec, double stamp_sec, const Pose3D& pose) {
  VisualPoseDecision output;
  if (!std::isfinite(now_sec) || !std::isfinite(stamp_sec) || stamp_sec <= 0.0 ||
      now_sec < stamp_sec || now_sec - stamp_sec > limits_.max_input_age_sec) {
    output.reason = "stale_or_invalid_visual_pose";
    return output;
  }
  if (!finite_pose(pose)) {
    output.reason = "non_finite_visual_pose";
    return output;
  }
  if (have_previous_ && stamp_sec <= previous_stamp_sec_) {
    output.reason = "non_monotonic_visual_pose";
    return output;
  }
  if (have_previous_) {
    output.translation_jump_m = std::sqrt(
        std::pow(pose.x - previous_pose_.x, 2.0) +
        std::pow(pose.y - previous_pose_.y, 2.0) +
        std::pow(pose.z - previous_pose_.z, 2.0));
    output.rotation_jump_rad = std::max({
        angle_delta(pose.roll, previous_pose_.roll),
        angle_delta(pose.pitch, previous_pose_.pitch),
        angle_delta(pose.yaw, previous_pose_.yaw)});
    if (output.translation_jump_m > limits_.max_translation_jump_m) {
      output.reason = "visual_translation_jump";
      return output;
    }
    if (output.rotation_jump_rad > limits_.max_rotation_jump_rad) {
      output.reason = "visual_rotation_jump";
      return output;
    }
  }
  have_previous_ = true;
  previous_stamp_sec_ = stamp_sec;
  previous_pose_ = pose;
  output.accepted = true;
  output.reason = "ok";
  return output;
}

PlanarPoseDecomposition decompose_visual_pose(const Pose3D& pose) {
  PlanarPoseDecomposition output;
  output.odom_to_base_footprint = {pose.x, pose.y, 0.0, 0.0, 0.0, pose.yaw};
  output.base_footprint_to_base_link = {0.0, 0.0, pose.z, pose.roll, pose.pitch, 0.0};
  return output;
}

}  // namespace sunhorizon_navigation_core
