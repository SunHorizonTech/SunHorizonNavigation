#include "sunhorizon_3d_localizer/tracking_odom_logic.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <Eigen/Geometry>

namespace sunhorizon_3d_localizer {
namespace {

double yaw(const Eigen::Matrix3d& rotation) {
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

bool valid_limit(double value) {
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

bool finite_transform(const Eigen::Matrix4d& transform) {
  return transform.array().isFinite().all();
}

double rotation_distance(
    const Eigen::Matrix4d& first, const Eigen::Matrix4d& second) {
  const Eigen::Matrix3d delta =
      first.block<3, 3>(0, 0).transpose() * second.block<3, 3>(0, 0);
  return std::abs(Eigen::AngleAxisd(delta).angle());
}

Eigen::Matrix4d planarize(const Eigen::Matrix4d& transform) {
  Eigen::Matrix4d output = Eigen::Matrix4d::Identity();
  output.block<3, 3>(0, 0) = Eigen::AngleAxisd(
      yaw(transform.block<3, 3>(0, 0)), Eigen::Vector3d::UnitZ()).toRotationMatrix();
  output(0, 3) = transform(0, 3);
  output(1, 3) = transform(1, 3);
  return output;
}

Eigen::Matrix4d planarize_with_z_reference(
    const Eigen::Matrix4d& transform, double z_reference_m) {
  if (!finite_transform(transform) || !std::isfinite(z_reference_m)) {
    throw std::invalid_argument("invalid_planar_pose_z_reference");
  }
  Eigen::Matrix4d output = planarize(transform);
  output(2, 3) = z_reference_m;
  return output;
}

TransformStepDecision evaluate_transform_step(
    const Eigen::Matrix4d& previous,
    const Eigen::Matrix4d& current,
    double maximum_translation_step_m,
    double maximum_rotation_step_rad) {
  TransformStepDecision output;
  if (!finite_transform(previous) || !finite_transform(current) ||
      !valid_limit(maximum_translation_step_m) ||
      !valid_limit(maximum_rotation_step_rad)) {
    output.reason = "transform_step_invalid_input";
    return output;
  }
  const Eigen::Matrix4d delta = previous.inverse() * current;
  output.translation_step_m = delta.block<3, 1>(0, 3).norm();
  output.rotation_step_rad = rotation_distance(previous, current);
  if (output.translation_step_m > maximum_translation_step_m) {
    output.reason = "translation_step_limit";
    return output;
  }
  if (output.rotation_step_rad > maximum_rotation_step_rad) {
    output.reason = "rotation_step_limit";
    return output;
  }
  output.accepted = true;
  output.reason = "ok";
  return output;
}

TrackingOdomGate::TrackingOdomGate(
    std::string expected_frame,
    std::string expected_child_frame,
    Eigen::Matrix4d prior_basis,
    Eigen::Matrix4d source_to_base,
    TrackingOdomLimits limits)
    : expected_frame_(std::move(expected_frame)),
      expected_child_frame_(std::move(expected_child_frame)),
      prior_basis_(std::move(prior_basis)),
      source_to_base_(std::move(source_to_base)),
      limits_(limits) {
  if (expected_frame_.empty() || expected_child_frame_.empty() ||
      expected_frame_ == expected_child_frame_ || !finite_transform(prior_basis_) ||
      !finite_transform(source_to_base_) || !valid_limit(limits_.maximum_gap_sec) ||
      !valid_limit(limits_.maximum_translation_rate_mps) ||
      !valid_limit(limits_.maximum_rotation_rate_radps) ||
      !valid_limit(limits_.maximum_translation_step_m) ||
      !valid_limit(limits_.maximum_rotation_step_rad)) {
    throw std::invalid_argument("invalid_tracking_odom_gate_configuration");
  }
}

TrackingOdomDecision TrackingOdomGate::reject(const std::string& reason) {
  latched_ = true;
  latched_reason_ = reason;
  TrackingOdomDecision output;
  output.latched = true;
  output.reason = reason;
  return output;
}

TrackingOdomDecision TrackingOdomGate::evaluate(const TrackingOdomSample& sample) {
  if (latched_) return reject(latched_reason_);
  if (sample.frame_id != expected_frame_ ||
      sample.child_frame_id != expected_child_frame_) {
    return reject("odometry_prior_frame_mismatch");
  }
  if (sample.stamp_ns <= 0 ||
      (previous_valid_ && sample.stamp_ns <= previous_stamp_ns_)) {
    return reject("odometry_prior_stamp_not_strictly_increasing");
  }
  if (!finite_transform(sample.raw_odom_to_source)) {
    return reject("odometry_prior_pose_non_finite");
  }

  TrackingOdomDecision output;
  output.odom_to_base =
      prior_basis_ * sample.raw_odom_to_source * prior_basis_.inverse() * source_to_base_;
  if (!finite_transform(output.odom_to_base)) {
    return reject("tracking_odom_transform_non_finite");
  }
  output.planar_odom_to_base = planarize(output.odom_to_base);

  if (previous_valid_) {
    output.has_previous = true;
    output.dt_sec = static_cast<double>(sample.stamp_ns - previous_stamp_ns_) * 1e-9;
    if (!std::isfinite(output.dt_sec) || output.dt_sec <= 0.0 ||
        output.dt_sec > limits_.maximum_gap_sec) {
      return reject("odometry_prior_gap");
    }
    const auto step = evaluate_transform_step(
        previous_odom_to_base_, output.odom_to_base,
        limits_.maximum_translation_step_m, limits_.maximum_rotation_step_rad);
    output.translation_step_m = step.translation_step_m;
    output.rotation_step_rad = step.rotation_step_rad;
    if (!step.accepted) {
      if (step.reason == "translation_step_limit") {
        return reject("odometry_prior_translation_step_limit");
      }
      if (step.reason == "rotation_step_limit") {
        return reject("odometry_prior_rotation_step_limit");
      }
      return reject("odometry_prior_step_invalid");
    }
    if (output.translation_step_m / output.dt_sec >
        limits_.maximum_translation_rate_mps) {
      return reject("odometry_prior_translation_rate_limit");
    }
    if (output.rotation_step_rad / output.dt_sec >
        limits_.maximum_rotation_rate_radps) {
      return reject("odometry_prior_rotation_rate_limit");
    }
    output.planar_relative_motion =
        previous_planar_odom_to_base_.inverse() * output.planar_odom_to_base;
  }

  previous_stamp_ns_ = sample.stamp_ns;
  previous_odom_to_base_ = output.odom_to_base;
  previous_planar_odom_to_base_ = output.planar_odom_to_base;
  previous_valid_ = true;
  output.accepted = true;
  output.reason = "ok";
  return output;
}

}  // namespace sunhorizon_3d_localizer
