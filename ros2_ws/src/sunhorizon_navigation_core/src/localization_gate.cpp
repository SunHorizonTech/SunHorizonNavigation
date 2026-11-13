#include "sunhorizon_navigation_core/localization_gate.hpp"

#include <cmath>

namespace sunhorizon_navigation_core {
namespace {

constexpr double kPi = 3.14159265358979323846;

double wrap_angle(double value) {
  while (value > kPi) value -= 2.0 * kPi;
  while (value < -kPi) value += 2.0 * kPi;
  return value;
}

}  // namespace

LocalizationGate::LocalizationGate(LocalizationLimits limits) : limits_(limits) {}

LocalizationDecision LocalizationGate::evaluate(
    const LocalizationMeasurement& measurement) {
  if (measurement.stamp_sec <= 0.0 || measurement.now_sec < measurement.stamp_sec ||
      measurement.now_sec - measurement.stamp_sec > limits_.measurement_timeout_sec) {
    return {false, "stale_measurement"};
  }
  if (!measurement.converged) {
    return {false, "scan_match_not_converged"};
  }
  if (!std::isfinite(measurement.score) || measurement.score < limits_.minimum_score) {
    return {false, "score_below_threshold"};
  }
  if (measurement.iterations > limits_.maximum_iterations) {
    return {false, "iteration_limit"};
  }
  if (measurement.initial_to_result_distance_m >
      limits_.maximum_initial_to_result_distance_m) {
    return {false, "initial_guess_disagreement"};
  }
  if (measurement.position_covariance > limits_.maximum_position_covariance) {
    return {false, "position_covariance"};
  }
  if (measurement.yaw_covariance > limits_.maximum_yaw_covariance) {
    return {false, "yaw_covariance"};
  }

  if (last_pose_) {
    const double dx = measurement.pose.x - last_pose_->x;
    const double dy = measurement.pose.y - last_pose_->y;
    const double dz = measurement.pose.z - last_pose_->z;
    if (std::sqrt(dx * dx + dy * dy + dz * dz) > limits_.maximum_position_jump_m) {
      return {false, "position_jump"};
    }
    if (std::abs(wrap_angle(measurement.pose.yaw - last_pose_->yaw)) >
        limits_.maximum_yaw_jump_rad) {
      return {false, "yaw_jump"};
    }
  }

  last_pose_ = measurement.pose;
  return {true, "ok"};
}

void LocalizationGate::reset() { last_pose_.reset(); }

}  // namespace sunhorizon_navigation_core
