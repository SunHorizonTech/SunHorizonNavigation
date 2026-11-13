#include "sunhorizon_3d_localizer/ndt_result_step_gate.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/LU>

#include "sunhorizon_3d_localizer/tracking_odom_logic.hpp"

namespace sunhorizon_3d_localizer {
namespace {

constexpr double kPi = 3.14159265358979323846;

bool valid_positive_limit(double value) {
  return std::isfinite(value) && value > 0.0;
}

double yaw(const Eigen::Matrix3d& rotation) {
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

double wrapped_yaw_step(
    const Eigen::Matrix4d& first, const Eigen::Matrix4d& second) {
  double delta = yaw(second.block<3, 3>(0, 0)) -
      yaw(first.block<3, 3>(0, 0));
  while (delta > kPi) delta -= 2.0 * kPi;
  while (delta < -kPi) delta += 2.0 * kPi;
  return std::abs(delta);
}

}  // namespace

bool valid_result_translation_step_limit(double limit_m, bool attended_profile) {
  return std::isfinite(limit_m) && limit_m > 0.0 &&
      limit_m <= (attended_profile ? 0.10 : 0.05);
}

Eigen::Matrix4d canonical_map_to_odom_from_planar_base_poses(
    const Eigen::Matrix4d& planar_map_to_base,
    const Eigen::Matrix4d& planar_odom_to_base) {
  if (!finite_transform(planar_map_to_base) ||
      !finite_transform(planar_odom_to_base)) {
    return Eigen::Matrix4d::Constant(
        std::numeric_limits<double>::quiet_NaN());
  }
  return planar_map_to_base * planar_odom_to_base.inverse();
}

void NdtResultStepGate::reset() {
  baseline_valid_ = false;
  baseline_stamp_ns_ = 0;
  baseline_map_to_odom_ = Eigen::Matrix4d::Identity();
}

NdtResultStepDecision NdtResultStepGate::evaluate(
    std::int64_t stamp_ns,
    const Eigen::Matrix4d& candidate_map_to_odom,
    const NdtResultStepLimits& limits) {
  if (!valid_positive_limit(limits.maximum_gap_sec) ||
      !valid_positive_limit(limits.maximum_translation_step_m) ||
      !valid_positive_limit(limits.maximum_yaw_step_rad)) {
    throw std::invalid_argument("invalid_ndt_result_step_limits");
  }

  NdtResultStepDecision decision;
  decision.has_baseline = baseline_valid_;
  if (stamp_ns <= 0 || (baseline_valid_ && stamp_ns <= baseline_stamp_ns_)) {
    decision.reason = "ndt_correction_stamp_not_strictly_increasing";
    return decision;
  }
  if (!finite_transform(candidate_map_to_odom)) {
    decision.reason = "ndt_correction_transform_non_finite";
    return decision;
  }

  bool recovered_gap = false;
  if (baseline_valid_) {
    decision.dt_sec = static_cast<double>(stamp_ns - baseline_stamp_ns_) * 1e-9;
    const double dx = candidate_map_to_odom(0, 3) -
        baseline_map_to_odom_(0, 3);
    const double dy = candidate_map_to_odom(1, 3) -
        baseline_map_to_odom_(1, 3);
    decision.translation_step_m = std::hypot(dx, dy);
    decision.yaw_step_rad = wrapped_yaw_step(
        baseline_map_to_odom_, candidate_map_to_odom);
    if (!std::isfinite(decision.dt_sec) || decision.dt_sec <= 0.0) {
      decision.reason = "ndt_correction_stamp_not_strictly_increasing";
      return decision;
    }
    recovered_gap = decision.dt_sec > limits.maximum_gap_sec;
    if (!std::isfinite(decision.translation_step_m) ||
        decision.translation_step_m > limits.maximum_translation_step_m) {
      decision.reason = "ndt_correction_translation_step_limit";
      return decision;
    }
    if (!std::isfinite(decision.yaw_step_rad) ||
        decision.yaw_step_rad > limits.maximum_yaw_step_rad) {
      decision.reason = "ndt_correction_yaw_step_limit";
      return decision;
    }
  } else {
    decision.establishes_baseline = true;
  }

  baseline_stamp_ns_ = stamp_ns;
  baseline_map_to_odom_ = candidate_map_to_odom;
  baseline_valid_ = true;
  decision.accepted = true;
  decision.gap_recovered = recovered_gap;
  decision.reason = recovered_gap ?
      "ok_after_result_gap_with_bounded_correction" : "ok";
  return decision;
}

}  // namespace sunhorizon_3d_localizer
