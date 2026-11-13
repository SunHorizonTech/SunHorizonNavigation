#include "sunhorizon_3d_localizer/direct_map_tracking_logic.hpp"

#include <cmath>
#include <stdexcept>

#include "sunhorizon_3d_localizer/tracking_odom_logic.hpp"

namespace sunhorizon_3d_localizer {

void DirectMapTrackingState::reset(
    const Eigen::Matrix4d& accepted_initial_map_to_source,
    const Eigen::Matrix4d& source_to_base) {
  if (!finite_transform(accepted_initial_map_to_source) ||
      !finite_transform(source_to_base)) {
    throw std::invalid_argument("direct_map_initial_pose_non_finite");
  }
  source_to_base_ = source_to_base;
  last_accepted_map_to_source_ = accepted_initial_map_to_source;
  last_accepted_planar_map_to_base_ = planarize(
      accepted_initial_map_to_source * source_to_base_);
  accepted_measurement_valid_ = false;
  last_accepted_stamp_ns_ = 0;
  initialized_ = true;
}

const Eigen::Matrix4d& DirectMapTrackingState::next_initial_guess() const {
  if (!initialized_) {
    throw std::logic_error("direct_map_initial_pose_not_accepted");
  }
  return last_accepted_map_to_source_;
}

DirectMapTrackingDecision DirectMapTrackingState::evaluate(
    std::int64_t stamp_ns,
    const Eigen::Matrix4d& candidate_map_to_source,
    const DirectMapTrackingLimits& limits) {
  if (!initialized_) {
    throw std::logic_error("direct_map_initial_pose_not_accepted");
  }
  DirectMapTrackingDecision decision;
  if (stamp_ns <= 0 ||
      (accepted_measurement_valid_ && stamp_ns <= last_accepted_stamp_ns_)) {
    decision.reason = "direct_map_stamp_not_strictly_increasing";
    return decision;
  }
  if (!finite_transform(candidate_map_to_source)) {
    decision.reason = "direct_map_candidate_transform_non_finite";
    return decision;
  }
  if (!std::isfinite(limits.maximum_translation_step_m) ||
      limits.maximum_translation_step_m <= 0.0 ||
      !std::isfinite(limits.maximum_rotation_step_rad) ||
      limits.maximum_rotation_step_rad <= 0.0 ||
      !std::isfinite(limits.maximum_translation_rate_mps) ||
      limits.maximum_translation_rate_mps <= 0.0 ||
      !std::isfinite(limits.maximum_rotation_rate_radps) ||
      limits.maximum_rotation_rate_radps <= 0.0) {
    throw std::invalid_argument("invalid_direct_map_tracking_limits");
  }

  const Eigen::Matrix4d candidate_planar_map_to_base = planarize(
      candidate_map_to_source * source_to_base_);
  const auto step = evaluate_transform_step(
      last_accepted_planar_map_to_base_, candidate_planar_map_to_base,
      limits.maximum_translation_step_m, limits.maximum_rotation_step_rad);
  decision.translation_step_m = step.translation_step_m;
  decision.rotation_step_rad = step.rotation_step_rad;
  if (!step.accepted) {
    if (step.reason == "translation_step_limit") {
      decision.reason = "direct_map_translation_step_limit";
    } else if (step.reason == "rotation_step_limit") {
      decision.reason = "direct_map_rotation_step_limit";
    } else {
      decision.reason = "direct_map_transform_step_invalid";
    }
    return decision;
  }

  if (accepted_measurement_valid_) {
    decision.has_previous_measurement = true;
    decision.dt_sec = static_cast<double>(stamp_ns - last_accepted_stamp_ns_) * 1e-9;
    if (!std::isfinite(decision.dt_sec) || decision.dt_sec <= 0.0) {
      decision.reason = "direct_map_stamp_not_strictly_increasing";
      return decision;
    }
    decision.translation_rate_mps = decision.translation_step_m / decision.dt_sec;
    decision.rotation_rate_radps = decision.rotation_step_rad / decision.dt_sec;
    if (!std::isfinite(decision.translation_rate_mps) ||
        decision.translation_rate_mps > limits.maximum_translation_rate_mps) {
      decision.reason = "direct_map_translation_rate_limit";
      return decision;
    }
    if (!std::isfinite(decision.rotation_rate_radps) ||
        decision.rotation_rate_radps > limits.maximum_rotation_rate_radps) {
      decision.reason = "direct_map_rotation_rate_limit";
      return decision;
    }
  }

  decision.output.map_to_source = candidate_map_to_source;
  decision.output.planar_map_to_base = candidate_planar_map_to_base;
  // The odom candidate deliberately uses map-equivalent numeric coordinates.
  // The frame names remain distinct, so the localization manager computes a
  // valid identity map->odom edge instead of a self-referential TF edge.
  decision.output.map_equivalent_odom_to_base =
      decision.output.planar_map_to_base;
  decision.output.map_to_odom = Eigen::Matrix4d::Identity();

  last_accepted_map_to_source_ = candidate_map_to_source;
  last_accepted_planar_map_to_base_ = candidate_planar_map_to_base;
  last_accepted_stamp_ns_ = stamp_ns;
  accepted_measurement_valid_ = true;
  decision.accepted = true;
  decision.reason = "ok";
  return decision;
}

}  // namespace sunhorizon_3d_localizer
