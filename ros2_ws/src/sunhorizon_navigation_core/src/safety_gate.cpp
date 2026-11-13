#include "sunhorizon_navigation_core/safety_gate.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sunhorizon_navigation_core {
namespace {

bool stale(double now, double stamp, double timeout) {
  return !std::isfinite(now) || !std::isfinite(stamp) ||
         !std::isfinite(timeout) || timeout <= 0.0 ||
         stamp <= 0.0 || now < stamp || (now - stamp) > timeout;
}

double clamp(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

double slew(double requested, double previous, double limit_per_sec, double dt) {
  if (dt <= 0.0) {
    return previous;
  }
  return previous + clamp(requested - previous, -limit_per_sec * dt, limit_per_sec * dt);
}

bool near_zero(const Twist2D& command) {
  return std::abs(command.vx) < 1e-6 && std::abs(command.vy) < 1e-6 &&
         std::abs(command.wz) < 1e-6;
}

bool valid_axis_limits(double maximum_velocity, double maximum_acceleration) {
  return std::isfinite(maximum_velocity) && maximum_velocity >= 0.0 &&
         std::isfinite(maximum_acceleration) && maximum_acceleration >= 0.0 &&
         (maximum_velocity == 0.0 || maximum_acceleration > 0.0);
}

bool valid_limits(const SafetyLimits& limits) {
  return std::isfinite(limits.candidate_timeout_sec) &&
         limits.candidate_timeout_sec > 0.0 &&
         std::isfinite(limits.platform_timeout_sec) &&
         limits.platform_timeout_sec > 0.0 &&
         std::isfinite(limits.obstacle_coverage_timeout_sec) &&
         limits.obstacle_coverage_timeout_sec > 0.0 &&
         std::isfinite(limits.localization_timeout_sec) &&
         limits.localization_timeout_sec > 0.0 &&
         std::isfinite(limits.tf_timeout_sec) && limits.tf_timeout_sec > 0.0 &&
         std::isfinite(limits.navigation_lease_timeout_sec) &&
         limits.navigation_lease_timeout_sec > 0.0 &&
         valid_axis_limits(limits.max_vx, limits.max_ax) &&
         valid_axis_limits(limits.max_vy, limits.max_ay) &&
         valid_axis_limits(limits.max_wz, limits.max_aw) &&
         std::isfinite(limits.max_waist_yaw_rad) &&
         limits.max_waist_yaw_rad >= 0.0;
}

}  // namespace

bool command_reduction_exceeds(
    const Twist2D& raw, const Twist2D& filtered, double epsilon) {
  if (!std::isfinite(raw.vx) || !std::isfinite(raw.vy) ||
      !std::isfinite(raw.wz) || !std::isfinite(filtered.vx) ||
      !std::isfinite(filtered.vy) || !std::isfinite(filtered.wz) ||
      !std::isfinite(epsilon) || epsilon <= 0.0) {
    return true;
  }
  const double raw_magnitude = std::max({
      std::abs(raw.vx), std::abs(raw.vy), std::abs(raw.wz)});
  const double filtered_magnitude = std::max({
      std::abs(filtered.vx), std::abs(filtered.vy), std::abs(filtered.wz)});
  return raw_magnitude > epsilon && filtered_magnitude + epsilon < raw_magnitude;
}

CollisionAuditClass parse_collision_audit_frame_id(const std::string& frame_id) {
  if (frame_id == "collision_clear") {
    return CollisionAuditClass::kClear;
  }
  if (frame_id == "collision_approach_slowdown") {
    return CollisionAuditClass::kApproachSlowdown;
  }
  if (frame_id == "collision_stop") {
    return CollisionAuditClass::kStop;
  }
  if (frame_id == "collision_source_invalid") {
    return CollisionAuditClass::kSourceInvalid;
  }
  return CollisionAuditClass::kInvalid;
}

bool collision_audit_requires_immediate_latch(
    CollisionAuditClass audit_class) {
  return audit_class == CollisionAuditClass::kStop ||
      audit_class == CollisionAuditClass::kSourceInvalid;
}

CollisionInterventionGate::CollisionInterventionGate(std::size_t required_evidence)
    : required_evidence_(std::max<std::size_t>(required_evidence, 1U)) {}

bool CollisionInterventionGate::observe(
    bool reduction_evidence, double steady_now_sec, double max_gap_sec) {
  if (!reduction_evidence || !std::isfinite(steady_now_sec) ||
      !std::isfinite(max_gap_sec) || max_gap_sec <= 0.0) {
    reset();
    return false;
  }
  if (streak_ == 0U || steady_now_sec < last_evidence_sec_ ||
      steady_now_sec - last_evidence_sec_ > max_gap_sec) {
    streak_ = 1U;
  } else {
    ++streak_;
  }
  last_evidence_sec_ = steady_now_sec;
  return streak_ >= required_evidence_;
}

void CollisionInterventionGate::reset() {
  streak_ = 0U;
  last_evidence_sec_ = 0.0;
}

SafetyGate::SafetyGate(SafetyLimits limits) : limits_(limits) {
  if (!valid_limits(limits_)) {
    throw std::invalid_argument("safety_limits_must_be_finite_and_bounded");
  }
}

std::string SafetyGate::first_failure(
    const SafetyInputs& inputs, const Twist2D& candidate) const {
  const bool geometry_hold =
      limits_.geometry_hold_enabled && inputs.geometry_hold_requested;
  if (stale(inputs.now_sec, inputs.platform_stamp_sec, limits_.platform_timeout_sec)) {
    return "stale_platform_state";
  }
  if (!inputs.mode_allowed) {
    return "robot_mode_not_allowed";
  }
  if (!inputs.ethercat_op) {
    return "ethercat_not_op";
  }
  if (!inputs.wkc_normal) {
    return "ethercat_wkc_not_proven";
  }
  if (!inputs.joint_feedback_fresh) {
    return "stale_joint_feedback";
  }
  if (!inputs.leg_joints_error_free) {
    return "leg_joint_error";
  }
  if (!inputs.leg_joints_operation_enabled) {
    return "leg_joint_not_operation_enabled";
  }
  if (!inputs.motion_ready) {
    return "motion_not_ready";
  }
  if (!inputs.navigation_mode_discovered) {
    return "navigation_mode_not_discovered";
  }
  if (inputs.platform_fault) {
    return "platform_fault";
  }
  if (!std::isfinite(inputs.waist_yaw_rad)) {
    return "non_finite_waist_yaw";
  }
  if (std::abs(inputs.waist_yaw_rad) > limits_.max_waist_yaw_rad) {
    return "waist_yaw_limit";
  }
  if (!inputs.obstacle_coverage_seen) {
    return "obstacle_coverage_missing";
  }
  if (stale(
          inputs.now_sec, inputs.obstacle_coverage_stamp_sec,
          limits_.obstacle_coverage_timeout_sec)) {
    return "stale_obstacle_coverage";
  }
  if (!geometry_hold && !inputs.obstacle_coverage_healthy) {
    return "obstacle_coverage_unhealthy";
  }
  if (!geometry_hold && !inputs.forward_coverage) {
    return "forward_obstacle_coverage_missing";
  }
  if (!geometry_hold && !inputs.near_field_coverage) {
    return "near_field_obstacle_coverage_missing";
  }
  if (stale(inputs.now_sec, inputs.localization_stamp_sec, limits_.localization_timeout_sec) ||
      !inputs.localization_healthy) {
    return "localization_unhealthy";
  }
  if (stale(inputs.now_sec, inputs.tf_stamp_sec, limits_.tf_timeout_sec) ||
      !inputs.tf_continuous) {
    return "tf_unhealthy";
  }
  if (!inputs.candidate_seen) {
    return geometry_hold ? "candidate_missing_during_geometry_hold" :
        "waiting_for_candidate";
  }
  if (stale(inputs.now_sec, inputs.candidate_stamp_sec, limits_.candidate_timeout_sec)) {
    return "candidate_timeout";
  }
  if (candidate.vx < -1e-6) {
    return "autonomous_reverse_disabled";
  }
  if (!std::isfinite(candidate.vx) || !std::isfinite(candidate.vy) ||
      !std::isfinite(candidate.wz)) {
    return "non_finite_command";
  }
  if (candidate.vx > limits_.max_vx + 1e-6) {
    return "forward_velocity_limit";
  }
  if (std::abs(candidate.vy) > limits_.max_vy + 1e-6) {
    return limits_.max_vy <= 1e-6 ?
        "lateral_axis_disabled" : "lateral_velocity_limit";
  }
  if (std::abs(candidate.wz) > limits_.max_wz + 1e-6) {
    return limits_.max_wz <= 1e-6 ?
        "yaw_axis_disabled" : "yaw_velocity_limit";
  }
  if (geometry_hold || !near_zero(candidate)) {
    if (!inputs.navigation_status_seen) {
      return "navigation_execution_lease_missing";
    }
    if (stale(
            inputs.now_sec, inputs.navigation_status_stamp_sec,
            limits_.navigation_lease_timeout_sec)) {
      return "navigation_execution_lease_stale";
    }
    if (!inputs.navigation_task_active) {
      return "navigation_execution_lease_inactive";
    }
  }
  return {};
}

Twist2D SafetyGate::clamp_and_slew(const Twist2D& candidate, double now_sec) {
  Twist2D limited{
      clamp(candidate.vx, 0.0, limits_.max_vx),
      clamp(candidate.vy, -limits_.max_vy, limits_.max_vy),
      clamp(candidate.wz, -limits_.max_wz, limits_.max_wz),
  };

  if (last_evaluation_sec_ <= 0.0 || now_sec <= last_evaluation_sec_) {
    last_output_ = {};
    last_evaluation_sec_ = now_sec;
    return last_output_;
  }

  const double dt = std::min(now_sec - last_evaluation_sec_, 0.25);
  last_output_.vx = slew(limited.vx, last_output_.vx, limits_.max_ax, dt);
  last_output_.vy = slew(limited.vy, last_output_.vy, limits_.max_ay, dt);
  last_output_.wz = slew(limited.wz, last_output_.wz, limits_.max_aw, dt);
  last_evaluation_sec_ = now_sec;
  return last_output_;
}

SafetyDecision SafetyGate::evaluate(const SafetyInputs& inputs, const Twist2D& candidate) {
  if (latched_) {
    last_output_ = {};
    last_evaluation_sec_ = inputs.now_sec;
    return {false, true, {}, latch_reason_};
  }

  const std::string failure = first_failure(inputs, candidate);
  if (!failure.empty()) {
    last_output_ = {};
    last_evaluation_sec_ = inputs.now_sec;
    if (failure != "waiting_for_candidate") {
      latched_ = true;
      latch_reason_ = failure;
    }
    return {false, latched_, {}, failure};
  }

  // Geometry Hold is a zero-speed decision only after every hard safety and
  // command check has passed. It must never retain a previous slew output.
  if (limits_.geometry_hold_enabled && inputs.geometry_hold_requested) {
    last_output_ = {};
    last_evaluation_sec_ = inputs.now_sec;
    return {false, false, {}, "geometry_hold", true};
  }

  // A downstream collision stop or an operator-requested zero must not be
  // stretched by the normal acceleration limiter.  Slew limiting is useful
  // when motion starts and while non-zero commands change, but zero is the
  // fail-safe value and therefore takes effect in the current evaluation.
  if (near_zero(candidate)) {
    last_output_ = {};
    last_evaluation_sec_ = inputs.now_sec;
    return {true, false, {}, "ok"};
  }

  return {true, false, clamp_and_slew(candidate, inputs.now_sec), "ok"};
}

bool SafetyGate::reset(
    const SafetyInputs& inputs, const Twist2D& candidate, std::string* reason) {
  // A caller cannot borrow the Hold coverage exemption to clear a hard latch.
  if (inputs.geometry_hold_requested) {
    if (reason) {
      *reason = "geometry_hold_must_clear_before_reset";
    }
    return false;
  }

  if (inputs.candidate_seen && !near_zero(candidate)) {
    if (reason) {
      *reason = "candidate_must_be_zero_before_reset";
    }
    return false;
  }

  SafetyInputs reset_inputs = inputs;
  reset_inputs.candidate_seen = false;
  const std::string failure = first_failure(reset_inputs, {});
  if (!failure.empty() && failure != "waiting_for_candidate") {
    if (reason) {
      *reason = failure;
    }
    return false;
  }

  latched_ = false;
  latch_reason_.clear();
  last_output_ = {};
  last_evaluation_sec_ = inputs.now_sec;
  if (reason) {
    *reason = "reset";
  }
  return true;
}

void SafetyGate::force_latch(const std::string& reason) {
  latched_ = true;
  latch_reason_ = reason.empty() ? "operator_stop" : reason;
  last_output_ = {};
}

}  // namespace sunhorizon_navigation_core
