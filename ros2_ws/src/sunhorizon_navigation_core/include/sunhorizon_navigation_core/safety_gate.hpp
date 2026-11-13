#pragma once

#include <cstddef>
#include <string>

#include "sunhorizon_navigation_core/types.hpp"

namespace sunhorizon_navigation_core {

bool command_reduction_exceeds(
    const Twist2D& raw, const Twist2D& filtered, double epsilon);

enum class CollisionAuditClass {
  kInvalid,
  kClear,
  kApproachSlowdown,
  kStop,
  kSourceInvalid,
};

CollisionAuditClass parse_collision_audit_frame_id(const std::string& frame_id);
bool collision_audit_requires_immediate_latch(CollisionAuditClass audit_class);

class CollisionInterventionGate {
 public:
  explicit CollisionInterventionGate(std::size_t required_evidence = 2U);
  bool observe(bool reduction_evidence, double steady_now_sec, double max_gap_sec);
  void reset();
  std::size_t streak() const { return streak_; }

 private:
  std::size_t required_evidence_{2U};
  std::size_t streak_{0U};
  double last_evidence_sec_{0.0};
};

struct SafetyLimits {
  double candidate_timeout_sec{0.25};
  double platform_timeout_sec{0.25};
  double obstacle_coverage_timeout_sec{0.30};
  double localization_timeout_sec{0.50};
  double tf_timeout_sec{0.25};
  double navigation_lease_timeout_sec{0.50};
  double max_vx{0.20};
  double max_vy{0.10};
  double max_wz{0.30};
  double max_ax{0.20};
  double max_ay{0.20};
  double max_aw{0.40};
  double max_waist_yaw_rad{0.10};
  bool geometry_hold_enabled{false};
};

struct SafetyInputs {
  double now_sec{0.0};
  bool candidate_seen{false};
  double candidate_stamp_sec{0.0};
  double platform_stamp_sec{0.0};
  bool obstacle_coverage_seen{false};
  double obstacle_coverage_stamp_sec{0.0};
  double localization_stamp_sec{0.0};
  double tf_stamp_sec{0.0};
  bool navigation_status_seen{false};
  double navigation_status_stamp_sec{0.0};
  bool navigation_task_active{false};
  bool mode_allowed{false};
  bool ethercat_op{false};
  bool wkc_normal{false};
  bool joint_feedback_fresh{false};
  bool leg_joints_error_free{false};
  bool leg_joints_operation_enabled{false};
  bool motion_ready{false};
  bool navigation_mode_discovered{false};
  bool platform_fault{true};
  bool obstacle_coverage_healthy{false};
  bool forward_coverage{false};
  bool near_field_coverage{false};
  bool rear_coverage{false};
  bool localization_healthy{false};
  bool tf_continuous{false};
  double waist_yaw_rad{0.0};
  // Internal result of validating one complete combined coverage decision.
  // This is not authorization from an external bare boolean; see
  // doc/geometry_hold.md for the caller's required evidence and state machine.
  bool geometry_hold_requested{false};
};

struct SafetyDecision {
  bool allowed{false};
  bool latched{false};
  Twist2D output;
  std::string reason;
  bool geometry_hold{false};
};

class SafetyGate {
 public:
  explicit SafetyGate(SafetyLimits limits = {});

  SafetyDecision evaluate(const SafetyInputs& inputs, const Twist2D& candidate);
  bool reset(const SafetyInputs& inputs, const Twist2D& candidate, std::string* reason);
  void force_latch(const std::string& reason);
  bool latched() const { return latched_; }
  const std::string& latch_reason() const { return latch_reason_; }

 private:
  std::string first_failure(const SafetyInputs& inputs, const Twist2D& candidate) const;
  Twist2D clamp_and_slew(const Twist2D& candidate, double now_sec);

  SafetyLimits limits_;
  bool latched_{false};
  std::string latch_reason_;
  Twist2D last_output_;
  double last_evaluation_sec_{0.0};
};

}  // namespace sunhorizon_navigation_core
