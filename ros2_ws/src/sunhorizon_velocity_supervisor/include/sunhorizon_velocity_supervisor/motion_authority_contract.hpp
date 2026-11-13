#pragma once

#include <chrono>
#include <cmath>
#include <string>

namespace sunhorizon_velocity_supervisor {

inline constexpr char kReceiptGatedNavigationMode[] =
    "receipt_gated_navigation";
inline constexpr char kAttendedFiveCmQualificationMode[] =
    "attended_five_cm_qualification_v1";
inline constexpr char kAttendedFiveCmQualificationAcknowledgement[] =
    "SUNHORIZON_ATTENDED_FIVE_CM_SUPERVISOR_V1";
inline constexpr char kAttendedFiveCmCandidateOwner[] =
    "/sunhorizon_attended_five_cm_supervisor_output_probe";
inline constexpr char kAttendedQualificationBudgetExhaustedReason[] =
    "attended_five_cm_qualification_nonzero_budget_exhausted";
inline constexpr char kAttendedYawQualificationMode[] =
    "attended_yaw_qualification_v1";
inline constexpr char kAttendedYawQualificationAcknowledgement[] =
    "SUNHORIZON_ATTENDED_YAW_SUPERVISOR_V1";
inline constexpr char kAttendedYawCandidateOwner[] =
    "/sunhorizon_attended_yaw_supervisor_output_probe";
inline constexpr char kAttendedYawQualificationBudgetExhaustedReason[] =
    "attended_yaw_qualification_nonzero_budget_exhausted";
inline constexpr char kAttendedForwardCalibrationMode[] =
    "attended_forward_calibration_v1";
inline constexpr char kAttendedForwardCalibrationAcknowledgement[] =
    "SUNHORIZON_ATTENDED_FORWARD_CALIBRATION_SUPERVISOR_V1";
inline constexpr char kAttendedForwardCalibrationCandidateOwner[] =
    "/sunhorizon_attended_forward_calibration_supervisor_output_probe";
inline constexpr char kAttendedForwardCalibrationBudgetExhaustedReason[] =
    "attended_forward_calibration_nonzero_budget_exhausted";
inline constexpr char kAttendedForwardPulseMode[] = "attended_forward_pulse_v1";
inline constexpr char kAttendedForwardPulseAcknowledgement[] =
    "SUNHORIZON_ATTENDED_FORWARD_PULSE_SUPERVISOR_V1";
inline constexpr char kAttendedForwardPulseCandidateOwner[] =
    "/sunhorizon_attended_forward_pulse";
inline constexpr char kAttendedForwardPulseBudgetExhaustedReason[] =
    "attended_forward_pulse_nonzero_budget_exhausted";
inline constexpr char kAttendedFirstClickNavigationMode[] =
    "attended_first_click_navigation_v1";
inline constexpr char kAttendedFirstClickNavigationAcknowledgement[] =
    "SUNHORIZON_ATTENDED_FIRST_CLICK_NAVIGATION_V1";
inline constexpr char kAttendedFirstClickNavigationBudgetExhaustedReason[] =
    "attended_first_click_navigation_nonzero_budget_exhausted";

inline constexpr char kAttendedHeadingCorrectionNavigationMode[] =
    "attended_heading_correction_navigation_v1";
inline constexpr char kAttendedHeadingCorrectionNavigationAcknowledgement[] =
    "SUNHORIZON_ATTENDED_HEADING_CORRECTION_NAVIGATION_V1";
inline constexpr char kAttendedHeadingCorrectionNavigationBudgetExhaustedReason[] =
    "attended_heading_correction_navigation_nonzero_budget_exhausted";

inline constexpr char kAttendedThreeMeterNavigationMode[] = "attended_three_meter_navigation_v1";
inline constexpr char kAttendedThreeMeterNavigationAcknowledgement[] = "SUNHORIZON_ATTENDED_THREE_METER_NAVIGATION_V1";

inline constexpr char kAttendedFiveMeterNavigationMode[] = "attended_five_meter_navigation_v1";
inline constexpr char kAttendedFiveMeterNavigationAcknowledgement[] = "SUNHORIZON_ATTENDED_FIVE_METER_NAVIGATION_V1";
// Explicit speed comparison: v1 remains capped at .05; v2 allows .08.
inline constexpr char kAttendedFiveMeterFasterNavigationMode[] = "attended_five_meter_navigation_v2";
inline constexpr char kAttendedFiveMeterFasterNavigationAcknowledgement[] = "SUNHORIZON_ATTENDED_FIVE_METER_NAVIGATION_V2";

inline bool is_five_meter_navigation_mode(const std::string& mode) {
  return mode == kAttendedFiveMeterNavigationMode ||
         mode == kAttendedFiveMeterFasterNavigationMode;
}

// This is intentionally compiled into the final-output process.  The attended
// Python probe is an observer/orchestrator, not the enforcement boundary for
// the maximum time that a non-zero command may remain authoritative.
class AttendedQualificationNonzeroBudget {
 public:
  using Clock = std::chrono::steady_clock;
  static constexpr auto kHardLimit = std::chrono::milliseconds(400);
  static constexpr auto kForwardPulseHardLimit = std::chrono::milliseconds(2000);
  static constexpr auto kForwardCalibrationHardLimit =
      std::chrono::milliseconds(12000);
  static constexpr auto kFirstClickNavigationHardLimit =
      std::chrono::milliseconds(45000);

  static constexpr auto kFiveMeterHardLimit = std::chrono::milliseconds(225000);
  static constexpr auto kThreeMeterHardLimit = std::chrono::milliseconds(135000);

  explicit AttendedQualificationNonzeroBudget(
      std::chrono::milliseconds hard_limit = kHardLimit)
      : hard_limit_(hard_limit) {}

  void observe_first_nonzero(Clock::time_point now) {
    if (spent_) return;
    spent_ = true;
    deadline_ = now + hard_limit_;
  }

  bool spent() const { return spent_; }
  bool reset_allowed() const { return !spent_; }
  bool expired(Clock::time_point now) const {
    return spent_ && now >= deadline_;
  }

 private:
  bool spent_{false};
  std::chrono::milliseconds hard_limit_;
  Clock::time_point deadline_{};
};

struct MotionAuthorityContract {
  std::string mode{kReceiptGatedNavigationMode};
  std::string qualification_acknowledgement;
  bool motion_output_enabled{false};
  bool velocity_calibration_accepted{false};
  bool vendor_watchdog_accepted{false};
  double vendor_vx_gain{1.0};
  double vendor_vx_deadzone_command{0.0};
  std::string candidate_topic;
  std::string collision_audit_topic;
  std::string final_topic;
  std::string map_frame;
  std::string base_frame;
  std::string platform_state_topic;
  std::string localization_status_topic;
  std::string obstacle_coverage_topic;
  std::string expected_candidate_publisher_fqn;
  double candidate_timeout_sec{0.0};
  double control_rate_hz{0.0};
  double max_vx{0.0};
  double max_vy{0.0};
  double max_wz{0.0};
  double max_ax{0.0};
  double max_ay{0.0};
  double max_aw{0.0};
};

inline bool exact_zero(double value) {
  return std::isfinite(value) && value == 0.0;
}

inline std::string motion_authority_contract_failure(
    const MotionAuthorityContract& contract) {
  if (contract.mode == kReceiptGatedNavigationMode) {
    if (!contract.qualification_acknowledgement.empty()) {
      return "qualification_acknowledgement_forbidden_in_navigation_mode";
    }
    if (contract.motion_output_enabled &&
        (!contract.velocity_calibration_accepted ||
         !contract.vendor_watchdog_accepted)) {
      return "live_output_requires_velocity_and_watchdog_receipts";
    }
    return {};
  }

  const bool attended_five_cm =
      contract.mode == kAttendedFiveCmQualificationMode;
  const bool attended_yaw = contract.mode == kAttendedYawQualificationMode;
  const bool attended_forward_calibration =
      contract.mode == kAttendedForwardCalibrationMode;
  const bool attended_forward_pulse = contract.mode == kAttendedForwardPulseMode;
  const bool attended_first_click_navigation =
      contract.mode == kAttendedFirstClickNavigationMode;
  const bool attended_five_meter = is_five_meter_navigation_mode(contract.mode);
  const bool attended_five_meter_faster = contract.mode == kAttendedFiveMeterFasterNavigationMode;
  const bool attended_three_meter = contract.mode == kAttendedThreeMeterNavigationMode;
  const bool attended_heading_correction =
      contract.mode == kAttendedHeadingCorrectionNavigationMode || attended_three_meter || attended_five_meter;
  if (!attended_five_cm && !attended_yaw &&
      !attended_forward_calibration && !attended_forward_pulse && !attended_first_click_navigation &&
      !attended_heading_correction) {
    return "motion_authority_mode_invalid";
  }
  std::string profile = "attended_five_cm";
  const char* expected_acknowledgement =
      kAttendedFiveCmQualificationAcknowledgement;
  const char* expected_candidate_owner = kAttendedFiveCmCandidateOwner;
  if (attended_yaw) {
    profile = "attended_yaw";
    expected_acknowledgement = kAttendedYawQualificationAcknowledgement;
    expected_candidate_owner = kAttendedYawCandidateOwner;
  } else if (attended_forward_calibration) {
    profile = "attended_forward_calibration";
    expected_acknowledgement =
        kAttendedForwardCalibrationAcknowledgement;
    expected_candidate_owner = kAttendedForwardCalibrationCandidateOwner;
  } else if (attended_forward_pulse) {
    profile = "attended_forward_pulse";
    expected_acknowledgement = kAttendedForwardPulseAcknowledgement;
    expected_candidate_owner = kAttendedForwardPulseCandidateOwner;
  } else if (attended_heading_correction) {
    profile = "attended_heading_correction_navigation";
    expected_acknowledgement = kAttendedHeadingCorrectionNavigationAcknowledgement;
    expected_candidate_owner = "/collision_monitor";
  } else if (attended_first_click_navigation) {
    profile = "attended_first_click_navigation";
    expected_acknowledgement =
        kAttendedFirstClickNavigationAcknowledgement;
    expected_candidate_owner = "/collision_monitor";
  }
  if (attended_three_meter) {
    profile = "attended_three_meter_navigation";
    expected_acknowledgement = kAttendedThreeMeterNavigationAcknowledgement;
  }
  if (attended_five_meter) {
    profile = "attended_five_meter_navigation";
    expected_acknowledgement = attended_five_meter_faster ?
        kAttendedFiveMeterFasterNavigationAcknowledgement : kAttendedFiveMeterNavigationAcknowledgement;
  }
  if (contract.qualification_acknowledgement != expected_acknowledgement) {
    return profile + "_qualification_acknowledgement_required";
  }
  if (!contract.motion_output_enabled) {
    return profile + "_qualification_requires_motion_output";
  }
  if (contract.velocity_calibration_accepted ||
      contract.vendor_watchdog_accepted) {
    return profile + "_qualification_must_not_claim_acceptance_receipts";
  }
  if (!std::isfinite(contract.vendor_vx_gain) ||
      contract.vendor_vx_gain != 1.0 ||
      !exact_zero(contract.vendor_vx_deadzone_command)) {
    return profile + "_qualification_requires_unity_unaccepted_adapter";
  }
  if (contract.candidate_topic !=
          "/sunhorizon/navigation/cmd_vel_candidate" ||
      contract.collision_audit_topic !=
          "/sunhorizon/navigation/collision_audit" ||
      contract.final_topic != "/navigation/cmd_vel" ||
      contract.map_frame != "map" || contract.base_frame != "base_footprint" ||
      contract.platform_state_topic != "/sunhorizon/platform/safety_state" ||
      contract.localization_status_topic !=
          "/sunhorizon/localization/status" ||
      contract.obstacle_coverage_topic !=
          "/sunhorizon/obstacle_coverage/status") {
    return profile + "_qualification_fixed_interface_mismatch";
  }
  if (contract.expected_candidate_publisher_fqn != expected_candidate_owner) {
    return profile + "_qualification_candidate_owner_mismatch";
  }
  if (!std::isfinite(contract.candidate_timeout_sec) ||
      contract.candidate_timeout_sec <= 0.0 ||
      contract.candidate_timeout_sec > 0.25 ||
      !std::isfinite(contract.control_rate_hz) ||
      contract.control_rate_hz < 50.0) {
    return profile + "_qualification_timing_contract_invalid";
  }
  const bool valid_five_cm_envelope =
      attended_five_cm && std::isfinite(contract.max_vx) &&
      contract.max_vx > 0.0 && contract.max_vx <= 0.05 &&
      exact_zero(contract.max_vy) && exact_zero(contract.max_wz);
  const bool valid_yaw_envelope =
      attended_yaw && exact_zero(contract.max_vx) &&
      exact_zero(contract.max_vy) && std::isfinite(contract.max_wz) &&
      contract.max_wz == 0.03;
  const bool valid_forward_calibration_envelope =
      attended_forward_calibration && std::isfinite(contract.max_vx) &&
      contract.max_vx == 0.05 && exact_zero(contract.max_vy) &&
      exact_zero(contract.max_wz);
  const bool valid_forward_pulse_envelope =
      attended_forward_pulse && std::isfinite(contract.max_vx) &&
      contract.max_vx == 0.05 && exact_zero(contract.max_vy) &&
      exact_zero(contract.max_wz);
  const bool valid_first_click_navigation_envelope =
      attended_first_click_navigation && std::isfinite(contract.max_vx) &&
      contract.max_vx > 0.0 && contract.max_vx <= 0.05 &&
      exact_zero(contract.max_vy) && exact_zero(contract.max_wz);
  // These are provisional attended command limits, not measured SI calibration.
  // The existing forward-only profile remains exact-zero on both extra axes.
  const bool valid_heading_correction_envelope =
      attended_heading_correction && std::isfinite(contract.max_vx) &&
      contract.max_vx > 0.0 && contract.max_vx <= (attended_five_meter_faster ? 0.08 : 0.05) &&
      exact_zero(contract.max_vy) && std::isfinite(contract.max_wz) &&
      contract.max_wz > 0.0 && contract.max_wz <= 0.03 &&
      std::isfinite(contract.max_ax) && contract.max_ax > 0.0 &&
      contract.max_ax <= 0.10 && exact_zero(contract.max_ay) &&
      std::isfinite(contract.max_aw) && contract.max_aw > 0.0 &&
      contract.max_aw <= 0.10;
  if (!valid_five_cm_envelope && !valid_yaw_envelope &&
      !valid_forward_calibration_envelope && !valid_forward_pulse_envelope &&
      !valid_first_click_navigation_envelope && !valid_heading_correction_envelope) {
    if (attended_heading_correction) {
      return "attended_heading_correction_navigation_motion_envelope_invalid";
    }
    if (attended_yaw) {
      return "attended_yaw_qualification_motion_envelope_invalid";
    }
    if (attended_forward_calibration) {
      return "attended_forward_calibration_motion_envelope_invalid";
    }
    if (attended_forward_pulse) {
      return "attended_forward_pulse_motion_envelope_invalid";
    }
    if (attended_first_click_navigation) {
      return "attended_first_click_navigation_motion_envelope_invalid";
    }
    return "attended_five_cm_qualification_motion_envelope_invalid";
  }
  return {};
}

}  // namespace sunhorizon_velocity_supervisor
