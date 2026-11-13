#pragma once
#include <cstdint>
#include <sstream>
#include <string>
#include <string_view>

namespace sunhorizon_navigation_runtime { namespace detail {
// A single locked observation of the retained gate, not new health evidence.
// The identity view is consumed before releasing the owner's state mutex.
struct CanonicalHealthSnapshot {
  bool has_state{false}, has_health{false}, has_status{false}, has_health_state{false};
  bool status_healthy{false}, measurement_accepted{false};
  uint64_t reset_epoch{0}, state_epoch{0}, health_epoch{0}, health_state_epoch{0};
  uint64_t state_sequence{0}, tracking_sequence{0}, health_state_sequence{0};
  std::string_view tracking_identity;
  int64_t check_ros_ns{0}, check_steady_ns{0}, source_ns{0};
  int64_t receipt_ros_ns{0}, receipt_steady_ns{0}, status_stamp_ns{0};
  int64_t health_ros_ns{0}, health_steady_ns{0}, tracking_budget_ns{0}, health_budget_ns{0};
};

inline const char * canonical_health_failure(const CanonicalHealthSnapshot & s) {
  if (!s.has_state) return "canonical_state_missing";
  if (!s.has_health) return "canonical_health_missing";
  if (!s.has_status) return "canonical_health_status_missing";
  if (!s.has_health_state) return "canonical_health_last_state_missing";
  if (s.state_epoch != s.reset_epoch) return "canonical_state_epoch_mismatch";
  if (s.health_epoch != s.reset_epoch) return "canonical_health_epoch_mismatch";
  if (s.health_state_epoch != s.reset_epoch) return "canonical_health_last_state_epoch_mismatch";
  if (!s.status_healthy) return "canonical_status_not_healthy";
  if (!s.measurement_accepted) return "canonical_measurement_not_accepted";
  if (s.status_stamp_ns != s.health_ros_ns) return "canonical_health_evaluation_stamp_mismatch";
  // Source latency is diagnostic only. Original receipt clocks still detect a
  // stopped stream; repeated health evaluation cannot renew those receipts.
  if (s.source_ns <= 0) return "tracking_source_invalid";
  if (s.receipt_ros_ns < s.source_ns) return "tracking_source_after_receipt";
  if (s.receipt_steady_ns <= 0) return "tracking_receipt_steady_invalid";
  if (s.check_ros_ns < s.receipt_ros_ns) return "tracking_ros_clock_regression";
  if (s.check_steady_ns < s.receipt_steady_ns) return "tracking_steady_clock_regression";
  if (s.check_ros_ns - s.receipt_ros_ns > s.tracking_budget_ns) return "tracking_receipt_ros_age_exceeded";
  if (s.check_steady_ns - s.receipt_steady_ns > s.tracking_budget_ns) return "tracking_receipt_steady_age_exceeded";
  if (s.health_ros_ns <= 0) return "health_evaluation_ros_invalid";
  if (s.health_steady_ns <= 0) return "health_evaluation_steady_invalid";
  if (s.check_ros_ns < s.health_ros_ns) return "health_ros_clock_regression";
  if (s.check_steady_ns < s.health_steady_ns) return "health_steady_clock_regression";
  if (s.check_ros_ns - s.health_ros_ns > s.health_budget_ns) return "health_ros_age_exceeded";
  if (s.check_steady_ns - s.health_steady_ns > s.health_budget_ns) return "health_steady_age_exceeded";
  return "";
}

// Temporary absence/unhealthy status or an elapsed original lease blocks motion,
// while identity, malformed evidence and clock contradictions remain fatal.
// Unknown future failures default to fatal rather than silently widening admission.
inline bool canonical_health_motion_unavailable(std::string_view failure) {
  return failure == "canonical_state_missing" || failure == "canonical_health_missing" ||
    failure == "canonical_health_last_state_missing" ||
    failure == "canonical_status_not_healthy" || failure == "canonical_measurement_not_accepted" ||
    failure == "tracking_receipt_ros_age_exceeded" ||
    failure == "tracking_receipt_steady_age_exceeded" || failure == "health_ros_age_exceeded" ||
    failure == "health_steady_age_exceeded";
}

inline std::string canonical_health_diagnostic(const CanonicalHealthSnapshot & s, const char * failure) {
  std::ostringstream out;
  out << "gate=" << failure << " check_ros_ns=" << s.check_ros_ns
      << " check_steady_ns=" << s.check_steady_ns
      << " tracking_source_ns=" << s.source_ns << " tracking_receipt_ros_ns=" << s.receipt_ros_ns
      << " tracking_receipt_steady_ns=" << s.receipt_steady_ns
      << " tracking_sequence=" << s.tracking_sequence << " tracking_identity=" << s.tracking_identity
      << " state_sequence=" << s.state_sequence << " reset_epoch=" << s.reset_epoch
      << " state_epoch=" << s.state_epoch << " health_epoch=" << s.health_epoch
      << " health_last_state_epoch=" << s.health_state_epoch
      << " health_last_state_sequence=" << s.health_state_sequence
      << " health_status_stamp_ns=" << s.status_stamp_ns
      << " health_evaluated_ros_ns=" << s.health_ros_ns
      << " health_evaluated_steady_ns=" << s.health_steady_ns
      << " tracking_budget_ns=" << s.tracking_budget_ns << " health_budget_ns=" << s.health_budget_ns
      << " has_state=" << s.has_state << " has_health=" << s.has_health
      << " has_status=" << s.has_status << " has_health_last_state=" << s.has_health_state
      << " status_healthy=" << s.status_healthy << " measurement_accepted=" << s.measurement_accepted
      << " canonical_queue_statistics=unavailable";
  return out.str();
}
}}  // namespace sunhorizon_navigation_runtime::detail
