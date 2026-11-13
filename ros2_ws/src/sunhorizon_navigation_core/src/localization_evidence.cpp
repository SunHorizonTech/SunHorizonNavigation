#include "sunhorizon_navigation_core/localization_evidence.hpp"

#include <cmath>
#include <stdexcept>

namespace sunhorizon_navigation_core {

LocalizationEvidenceGate::LocalizationEvidenceGate(LocalizationEvidenceLimits limits)
    : limits_(limits) {
  if (!std::isfinite(limits_.maximum_stamp_delta_sec) ||
      limits_.maximum_stamp_delta_sec < 0.0) {
    throw std::invalid_argument("invalid localization evidence limits");
  }
}

LocalizationEvidenceDecision LocalizationEvidenceGate::evaluate(
    const LocalizationEvidence& evidence) const {
  if (!std::isfinite(evidence.pose_stamp_sec) ||
      !std::isfinite(evidence.evidence_stamp_sec) || evidence.pose_stamp_sec <= 0.0 ||
      evidence.evidence_stamp_sec <= 0.0) {
    return {false, "localization_evidence_invalid_stamp"};
  }
  if (std::abs(evidence.pose_stamp_sec - evidence.evidence_stamp_sec) >
      limits_.maximum_stamp_delta_sec) {
    return {false, "localization_evidence_stamp_mismatch"};
  }
  if (evidence.global_match_id <= 0 && evidence.proximity_match_id <= 0 &&
      evidence.landmark_match_id <= 0) {
    return {false, "localization_match_evidence_missing"};
  }
  return {true, "ok"};
}

}  // namespace sunhorizon_navigation_core
