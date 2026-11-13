#pragma once

#include <string>

namespace sunhorizon_navigation_core {

struct LocalizationEvidenceLimits {
  double maximum_stamp_delta_sec{0.005};
};

struct LocalizationEvidence {
  double pose_stamp_sec{0.0};
  double evidence_stamp_sec{0.0};
  int global_match_id{0};
  int proximity_match_id{0};
  int landmark_match_id{0};
};

struct LocalizationEvidenceDecision {
  bool accepted{false};
  std::string reason;
};

class LocalizationEvidenceGate {
 public:
  explicit LocalizationEvidenceGate(LocalizationEvidenceLimits limits = {});
  LocalizationEvidenceDecision evaluate(const LocalizationEvidence& evidence) const;

 private:
  LocalizationEvidenceLimits limits_;
};

}  // namespace sunhorizon_navigation_core
