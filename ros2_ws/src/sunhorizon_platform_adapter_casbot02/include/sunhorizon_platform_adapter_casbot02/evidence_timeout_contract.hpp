#pragma once

#include <cmath>

namespace sunhorizon_platform_adapter_casbot02 {

// Robot A's EtherCAT proof producer is slower than PMU evidence. Keep its
// allowance separate and bounded so tuning it cannot weaken other safety data.
inline constexpr double kMaxEthercatEvidenceTimeoutSec = 1.5;

inline bool valid_ethercat_evidence_timeout(double timeout_sec) {
  return std::isfinite(timeout_sec) && timeout_sec > 0.0 &&
      timeout_sec <= kMaxEthercatEvidenceTimeoutSec;
}

}  // namespace sunhorizon_platform_adapter_casbot02
