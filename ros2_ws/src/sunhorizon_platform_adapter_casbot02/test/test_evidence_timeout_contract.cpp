#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <limits>

#include "sunhorizon_platform_adapter_casbot02/evidence_timeout_contract.hpp"

using sunhorizon_platform_adapter_casbot02::kMaxEthercatEvidenceTimeoutSec;
using sunhorizon_platform_adapter_casbot02::valid_ethercat_evidence_timeout;

int main() {
  assert(valid_ethercat_evidence_timeout(0.001));
  assert(valid_ethercat_evidence_timeout(kMaxEthercatEvidenceTimeoutSec));
  assert(!valid_ethercat_evidence_timeout(0.0));
  assert(!valid_ethercat_evidence_timeout(-0.1));
  assert(!valid_ethercat_evidence_timeout(kMaxEthercatEvidenceTimeoutSec + 0.001));
  assert(!valid_ethercat_evidence_timeout(
      std::numeric_limits<double>::infinity()));
  assert(!valid_ethercat_evidence_timeout(
      std::numeric_limits<double>::quiet_NaN()));
  return 0;
}
