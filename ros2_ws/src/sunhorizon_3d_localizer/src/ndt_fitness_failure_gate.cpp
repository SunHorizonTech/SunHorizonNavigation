#include "sunhorizon_3d_localizer/ndt_fitness_failure_gate.hpp"

#include <limits>
#include <stdexcept>

namespace sunhorizon_3d_localizer {

NdtFitnessFailureGate::NdtFitnessFailureGate(
    std::uint32_t minimum_consecutive_failures_for_lost)
    : minimum_consecutive_failures_for_lost_(
          minimum_consecutive_failures_for_lost) {
  if (minimum_consecutive_failures_for_lost_ == 0) {
    throw std::invalid_argument(
        "minimum_consecutive_failures_for_lost_must_be_positive");
  }
}

NdtFitnessFailureDecision NdtFitnessFailureGate::observe_failure() {
  if (consecutive_failures_ < std::numeric_limits<std::uint32_t>::max()) {
    ++consecutive_failures_;
  }
  return consecutive_failures_ >= minimum_consecutive_failures_for_lost_
      ? NdtFitnessFailureDecision::kLost
      : NdtFitnessFailureDecision::kHoldLastAccepted;
}

void NdtFitnessFailureGate::observe_success() { consecutive_failures_ = 0; }

void NdtFitnessFailureGate::reset() { consecutive_failures_ = 0; }

std::uint32_t NdtFitnessFailureGate::consecutive_failures() const {
  return consecutive_failures_;
}

std::uint32_t
NdtFitnessFailureGate::minimum_consecutive_failures_for_lost() const {
  return minimum_consecutive_failures_for_lost_;
}

}  // namespace sunhorizon_3d_localizer
