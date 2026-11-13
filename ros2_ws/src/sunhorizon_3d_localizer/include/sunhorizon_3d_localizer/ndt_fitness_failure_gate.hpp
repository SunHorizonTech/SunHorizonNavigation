#pragma once

#include <cstdint>

namespace sunhorizon_3d_localizer {

enum class NdtFitnessFailureDecision {
  kHoldLastAccepted,
  kLost,
};

class NdtFitnessFailureGate {
 public:
  explicit NdtFitnessFailureGate(
      std::uint32_t minimum_consecutive_failures_for_lost = 2);

  NdtFitnessFailureDecision observe_failure();
  void observe_success();
  void reset();

  std::uint32_t consecutive_failures() const;
  std::uint32_t minimum_consecutive_failures_for_lost() const;

 private:
  std::uint32_t minimum_consecutive_failures_for_lost_{2};
  std::uint32_t consecutive_failures_{0};
};

}  // namespace sunhorizon_3d_localizer
