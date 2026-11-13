#include <stdexcept>

#include <gtest/gtest.h>

#include "sunhorizon_3d_localizer/ndt_fitness_failure_gate.hpp"

using sunhorizon_3d_localizer::NdtFitnessFailureDecision;
using sunhorizon_3d_localizer::NdtFitnessFailureGate;

TEST(NdtFitnessFailureGate, HoldsOneIsolatedFailure) {
  NdtFitnessFailureGate gate(2);
  EXPECT_EQ(
      gate.observe_failure(), NdtFitnessFailureDecision::kHoldLastAccepted);
  EXPECT_EQ(gate.consecutive_failures(), 1U);
  gate.observe_success();
  EXPECT_EQ(gate.consecutive_failures(), 0U);
}

TEST(NdtFitnessFailureGate, LosesOnConfiguredConsecutiveFailure) {
  NdtFitnessFailureGate gate(2);
  EXPECT_EQ(
      gate.observe_failure(), NdtFitnessFailureDecision::kHoldLastAccepted);
  EXPECT_EQ(gate.observe_failure(), NdtFitnessFailureDecision::kLost);
  EXPECT_EQ(gate.observe_failure(), NdtFitnessFailureDecision::kLost);
  gate.reset();
  EXPECT_EQ(gate.consecutive_failures(), 0U);
}

TEST(NdtFitnessFailureGate, RejectsZeroThreshold) {
  EXPECT_THROW(NdtFitnessFailureGate(0), std::invalid_argument);
}
