#include <gtest/gtest.h>

#include "sunhorizon_navigation_server/localization_health_gate.hpp"

namespace {

using sunhorizon_navigation_server::LocalizationHealthGate;

TEST(LocalizationHealthGate, RequiresPostAcknowledgementEvidenceForFullDwell) {
  LocalizationHealthGate gate(100);
  gate.arm(10U, 1000);

  EXPECT_FALSE(gate.observe(10U, 1001, true, true));
  EXPECT_FALSE(gate.observe(11U, 1000, true, true));
  EXPECT_FALSE(gate.observe(11U, 999, true, true));
  EXPECT_FALSE(gate.observe(11U, 1001, true, true));
  EXPECT_FALSE(gate.observe(12U, 1100, true, true));
  EXPECT_TRUE(gate.observe(13U, 1101, true, true));
}

TEST(LocalizationHealthGate, LostAndUnacceptedEvidenceResetDwell) {
  LocalizationHealthGate gate(100);
  gate.arm(4U, 2000);

  EXPECT_FALSE(gate.observe(5U, 2001, true, true));
  EXPECT_FALSE(gate.observe(6U, 2051, true, true));
  EXPECT_FALSE(gate.observe(7U, 2061, true, false));
  EXPECT_FALSE(gate.observe(8U, 2071, true, true));
  EXPECT_FALSE(gate.observe(9U, 2161, false, true));
  EXPECT_FALSE(gate.observe(10U, 2171, true, true));
  EXPECT_FALSE(gate.observe(11U, 2270, true, true));
  EXPECT_TRUE(gate.observe(12U, 2271, true, true));
}

TEST(LocalizationHealthGate, NonIncreasingEvidenceResetsDwell) {
  LocalizationHealthGate gate(100);
  gate.arm(7U, 3000);

  EXPECT_FALSE(gate.observe(8U, 3001, true, true));
  EXPECT_FALSE(gate.observe(9U, 3051, true, true));
  EXPECT_FALSE(gate.observe(10U, 3041, true, true));
  EXPECT_FALSE(gate.observe(11U, 3101, true, true));
  EXPECT_TRUE(gate.observe(12U, 3201, true, true));
}

TEST(LocalizationHealthGate, DisarmInvalidatesOutstandingEvidence) {
  LocalizationHealthGate gate(100);
  gate.arm(7U, 4000);
  EXPECT_FALSE(gate.observe(8U, 4001, true, true));
  gate.disarm();

  EXPECT_FALSE(gate.newer_than_arm(9U, 4101));
  EXPECT_FALSE(gate.observe(9U, 4101, true, true));
}

TEST(LocalizationHealthGate, RearmClearsPartialDwell) {
  LocalizationHealthGate gate(100);
  gate.arm(1U, 5000);
  EXPECT_FALSE(gate.observe(2U, 5001, true, true));
  EXPECT_FALSE(gate.observe(3U, 5051, true, true));
  gate.arm(3U, 5051);
  EXPECT_FALSE(gate.observe(4U, 5101, true, true));
  EXPECT_FALSE(gate.observe(5U, 5150, true, true));
  EXPECT_TRUE(gate.observe(6U, 5201, true, true));
}

TEST(LocalizationHealthGate, RejectsNonPositiveDwell) {
  EXPECT_THROW(LocalizationHealthGate(0), std::invalid_argument);
  EXPECT_THROW(LocalizationHealthGate(-1), std::invalid_argument);
}

}  // namespace
