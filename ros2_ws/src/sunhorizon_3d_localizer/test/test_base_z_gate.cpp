#include <limits>

#include <gtest/gtest.h>

#include <sunhorizon_3d_localizer/base_z_gate.hpp>

TEST(BaseZGate, AcceptsCandidateWithinMapRelativeBand) {
  EXPECT_TRUE(sunhorizon_3d_localizer::base_z_within_reference(
      -0.300, -0.21926, 0.15));
  EXPECT_TRUE(sunhorizon_3d_localizer::base_z_within_reference(
      -0.080, -0.21926, 0.15));
}

TEST(BaseZGate, RejectsCandidateOutsideMapRelativeBand) {
  EXPECT_FALSE(sunhorizon_3d_localizer::base_z_within_reference(
      -0.370, -0.21926, 0.15));
  EXPECT_FALSE(sunhorizon_3d_localizer::base_z_within_reference(
      -0.06925, -0.21926, 0.15));
}

TEST(BaseZGate, RejectsInvalidContract) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(sunhorizon_3d_localizer::base_z_within_reference(
      nan, -0.21926, 0.15));
  EXPECT_FALSE(sunhorizon_3d_localizer::base_z_within_reference(
      -0.21926, nan, 0.15));
  EXPECT_FALSE(sunhorizon_3d_localizer::base_z_within_reference(
      -0.21926, -0.21926, 0.0));
}
