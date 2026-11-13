#include <gtest/gtest.h>

#include <limits>

#include "sunhorizon_3d_localizer/local_map_target_logic.hpp"

namespace {

using sunhorizon_3d_localizer::LocalMapTargetCenter;
using sunhorizon_3d_localizer::LocalMapTargetCropLimits;
using sunhorizon_3d_localizer::local_map_target_contains;
using sunhorizon_3d_localizer::local_map_target_has_minimum_points;
using sunhorizon_3d_localizer::local_map_target_refresh_required;

TEST(LocalMapTargetCrop, SelectsClosedXyCylinderWithBoundedRelativeZ) {
  const LocalMapTargetCenter center{10.0, -4.0, 1.5};
  const LocalMapTargetCropLimits limits{5.0, 2.0};

  EXPECT_TRUE(local_map_target_contains(13.0, 0.0, 3.5, center, limits));
  EXPECT_TRUE(local_map_target_contains(7.0, -8.0, -0.5, center, limits));
  EXPECT_FALSE(local_map_target_contains(13.01, 0.0, 1.5, center, limits));
  EXPECT_FALSE(local_map_target_contains(10.0, -4.0, 3.5001, center, limits));
}

TEST(LocalMapTargetCrop, RejectsNonFinitePointsAndInvalidLimits) {
  const LocalMapTargetCenter center{0.0, 0.0, 0.0};
  EXPECT_FALSE(local_map_target_contains(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, center, {5.0, 2.0}));
  EXPECT_FALSE(local_map_target_contains(0.0, 0.0, 0.0, center, {0.0, 2.0}));
  EXPECT_FALSE(local_map_target_contains(0.0, 0.0, 0.0, center, {5.0, -1.0}));
}

TEST(LocalMapTargetRefresh, UsesXyDistanceAndIncludesThresholdBoundary) {
  const LocalMapTargetCenter current{1.0, 2.0, -5.0};

  EXPECT_FALSE(local_map_target_refresh_required(
      current, {3.99, 2.0, 100.0}, 3.0));
  EXPECT_TRUE(local_map_target_refresh_required(
      current, {4.0, 2.0, 100.0}, 3.0));
  EXPECT_TRUE(local_map_target_refresh_required(
      current, {1.0, -1.01, -100.0}, 3.0));
}

TEST(LocalMapTargetRefresh, InvalidInputsCannotTriggerRefresh) {
  const LocalMapTargetCenter current{0.0, 0.0, 0.0};
  EXPECT_FALSE(local_map_target_refresh_required(current, {1.0, 0.0, 0.0}, 0.0));
  EXPECT_FALSE(local_map_target_refresh_required(
      current, {std::numeric_limits<double>::infinity(), 0.0, 0.0}, 1.0));
}

TEST(LocalMapTargetReadiness, RequiresConfiguredMinimumPointCount) {
  EXPECT_FALSE(local_map_target_has_minimum_points(999U, 1000U));
  EXPECT_TRUE(local_map_target_has_minimum_points(1000U, 1000U));
  EXPECT_TRUE(local_map_target_has_minimum_points(1001U, 1000U));
  EXPECT_FALSE(local_map_target_has_minimum_points(1000U, 0U));
}

}  // namespace
