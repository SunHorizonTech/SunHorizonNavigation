#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "sunhorizon_rgbd_adapter/rgbd_freshness.hpp"

using sunhorizon_rgbd_adapter::measurement_age_seconds;
using sunhorizon_rgbd_adapter::measurement_fresh;

TEST(RgbdFreshness, PreservesSignedAgeAndRejectsFutureMeasurement) {
  EXPECT_NEAR(measurement_age_seconds(10.0, 9.7), 0.3, 1e-12);
  EXPECT_NEAR(measurement_age_seconds(10.0, 10.1), -0.1, 1e-12);
  EXPECT_FALSE(measurement_fresh(-0.1, 0.3));
}

TEST(RgbdFreshness, AcceptsExactBoundaryAndRejectsOlderMeasurement) {
  EXPECT_TRUE(measurement_fresh(0.3, 0.3));
  EXPECT_FALSE(measurement_fresh(0.300001, 0.3));
}

TEST(RgbdFreshness, RejectsInvalidAgeStampAndTimeout) {
  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(std::isinf(measurement_age_seconds(10.0, 0.0)));
  EXPECT_TRUE(std::isinf(measurement_age_seconds(nan, 9.9)));
  EXPECT_FALSE(measurement_fresh(infinity, 0.3));
  EXPECT_FALSE(measurement_fresh(nan, 0.3));
  EXPECT_FALSE(measurement_fresh(0.1, infinity));
  EXPECT_FALSE(measurement_fresh(0.1, 0.0));
}
