#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "sunhorizon_3d_localizer/ndt_registration_quality.hpp"

namespace sunhorizon_3d_localizer {
namespace {

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

TEST(NdtRegistrationQuality, ComputesOverlapAndInlierRmse) {
  PointCloud target;
  target.push_back(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
  target.push_back(pcl::PointXYZ(1.0F, 0.0F, 0.0F));
  target.push_back(pcl::PointXYZ(2.0F, 0.0F, 0.0F));

  PointCloud aligned;
  aligned.push_back(pcl::PointXYZ(0.1F, 0.0F, 0.0F));
  aligned.push_back(pcl::PointXYZ(1.2F, 0.0F, 0.0F));
  aligned.push_back(pcl::PointXYZ(10.0F, 0.0F, 0.0F));

  const auto quality = compute_ndt_registration_quality(
      aligned, target, 0.5);
  EXPECT_EQ(quality.source_points, 3U);
  EXPECT_EQ(quality.target_points, 3U);
  EXPECT_EQ(quality.inlier_points, 2U);
  EXPECT_NEAR(
      quality.all_point_mean_squared_nearest_neighbor_m2,
      (0.01 + 0.04 + 64.0) / 3.0, 1e-5);
  EXPECT_NEAR(quality.source_inlier_fraction, 2.0 / 3.0, 1e-12);
  EXPECT_NEAR(
      quality.source_to_target_inlier_rmse_m, std::sqrt(0.025), 1e-6);
}

TEST(NdtRegistrationQuality, ReportsZeroOverlapWithoutInventingRmse) {
  PointCloud target;
  target.push_back(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
  PointCloud aligned;
  aligned.push_back(pcl::PointXYZ(5.0F, 0.0F, 0.0F));

  const auto quality = compute_ndt_registration_quality(
      aligned, target, 0.25);
  EXPECT_EQ(quality.inlier_points, 0U);
  EXPECT_DOUBLE_EQ(quality.all_point_mean_squared_nearest_neighbor_m2, 25.0);
  EXPECT_DOUBLE_EQ(quality.source_inlier_fraction, 0.0);
  EXPECT_TRUE(std::isnan(quality.source_to_target_inlier_rmse_m));
}

TEST(NdtRegistrationQuality, CountsInvalidSourcePointsAsNonOverlapping) {
  PointCloud target;
  target.push_back(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
  PointCloud aligned;
  aligned.push_back(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
  aligned.push_back(pcl::PointXYZ(
      std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F));

  const auto quality = compute_ndt_registration_quality(
      aligned, target, 0.25);
  EXPECT_EQ(quality.source_points, 2U);
  EXPECT_EQ(quality.inlier_points, 1U);
  EXPECT_TRUE(std::isnan(
      quality.all_point_mean_squared_nearest_neighbor_m2));
  EXPECT_DOUBLE_EQ(quality.source_inlier_fraction, 0.5);
  EXPECT_DOUBLE_EQ(quality.source_to_target_inlier_rmse_m, 0.0);
}

TEST(NdtRegistrationQuality, RejectsInvalidInlierDistance) {
  const PointCloud empty;
  EXPECT_THROW(
      compute_ndt_registration_quality(empty, empty, 0.0),
      std::invalid_argument);
  EXPECT_THROW(
      compute_ndt_registration_quality(
          empty, empty, std::numeric_limits<double>::infinity()),
      std::invalid_argument);
}

TEST(NdtRegistrationQuality, ReportsTargetCardinalityWithoutSource) {
  PointCloud target;
  target.push_back(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
  const PointCloud empty_source;

  const auto quality = compute_ndt_registration_quality(
      empty_source, target, 0.25);
  EXPECT_EQ(quality.source_points, 0U);
  EXPECT_EQ(quality.target_points, 1U);
  EXPECT_TRUE(std::isnan(quality.source_inlier_fraction));
  EXPECT_TRUE(std::isnan(
      quality.all_point_mean_squared_nearest_neighbor_m2));
  EXPECT_TRUE(std::isnan(quality.source_to_target_inlier_rmse_m));
}

TEST(NdtRegistrationQuality, ReusesIndexAndReplacesTargetExplicitly) {
  auto target = PointCloud::Ptr(new PointCloud());
  target->push_back(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
  NdtRegistrationQualityIndex index;
  index.set_target(target);

  PointCloud aligned;
  aligned.push_back(pcl::PointXYZ(0.1F, 0.0F, 0.0F));
  EXPECT_DOUBLE_EQ(index.compute(aligned, 0.25).source_inlier_fraction, 1.0);
  EXPECT_DOUBLE_EQ(index.compute(aligned, 0.25).source_inlier_fraction, 1.0);

  auto distant = PointCloud::Ptr(new PointCloud());
  distant->push_back(pcl::PointXYZ(5.0F, 0.0F, 0.0F));
  index.set_target(distant);
  EXPECT_DOUBLE_EQ(index.compute(aligned, 0.25).source_inlier_fraction, 0.0);
  index.clear();
  EXPECT_TRUE(std::isnan(index.compute(aligned, 0.25).source_inlier_fraction));
}

}  // namespace
}  // namespace sunhorizon_3d_localizer
