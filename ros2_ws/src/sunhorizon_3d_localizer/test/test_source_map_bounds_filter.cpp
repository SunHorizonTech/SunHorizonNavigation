#include <gtest/gtest.h>

#include <Eigen/Core>

#include <limits>
#include <stdexcept>

#include "sunhorizon_3d_localizer/source_map_bounds_filter.hpp"

namespace sunhorizon_3d_localizer {
namespace {

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

TEST(SourceMapBoundsFilter, UsesInitialMapToSourceTransformAndKeepsSourceFrame) {
  PointCloud target;
  target.push_back(pcl::PointXYZ(10.0F, 20.0F, -1.0F));
  target.push_back(pcl::PointXYZ(12.0F, 22.0F, 1.0F));
  const auto bounds = compute_map_axis_aligned_bounds(target);

  PointCloud source;
  source.push_back(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
  source.push_back(pcl::PointXYZ(1.0F, -1.0F, 0.0F));
  source.push_back(pcl::PointXYZ(5.0F, 5.0F, 0.0F));
  Eigen::Matrix4f initial = Eigen::Matrix4f::Identity();
  // 90 degree source-to-map yaw followed by translation. The second source
  // point becomes map (11, 21, 0), proving transform direction and rotation.
  initial(0, 0) = 0.0F;
  initial(0, 1) = -1.0F;
  initial(1, 0) = 1.0F;
  initial(1, 1) = 0.0F;
  initial(0, 3) = 10.0F;
  initial(1, 3) = 20.0F;

  const auto result = filter_source_by_map_bounds(
      source, initial, bounds, {0.0, 0.60, 2U});
  ASSERT_TRUE(result.accepted);
  EXPECT_EQ(result.input_points, 3U);
  EXPECT_EQ(result.retained_points, 2U);
  EXPECT_EQ(result.rejected_points, 1U);
  EXPECT_NEAR(result.retained_fraction, 2.0 / 3.0, 1e-12);
  ASSERT_EQ(result.retained->size(), 2U);
  EXPECT_FLOAT_EQ(result.retained->at(0).x, 0.0F);
  EXPECT_FLOAT_EQ(result.retained->at(1).x, 1.0F);
  EXPECT_FLOAT_EQ(result.retained->at(1).y, -1.0F);
}

TEST(SourceMapBoundsFilter, MarginIsInclusive) {
  PointCloud target;
  target.push_back(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
  const auto bounds = compute_map_axis_aligned_bounds(target);
  PointCloud source;
  source.push_back(pcl::PointXYZ(1.0F, -1.0F, 1.0F));

  const auto result = filter_source_by_map_bounds(
      source, Eigen::Matrix4f::Identity(), bounds, {1.0, 1.0, 1U});
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.retained_points, 1U);
}

TEST(SourceMapBoundsFilter, RejectsTooFewPointsBeforeFraction) {
  PointCloud target;
  target.push_back(pcl::PointXYZ(-1.0F, -1.0F, -1.0F));
  target.push_back(pcl::PointXYZ(1.0F, 1.0F, 1.0F));
  const auto bounds = compute_map_axis_aligned_bounds(target);
  PointCloud source;
  source.push_back(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
  source.push_back(pcl::PointXYZ(0.5F, 0.0F, 0.0F));

  const auto result = filter_source_by_map_bounds(
      source, Eigen::Matrix4f::Identity(), bounds, {0.0, 0.5, 3U});
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "source_map_bounds_retained_points_limit");
}

TEST(SourceMapBoundsFilter, RejectsTinySubsetEvenWhenPointMinimumPasses) {
  PointCloud target;
  target.push_back(pcl::PointXYZ(-1.0F, -1.0F, -1.0F));
  target.push_back(pcl::PointXYZ(1.0F, 1.0F, 1.0F));
  const auto bounds = compute_map_axis_aligned_bounds(target);
  PointCloud source;
  source.push_back(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
  source.push_back(pcl::PointXYZ(0.5F, 0.0F, 0.0F));
  source.push_back(pcl::PointXYZ(5.0F, 0.0F, 0.0F));
  source.push_back(pcl::PointXYZ(6.0F, 0.0F, 0.0F));

  const auto result = filter_source_by_map_bounds(
      source, Eigen::Matrix4f::Identity(), bounds, {0.0, 0.6, 2U});
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "source_map_bounds_retained_fraction_limit");
  EXPECT_DOUBLE_EQ(result.retained_fraction, 0.5);
}

TEST(SourceMapBoundsFilter, InvalidGeometryFailsClosed) {
  PointCloud target;
  target.push_back(pcl::PointXYZ(
      std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F));
  EXPECT_THROW(compute_map_axis_aligned_bounds(target), std::invalid_argument);

  PointCloud source;
  source.push_back(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
  Eigen::Matrix4f invalid = Eigen::Matrix4f::Identity();
  invalid(0, 0) = std::numeric_limits<float>::infinity();
  const MapAxisAlignedBounds bounds{-1.0, -1.0, -1.0, 1.0, 1.0, 1.0};
  EXPECT_THROW(
      filter_source_by_map_bounds(source, invalid, bounds, {1.0, 0.6, 1U}),
      std::invalid_argument);
  EXPECT_THROW(
      filter_source_by_map_bounds(
          source, Eigen::Matrix4f::Identity(), bounds, {1.0, 0.0, 1U}),
      std::invalid_argument);
}

}  // namespace
}  // namespace sunhorizon_3d_localizer
