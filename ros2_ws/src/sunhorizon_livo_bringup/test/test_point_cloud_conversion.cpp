#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "sunhorizon_livo_bringup/point_cloud_conversion.hpp"

namespace
{

sensor_msgs::msg::PointCloud2 make_cloud(bool all_invalid)
{
  using sensor_msgs::msg::PointField;
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "airy_lidar";
  cloud.is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    6,
    "x", 1, PointField::FLOAT32,
    "y", 1, PointField::FLOAT32,
    "z", 1, PointField::FLOAT32,
    "intensity", 1, PointField::UINT8,
    "timestamp", 1, PointField::FLOAT32,
    "ring", 1, PointField::UINT16);
  modifier.resize(3U);

  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<std::uint8_t> intensity(cloud, "intensity");
  sensor_msgs::PointCloud2Iterator<float> timestamp(cloud, "timestamp");
  sensor_msgs::PointCloud2Iterator<std::uint16_t> ring(cloud, "ring");
  for (std::size_t index = 0; index < 3U;
    ++index, ++x, ++y, ++z, ++intensity, ++timestamp, ++ring)
  {
    *x = all_invalid || index == 1U ?
      std::numeric_limits<float>::quiet_NaN() : static_cast<float>(index + 1U);
    *y = static_cast<float>(index + 4U);
    *z = static_cast<float>(index + 7U);
    *intensity = static_cast<std::uint8_t>(11U * (index + 1U));
    *timestamp = 0.01F * static_cast<float>(index + 1U);
    *ring = static_cast<std::uint16_t>(index + 10U);
  }
  return cloud;
}

TEST(PointCloudConversion, RemovesNonFiniteReturnsAndPreservesFiniteFields)
{
  auto input = make_cloud(false);
  sensor_msgs::msg::PointCloud2 output;
  std::string reason;

  ASSERT_TRUE(sunhorizon_livo_bringup::convert_airy_points(input, &output, &reason));
  EXPECT_TRUE(reason.empty());
  EXPECT_EQ(output.height, 1U);
  EXPECT_EQ(output.width, 2U);
  EXPECT_TRUE(output.is_dense);

  sensor_msgs::PointCloud2ConstIterator<float> x(output, "x");
  sensor_msgs::PointCloud2ConstIterator<float> intensity(output, "intensity");
  sensor_msgs::PointCloud2ConstIterator<double> timestamp(output, "timestamp");
  sensor_msgs::PointCloud2ConstIterator<std::uint16_t> ring(output, "ring");
  EXPECT_FLOAT_EQ(*x, 1.0F);
  EXPECT_FLOAT_EQ(*intensity, 11.0F);
  EXPECT_DOUBLE_EQ(*timestamp, static_cast<double>(0.01F));
  EXPECT_EQ(*ring, 10U);
  ++x;
  ++intensity;
  ++timestamp;
  ++ring;
  EXPECT_FLOAT_EQ(*x, 3.0F);
  EXPECT_FLOAT_EQ(*intensity, 33.0F);
  EXPECT_DOUBLE_EQ(*timestamp, static_cast<double>(0.03F));
  EXPECT_EQ(*ring, 12U);
}

TEST(PointCloudConversion, RejectsScanWithNoFiniteReturns)
{
  auto input = make_cloud(true);
  sensor_msgs::msg::PointCloud2 output;
  std::string reason;

  EXPECT_FALSE(sunhorizon_livo_bringup::convert_airy_points(input, &output, &reason));
  EXPECT_EQ(reason, "airy_pointcloud_no_finite_points");
}

}  // namespace
