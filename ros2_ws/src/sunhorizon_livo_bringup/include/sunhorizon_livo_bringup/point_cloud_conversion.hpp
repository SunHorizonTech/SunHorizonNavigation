#pragma once

#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace sunhorizon_livo_bringup
{

// Convert Robot A's Airy layout to the pinned FAST-LIVO2 layout. Individual
// non-finite returns are normal LiDAR no-return samples and are removed; an
// empty or entirely invalid scan is rejected.
bool convert_airy_points(
  const sensor_msgs::msg::PointCloud2 & input,
  sensor_msgs::msg::PointCloud2 * output,
  std::string * reason);

}  // namespace sunhorizon_livo_bringup
