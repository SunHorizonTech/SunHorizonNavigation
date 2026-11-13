#pragma once

#include "sunhorizon_rgbd_adapter/dynamic_self_model.hpp"

namespace sunhorizon_rgbd_adapter {
// Equal source/anchor timestamps need only the exact-time relative chain.
// Different timestamps retain odom motion compensation; neither path uses latest.
inline geometry_msgs::msg::TransformStamped spatial_source_transform(
    tf2_ros::Buffer& buffer, const std::string& common_frame,
    const std::string& optical_frame, int64_t source_ns, int64_t anchor_ns,
    rcl_clock_type_t clock_type, int64_t steady_deadline) {
  const rclcpp::Time source_stamp(source_ns, clock_type);
  if (source_ns == anchor_ns)
    return buffer.lookupTransform(common_frame, optical_frame, source_stamp,
        dynamic_self_model::remaining(steady_deadline));
  return buffer.lookupTransform(common_frame, rclcpp::Time(anchor_ns, clock_type),
      optical_frame, source_stamp, "odom", dynamic_self_model::remaining(steady_deadline));
}
}  // namespace sunhorizon_rgbd_adapter
