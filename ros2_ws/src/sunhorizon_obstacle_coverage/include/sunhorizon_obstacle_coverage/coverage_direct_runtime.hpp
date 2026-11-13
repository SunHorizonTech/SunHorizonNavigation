#pragma once
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sunhorizon_navigation_interfaces/msg/rgbd_status.hpp>
#include <sunhorizon_navigation_interfaces/msg/spatial_coverage_evidence.hpp>
#include "sunhorizon_obstacle_coverage/spatial_delivery_result.hpp"

namespace sunhorizon_obstacle_coverage {
using SpatialWire=sunhorizon_navigation_interfaces::msg::SpatialCoverageEvidence;
struct CoverageDirectOptions {
  rclcpp::NodeOptions node_options;
  std::shared_ptr<tf2_ros::Buffer> transforms;
  // Actual producer objects, obtained from the one already-constructed owner.
  // No caller-supplied GID or synthetic MessageInfo is accepted.
  rclcpp::Node::SharedPtr spatial_owner;
  rclcpp::Publisher<SpatialWire>::SharedPtr spatial_publisher;
  std::function<bool()> producer_failed;
};
class CoverageDirectRuntime {
 public:
  virtual ~CoverageDirectRuntime()=default;
  virtual rclcpp::Node::SharedPtr node()=0;
  // Call only on dedicated delivery workers, never SDK/FK/engine callbacks.
  // Source stamps are unchanged; receipt is the ORIGINAL SDK first receipt.
  virtual void receive_airy(sensor_msgs::msg::PointCloud2::ConstSharedPtr,int64_t receipt_ros_ns)=0;
  virtual void receive_belly(sensor_msgs::msg::PointCloud2::ConstSharedPtr,int64_t receipt_ros_ns)=0;
  virtual void receive_rgbd(sunhorizon_navigation_interfaces::msg::RgbdStatus::ConstSharedPtr)=0;
  virtual void receive_region(diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr)=0;
  // MotionUnavailable is delivered but not accepted and cannot qualify startup.
  // The original tracker stays latched; no new frame conceals a validity gap.
  virtual SpatialDeliveryResult receive_spatial(SpatialWire::ConstSharedPtr)=0;
  virtual void revoke(const std::string&)=0;
  virtual void stop()=0; // After callers and the node's executor are quiescent.
};
std::shared_ptr<CoverageDirectRuntime> make_coverage_direct_runtime(CoverageDirectOptions);
rclcpp::Node::SharedPtr make_coverage_standalone(const rclcpp::NodeOptions& = rclcpp::NodeOptions());
}  // namespace sunhorizon_obstacle_coverage
