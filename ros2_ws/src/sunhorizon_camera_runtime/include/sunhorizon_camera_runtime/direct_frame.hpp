#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "sunhorizon_camera_runtime/callback_timing.hpp"
#include "sunhorizon_camera_runtime/direct_source_identity.hpp"
#include "sunhorizon_camera_runtime/depth_measurement_quality.hpp"

namespace sunhorizon_camera_runtime {
struct CameraStreamTime {
  std::uint64_t device_us{0}, global_us{0}, sdk_system_us{0};
};
// Immutable, owning snapshot. The SDK buffers are copied once into these image
// owners. No SDK buffer or mutable producer alias escapes through this API.
struct CameraFrame {
  using ConstSharedPtr = std::shared_ptr<const CameraFrame>;
  std::string role;
  std::shared_ptr<const CameraSourceIdentity> source_identity;
  sensor_msgs::msg::Image::ConstSharedPtr depth;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr depth_info;
  sensor_msgs::msg::Image::ConstSharedPtr color;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr color_info;
  // The exported 16UC1 image is normalized to millimetres, independently of
  // the original sensor value scale recorded in sdk_depth_scale_m.
  double depth_scale_m{0.001};
  double sdk_depth_scale_m{0.0};
  builtin_interfaces::msg::Time source_stamp;
  std::int64_t first_sdk_callback_ros_ns{-1};
  std::int64_t first_sdk_callback_steady_ns{-1};
  std::uint64_t sequence{0};
  CameraStreamTime depth_time, color_time;
  std::uint64_t depth_frame_index{0};
  // All ingress timestamp/shape/pair checks passed. Freshness and future
  // runtime failures are independently delivered through CameraFaultSink.
  bool source_validated{false};
  DepthMeasurementQuality::ConstSharedPtr measurement_quality;
  CallbackFrameTiming timing;
};
struct CameraFault {
  std::string reason;
  std::int64_t ros_ns{-1}, steady_ns{-1};
  bool fatal_latched{true};
};
// Both callbacks must perform bounded, nonblocking admission only and must
// outlive the owner. No ROS Image/CameraInfo subscriptions are used. Throwing
// frame sinks latch a fatal fault; fault sink exceptions cannot escape.
using CameraFrameSink = std::function<void(CameraFrame::ConstSharedPtr)>;
using CameraFaultSink = std::function<void(const CameraFault &)>;
// Empty sinks retain the standalone executable's legacy default behaviour.
// A frame sink selects latest-worker-v1 by default and requires a fault sink.
// raw_ros_publish_enabled defaults false for direct mode, true for standalone.
rclcpp::Node::SharedPtr create_camera_runtime(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
  CameraFrameSink frame_sink = {}, CameraFaultSink fault_sink = {});
// Stop admission, stop SDK capture, discard pending work and join all workers.
// Invoke from the external owner, never from a sink callback. The node must
// remain alive until executor callbacks have stopped before its final reset.
void stop_camera_runtime(const rclcpp::Node::SharedPtr & node) noexcept;
bool camera_runtime_failed(const rclcpp::Node::SharedPtr & node) noexcept;
}  // namespace sunhorizon_camera_runtime
