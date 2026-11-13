#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "sunhorizon_camera_runtime/direct_frame.hpp"
#include "sunhorizon_navigation_interfaces/msg/rgbd_status.hpp"
#include "sunhorizon_perception_runtime/rgbd_health_core.hpp"

namespace sunhorizon_perception_runtime {
// Reads the original rgbd_adapter.yaml names; optional prefix scopes those
// exact field names. Already-declared parameters are read without redeclaration.
RgbdHealthConfig read_rgbd_health_config(rclcpp::Node & node, const std::string & prefix = "");

class RgbdHealth {
 public:
  explicit RgbdHealth(rclcpp::Node & node, const std::string & prefix = "");
  explicit RgbdHealth(RgbdHealthConfig config);
  // Call on the role's computation worker, never in the SDK admission sink:
  // this validates sampled pixels but performs no projection or publication.
  RgbdAdmission admit(const sunhorizon_camera_runtime::CameraFrame & frame,
    std::int64_t now_ros_ns, std::int64_t now_steady_ns);
  // Commit ONLY after successful computation and body-cloud publication.
  // The exact cloud stamp must match the admitted original depth source stamp.
  bool complete(const sunhorizon_camera_runtime::CameraFrame & frame,
    std::int64_t cloud_source_stamp_ns, std::int64_t now_ros_ns, std::int64_t now_steady_ns);
  void reject_computation(const sunhorizon_camera_runtime::CameraFrame & frame, const std::string & reason, bool upstream_until_completion=false);
  void fault(const std::string & reason);
  sunhorizon_navigation_interfaces::msg::RgbdStatus snapshot(
    std::int64_t now_ros_ns, std::int64_t now_steady_ns);
  const RgbdHealthConfig & config() const noexcept;
 private:
  RgbdHealthCore core_;
};
}  // namespace sunhorizon_perception_runtime
