#include <stdexcept>
#include "sunhorizon_camera_runtime/direct_frame.hpp"

namespace sunhorizon_camera_runtime {
rclcpp::Node::SharedPtr create_camera_runtime(const rclcpp::NodeOptions &,
  CameraFrameSink, CameraFaultSink)
{
  throw std::runtime_error("camera runtime was built without the official OrbbecSDK; "
    "rebuild with SUNHORIZON_CAMERA_RUNTIME_REQUIRE_SDK=ON and ORBBEC_SDK_ROOT");
}
void stop_camera_runtime(const rclcpp::Node::SharedPtr &) noexcept {}
bool camera_runtime_failed(const rclcpp::Node::SharedPtr &) noexcept {return true;}
}  // namespace sunhorizon_camera_runtime
