#include <cstdio>
#include <exception>
#include "sunhorizon_camera_runtime/direct_frame.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = sunhorizon_camera_runtime::create_camera_runtime();
    rclcpp::spin(node);
    const bool failed = sunhorizon_camera_runtime::camera_runtime_failed(node);
    sunhorizon_camera_runtime::stop_camera_runtime(node);
    node.reset();
    if (rclcpp::ok()) rclcpp::shutdown();
    return failed ? 1 : 0;
  } catch (const std::exception & error) {
    std::fprintf(stderr, "sunhorizon camera startup failure: %s\n", error.what());
  } catch (...) {
    std::fprintf(stderr, "sunhorizon camera startup failure: unknown exception\n");
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return 1;
}
