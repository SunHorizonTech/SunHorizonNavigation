#include <rclcpp/rclcpp.hpp>
#include <exception>
namespace sunhorizon_rgbd_adapter {rclcpp::Node::SharedPtr make_pointcloud_self_exclusion_ros_node();}
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(sunhorizon_rgbd_adapter::make_pointcloud_self_exclusion_ros_node());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("sunhorizon_pointcloud_self_exclusion"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
