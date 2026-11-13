#include <rclcpp/rclcpp.hpp>
#include "sunhorizon_localization_manager/localization_direct_runtime.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(sunhorizon_localization_manager::make_localization_ros_node());
  rclcpp::shutdown();
  return 0;
}
