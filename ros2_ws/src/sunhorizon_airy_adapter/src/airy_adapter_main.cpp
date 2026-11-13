#include "sunhorizon_airy_adapter/direct_adapter.hpp"
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto adapter = sunhorizon_airy_adapter::make_airy_adapter_node();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3U);
  executor.add_node(adapter);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
