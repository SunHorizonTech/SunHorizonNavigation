#include "sunhorizon_obstacle_coverage/coverage_direct_runtime.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>
int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  auto node=sunhorizon_obstacle_coverage::make_coverage_standalone();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),3U);
  executor.add_node(node);executor.spin();rclcpp::shutdown();return 0;
}
