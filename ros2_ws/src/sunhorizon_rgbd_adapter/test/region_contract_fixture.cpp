// Synthetic test fixture only. No rclcpp::init, ROS entities, TF, or robot access.
#include <iostream>
#include "sunhorizon_rgbd_adapter/region_depth_producer.hpp"
int main(int argc, char** argv) {
  const bool typed = argc == 2 && std::string(argv[1]) == "--typed-evaluation-diagnostics";
  if (argc != 1 && !typed) return 2;
  using namespace sunhorizon_rgbd_adapter;
  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = "camera"; info.width = info.height = 16;
  info.distortion_model = "plumb_bob"; info.d = {0., 0., 0., 0., 0.};
  info.k = {20., 0., 8., 0., 20., 8., 0., 0., 1.};
  info.r = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
  info.p = {20., 0., 8., 0., 0., 20., 8., 0., 0., 0., 1., 0.};
  RegionProducerOptions options;
  options.typed_evaluation_diagnostics_enabled = typed;
  options.region_id = "test_region";
  options.base_frame = "base"; options.optical_frame = "camera";
  options.depth_topic = "/region_test/image"; options.info_topic = "/region_test/info";
  options.diagnostics_topic = "/region_test/support";
  options.box_min = {-.1, -.1, .4}; options.box_max = {.1, .1, .45};
  options.accepted = {true, true, true, true, true};  // Synthetic model only.
  options.expected_calibration = region_calibration_contract(info);
  std::cout << options.expected_calibration << '\n' << region_producer_contract(options) << '\n';
}
