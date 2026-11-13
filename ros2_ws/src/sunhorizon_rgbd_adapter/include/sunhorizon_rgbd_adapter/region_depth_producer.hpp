#pragma once
#include <array>
#include <functional>
#include <memory>
#include <string>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sunhorizon_obstacle_coverage/region_depth_support.hpp>
#include "sunhorizon_rgbd_adapter/region_depth_pairing.hpp"
#include "sunhorizon_rgbd_adapter/region_tf_diagnostics.hpp"

namespace sunhorizon_rgbd_adapter {
struct RegionProducerOptions {
  std::string region_id, base_frame, optical_frame, depth_topic, info_topic, diagnostics_topic;
  std::string expected_calibration, expected_contract;
  std::array<double, 3> box_min{}, box_max{};
  double clear_margin_m{.02}, depth_scale_m{.001};
  double minimum_depth_m{.2}, maximum_depth_m{8.}, max_age_sec{.3}, tf_wait_sec{0.};
  sunhorizon_obstacle_coverage::RegionCalibrationContract accepted;
  bool shadow_only_enabled{false};
  bool hold_complete_measurement_enabled{false};
  bool typed_evaluation_diagnostics_enabled{false};
};
// Same function is used to validate actual inputs and form the pinned contract.
std::string region_calibration_contract(const sensor_msgs::msg::CameraInfo& info);
std::string region_producer_contract(const RegionProducerOptions& options);

// Owned only when explicitly enabled. Image/Info methods must use the same
// mutually-exclusive executor group; the TF listener has its own bounded buffer.
class RegionDepthProducer {
 public:
  using Image = sensor_msgs::msg::Image;
  using Info = sensor_msgs::msg::CameraInfo;
  using Publish = std::function<void(const diagnostic_msgs::msg::DiagnosticArray&)>;
  using Lookup = std::function<geometry_msgs::msg::TransformStamped(
      const std::string&, const std::string&, int64_t, double)>;
  RegionDepthProducer(RegionProducerOptions options, rclcpp::Clock::SharedPtr clock,
                      std::string node_name, Publish publish, Lookup lookup = {});
  // Synchronous direct path. The immutable image/Info and unfiltered axial-Z
  // raster are borrowed only during this call; no XYZ/projection/model work.
  // Lookup must read the same authoritative exact-source TF cache. A supplied
  // Lookup suppresses the legacy private TF listener entirely.
  void prepared(Image::ConstSharedPtr image, Info::ConstSharedPtr info,
      const float* raw_depth_m, std::size_t elements, uint64_t frame_sequence,
      int64_t first_receipt_ros_ns, int64_t first_receipt_steady_ns);
  void image(Image::ConstSharedPtr message);
  void info(Info::ConstSharedPtr message);
 private:
  friend struct RegionDepthProducerTestPeer;  // Deterministic report-boundary tests; no runtime hook.
  using Pairing = RegionExactPairing<Image, Info>;
  void begin_image(Image::ConstSharedPtr message, uint64_t sequence,
      int64_t receipt_ros_ns, int64_t receipt_steady_ns);
  void evaluate();
  void reset_shadow() { shadow_evaluated_ = false; shadow_support_ = {}; }
  void report(const std::string& reason,
              sunhorizon_obstacle_coverage::RegionDepthSupport support = {});
  RegionProducerOptions options_;
  rclcpp::Clock::SharedPtr clock_;
  std::string node_name_, contract_;
  Publish publish_;
  Lookup lookup_;
  const float* prepared_depth_{nullptr};
  std::size_t prepared_elements_{0};
  Pairing pairs_;
  RegionClockGuard report_clock_;
  RegionTfDiagnostics tf_diagnostics_;
  bool shadow_evaluated_{false};
  sunhorizon_obstacle_coverage::RegionDepthSupport shadow_support_;
  Pairing::Pending current_;
  Pairing::Pending last_reported_;  // Metadata only; protects a held measurement from late Info conflict.
  uint64_t frame_sequence_{0}, evaluation_sequence_{0};
  int64_t paired_stamp_{0}, tf_stamp_{0}, work_wall_ns_{0}, work_cpu_ns_{0};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};
}  // namespace sunhorizon_rgbd_adapter
