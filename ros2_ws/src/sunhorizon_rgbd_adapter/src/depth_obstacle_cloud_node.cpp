#include <cmath>
#include <algorithm>
#include <vector>
#include <chrono>
#include <ctime>
#include <memory>
#include <stdexcept>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include "sunhorizon_rgbd_adapter/depth_components.hpp"
#include "sunhorizon_rgbd_adapter/depth_projection.hpp"
#include "sunhorizon_rgbd_adapter/rgbd_freshness.hpp"
#include "sunhorizon_rgbd_adapter/region_depth_producer.hpp"

namespace sunhorizon_rgbd_adapter {
namespace {
int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
int64_t thread_cpu_ns() {
  timespec value{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value) != 0) return -1;
  return int64_t(value.tv_sec) * 1000000000 + value.tv_nsec;
}
struct StageTiming {
  bool executed{false};
  int64_t wall_ns{0}, cpu_ns{0};
};
// Scoped only around the existing three operations; exceptions retain elapsed time.
class StageTimer {
 public:
  explicit StageTimer(StageTiming& result)
      : result_(result), wall_(steady_ns()), cpu_(thread_cpu_ns()) {
    result_.executed = true;
  }
  ~StageTimer() {
    const auto cpu = thread_cpu_ns();
    result_.wall_ns = steady_ns() - wall_;
    result_.cpu_ns = cpu_ >= 0 && cpu >= cpu_ ? cpu - cpu_ : -1;
  }
 private:
  StageTiming& result_;
  int64_t wall_, cpu_;
};
}
class DepthObstacleCloud final : public rclcpp::Node {
 public:
  DepthObstacleCloud() : Node("sunhorizon_depth_obstacle_cloud") {
    enabled_ = param("enabled", false);
    accepted_ = param("projection_accepted", false);
    scale_ = param("depth_scale_m", .001);
    minimum_ = param("min_depth_m", .2); maximum_ = param("max_depth_m", 8.);
    component_max_depth_ = param("component_filter_max_depth_m", maximum_);
    max_age_ = param("max_cloud_age_sec", .3);
    delta_ = param("component_neighbor_delta_m", .05);
    pixels_ = param<int64_t>("component_min_pixels", 0);
    stride_ = param<int64_t>("pointcloud_pixel_stride", 2);
    if (!std::isfinite(scale_) || scale_ <= 0 || !std::isfinite(minimum_) || minimum_ < 0 ||
        !std::isfinite(maximum_) || maximum_ <= minimum_ || !std::isfinite(max_age_) ||
        max_age_ <= 0 || max_age_ > 1 || !std::isfinite(delta_) || delta_ <= 0 ||
        !std::isfinite(component_max_depth_) || component_max_depth_ < minimum_ ||
        component_max_depth_ > maximum_ ||
        stride_ < 1 || stride_ > 16 || pixels_ < 0 || pixels_ > static_cast<int64_t>(kMaxDepthComponentPixels) ||
        (enabled_ && (!accepted_ || pixels_ == 0))) {
      throw std::invalid_argument("invalid_depth_obstacle_profile");
    }
    const auto image = param<std::string>("depth_topic", "");
    const auto info = param<std::string>("camera_info_topic", "");
    const auto output = param<std::string>("output_topic", "");
    const auto diagnostics = param<std::string>("diagnostics_topic", "");
    if (image.empty() || info.empty() || output.empty() || diagnostics.empty() ||
        image == output || info == output) throw std::invalid_argument("invalid_depth_obstacle_topics");
    const auto input_qos = rclcpp::QoS(1).best_effort().durability_volatile();
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output, rclcpp::QoS(1).reliable());
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics, 1);
    // Disabled means no region TF listener, raster, cache, or extra diagnostic.
    if (param("region_support_enabled", false)) {
      if (!enabled_ || !accepted_) throw std::invalid_argument("region_requires_enabled_depth_projection");
      RegionProducerOptions o;
      o.region_id = param<std::string>("region_id", "");
      o.base_frame = param<std::string>("region_base_frame", "base_footprint");
      o.optical_frame = param<std::string>("region_optical_frame", "");
      o.depth_topic = image; o.info_topic = info;
      o.diagnostics_topic = param<std::string>("region_support_diagnostics_topic", "");
      if (o.diagnostics_topic.empty() || o.diagnostics_topic == diagnostics ||
          o.diagnostics_topic == output || o.diagnostics_topic == image || o.diagnostics_topic == info)
        throw std::invalid_argument("invalid_region_diagnostics_topic");
      region_diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          o.diagnostics_topic, rclcpp::QoS(1).reliable().durability_volatile());
      const auto minimum = param<std::vector<double>>("region_box_min_xyz", {});
      const auto maximum = param<std::vector<double>>("region_box_max_xyz", {});
      if (minimum.size() != 3 || maximum.size() != 3)
        throw std::invalid_argument("invalid_region_box_size");
      std::copy(minimum.begin(), minimum.end(), o.box_min.begin());
      std::copy(maximum.begin(), maximum.end(), o.box_max.begin());
      o.clear_margin_m = param("region_clear_margin_m", .02);
      o.tf_wait_sec = param("region_tf_wait_sec", 0.);
      o.shadow_only_enabled = param("region_shadow_only_enabled", false);
      o.hold_complete_measurement_enabled = param("region_hold_complete_measurement_enabled", false);
      o.typed_evaluation_diagnostics_enabled = param("region_typed_evaluation_diagnostics_enabled", false);
      o.depth_scale_m = scale_; o.minimum_depth_m = minimum_; o.maximum_depth_m = maximum_;
      o.max_age_sec = max_age_;
      o.expected_calibration = param<std::string>("region_expected_calibration", "");
      o.expected_contract = param<std::string>("region_expected_contract", "");
      o.accepted.rectified_pinhole_verified = param("region_rectified_pinhole_accepted", false);
      o.accepted.metric_axial_depth_verified = param("region_metric_axial_depth_accepted", false);
      o.accepted.depth_intrinsics_registration_verified = param("region_depth_intrinsics_registration_accepted", false);
      o.accepted.optical_to_base_extrinsics_verified = param("region_optical_to_base_extrinsics_accepted", false);
      o.accepted.source_time_and_transform_verified = param("region_source_time_transform_accepted", false);
      region_ = std::make_unique<RegionDepthProducer>(std::move(o), get_clock(), get_name(),
          [this](const diagnostic_msgs::msg::DiagnosticArray& m) { region_diagnostics_->publish(m); });
    }
    info_ = create_subscription<sensor_msgs::msg::CameraInfo>(info, input_qos,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
          calibration_ = m;
          if (region_) region_->info(m);
        });
    image_ = create_subscription<sensor_msgs::msg::Image>(image, input_qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
          if (region_) region_->image(m);
          receive(*m);
        });
  }
 private:
  template<class T> T param(const std::string& name, T value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor; descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }
  bool fresh(double stamp) {
    return measurement_fresh(measurement_age_seconds(now().seconds(), stamp), max_age_);
  }
  void report(const std::string& reason, std::size_t removed = 0) {
    diagnostic_msgs::msg::DiagnosticArray array; array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = get_name(); status.message = reason;
    status.level = reason == "filtered" ? status.OK : status.WARN;
    diagnostic_msgs::msg::KeyValue kv; kv.key = "component_mask_removed_pixels"; kv.value = std::to_string(removed);
    status.values.push_back(kv);
    const auto add = [&status](const std::string& key, const std::string& value) {
      diagnostic_msgs::msg::KeyValue entry; entry.key = key; entry.value = value;
      status.values.push_back(std::move(entry));
    };
    add("diagnostic_event", "depth_callback");
    add("callback_sequence", std::to_string(callback_sequence_));
    add("source_stamp_ns", std::to_string(source_stamp_ns_));
    add("receipt_ros_ns", std::to_string(receipt_ros_ns_));
    add("receipt_steady_ns", std::to_string(receipt_steady_ns_));
    add("input_age_sec", std::to_string(input_age_sec_));
    add("input_nonmonotonic", input_nonmonotonic_ ? "true" : "false");
    const auto stage = [&add](const char* name, const StageTiming& timing) {
      const std::string prefix(name);
      add(prefix + "_executed", timing.executed ? "true" : "false");
      add(prefix + "_wall_ns", std::to_string(timing.wall_ns));
      add(prefix + "_thread_cpu_ns", std::to_string(timing.cpu_ns));
    };
    stage("mask", mask_timing_); stage("project", project_timing_); stage("publish", publish_timing_);
    array.status.push_back(status); diagnostics_->publish(array);
  }
  void receive(const sensor_msgs::msg::Image& image) {
    receipt_steady_ns_ = steady_ns();
    receipt_ros_ns_ = now().nanoseconds();
    source_stamp_ns_ = int64_t(image.header.stamp.sec) * 1000000000 + image.header.stamp.nanosec;
    input_age_sec_ = static_cast<double>(receipt_ros_ns_ - source_stamp_ns_) / 1.e9;
    ++callback_sequence_;
    mask_timing_ = {}; project_timing_ = {}; publish_timing_ = {};
    const double stamp = rclcpp::Time(image.header.stamp).seconds();
    input_nonmonotonic_ = stamp <= last_stamp_;
    if (!enabled_) return;
    if (!fresh(stamp) || stamp <= last_stamp_) { report("stale_or_nonmonotonic_depth"); return; }
    last_stamp_ = stamp;
    if (!calibration_) { report("camera_info_missing"); return; }
    const auto& mask = [&]() -> const DepthComponentMask& {
      StageTimer timer(mask_timing_);
      return depth_component_mask({image.data.data(), image.data.size(), image.width,
          image.height, image.step, image.is_bigendian != 0}, scale_, minimum_, maximum_, delta_,
          pixels_, component_workspace_);
    }();
    if (!mask.valid) { report(mask.reason); return; }
    try {
      sensor_msgs::msg::PointCloud2 cloud;
      {
        StageTimer timer(project_timing_);
        cloud = project_depth(image, *calibration_, scale_, minimum_, maximum_, stride_,
                              &mask.keep, component_max_depth_);
      }
      if (!fresh(stamp)) { report("depth_became_stale_during_filter"); return; }
      {
        StageTimer timer(publish_timing_);
        publisher_->publish(std::move(cloud));
      }
      report("filtered", mask.removed_pixels);
    } catch (const std::invalid_argument& e) { report(e.what()); }
  }
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr region_diagnostics_;
  std::unique_ptr<RegionDepthProducer> region_;
  bool enabled_{false}, accepted_{false};
  double scale_, minimum_, maximum_, delta_, max_age_, component_max_depth_, last_stamp_{0};
  int64_t pixels_, stride_;
  DepthComponentWorkspace component_workspace_;
  uint64_t callback_sequence_{0};
  int64_t source_stamp_ns_{0}, receipt_ros_ns_{0}, receipt_steady_ns_{0};
  double input_age_sec_{0.};
  bool input_nonmonotonic_{false};
  StageTiming mask_timing_, project_timing_, publish_timing_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr calibration_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
};
}
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<sunhorizon_rgbd_adapter::DepthObstacleCloud>()); }
  catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("depth_obstacle_cloud"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
