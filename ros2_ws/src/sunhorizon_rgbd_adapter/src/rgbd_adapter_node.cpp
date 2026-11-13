#include "sunhorizon_rgbd_adapter/depth_components.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sunhorizon_navigation_core/rgbd_validation.hpp>
#include <sunhorizon_navigation_interfaces/msg/rgbd_status.hpp>

#include "sunhorizon_rgbd_adapter/rgbd_freshness.hpp"
#include "sunhorizon_rgbd_adapter/depth_projection.hpp"
#include "sunhorizon_rgbd_adapter/rgbd_status_snapshot.hpp"
#include "sunhorizon_rgbd_adapter/belly_stage_diagnostics.hpp"
#include "sunhorizon_obstacle_coverage/rgbd_upstream_diagnostics.hpp"

namespace {
namespace ud = sunhorizon_obstacle_coverage::upstream_diagnostics;

using CameraInfo = sensor_msgs::msg::CameraInfo;
using Image = sensor_msgs::msg::Image;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using RgbdStatus = sunhorizon_navigation_interfaces::msg::RgbdStatus;
using sunhorizon_navigation_core::CameraIntrinsics;
using sunhorizon_navigation_core::ImageMetadata;
using sunhorizon_navigation_core::RgbdPairValidator;
using sunhorizon_navigation_core::RgbdValidationLimits;
using sunhorizon_rgbd_adapter::measurement_age_seconds;
using sunhorizon_rgbd_adapter::measurement_fresh;
using sunhorizon_rgbd_adapter::BellyStageFrame;
using sunhorizon_rgbd_adapter::BellyExit;

int64_t diagnostic_steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

double stamp_seconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

CameraIntrinsics intrinsics(const CameraInfo& input) {
  return {input.width, input.height, input.k[0], input.k[4], input.k[2], input.k[5],
          input.header.frame_id};
}

ImageMetadata metadata(const Image& input) {
  return {stamp_seconds(input.header.stamp), input.width, input.height, input.encoding,
          input.header.frame_id};
}



rclcpp::QoS latest_sensor_qos() {
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

class RgbdAdapter final : public rclcpp::Node {
 public:
  explicit RgbdAdapter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("sunhorizon_rgbd_adapter", options), validator_(declare_validation_limits()) {
    belly_stage_diagnostics_enabled_ = declare_parameter("belly_stage_diagnostics_enabled", false);
    const auto upstream_endpoint = ud::endpoint_from_environment();
    upstream_diagnostics_enabled_ = !upstream_endpoint.empty();
    if (upstream_diagnostics_enabled_) {
      upstream_callback_queue_ = std::make_unique<ud::Queue<>>();
      upstream_status_queue_ = std::make_unique<ud::Queue<>>();
      upstream_exporter_ = std::make_unique<ud::Exporter>(*upstream_callback_queue_,
          *upstream_status_queue_, upstream_endpoint, "producer");
    }
    target_rate_hz_ = declare_parameter("target_rate_hz", 15.0);
    max_pair_delta_sec_ = get_parameter("max_pair_delta_sec").as_double();
    status_rate_hz_ = declare_parameter("status_rate_hz", 20.0);
    freshness_timeout_sec_ = declare_parameter("freshness_timeout_sec", 0.30);
    statistics_window_sec_ = declare_parameter("statistics_window_sec", 10.0);
    minimum_healthy_pair_rate_hz_ = declare_parameter("minimum_healthy_pair_rate_hz", 12.0);
    maximum_rejected_pair_ratio_ = declare_parameter("maximum_rejected_pair_ratio", 0.05);
    const int64_t requested_queue_size = declare_parameter<int64_t>("queue_size", 2);
    if (requested_queue_size < 2 || requested_queue_size > 64) {
      throw std::runtime_error("queue_size must be in [2, 64]");
    }
    queue_size_ = static_cast<std::size_t>(requested_queue_size);
    depth_scale_m_ = declare_parameter("depth_scale_m", 0.001);
    min_depth_m_ = declare_parameter("min_depth_m", 0.20);
    max_depth_m_ = declare_parameter("max_depth_m", 8.0);
    minimum_valid_depth_ratio_ = declare_parameter("minimum_valid_depth_ratio", 0.01);
    pointcloud_pixel_stride_ = declare_parameter("pointcloud_pixel_stride", 2);
    chest_component_filter_enabled_ = declare_parameter("chest_component_filter_enabled", false);
    chest_component_neighbor_delta_m_ = declare_parameter("chest_component_neighbor_delta_m", 0.0);
    chest_component_min_pixels_ = declare_parameter<int64_t>("chest_component_min_pixels", 0);
    chest_near_component_min_pixels_ = declare_parameter<int64_t>("chest_near_component_min_pixels", 0);
    chest_near_component_max_depth_m_ = declare_parameter("chest_near_component_max_depth_m", 0.0);
    chest_alignment_accepted_ = declare_parameter(
        "chest_alignment_calibration_accepted", false);
    chest_depth_scale_accepted_ = declare_parameter("chest_depth_scale_accepted", false);
    belly_projection_accepted_ = declare_parameter(
        "belly_projection_calibration_accepted", false);
    belly_depth_scale_accepted_ = declare_parameter("belly_depth_scale_accepted", false);
    calibration_report_sha256_ = declare_parameter("calibration_report_sha256", "");
    validate_parameters();

    const auto chest_color_topic = declare_parameter(
        "chest_color_topic", "/sunhorizon/raw/rgbd/chest/color/image");
    const auto chest_depth_topic = declare_parameter(
        "chest_depth_topic", "/sunhorizon/raw/rgbd/chest/aligned_depth/image");
    const auto chest_color_info_topic = declare_parameter(
        "chest_color_info_topic", "/sunhorizon/raw/rgbd/chest/color/camera_info");
    const auto chest_depth_info_topic = declare_parameter(
        "chest_depth_info_topic", "/sunhorizon/raw/rgbd/chest/aligned_depth/camera_info");
    const auto belly_depth_topic = declare_parameter(
        "belly_depth_topic", "/sunhorizon/raw/rgbd/belly/depth/image");
    const auto belly_depth_info_topic = declare_parameter(
        "belly_depth_info_topic", "/sunhorizon/raw/rgbd/belly/depth/camera_info");

    color_pub_ = create_publisher<Image>(
        declare_parameter("output_color_topic", "/sunhorizon/rgbd/color/image"),
        rclcpp::SensorDataQoS());
    depth_pub_ = create_publisher<Image>(
        declare_parameter("output_depth_topic", "/sunhorizon/rgbd/depth/image"),
        rclcpp::SensorDataQoS());
    color_info_pub_ = create_publisher<CameraInfo>(
        declare_parameter("output_color_info_topic", "/sunhorizon/rgbd/color/camera_info"), 10);
    depth_info_pub_ = create_publisher<CameraInfo>(
        declare_parameter("output_depth_info_topic", "/sunhorizon/rgbd/depth/camera_info"), 10);
    chest_points_pub_ = create_publisher<PointCloud2>(
        declare_parameter("output_chest_points_topic", "/sunhorizon/rgbd/points"),
        rclcpp::SensorDataQoS());
    belly_points_pub_ = create_publisher<PointCloud2>(
        declare_parameter("output_belly_points_topic", "/sunhorizon/rgbd/belly/points"),
        rclcpp::SensorDataQoS());
    status_pub_ = create_publisher<RgbdStatus>(
        declare_parameter("output_status_topic", "/sunhorizon/rgbd/status"), 10);

    // Each stream serializes its own queue, calibration and projection state.
    // Status never waits for image computation or a DDS publication.
    chest_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    belly_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    status_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions chest_options;
    chest_options.callback_group = chest_group_;
    rclcpp::SubscriptionOptions belly_options;
    belly_options.callback_group = belly_group_;

    color_sub_ = create_subscription<Image>(
        chest_color_topic, latest_sensor_qos(),
        std::bind(&RgbdAdapter::on_color, this, std::placeholders::_1), chest_options);
    depth_sub_ = create_subscription<Image>(
        chest_depth_topic, latest_sensor_qos(),
        std::bind(&RgbdAdapter::on_depth, this, std::placeholders::_1), chest_options);
    color_info_sub_ = create_subscription<CameraInfo>(
        chest_color_info_topic, latest_sensor_qos(),
        [this](CameraInfo::SharedPtr message) {
          color_info_ = std::move(message);
          update_chest_info_status();
        }, chest_options);
    depth_info_sub_ = create_subscription<CameraInfo>(
        chest_depth_info_topic, latest_sensor_qos(),
        [this](CameraInfo::SharedPtr message) {
          depth_info_ = std::move(message);
          update_chest_info_status();
        }, chest_options);
    belly_depth_sub_ = create_subscription<Image>(
        belly_depth_topic, latest_sensor_qos(),
        std::bind(&RgbdAdapter::on_belly_depth, this, std::placeholders::_1), belly_options);
    belly_info_sub_ = create_subscription<CameraInfo>(
        belly_depth_info_topic, latest_sensor_qos(),
        [this](CameraInfo::SharedPtr message) {
          belly_info_ = std::move(message);
        }, belly_options);

    const auto period = std::chrono::duration<double>(1.0 / status_rate_hz_);
    status_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&RgbdAdapter::publish_status, this), status_group_);
    startup_stamp_sec_ = get_clock()->now().seconds();
  }

 private:
#ifdef SUNHORIZON_RGBD_ADAPTER_TEST
  friend struct RgbdAdapterTestPeer;
  // Only present in the test translation unit, never in the installed node.
  std::function<void()> before_chest_mask_;
  std::function<void()> before_chest_points_publish_;
  std::function<void()> before_belly_points_publish_;
  std::function<void(const RgbdStatus&)> status_test_observer_;
  std::function<void(const ud::Record&, const RgbdStatus&)> upstream_status_test_observer_;
#endif

  void update_chest_info_status() {
    std::string reason;
    const bool valid = color_info_ && depth_info_ &&
        sunhorizon_navigation_core::valid_camera_intrinsics(intrinsics(*color_info_), &reason) &&
        sunhorizon_navigation_core::valid_camera_intrinsics(intrinsics(*depth_info_), &reason);
    status_.update([valid](auto& state) { state.chest_info_valid = valid; });
  }

  RgbdValidationLimits declare_validation_limits() {
    RgbdValidationLimits limits;
    limits.max_pair_delta_sec = declare_parameter("max_pair_delta_sec", 0.005);
    return limits;
  }

  void validate_parameters() const {
    const bool finite = std::isfinite(target_rate_hz_) &&
        std::isfinite(status_rate_hz_) && std::isfinite(freshness_timeout_sec_) &&
        std::isfinite(statistics_window_sec_) &&
        std::isfinite(minimum_healthy_pair_rate_hz_) &&
        std::isfinite(maximum_rejected_pair_ratio_) &&
        std::isfinite(max_pair_delta_sec_) && std::isfinite(depth_scale_m_) &&
        std::isfinite(min_depth_m_) && std::isfinite(max_depth_m_) &&
        std::isfinite(minimum_valid_depth_ratio_);
    if (!finite || target_rate_hz_ <= 0.0 || status_rate_hz_ <= 0.0 || freshness_timeout_sec_ <= 0.0 ||
        statistics_window_sec_ <= 0.0 || minimum_healthy_pair_rate_hz_ <= 0.0 ||
        maximum_rejected_pair_ratio_ < 0.0 || maximum_rejected_pair_ratio_ >= 1.0 ||
        queue_size_ < 2U || depth_scale_m_ <= 0.0 || min_depth_m_ < 0.0 ||
        max_depth_m_ <= min_depth_m_ || minimum_valid_depth_ratio_ <= 0.0 ||
        minimum_valid_depth_ratio_ > 1.0 || pointcloud_pixel_stride_ < 1) {
      throw std::runtime_error("invalid RGB-D adapter parameter");
    }
    if (chest_component_filter_enabled_ &&
        (!std::isfinite(chest_component_neighbor_delta_m_) || chest_component_neighbor_delta_m_ <= 0 ||
         chest_component_min_pixels_ <= 0 ||
         static_cast<uint64_t>(chest_component_min_pixels_) >
             sunhorizon_rgbd_adapter::kMaxDepthComponentPixels)) {
      throw std::runtime_error("enabled chest component filter requires explicit valid thresholds");
    }
    if (chest_near_component_min_pixels_ < 0 ||
        static_cast<uint64_t>(chest_near_component_min_pixels_) > sunhorizon_rgbd_adapter::kMaxDepthComponentPixels ||
        (chest_near_component_min_pixels_ &&
         (!chest_component_filter_enabled_ || chest_near_component_min_pixels_ < chest_component_min_pixels_ ||
          !std::isfinite(chest_near_component_max_depth_m_) ||
          chest_near_component_max_depth_m_ < min_depth_m_ || chest_near_component_max_depth_m_ > max_depth_m_))) {
      throw std::runtime_error("invalid chest near component filter thresholds");
    }
    const bool any_calibration_accepted = chest_alignment_accepted_ ||
        chest_depth_scale_accepted_ || belly_projection_accepted_ || belly_depth_scale_accepted_;
    if (any_calibration_accepted && calibration_report_sha256_.size() != 64U) {
      throw std::runtime_error(
          "accepted RGB-D calibration requires a 64-character calibration_report_sha256");
    }
  }

  bool depth_image_valid(const Image& image, double* valid_ratio) const {
    if ((image.encoding != sensor_msgs::image_encodings::TYPE_16UC1 &&
         image.encoding != sensor_msgs::image_encodings::MONO16) ||
        image.width == 0U || image.height == 0U || image.step < image.width * 2U ||
        image.data.size() < static_cast<std::size_t>(image.step) * image.height) {
      *valid_ratio = 0.0;
      return false;
    }
    std::size_t sampled = 0U;
    std::size_t valid = 0U;
    const uint32_t stride = static_cast<uint32_t>(std::max(2, pointcloud_pixel_stride_ * 2));
    for (uint32_t row = 0; row < image.height; row += stride) {
      for (uint32_t column = 0; column < image.width; column += stride) {
        const std::size_t offset = static_cast<std::size_t>(row) * image.step + column * 2U;
        uint16_t raw = 0U;
        std::memcpy(&raw, image.data.data() + offset, sizeof(raw));
        if (image.is_bigendian) raw = static_cast<uint16_t>((raw >> 8U) | (raw << 8U));
        const double meters = static_cast<double>(raw) * depth_scale_m_;
        ++sampled;
        if (raw != 0U && meters >= min_depth_m_ && meters <= max_depth_m_) ++valid;
      }
    }
    *valid_ratio = sampled == 0U ? 0.0 : static_cast<double>(valid) / sampled;
    return *valid_ratio >= minimum_valid_depth_ratio_;
  }

  PointCloud2 make_pointcloud(const Image& depth, const CameraInfo& info,
                             const std::vector<uint8_t>* retained_pixels = nullptr) const {
    return sunhorizon_rgbd_adapter::project_depth(depth, info, depth_scale_m_,
        min_depth_m_, max_depth_m_, pointcloud_pixel_stride_, retained_pixels);
  }

  void on_color(Image::SharedPtr message) {
    color_queue_.push_back(std::move(message));
    trim_queue(&color_queue_);
    try_pair();
  }

  void on_depth(Image::SharedPtr message) {
    depth_queue_.push_back(std::move(message));
    trim_queue(&depth_queue_);
    try_pair();
  }

  void trim_queue(std::deque<Image::SharedPtr>* queue) {
    while (queue->size() > queue_size_) {
      queue->pop_front();
      status_.rejected(get_clock()->now().seconds(), statistics_window_sec_);
    }
  }

  void try_pair() {
    if (color_queue_.empty() || depth_queue_.empty() || !color_info_ || !depth_info_) return;
    while (!color_queue_.empty() && !depth_queue_.empty()) {
      const double color_stamp = stamp_seconds(color_queue_.front()->header.stamp);
      auto closest = depth_queue_.begin();
      double closest_delta = std::numeric_limits<double>::infinity();
      for (auto iterator = depth_queue_.begin(); iterator != depth_queue_.end(); ++iterator) {
        const double delta = std::abs(color_stamp - stamp_seconds((*iterator)->header.stamp));
        if (delta < closest_delta) {
          closest = iterator;
          closest_delta = delta;
        }
      }
      if (closest_delta <= max_pair_delta_sec_) {
        const auto color = color_queue_.front();
        const auto depth = *closest;
        color_queue_.pop_front();
        depth_queue_.erase(closest);
        process_pair(*color, *depth);
        continue;
      }
      const double newest_depth = stamp_seconds(depth_queue_.back()->header.stamp);
      const double oldest_depth = stamp_seconds(depth_queue_.front()->header.stamp);
      if (color_stamp < newest_depth - max_pair_delta_sec_) {
        color_queue_.pop_front();
      } else if (oldest_depth < color_stamp - max_pair_delta_sec_) {
        depth_queue_.pop_front();
      } else {
        break;
      }
      status_.rejected(get_clock()->now().seconds(), statistics_window_sec_);
    }
  }

  void process_pair(const Image& color, const Image& depth) {
    const double stamp = stamp_seconds(color.header.stamp);
    if (last_output_stamp_sec_ > 0.0 && stamp - last_output_stamp_sec_ < 1.0 / target_rate_hz_) {
      return;
    }
    const auto decision = validator_.evaluate(
        metadata(color), metadata(depth), intrinsics(*color_info_), intrinsics(*depth_info_),
        chest_alignment_accepted_ && chest_depth_scale_accepted_);
    status_.update([&](auto& state) { state.last_pair_delta_sec = decision.pair_delta_sec; });
    if (!decision.stream_accepted) {
      status_.rejected(get_clock()->now().seconds(), statistics_window_sec_);
      status_.update([&](auto& state) { state.last_rejection_reason = decision.reason; });
      return;
    }
    double valid_ratio = 0.0;
    if (!depth_image_valid(depth, &valid_ratio)) {
      status_.rejected(get_clock()->now().seconds(), statistics_window_sec_);
      status_.update([](auto& state) { state.last_rejection_reason = "chest_depth_invalid_or_empty"; });
      return;
    }
    // This field describes the currently failing pair, not an irreversible
    // historical latch. Rolling rejection counters remain available below.
    last_output_stamp_sec_ = stamp;
    status_.accepted(get_clock()->now().seconds(), stamp, statistics_window_sec_);
    color_pub_->publish(color);
    Image normalized_depth = depth;
    // The input pair is accepted with a bounded delta, but downstream RGB-D
    // consumers use exact synchronization.  Both aligned images therefore
    // represent one normalized observation and must carry one timestamp/frame.
    normalized_depth.header = color.header;
    depth_pub_->publish(normalized_depth);
    CameraInfo normalized_info = *color_info_;
    normalized_info.header = color.header;
    color_info_pub_->publish(normalized_info);
    CameraInfo normalized_depth_info = *depth_info_;
    normalized_depth_info.header = normalized_depth.header;
    depth_info_pub_->publish(normalized_depth_info);
    if (decision.pointcloud_allowed) {
      const sunhorizon_rgbd_adapter::DepthComponentMask* components = nullptr;
      if (chest_component_filter_enabled_) {
#ifdef SUNHORIZON_RGBD_ADAPTER_TEST
        if (before_chest_mask_) before_chest_mask_();
#endif
        components = &sunhorizon_rgbd_adapter::depth_component_mask(
            {depth.data.data(), depth.data.size(), depth.width, depth.height, depth.step,
             depth.is_bigendian != 0}, depth_scale_m_, min_depth_m_, max_depth_m_,
            chest_component_neighbor_delta_m_, static_cast<std::size_t>(chest_component_min_pixels_),
            chest_workspace_, static_cast<std::size_t>(chest_near_component_min_pixels_),
            chest_near_component_max_depth_m_);
        if (!components->valid) {
          status_.update([&](auto& state) {
            state.last_rejection_reason = "chest_component_filter:" + components->reason;
          });
          return;
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
            "chest_depth_components valid=%zu retained=%zu removed=%zu islands=%zu delta_m=%.3f min_pixels=%ld near_min_pixels=%ld near_max_depth_m=%.3f",
            components->valid_pixels, components->valid_pixels - components->removed_pixels,
            components->removed_pixels, components->removed_components,
            chest_component_neighbor_delta_m_, static_cast<long>(chest_component_min_pixels_),
            static_cast<long>(chest_near_component_min_pixels_), chest_near_component_max_depth_m_);
      }
      const auto cloud = make_pointcloud(normalized_depth, normalized_depth_info,
          components ? &components->keep : nullptr);
#ifdef SUNHORIZON_RGBD_ADAPTER_TEST
      if (before_chest_points_publish_) before_chest_points_publish_();
#endif
      chest_points_pub_->publish(cloud);
      status_.update([stamp](auto& state) { state.chest_points_stamp = stamp; });
    }
  }

  void on_belly_depth(const Image::SharedPtr input) {
    const int64_t exact_source_ns = upstream_diagnostics_enabled_ ?
        static_cast<int64_t>(input->header.stamp.sec) * 1000000000LL + input->header.stamp.nanosec : 0;
    const uint64_t callback_sequence = upstream_diagnostics_enabled_ ? ++upstream_callback_sequence_ : 0;
    std::optional<ud::Stage> upstream_stage;
    if (upstream_diagnostics_enabled_) upstream_stage.emplace(*upstream_callback_queue_,
        callback_sequence, exact_source_ns, get_clock()->now().nanoseconds());
    // Disabled path takes no diagnostic clocks or locks. The scope marks every
    // early return/exception separately; only the original successful path is published.
    std::optional<sunhorizon_rgbd_adapter::BellyStageScope<int64_t (*)()>> diagnostic;
    if (belly_stage_diagnostics_enabled_) {
      const int64_t source_ns = static_cast<int64_t>(input->header.stamp.sec) * 1000000000LL +
          input->header.stamp.nanosec;
      diagnostic.emplace(belly_stage_diagnostics_, source_ns,
          get_clock()->now().nanoseconds(), &diagnostic_steady_ns);
    }
    const double stamp = stamp_seconds(input->header.stamp);
    if (stamp <= 0.0 || (last_belly_depth_stamp_sec_ > 0.0 && stamp <= last_belly_depth_stamp_sec_)) {
      status_.update([](auto& state) { state.belly_reason = "belly_non_monotonic_timestamp"; });
      if (diagnostic) diagnostic->finish(BellyExit::kTimestampRejected);
      if (upstream_stage) upstream_stage->finish(2);
      return;
    }
    last_belly_depth_stamp_sec_ = stamp;
    status_.update([&](auto& state) {
      state.belly_depth_stamp = stamp;
      if (upstream_diagnostics_enabled_) {
        state.belly_depth_source_ns = exact_source_ns;
        state.belly_depth_callback_sequence = callback_sequence;
      }
    });
    if (!belly_info_) {
      status_.update([](auto& state) { state.belly_reason = "belly_camera_info_missing"; });
      if (diagnostic) diagnostic->finish(BellyExit::kInfoMissing);
      if (upstream_stage) upstream_stage->finish(2);
      return;
    }
    std::string info_reason;
    const bool belly_info_valid = sunhorizon_navigation_core::valid_camera_intrinsics(
        intrinsics(*belly_info_), &info_reason);
    status_.update([belly_info_valid](auto& state) { state.belly_info_valid = belly_info_valid; });
    double valid_ratio = 0.0;
    if (!belly_info_valid || !depth_image_valid(*input, &valid_ratio)) {
      status_.update([&](auto& state) {
        state.belly_reason = belly_info_valid ? "belly_depth_invalid_or_empty" : info_reason;
      });
      if (diagnostic) diagnostic->finish(BellyExit::kInvalidDepthOrInfo);
      if (upstream_stage) upstream_stage->finish(2);
      return;
    }
    if (!belly_projection_accepted_ || !belly_depth_scale_accepted_) {
      status_.update([](auto& state) { state.belly_reason = "belly_calibration_not_accepted"; });
      if (diagnostic) diagnostic->finish(BellyExit::kCalibrationRejected);
      if (upstream_stage) upstream_stage->finish(2);
      return;
    }
    if (diagnostic) diagnostic->stage(&BellyStageFrame::projection_begin_ns);
    if (upstream_stage) upstream_stage->emit(ud::Event::kProjectionBegin);
    const auto cloud = make_pointcloud(*input, *belly_info_);
    if (diagnostic) diagnostic->stage(&BellyStageFrame::projection_end_ns);
    if (upstream_stage) upstream_stage->emit(ud::Event::kProjectionEnd);
    if (diagnostic) diagnostic->stage(&BellyStageFrame::publish_begin_ns);
    if (upstream_stage) upstream_stage->emit(ud::Event::kPointsPublishBegin);
#ifdef SUNHORIZON_RGBD_ADAPTER_TEST
    if (before_belly_points_publish_) before_belly_points_publish_();
#endif
    belly_points_pub_->publish(cloud);
    if (diagnostic) diagnostic->stage(&BellyStageFrame::publish_return_ns);
    if (upstream_stage) upstream_stage->emit(ud::Event::kPointsPublishReturn);
    status_.update([&](auto& state) {
      state.belly_points_stamp = stamp;
      if (upstream_diagnostics_enabled_) {
        state.belly_points_source_ns = exact_source_ns;
        state.belly_points_callback_sequence = callback_sequence;
      }
      state.belly_reason = "ok";
    });
    if (diagnostic) diagnostic->finish(BellyExit::kPublished);
    if (upstream_stage) upstream_stage->finish(1);
  }

  void publish_status() {
    const auto snapshot_begin_ns = upstream_diagnostics_enabled_ ? ud::steady_ns() : 0;
    const auto snapshot = status_.snapshot(get_clock()->now().seconds(), statistics_window_sec_);
    const auto snapshot_end_ns = upstream_diagnostics_enabled_ ? ud::steady_ns() : 0;
    // Age is calculated after the short snapshot lock has been released.
    const auto now_message = get_clock()->now();
    const double now = now_message.seconds();
    RgbdStatus output;
    output.header.stamp = now_message;
    output.header.frame_id = "base_link";
    output.chest_pair_age_sec = measurement_age_seconds(
        now, snapshot.chest_pair_stamp);
    output.belly_depth_age_sec = measurement_age_seconds(
        now, snapshot.belly_depth_stamp);
    output.chest_pointcloud_age_sec = measurement_age_seconds(
        now, snapshot.chest_points_stamp);
    output.belly_pointcloud_age_sec = measurement_age_seconds(
        now, snapshot.belly_points_stamp);
    output.chest_pair_fresh = measurement_fresh(
        output.chest_pair_age_sec, freshness_timeout_sec_);
    output.chest_camera_info_valid = snapshot.chest_info_valid;
    output.chest_alignment_accepted = chest_alignment_accepted_;
    output.chest_depth_scale_accepted = chest_depth_scale_accepted_;
    output.chest_pointcloud_healthy = measurement_fresh(
        output.chest_pointcloud_age_sec, freshness_timeout_sec_);
    output.belly_depth_fresh = measurement_fresh(
        output.belly_depth_age_sec, freshness_timeout_sec_);
    output.belly_camera_info_valid = snapshot.belly_info_valid;
    output.belly_depth_scale_accepted = belly_depth_scale_accepted_;
    output.belly_pointcloud_healthy = measurement_fresh(
        output.belly_pointcloud_age_sec, freshness_timeout_sec_);
    output.last_pair_delta_ms = snapshot.last_pair_delta_sec * 1000.0;
    output.accepted_pair_rate_hz =
        static_cast<double>(snapshot.window_accepted) / statistics_window_sec_;
    const auto attempts = snapshot.window_accepted + snapshot.window_rejected;
    output.rejected_pair_ratio = attempts == 0U ? 1.0 :
        static_cast<double>(snapshot.window_rejected) / static_cast<double>(attempts);
    output.accepted_pairs = snapshot.accepted_pairs;
    output.rejected_pairs = snapshot.rejected_pairs;

    std::ostringstream reason;
    if (snapshot.statistics_overflow) reason << "pair_statistics_overflow;";
    if (!output.chest_pair_fresh) reason << "chest_pair_stale;";
    if (!output.chest_camera_info_valid) reason << "chest_camera_info_invalid;";
    if (!output.chest_alignment_accepted) reason << "chest_alignment_not_accepted;";
    if (!output.chest_depth_scale_accepted) reason << "chest_depth_scale_not_accepted;";
    if (!output.chest_pointcloud_healthy) reason << "chest_pointcloud_stale;";
    if (!output.belly_depth_fresh) reason << "belly_depth_stale;";
    if (!output.belly_camera_info_valid) reason << "belly_camera_info_invalid;";
    if (!belly_projection_accepted_) reason << "belly_projection_not_accepted;";
    if (!output.belly_depth_scale_accepted) reason << "belly_depth_scale_not_accepted;";
    if (!output.belly_pointcloud_healthy) reason << "belly_pointcloud_stale;";
    if (output.accepted_pair_rate_hz < minimum_healthy_pair_rate_hz_)
      reason << "pair_rate_low;";
    if (output.rejected_pair_ratio > maximum_rejected_pair_ratio_)
      reason << "pair_rejection_ratio_high;";
    if (!snapshot.last_rejection_reason.empty()) reason << "last_pair_rejection=" << snapshot.last_rejection_reason << ';';
    if (!snapshot.belly_reason.empty() && snapshot.belly_reason != "ok")
      reason << "belly=" << snapshot.belly_reason << ';';
    output.reason = reason.str().empty() ? "ok" : reason.str();
    const bool all_healthy = output.reason == "ok";
    if (all_healthy) output.state = RgbdStatus::STATE_HEALTHY;
    else if (now - startup_stamp_sec_ < statistics_window_sec_)
      output.state = RgbdStatus::STATE_INITIALIZING;
    else output.state = RgbdStatus::STATE_UNHEALTHY;
    ud::Record upstream;
    if (upstream_diagnostics_enabled_) {
      upstream.sequence = ++upstream_status_sequence_;
      upstream.status_header_ns = now_message.nanoseconds();
      upstream.header_ambiguous = upstream_header_identity_.observe(upstream.status_header_ns);
      upstream.depth_source_ns = snapshot.belly_depth_source_ns;
      upstream.points_source_ns = snapshot.belly_points_source_ns;
      upstream.depth_callback_sequence = snapshot.belly_depth_callback_sequence;
      upstream.points_callback_sequence = snapshot.belly_points_callback_sequence;
      upstream.snapshot_begin_ns = snapshot_begin_ns; upstream.snapshot_end_ns = snapshot_end_ns;
      upstream.ages = ud::age_bits(output); upstream.health = ud::health_bits(output);
      upstream.input_seen = snapshot.belly_depth_callback_sequence != 0;
      upstream.event = ud::Event::kStatusPublishBegin; upstream.event_steady_ns = ud::steady_ns();
      upstream_status_queue_->try_push(upstream);
    }
    status_pub_->publish(output);
    if (upstream_diagnostics_enabled_) {
      upstream.event = ud::Event::kStatusPublishReturn; upstream.event_steady_ns = ud::steady_ns();
      upstream_status_queue_->try_push(upstream);
#ifdef SUNHORIZON_RGBD_ADAPTER_TEST
      if (upstream_status_test_observer_) upstream_status_test_observer_(upstream, output);
#endif
    }
    if (belly_stage_diagnostics_enabled_) {
      // Separate diagnostic snapshot: its sequence/source identify the frame;
      // do not attribute last_finished to the earlier business status snapshot.
      const auto diagnostic = belly_stage_diagnostics_.snapshot();
      const auto observed_steady_ns = diagnostic_steady_ns();
      if (sunhorizon_rgbd_adapter::belly_diagnostic_needed(diagnostic, observed_steady_ns,
          output.belly_depth_age_sec, output.belly_pointcloud_age_sec)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "Belly stage diagnostic (wall includes scheduling; not a safety gate): %s",
            sunhorizon_rgbd_adapter::format_belly_stages(diagnostic, observed_steady_ns,
                now_message.nanoseconds(), snapshot.belly_depth_stamp,
                snapshot.belly_points_stamp).c_str());
      }
    }
#ifdef SUNHORIZON_RGBD_ADAPTER_TEST
    if (status_test_observer_) status_test_observer_(output);
#endif
  }

  RgbdPairValidator validator_;
  sunhorizon_rgbd_adapter::RgbdStatusSnapshot status_;
  sunhorizon_rgbd_adapter::BellyStageDiagnostics belly_stage_diagnostics_;
  bool belly_stage_diagnostics_enabled_{false};
  sunhorizon_rgbd_adapter::DepthComponentWorkspace chest_workspace_;
  rclcpp::CallbackGroup::SharedPtr chest_group_;
  rclcpp::CallbackGroup::SharedPtr belly_group_;
  rclcpp::CallbackGroup::SharedPtr status_group_;
  std::deque<Image::SharedPtr> color_queue_;
  std::deque<Image::SharedPtr> depth_queue_;
  CameraInfo::SharedPtr color_info_;
  CameraInfo::SharedPtr depth_info_;
  CameraInfo::SharedPtr belly_info_;
  double target_rate_hz_{15.0};
  double status_rate_hz_{20.0};
  double freshness_timeout_sec_{0.30};
  double statistics_window_sec_{10.0};
  double minimum_healthy_pair_rate_hz_{12.0};
  double maximum_rejected_pair_ratio_{0.05};
  std::size_t queue_size_{30U};
  double max_pair_delta_sec_{0.005};
  double depth_scale_m_{0.001};
  double min_depth_m_{0.20};
  double max_depth_m_{8.0};
  double minimum_valid_depth_ratio_{0.01};
  int pointcloud_pixel_stride_{2};
  bool chest_component_filter_enabled_{false};
  double chest_component_neighbor_delta_m_{0.0};
  int64_t chest_component_min_pixels_{0};
  int64_t chest_near_component_min_pixels_{0};
  double chest_near_component_max_depth_m_{0.0};
  bool chest_alignment_accepted_{false};
  bool chest_depth_scale_accepted_{false};
  bool belly_projection_accepted_{false};
  bool belly_depth_scale_accepted_{false};
  std::string calibration_report_sha256_;
  double startup_stamp_sec_{0.0};
  double last_output_stamp_sec_{0.0};
  double last_belly_depth_stamp_sec_{0.0};
  rclcpp::Publisher<Image>::SharedPtr color_pub_;
  rclcpp::Publisher<Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr color_info_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr depth_info_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr chest_points_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr belly_points_pub_;
  rclcpp::Publisher<RgbdStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<Image>::SharedPtr color_sub_;
  rclcpp::Subscription<Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr color_info_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr depth_info_sub_;
  rclcpp::Subscription<Image>::SharedPtr belly_depth_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr belly_info_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  bool upstream_diagnostics_enabled_{false};
  uint64_t upstream_callback_sequence_{0}, upstream_status_sequence_{0};
  ud::HeaderIdentity upstream_header_identity_;
  std::unique_ptr<ud::Queue<>> upstream_callback_queue_, upstream_status_queue_;
  // Destroy exporter before its queues. No external/DDS wait during shutdown.
  std::unique_ptr<ud::Exporter> upstream_exporter_;
};

}  // namespace

#ifndef SUNHORIZON_RGBD_ADAPTER_TEST
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RgbdAdapter>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
#endif
