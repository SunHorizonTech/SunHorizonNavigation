#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "sunhorizon_airy_adapter/header_time_rebaser.hpp"
#include "sunhorizon_airy_adapter/imu_normalization.hpp"
#include "sunhorizon_airy_adapter/qos_history.hpp"
#include "sunhorizon_airy_adapter/direct_adapter.hpp"

namespace {

// Live tracking subscriptions must not inherit the deep output histories. A new
// reliable reader can otherwise drain hundreds of already queued IMU samples;
// the first old sample then initializes the device-to-ROS clock and makes the
// following live samples appear to come from the future. Output histories stay
// deep for downstream consumers. Before clock lock the owned raw boundary keeps
// only the newest 80 ms of nominal 200 Hz IMU and four 10 Hz point frames. Once
// the epoch is locked and ingress is promoted to reliable, IMU history becomes
// deep enough to absorb a temporarily slow point-cloud callback while points
// remain shallow. Fixed-clock rosbag replay retains deep input histories.
double time_seconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time stamp_from_seconds(double stamp_sec) {
  const auto stamp_ns = static_cast<std::int64_t>(std::llround(stamp_sec * 1e9));
  if (stamp_ns <= 0) {
    throw std::runtime_error("rebased_header_stamp_non_positive");
  }
  builtin_interfaces::msg::Time output;
  output.sec = static_cast<std::int32_t>(stamp_ns / 1000000000LL);
  output.nanosec = static_cast<std::uint32_t>(stamp_ns % 1000000000LL);
  return output;
}

rclcpp::QoS sensor_qos(std::size_t depth, bool reliable) {
  rclcpp::QoS qos{rclcpp::KeepLast(depth)};
  qos.durability_volatile();
  if (reliable) {
    qos.reliable();
  } else {
    qos.best_effort();
  }
  return qos;
}

class AiryAdapter final : public rclcpp::Node, public sunhorizon_airy_adapter::AiryDirectAdapter {
 public:
  using Stamp = sunhorizon_airy_adapter::AiryDirectStamp;
  AiryAdapter(const rclcpp::NodeOptions& options, bool direct = false,
    sunhorizon_airy_adapter::AiryDirectSinks sinks = {}, std::string points_owner = {}, std::string imu_owner = {})
      : Node("sunhorizon_airy_adapter", options), direct_(direct), sinks_(std::move(sinks)),
        points_owner_(std::move(points_owner)), imu_owner_(std::move(imu_owner)) {
    if (direct_ && (!sinks_.points || !sinks_.imu || points_owner_.empty() || imu_owner_.empty()))
      throw std::invalid_argument("direct_airy_requires_actual_owners_and_consumers");
    input_points_topic_ =
      declare_parameter("input_points_topic", "/sunhorizon/raw/airy/points");
    input_imu_topic_ =
      declare_parameter("input_imu_topic", "/sunhorizon/raw/airy/imu");
    expected_input_points_frame_ =
        declare_parameter("expected_input_points_frame", "rslidar");
    expected_input_imu_frame_ = declare_parameter("expected_input_imu_frame", "rslidar");
    const auto output_points = declare_parameter("output_points_topic", "/sunhorizon/airy/points");
    const auto output_imu = declare_parameter("output_imu_topic", "/sunhorizon/airy/imu");
    output_points_frame_ = declare_parameter("output_points_frame", "airy_lidar");
    output_imu_frame_ = declare_parameter("output_imu_frame", std::string{});
    linear_acceleration_scale_ = declare_parameter(
        "linear_acceleration_scale",
        sunhorizon_airy_adapter::kDefaultLinearAccelerationScale);
    input_orientation_available_ = declare_parameter("input_orientation_available", false);
    timestamp_field_ = declare_parameter("timestamp_field", "timestamp");
    timestamp_mode_ = declare_parameter("timestamp_mode", "scan_relative");
    timestamp_field_datatype_ = declare_parameter("timestamp_field_datatype", "float32");
    maximum_scan_duration_sec_ = declare_parameter("maximum_scan_duration_sec", 0.20);
    maximum_future_stamp_sec_ = declare_parameter("maximum_future_stamp_sec", 0.10);
    header_time_mode_ = declare_parameter("header_time_mode", "preserve");
    const double fixed_header_time_offset_sec = declare_parameter(
        "fixed_header_time_offset_sec", 0.0);
    const double tracking_target_delay_sec = declare_parameter(
        "tracking_target_delay_sec", 0.02);
    const double maximum_clock_offset_slew_rate = declare_parameter(
        "maximum_clock_offset_slew_rate", 0.002);
    const double tracking_initialization_warmup_sec = declare_parameter(
        "tracking_initialization_warmup_sec", 0.5);
    point_calibration_valid_ = declare_parameter("point_calibration_valid", false);
    imu_calibration_valid_ = declare_parameter("imu_axis_calibration_valid", false);
    reliable_sensor_qos_ = declare_parameter("reliable_sensor_qos", false);

    const auto translation = declare_parameter<std::vector<double>>(
        "point_translation_xyz", {0.0, 0.0, 0.0});
    const auto point_rotation = declare_parameter<std::vector<double>>(
        "point_rotation_xyzw", {0.0, 0.0, 0.0, 1.0});
    const auto imu_rotation = declare_parameter<std::vector<double>>(
        "imu_rotation_xyzw", {0.0, 0.0, 0.0, 1.0});
    if (translation.size() != 3 || point_rotation.size() != 4 || imu_rotation.size() != 4) {
      throw std::runtime_error("translation must have 3 values and rotations 4 values");
    }
    if (expected_input_points_frame_.empty() || expected_input_imu_frame_.empty() ||
        output_points_frame_.empty()) {
      throw std::runtime_error("Airy input and output frames must be explicit");
    }
    if (timestamp_mode_ != "absolute" && timestamp_mode_ != "scan_relative") {
      throw std::runtime_error("timestamp_mode must be absolute or scan_relative");
    }
    if (timestamp_field_datatype_ != "float32" && timestamp_field_datatype_ != "float64") {
      throw std::runtime_error("timestamp_field_datatype must be float32 or float64");
    }
    if (header_time_mode_ != "preserve" &&
        header_time_mode_ != "first_imu_to_ros" &&
        header_time_mode_ != "fixed_offset" &&
        header_time_mode_ != "tracking_imu_to_ros") {
      throw std::runtime_error(
          "header_time_mode must be preserve, first_imu_to_ros, fixed_offset, "
          "or tracking_imu_to_ros");
    }
    if (!std::isfinite(maximum_scan_duration_sec_) || maximum_scan_duration_sec_ <= 0.0 ||
        !std::isfinite(maximum_future_stamp_sec_) || maximum_future_stamp_sec_ < 0.0) {
      throw std::runtime_error("timestamp bounds must be finite and non-negative");
    }
    header_time_rebaser_ = std::make_unique<sunhorizon_airy_adapter::HeaderTimeRebaser>(
        sunhorizon_airy_adapter::parse_header_time_mode(header_time_mode_),
        fixed_header_time_offset_sec, maximum_future_stamp_sec_,
        tracking_target_delay_sec, maximum_clock_offset_slew_rate,
        tracking_initialization_warmup_sec);
    translation_.setValue(translation[0], translation[1], translation[2]);
    point_calibration_.setValue(
        point_rotation[0], point_rotation[1], point_rotation[2], point_rotation[3]);
    imu_calibration_.setValue(
        imu_rotation[0], imu_rotation[1], imu_rotation[2], imu_rotation[3]);
    if (!std::isfinite(point_calibration_.length2()) || point_calibration_.length2() < 1e-12 ||
        !std::isfinite(imu_calibration_.length2()) || imu_calibration_.length2() < 1e-12) {
      throw std::runtime_error("point or IMU rotation is invalid");
    }
    point_calibration_.normalize();
    imu_calibration_.normalize();
    point_rotation_ = tf2::Matrix3x3(point_calibration_);
    imu_rotation_ = tf2::Matrix3x3(imu_calibration_);

    const auto imu_config_status = sunhorizon_airy_adapter::validate_imu_normalization_config(
        linear_acceleration_scale_, output_imu_frame_, output_points_frame_);
    if (imu_config_status != sunhorizon_airy_adapter::ImuNormalizationStatus::kOk) {
      throw std::runtime_error(
          sunhorizon_airy_adapter::imu_normalization_status_reason(imu_config_status));
    }

    const bool live_tracking_clock =
        header_time_mode_ == "tracking_imu_to_ros";
    // A reliable live reader receives the driver's pre-existing history after
    // discovery.  Those old IMU samples bias the device-to-ROS epoch before
    // the shallow application queue can discard them.  Keep the live ingress
    // latest-only, while retaining reliable owned outputs for LIVO.
    reliable_input_sensor_qos_ = reliable_sensor_qos_ && !live_tracking_clock;
    reliable_output_sensor_qos_ = reliable_sensor_qos_;
    live_input_qos_promotion_pending_ =
        reliable_sensor_qos_ && live_tracking_clock;
    live_tracking_clock_ = live_tracking_clock;
    points_callback_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    imu_callback_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    diagnostics_callback_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    const auto output_points_qos = sensor_qos(
        sunhorizon_airy_adapter::kOutputPointQueueDepth,
        reliable_output_sensor_qos_);
    const auto output_imu_qos = sensor_qos(
        sunhorizon_airy_adapter::kOutputImuQueueDepth,
        reliable_output_sensor_qos_);
    if (!direct_) points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        output_points, output_points_qos);
    if (!direct_) imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
        output_imu, output_imu_qos);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/sunhorizon/airy/diagnostics", 10);
    if (!direct_) configure_input_subscriptions(reliable_input_sensor_qos_.load());
    timer_ = create_wall_timer(
        std::chrono::milliseconds(250),
        std::bind(&AiryAdapter::publish_diagnostics, this),
        diagnostics_callback_group_);

    if (!point_calibration_valid_ || !imu_calibration_valid_) {
      RCLCPP_ERROR(get_logger(),
          "Point and/or IMU axis calibration is not accepted; affected output is blocked");
    }
    RCLCPP_INFO(
        get_logger(),
        "Airy sensor QoS input=%s output=%s promotion_pending=%s "
        "input_depths=%zu/%zu output_depths=%zu/%zu",
        reliable_input_sensor_qos_.load() ? "reliable" : "best_effort",
        reliable_output_sensor_qos_ ? "reliable" : "best_effort",
        live_input_qos_promotion_pending_.load() ? "true" : "false",
        active_input_point_queue_depth_.load(), active_input_imu_queue_depth_.load(),
        sunhorizon_airy_adapter::kOutputPointQueueDepth,
        sunhorizon_airy_adapter::kOutputImuQueueDepth);
  }

  rclcpp::Node::SharedPtr node() override { return rclcpp::Node::shared_from_this(); }
  void receive_points(sensor_msgs::msg::PointCloud2::ConstSharedPtr message, const Stamp& stamp) override {
    std::lock_guard<std::mutex> lock(points_direct_mutex_);
    validate_direct(message, stamp, points_owner_, point_direct_sequence_);
    Stamp work = stamp; work.normalization_start_steady_ns = steady_ns();
    work.normalization_thread_cpu_ns = thread_cpu_ns();
    on_points(std::move(message), false, &work);
  }
  void receive_imu(sensor_msgs::msg::Imu::ConstSharedPtr message, const Stamp& stamp) override {
    std::lock_guard<std::mutex> lock(imu_direct_mutex_);
    validate_direct(message, stamp, imu_owner_, imu_direct_sequence_);
    Stamp work = stamp; work.normalization_start_steady_ns = steady_ns();
    work.normalization_thread_cpu_ns = thread_cpu_ns();
    on_imu(std::move(message), false, &work);
  }
 private:
  static int64_t steady_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  static int64_t thread_cpu_ns() {
    timespec t{};
    if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t) != 0) throw std::runtime_error("thread_cpu_clock_unavailable");
    return int64_t(t.tv_sec)*1000000000LL+t.tv_nsec;
  }
  template<class Message> void validate_direct(const Message& message, const Stamp& stamp,
      const std::string& owner, uint64_t& last_sequence) {
    if (!direct_ || !message || stamp.source_identity != owner || !stamp.sequence ||
        stamp.sequence <= last_sequence || stamp.raw_source_ns <= 0 || stamp.source_ns != stamp.raw_source_ns ||
        int64_t(message->header.stamp.sec)*1000000000LL + message->header.stamp.nanosec != stamp.raw_source_ns ||
        stamp.first_receipt_ros_ns <= 0 || stamp.first_receipt_steady_ns <= 0 ||
        stamp.first_receipt_ros_ns > get_clock()->now().nanoseconds() ||
        stamp.first_receipt_steady_ns > std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count())
      throw std::invalid_argument("invalid_original_direct_airy_input");
    last_sequence = stamp.sequence;
  }
  void configure_input_subscriptions(bool reliable) {
    reliable_input_sensor_qos_.store(reliable);
    const auto active_depths = sunhorizon_airy_adapter::input_queue_depths(
        live_tracking_clock_, reliable && live_tracking_clock_);
    active_input_point_queue_depth_.store(active_depths.points);
    active_input_imu_queue_depth_.store(active_depths.imu);
    const auto input_points_qos = sensor_qos(
        active_depths.points, reliable);
    const auto input_imu_qos = sensor_qos(
        active_depths.imu, reliable);
    rclcpp::SubscriptionOptions point_options;
    point_options.callback_group = points_callback_group_;
    rclcpp::SubscriptionOptions imu_options;
    imu_options.callback_group = imu_callback_group_;
    points_sub_.reset();
    imu_sub_.reset();
    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_points_topic_, input_points_qos,
        [this, reliable](sensor_msgs::msg::PointCloud2::SharedPtr message) {
          on_points(std::move(message), reliable);
        },
        point_options);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        input_imu_topic_, input_imu_qos,
        [this, reliable](sensor_msgs::msg::Imu::SharedPtr message) {
          on_imu(std::move(message), reliable);
        },
        imu_options);
  }

  void maybe_promote_live_input_qos() {
    bool header_time_initialized = false;
    {
      std::lock_guard<std::mutex> lock(header_time_mutex_);
      header_time_initialized = header_time_rebaser_->initialized();
    }
    if (direct_ || !live_input_qos_promotion_pending_.load() ||
        !header_time_initialized) {
      return;
    }
    live_input_qos_promotion_pending_.store(false);
    reliable_generation_point_received_.store(false);
    reliable_generation_imu_received_.store(false);
    reliable_live_ready_.store(false);
    configure_input_subscriptions(true);
    RCLCPP_WARN(
        get_logger(),
        "Promoted live Airy driver ingress to reliable after clock epoch initialization "
        "with input depths=%zu/%zu",
        active_input_point_queue_depth_.load(), active_input_imu_queue_depth_.load());
  }

  const sensor_msgs::msg::PointField* find_field(
      const sensor_msgs::msg::PointCloud2& cloud, const std::string& name) const {
    const auto it = std::find_if(cloud.fields.begin(), cloud.fields.end(),
        [&name](const auto& field) { return field.name == name; });
    return it == cloud.fields.end() ? nullptr : &*it;
  }

  bool timestamp_range_float32(
      const sensor_msgs::msg::PointCloud2& cloud, double* minimum, double* maximum,
      std::string* reason) const {
    try {
      sensor_msgs::PointCloud2ConstIterator<float> timestamps(cloud, timestamp_field_);
      for (; timestamps != timestamps.end(); ++timestamps) {
        const double value = static_cast<double>(*timestamps);
        if (!std::isfinite(value)) {
          *reason = "non_finite_point_timestamp";
          return false;
        }
        *minimum = std::min(*minimum, value);
        *maximum = std::max(*maximum, value);
      }
    } catch (const std::runtime_error&) {
      *reason = "invalid_timestamp_layout";
      return false;
    }
    return true;
  }

  bool timestamp_range_float64(
      const sensor_msgs::msg::PointCloud2& cloud, double* minimum, double* maximum,
      std::string* reason) const {
    try {
      sensor_msgs::PointCloud2ConstIterator<double> timestamps(cloud, timestamp_field_);
      for (; timestamps != timestamps.end(); ++timestamps) {
        const double value = *timestamps;
        if (!std::isfinite(value)) {
          *reason = "non_finite_point_timestamp";
          return false;
        }
        *minimum = std::min(*minimum, value);
        *maximum = std::max(*maximum, value);
      }
    } catch (const std::runtime_error&) {
      *reason = "invalid_timestamp_layout";
      return false;
    }
    return true;
  }

  bool validate_timestamps(const sensor_msgs::msg::PointCloud2& cloud, std::string* reason) {
    const auto* field = find_field(cloud, timestamp_field_);
    if (field == nullptr) {
      *reason = "missing_timestamp_field";
      return false;
    }
    const auto expected_datatype = timestamp_field_datatype_ == "float32"
        ? sensor_msgs::msg::PointField::FLOAT32
        : sensor_msgs::msg::PointField::FLOAT64;
    if (field->datatype != expected_datatype) {
      *reason = "timestamp_datatype_mismatch";
      return false;
    }
    if (field->count != 1U) {
      *reason = "timestamp_field_count_mismatch";
      return false;
    }
    double minimum = std::numeric_limits<double>::infinity();
    double maximum = -std::numeric_limits<double>::infinity();
    const bool range_valid = timestamp_field_datatype_ == "float32"
        ? timestamp_range_float32(cloud, &minimum, &maximum, reason)
        : timestamp_range_float64(cloud, &minimum, &maximum, reason);
    if (!range_valid) {
      return false;
    }
    if (!std::isfinite(minimum) || maximum < minimum ||
        maximum - minimum > maximum_scan_duration_sec_) {
      *reason = "scan_timestamp_span_invalid";
      return false;
    }
    const double header = time_seconds(cloud.header.stamp);
    const double now = get_clock()->now().seconds();
    if (header > now + maximum_future_stamp_sec_) {
      *reason = "cloud_header_in_future";
      return false;
    }
    if (timestamp_mode_ == "absolute") {
      if (minimum < 1.0e8 || std::abs(maximum - header) > maximum_scan_duration_sec_ + 0.05) {
        *reason = "absolute_timestamp_header_mismatch";
        return false;
      }
      if (maximum > now + maximum_future_stamp_sec_) {
        *reason = "point_timestamp_in_future";
        return false;
      }
    } else if (timestamp_mode_ == "scan_relative") {
      constexpr double kRelativeTimestampToleranceSec = 1.0e-6;
      if (minimum < -kRelativeTimestampToleranceSec ||
          maximum > maximum_scan_duration_sec_ + kRelativeTimestampToleranceSec) {
        *reason = "scan_relative_timestamp_out_of_bounds";
        return false;
      }
    } else {
      *reason = "timestamp_mode_mismatch";
      return false;
    }
    return true;
  }

  bool rebase_header(
      builtin_interfaces::msg::Time* stamp, bool may_initialize,
      std::string* reason, int64_t original_receipt_ns = 0) {
    const double raw_stamp_sec = time_seconds(*stamp);
    std::lock_guard<std::mutex> lock(header_time_mutex_);
    const auto result = header_time_rebaser_->rebase(
        raw_stamp_sec, original_receipt_ns > 0 ? double(original_receipt_ns)*1e-9 : get_clock()->now().seconds(), may_initialize);
    if (result.initialized_now) {
      RCLCPP_WARN(
          get_logger(),
          "Initialized Airy device-to-ROS header offset to %.9f s in %s mode",
          header_time_rebaser_->offset_sec(), header_time_mode_.c_str());
    }
    if (!result.accepted) {
      *reason = result.reason;
      return false;
    }
    try {
      *stamp = stamp_from_seconds(result.shifted_stamp_sec);
    } catch (const std::runtime_error& error) {
      *reason = error.what();
      return false;
    }
    return true;
  }

  void set_last_reason(const std::string& reason) {
    std::lock_guard<std::mutex> lock(reason_mutex_);
    last_reason_ = reason;
  }

  std::string last_reason() const {
    std::lock_guard<std::mutex> lock(reason_mutex_);
    return last_reason_;
  }

  void on_points(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr input,
      bool reliable_generation, const Stamp* direct_stamp = nullptr) {
    ++point_inputs_;
    if (input->header.frame_id != expected_input_points_frame_) {
      set_last_reason("unexpected_point_input_frame");
      ++point_rejections_;
      return;
    }
    if (!point_calibration_valid_) {
      set_last_reason("point_calibration_not_accepted");
      ++point_rejections_;
      return;
    }
    std::string reason;
    if (!validate_timestamps(*input, &reason)) {
      set_last_reason(reason);
      ++point_rejections_;
      return;
    }
    auto output = *input;
    output.header.frame_id = output_points_frame_;
    if (!rebase_header(&output.header.stamp, false, &reason, direct_stamp ? direct_stamp->first_receipt_ros_ns : 0)) {
      set_last_reason(reason);
      ++point_rejections_;
      return;
    }
    const double output_stamp_sec = time_seconds(output.header.stamp);
    if (output_stamp_sec <= last_output_point_header_sec_.load()) {
      set_last_reason("rebased_point_header_not_strictly_increasing");
      ++point_rejections_;
      return;
    }
    try {
      sensor_msgs::PointCloud2Iterator<float> x(output, "x");
      sensor_msgs::PointCloud2Iterator<float> y(output, "y");
      sensor_msgs::PointCloud2Iterator<float> z(output, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        const tf2::Vector3 transformed =
            point_rotation_ * tf2::Vector3(*x, *y, *z) + translation_;
        *x = static_cast<float>(transformed.x());
        *y = static_cast<float>(transformed.y());
        *z = static_cast<float>(transformed.z());
      }
    } catch (const std::runtime_error&) {
      set_last_reason("missing_float32_xyz_fields");
      ++point_rejections_;
      return;
    }
    last_point_receipt_ns_.store(direct_stamp ? direct_stamp->first_receipt_ros_ns : get_clock()->now().nanoseconds());
    last_output_point_header_sec_.store(output_stamp_sec);
    set_last_reason("ok");
    ++point_outputs_;
    if (direct_stamp) {
      Stamp stamp = *direct_stamp;
      stamp.source_ns = int64_t(output.header.stamp.sec)*1000000000LL + output.header.stamp.nanosec;
      stamp.source_identity = std::string(get_fully_qualified_name()) + "#points";
      if (direct_stamp->ray_origins) {
        if(direct_stamp->ray_origins->size()!=std::size_t(output.width)*output.height)
          throw std::runtime_error("airy_ray_origin_record_count_mismatch");
        auto origins=std::make_shared<std::vector<std::array<float,3>>>();
        origins->reserve(direct_stamp->ray_origins->size());
        for(const auto& origin:*direct_stamp->ray_origins) {
          const auto value=point_rotation_*tf2::Vector3(origin[0],origin[1],origin[2])+translation_;
          origins->push_back({float(value.x()),float(value.y()),float(value.z())});
        }
        stamp.ray_origins=std::move(origins);
      }
      stamp.normalization_end_steady_ns = steady_ns();
      stamp.normalization_thread_cpu_ns = thread_cpu_ns() - stamp.normalization_thread_cpu_ns;
      sinks_.points(std::make_shared<const sensor_msgs::msg::PointCloud2>(std::move(output)), stamp);
    } else points_pub_->publish(output);
    if (reliable_generation && live_tracking_clock_ &&
        reliable_input_sensor_qos_.load() &&
        !live_input_qos_promotion_pending_.load()) {
      reliable_generation_point_received_.store(true);
      reliable_live_ready_.store(reliable_generation_imu_received_.load());
    }
  }

  void on_imu(
      const sensor_msgs::msg::Imu::ConstSharedPtr input,
      bool reliable_generation, const Stamp* direct_stamp = nullptr) {
    ++imu_inputs_;
    if (input->header.frame_id != expected_input_imu_frame_) {
      set_last_reason("unexpected_imu_input_frame");
      ++imu_rejections_;
      return;
    }
    auto result = sunhorizon_airy_adapter::normalize_imu(
        *input, imu_calibration_, imu_rotation_, imu_calibration_valid_,
        input_orientation_available_, linear_acceleration_scale_, output_imu_frame_,
        output_points_frame_);
    std::string reason =
        sunhorizon_airy_adapter::imu_normalization_status_reason(result.status);
    if (!result.message.has_value()) {
      set_last_reason(reason);
      ++imu_rejections_;
      return;
    }
    if (!rebase_header(&result.message->header.stamp, true, &reason, direct_stamp ? direct_stamp->first_receipt_ros_ns : 0)) {
      set_last_reason(reason);
      ++imu_rejections_;
      return;
    }
    const double output_stamp_sec = time_seconds(result.message->header.stamp);
    if (output_stamp_sec <= last_output_imu_header_sec_.load()) {
      set_last_reason("rebased_imu_header_not_strictly_increasing");
      ++imu_rejections_;
      return;
    }
    last_imu_receipt_ns_.store(direct_stamp ? direct_stamp->first_receipt_ros_ns : get_clock()->now().nanoseconds());
    last_output_imu_header_sec_.store(output_stamp_sec);
    set_last_reason("ok");
    ++imu_outputs_;
    if (direct_stamp) {
      Stamp stamp = *direct_stamp;
      stamp.normalization_end_steady_ns = steady_ns();
      stamp.normalization_thread_cpu_ns = thread_cpu_ns() - stamp.normalization_thread_cpu_ns;
      stamp.source_ns = int64_t(result.message->header.stamp.sec)*1000000000LL + result.message->header.stamp.nanosec;
      stamp.source_identity = std::string(get_fully_qualified_name()) + "#imu";
      sinks_.imu(std::make_shared<const sensor_msgs::msg::Imu>(std::move(*result.message)), stamp);
    } else imu_pub_->publish(*result.message);
    if (reliable_generation && live_tracking_clock_ &&
        reliable_input_sensor_qos_.load() &&
        !live_input_qos_promotion_pending_.load()) {
      reliable_generation_imu_received_.store(true);
      reliable_live_ready_.store(reliable_generation_point_received_.load());
    }
  }

  void publish_diagnostics() {
    maybe_promote_live_input_qos();
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = get_clock()->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "sunhorizon_airy_adapter";
    status.hardware_id = "Airy";
    const auto now_ns = get_clock()->now().nanoseconds();
    const auto last_point_ns = last_point_receipt_ns_.load();
    const auto last_imu_ns = last_imu_receipt_ns_.load();
    const bool fresh = last_point_ns > 0 && last_imu_ns > 0 &&
        static_cast<double>(now_ns - last_point_ns) * 1e-9 < 1.0 &&
        static_cast<double>(now_ns - last_imu_ns) * 1e-9 < 1.0;
    const bool calibrated = point_calibration_valid_ && imu_calibration_valid_;
    const bool transport_ready =
        direct_ || !live_tracking_clock_ || !reliable_sensor_qos_ || reliable_live_ready_.load();
    status.level = calibrated && fresh && transport_ready
        ? diagnostic_msgs::msg::DiagnosticStatus::OK
        : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = !calibrated ? "point_or_imu_calibration_not_accepted"
        : !transport_ready ? "reliable_live_transport_not_ready"
        : fresh ? "ok" : "sensor_input_stale";
    status.values.resize(19);
    status.values[18].key = "input_transport";
    status.values[18].value = direct_ ? "direct" : "dds";
    status.values[0].key = "point_inputs";
    status.values[0].value = std::to_string(point_inputs_.load());
    status.values[1].key = "imu_inputs";
    status.values[1].value = std::to_string(imu_inputs_.load());
    status.values[2].key = "point_rejections";
    status.values[2].value = std::to_string(point_rejections_.load());
    status.values[3].key = "last_reason";
    status.values[3].value = last_reason();
    status.values[4].key = "imu_rejections";
    status.values[4].value = std::to_string(imu_rejections_.load());
    status.values[5].key = "point_outputs";
    status.values[5].value = std::to_string(point_outputs_.load());
    status.values[6].key = "imu_outputs";
    status.values[6].value = std::to_string(imu_outputs_.load());
    status.values[7].key = "reliable_sensor_qos";
    status.values[7].value = reliable_sensor_qos_ ? "true" : "false";
    status.values[8].key = "header_time_mode";
    status.values[8].value = header_time_mode_;
    status.values[9].key = "header_time_offset_sec";
    bool header_time_initialized = false;
    {
      std::lock_guard<std::mutex> lock(header_time_mutex_);
      status.values[9].value = std::to_string(header_time_rebaser_->offset_sec());
      status.values[10].key = "header_time_last_observed_offset_sec";
      status.values[10].value = std::to_string(
          header_time_rebaser_->last_observed_offset_sec());
      status.values[11].key = "header_time_last_adjustment_sec";
      status.values[11].value = std::to_string(
          header_time_rebaser_->last_offset_adjustment_sec());
      header_time_initialized = header_time_rebaser_->initialized();
    }
    status.values[12].key = "maximum_future_stamp_sec";
    status.values[12].value = std::to_string(maximum_future_stamp_sec_);
    status.values[13].key = "header_time_offset_initialized";
    status.values[13].value = header_time_initialized ? "true" : "false";
    status.values[14].key = "reliable_input_sensor_qos";
    status.values[14].value = reliable_input_sensor_qos_.load() ? "true" : "false";
    status.values[15].key = "reliable_output_sensor_qos";
    status.values[15].value = reliable_output_sensor_qos_ ? "true" : "false";
    status.values[16].key = "live_input_qos_promotion_pending";
    status.values[16].value =
        live_input_qos_promotion_pending_.load() ? "true" : "false";
    status.values[17].key = "reliable_live_ready";
    status.values[17].value = reliable_live_ready_.load() ? "true" : "false";
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  const bool direct_;
  sunhorizon_airy_adapter::AiryDirectSinks sinks_;
  std::string points_owner_, imu_owner_;
  std::mutex points_direct_mutex_, imu_direct_mutex_;
  uint64_t point_direct_sequence_{0}, imu_direct_sequence_{0};
  std::string expected_input_points_frame_;
  std::string expected_input_imu_frame_;
  std::string input_points_topic_;
  std::string input_imu_topic_;
  std::string output_points_frame_;
  std::string output_imu_frame_;
  std::string timestamp_field_;
  std::string timestamp_mode_;
  std::string timestamp_field_datatype_;
  std::string header_time_mode_;
  bool point_calibration_valid_{false};
  bool imu_calibration_valid_{false};
  bool input_orientation_available_{false};
  bool reliable_sensor_qos_{false};
  std::atomic<bool> reliable_input_sensor_qos_{false};
  bool reliable_output_sensor_qos_{false};
  std::atomic<bool> live_input_qos_promotion_pending_{false};
  bool live_tracking_clock_{false};
  std::atomic<bool> reliable_generation_point_received_{false};
  std::atomic<bool> reliable_generation_imu_received_{false};
  std::atomic<bool> reliable_live_ready_{false};
  std::atomic<std::size_t> active_input_point_queue_depth_{0U};
  std::atomic<std::size_t> active_input_imu_queue_depth_{0U};
  double maximum_scan_duration_sec_{0.2};
  double maximum_future_stamp_sec_{0.1};
  double linear_acceleration_scale_{sunhorizon_airy_adapter::kDefaultLinearAccelerationScale};
  std::atomic<double> last_output_point_header_sec_{0.0};
  std::atomic<double> last_output_imu_header_sec_{0.0};
  std::unique_ptr<sunhorizon_airy_adapter::HeaderTimeRebaser> header_time_rebaser_;
  tf2::Vector3 translation_;
  tf2::Quaternion point_calibration_;
  tf2::Quaternion imu_calibration_;
  tf2::Matrix3x3 point_rotation_;
  tf2::Matrix3x3 imu_rotation_;
  std::atomic<std::int64_t> last_point_receipt_ns_{0};
  std::atomic<std::int64_t> last_imu_receipt_ns_{0};
  mutable std::mutex header_time_mutex_;
  mutable std::mutex reason_mutex_;
  std::string last_reason_{"waiting_for_input"};
  std::atomic<uint64_t> point_inputs_{0};
  std::atomic<uint64_t> imu_inputs_{0};
  std::atomic<uint64_t> point_rejections_{0};
  std::atomic<uint64_t> imu_rejections_{0};
  std::atomic<uint64_t> point_outputs_{0};
  std::atomic<uint64_t> imu_outputs_{0};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::CallbackGroup::SharedPtr points_callback_group_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::CallbackGroup::SharedPtr diagnostics_callback_group_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

namespace sunhorizon_airy_adapter {
std::shared_ptr<AiryDirectAdapter> make_airy_direct_adapter(const rclcpp::NodeOptions& options,
    AiryDirectSinks sinks, std::string points_owner, std::string imu_owner) {
  return std::make_shared<AiryAdapter>(options, true, std::move(sinks), std::move(points_owner), std::move(imu_owner));
}
rclcpp::Node::SharedPtr make_airy_adapter_node(const rclcpp::NodeOptions& options) {
  return std::make_shared<AiryAdapter>(options);
}
}
