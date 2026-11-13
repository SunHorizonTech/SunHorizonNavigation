#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sunhorizon_navigation_interfaces/msg/localization_status.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "sunhorizon_navigation_core/latest_observer.hpp"
#include "sunhorizon_livo_bringup/candidate_admission_diagnostics.hpp"
#include "sunhorizon_livo_bringup/causal_body_velocity.hpp"
#include "sunhorizon_livo_bringup/livo_direct_adapter.hpp"
#include "sunhorizon_livo_bringup/locked_time_snapshot.hpp"
#include "sunhorizon_livo_bringup/measurement_continuity.hpp"
#include "sunhorizon_livo_bringup/point_cloud_conversion.hpp"
#include "sunhorizon_livo_bringup/measurement_freshness.hpp"
#include "sunhorizon_livo_bringup/propagation_watchdog.hpp"
#include "sunhorizon_livo_bringup/propagation_causal_audit.hpp"
#include "sunhorizon_livo_bringup/timestamp_rate_gate.hpp"

namespace
{

using LocalizationStatus = sunhorizon_navigation_interfaces::msg::LocalizationStatus;
constexpr char kRos2PortCommit[] = "a09599a288c56b832114d7ad9e898248a4111d9e";
constexpr char kOfficialAiryReferenceCommit[] =
  "0d2c0346107b75b59934975adec9a6eeeb913c64";
constexpr char kPointsTopic[] = "/sunhorizon/airy/points";
constexpr char kImuTopic[] = "/sunhorizon/airy/imu";
constexpr char kLiveImageTopic[] = "/sunhorizon/raw/rgbd/chest/color/image";
constexpr char kLiveCameraInfoTopic[] =
  "/sunhorizon/raw/rgbd/chest/color/camera_info";
constexpr char kReplayImageTopic[] = "/sunhorizon/rgbd/color/image";
constexpr char kReplayCameraInfoTopic[] = "/sunhorizon/rgbd/color/camera_info";
constexpr char kVendorPointsTopic[] = "/sunhorizon/livo/vendor/points";
constexpr char kVendorImageTopic[] = "/sunhorizon/livo/vendor/image";
constexpr char kVendorOdometryTopic[] = "/sunhorizon/livo/vendor/odom";
constexpr char kVendorPropagatedOdometryTopic[] =
  "/sunhorizon/livo/vendor/imu_propagate";
constexpr char kOdometryCandidateTopic[] =
  "/sunhorizon/localization/livo/odom_candidate";
constexpr char kOdometryStatusTopic[] =
  "/sunhorizon/localization/livo/odometry_status";
constexpr char kAdapterDiagnosticsTopic[] =
  "/sunhorizon/livo/adapter_diagnostics";

const std::vector<std::string> kRequiredPointFields{
  "x", "y", "z", "intensity", "timestamp", "ring"};

bool finite(double value)
{
  return std::isfinite(value);
}

class PendingCallbackGuard
{
public:
  explicit PendingCallbackGuard(std::atomic_uint & pending)
  : pending_(pending)
  {
    pending_.fetch_add(1U, std::memory_order_acq_rel);
  }

  ~PendingCallbackGuard()
  {
    pending_.fetch_sub(1U, std::memory_order_release);
  }

  PendingCallbackGuard(const PendingCallbackGuard &) = delete;
  PendingCallbackGuard & operator=(const PendingCallbackGuard &) = delete;

private:
  std::atomic_uint & pending_;
};

rclcpp::QoS sensor_qos(std::size_t depth, bool reliable)
{
  rclcpp::QoS qos{rclcpp::KeepLast(depth)};
  qos.durability_volatile();
  if (reliable) {
    qos.reliable();
  } else {
    qos.best_effort();
  }
  return qos;
}

class LivoAdapter final : public rclcpp::Node,
  public sunhorizon_livo_bringup::LivoDirectAdapter
{
public:
  explicit LivoAdapter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
    bool direct_adapter = false, sunhorizon_livo_bringup::LivoDirectSinks sinks = {})
  : Node("sunhorizon_livo_adapter", options), direct_adapter_(direct_adapter),
    direct_sinks_(std::move(sinks)),
    visual_inputs_required_(!direct_adapter_ || direct_sinks_.vendor_images_enabled)
  {
    mode_ = declare_parameter("mode", "mapping");
    const auto ros2_commit = declare_parameter("vendor_ros2_commit", std::string{});
    const auto airy_commit = declare_parameter(
      "official_airy_reference_commit", std::string{});
    const auto points_topic = declare_parameter("input_points_topic", kPointsTopic);
    const auto imu_topic = declare_parameter("input_imu_topic", kImuTopic);
    const auto image_topic = declare_parameter("input_image_topic", kLiveImageTopic);
    const auto camera_info_topic = declare_parameter(
      "input_camera_info_topic", kLiveCameraInfoTopic);
    const auto vendor_points_topic = declare_parameter(
      "vendor_points_topic", kVendorPointsTopic);
    vendor_point_rate_hz_ = declare_parameter("vendor_point_rate_hz", 10.0);
    const auto vendor_image_topic = declare_parameter(
      "vendor_image_topic", kVendorImageTopic);
    vendor_image_rate_hz_ = declare_parameter("vendor_image_rate_hz", 15.0);
    const auto vendor_odometry_topic = declare_parameter(
      "vendor_odometry_topic", kVendorOdometryTopic);
    const auto vendor_propagated_odometry_topic = declare_parameter(
      "vendor_propagated_odometry_topic", kVendorPropagatedOdometryTopic);
    const auto output_odometry_topic = declare_parameter(
      "output_odometry_topic", kOdometryCandidateTopic);
    const auto output_status_topic = declare_parameter(
      "output_odometry_status_topic", kOdometryStatusTopic);
    expected_vendor_frame_ = declare_parameter(
      "expected_vendor_frame", "camera_init");
    expected_vendor_child_frame_ = declare_parameter(
      "expected_vendor_child_frame", "aft_mapped");
    output_odometry_frame_ = declare_parameter("output_odometry_frame", "odom");
    output_base_frame_ = declare_parameter("output_base_frame", "base_footprint");
    const bool vendor_child_to_base_accepted = declare_parameter(
      "vendor_child_to_base_transform_accepted", false);
    const auto vendor_child_to_base = declare_parameter<std::vector<double>>(
      "vendor_child_to_base_xyz_qxyzw",
      {-0.091935097661, 0.005730023155, -0.214831103888,
        0.003078658613, 0.000905027588, -0.001042186395, 0.999994308301});
    rcl_interfaces::msg::ParameterDescriptor body_feedback_descriptor;
    body_feedback_descriptor.read_only=true;
    body_velocity_enabled_ = declare_parameter("body_velocity_feedback_enabled",false,body_feedback_descriptor);
    if (body_velocity_enabled_ && (!direct_adapter_ ||
        direct_sinks_.body_transform_tf_mode != "attended_measured_direct" ||
        !direct_sinks_.source_time_vendor_child_from_base))
      throw std::runtime_error("body_velocity_requires_consistent_dynamic_tf_owner");
    const auto required_fields = declare_parameter<std::vector<std::string>>(
      "required_point_fields", kRequiredPointFields);
    const auto input_intensity_datatype = declare_parameter(
      "input_intensity_datatype", "uint8");
    const auto vendor_intensity_datatype = declare_parameter(
      "vendor_intensity_datatype", "float32");
    const auto input_timestamp_datatype = declare_parameter(
      "input_timestamp_datatype", "float32");
    const auto vendor_timestamp_datatype = declare_parameter(
      "vendor_timestamp_datatype", "float64");
    maximum_input_age_sec_ = declare_parameter("maximum_input_age_sec", 0.5);
    const double maximum_vendor_odometry_measurement_gap_sec = declare_parameter(
      "maximum_vendor_odometry_measurement_gap_sec", 0.5);
    const double maximum_vendor_propagated_odometry_gap_sec = declare_parameter(
      "maximum_vendor_propagated_odometry_gap_sec", 0.25);
    const double maximum_vendor_propagation_horizon_sec = declare_parameter(
      "maximum_vendor_propagation_horizon_sec", 0.5);
    const double status_rate_hz = declare_parameter("status_rate_hz", 10.0);
    reliable_sensor_qos_ = declare_parameter("reliable_sensor_qos", false);
    reliable_input_image_qos_ = declare_parameter(
      "reliable_input_image_qos", false);
    reliable_image_qos_ = declare_parameter("reliable_image_qos", true);
    reliable_odometry_qos_ = declare_parameter("reliable_odometry_qos", false);
    use_propagated_odometry_ = declare_parameter("use_propagated_odometry", false);
    const bool retain_replay_odometry_history =
      get_parameter("use_sim_time").as_bool();

    if (mode_ != "mapping" && mode_ != "localization") {
      throw std::runtime_error("mode must be mapping or localization");
    }
    if (ros2_commit != kRos2PortCommit || airy_commit != kOfficialAiryReferenceCommit) {
      throw std::runtime_error("FAST-LIVO2 provenance parameters do not match pinned commits");
    }
    require_equal(points_topic, kPointsTopic, "input_points_topic");
    require_equal(imu_topic, kImuTopic, "input_imu_topic");
    const bool live_camera_pair =
      image_topic == kLiveImageTopic && camera_info_topic == kLiveCameraInfoTopic;
    const bool replay_camera_pair =
      image_topic == kReplayImageTopic && camera_info_topic == kReplayCameraInfoTopic;
    if (!live_camera_pair && !replay_camera_pair) {
      throw std::runtime_error(
              "input_image_topic/input_camera_info_topic must be the owned live raw "
              "pair or sealed replay normalized pair");
    }
    require_equal(vendor_points_topic, kVendorPointsTopic, "vendor_points_topic");
    require_equal(vendor_image_topic, kVendorImageTopic, "vendor_image_topic");
    require_equal(vendor_odometry_topic, kVendorOdometryTopic, "vendor_odometry_topic");
    require_equal(
      vendor_propagated_odometry_topic, kVendorPropagatedOdometryTopic,
      "vendor_propagated_odometry_topic");
    require_equal(output_odometry_topic, kOdometryCandidateTopic, "output_odometry_topic");
    require_equal(output_status_topic, kOdometryStatusTopic, "output_odometry_status_topic");
    if (required_fields != kRequiredPointFields) {
      throw std::runtime_error("required_point_fields must be x/y/z/intensity/timestamp/ring");
    }
    if (input_intensity_datatype != "uint8" || vendor_intensity_datatype != "float32" ||
      input_timestamp_datatype != "float32" || vendor_timestamp_datatype != "float64")
    {
      throw std::runtime_error(
        "Airy uint8 intensity/float32 timestamp to FAST-LIVO2 "
        "float32 intensity/float64 timestamp contract required");
    }
    if (!finite(maximum_input_age_sec_) || maximum_input_age_sec_ <= 0.0 ||
      !finite(maximum_vendor_odometry_measurement_gap_sec) ||
      maximum_vendor_odometry_measurement_gap_sec <= 0.0 ||
      !finite(maximum_vendor_propagated_odometry_gap_sec) ||
      maximum_vendor_propagated_odometry_gap_sec <= 0.0 ||
      !finite(maximum_vendor_propagation_horizon_sec) ||
      maximum_vendor_propagation_horizon_sec <= 0.0 ||
      !finite(vendor_point_rate_hz_) || vendor_point_rate_hz_ <= 0.0 ||
      !finite(vendor_image_rate_hz_) || vendor_image_rate_hz_ <= 0.0 ||
      !finite(status_rate_hz) || status_rate_hz <= 0.0)
    {
      throw std::runtime_error("freshness limits must be finite and positive");
    }
    const bool accepted_localization_point_rate =
      std::abs(vendor_point_rate_hz_ - 6.0) <= 1e-9 ||
      std::abs(vendor_point_rate_hz_ - 8.0) <= 1e-9 ||
      std::abs(vendor_point_rate_hz_ - 10.0) <= 1e-9;
    if ((mode_ == "localization" && !accepted_localization_point_rate) ||
      (mode_ == "mapping" && std::abs(vendor_point_rate_hz_ - 10.0) > 1e-9))
    {
      throw std::runtime_error(
        "vendor_point_rate_hz must be 6, 8 or 10 Hz for localization and 10 Hz for mapping");
    }
    const bool accepted_localization_image_rate =
      std::abs(vendor_image_rate_hz_ - 6.0) <= 1e-9 ||
      std::abs(vendor_image_rate_hz_ - 8.0) <= 1e-9 ||
      std::abs(vendor_image_rate_hz_ - 10.0) <= 1e-9;
    if ((mode_ == "localization" && !accepted_localization_image_rate) ||
      (mode_ == "mapping" && std::abs(vendor_image_rate_hz_ - 15.0) > 1e-9))
    {
      throw std::runtime_error(
        "vendor_image_rate_hz must be 6, 8 or 10 Hz for localization and 15 Hz for mapping");
    }
    if (!vendor_child_to_base_accepted || vendor_child_to_base.size() != 7U ||
      expected_vendor_frame_.empty() || expected_vendor_child_frame_.empty() ||
      output_odometry_frame_ != "odom" || output_base_frame_ != "base_footprint")
    {
      throw std::runtime_error(
        "accepted vendor-child to canonical-base transform is required");
    }
    const tf2::Vector3 translation(
      vendor_child_to_base[0], vendor_child_to_base[1], vendor_child_to_base[2]);
    tf2::Quaternion rotation(
      vendor_child_to_base[3], vendor_child_to_base[4],
      vendor_child_to_base[5], vendor_child_to_base[6]);
    if (!finite(translation.x()) || !finite(translation.y()) ||
      !finite(translation.z()) || !finite(rotation.x()) || !finite(rotation.y()) ||
      !finite(rotation.z()) || !finite(rotation.w()) || rotation.length2() < 1e-12)
    {
      throw std::runtime_error("vendor-child to base transform is invalid");
    }
    rotation.normalize();
    vendor_child_to_base_.setOrigin(translation);
    vendor_child_to_base_.setRotation(rotation);
    propagation_watchdog_.emplace(
      maximum_vendor_odometry_measurement_gap_sec,
      maximum_vendor_propagated_odometry_gap_sec,
      maximum_vendor_propagation_horizon_sec);
    vendor_point_rate_gate_.emplace(vendor_point_rate_hz_);
    vendor_image_rate_gate_.emplace(vendor_image_rate_hz_);

    const auto points_qos = sensor_qos(
      reliable_sensor_qos_ ? 32U : 4U, reliable_sensor_qos_);
    const auto imu_qos = sensor_qos(
      reliable_sensor_qos_ ? 2000U : 100U, reliable_sensor_qos_);
    const auto input_image_qos = sensor_qos(
      reliable_input_image_qos_ ? 1U : 4U, reliable_input_image_qos_);
    const auto input_camera_info_qos = sensor_qos(
      reliable_input_image_qos_ ? 1U : 4U, reliable_input_image_qos_);
    const auto vendor_image_qos = sensor_qos(
      reliable_image_qos_ ? 1U : 4U, reliable_image_qos_);
    // Reliability and replay retention are independent. Robot A uses reliable
    // transport live, but live odometry is state rather than a replay log: a
    // delayed executor must jump directly to the newest sample. Only sealed
    // simulated-time replay retains a bounded FIFO history.
    const std::size_t vendor_odometry_depth =
      retain_replay_odometry_history ? 10U : 1U;
    const std::size_t vendor_propagated_odometry_depth =
      retain_replay_odometry_history ? 1000U : 1U;
    const std::size_t odometry_candidate_depth =
      retain_replay_odometry_history ? 10U : 1U;
    const auto vendor_odometry_qos = sensor_qos(
      vendor_odometry_depth, reliable_odometry_qos_);
    const auto vendor_propagated_odometry_qos = sensor_qos(
      vendor_propagated_odometry_depth, reliable_odometry_qos_);
    point_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    sensor_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    image_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    corrected_odometry_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    propagated_odometry_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    if (direct_adapter_) {
      timer_callback_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, true);
    } else {
      timer_callback_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    }
    rclcpp::SubscriptionOptions point_subscription_options;
    point_subscription_options.callback_group = point_callback_group_;
    rclcpp::SubscriptionOptions sensor_subscription_options;
    sensor_subscription_options.callback_group = sensor_callback_group_;
    rclcpp::SubscriptionOptions image_subscription_options;
    image_subscription_options.callback_group = image_callback_group_;
    rclcpp::SubscriptionOptions corrected_odometry_subscription_options;
    corrected_odometry_subscription_options.callback_group =
      corrected_odometry_callback_group_;
    rclcpp::SubscriptionOptions propagated_odometry_subscription_options;
    propagated_odometry_subscription_options.callback_group =
      propagated_odometry_callback_group_;
    if (!direct_adapter_) {
      vendor_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        vendor_points_topic, points_qos);
      vendor_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        vendor_image_topic, vendor_image_qos);
    } else if (!direct_sinks_.vendor_points || !direct_sinks_.vendor_imu ||
      (visual_inputs_required_ && !direct_sinks_.vendor_image) || !direct_sinks_.candidate) {
      throw std::runtime_error("livo_direct_adapter_required_sink_missing");
    }
    odometry_candidate_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      output_odometry_topic,
      rclcpp::QoS(rclcpp::KeepLast(odometry_candidate_depth)).reliable());
    odometry_status_pub_ = create_publisher<LocalizationStatus>(
      output_status_topic, rclcpp::QoS(10).reliable().transient_local());
    adapter_diagnostics_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(kAdapterDiagnosticsTopic, 10);

    if (!direct_adapter_) {
    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic, points_qos,
      std::bind(&LivoAdapter::on_points, this, std::placeholders::_1),
      point_subscription_options);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, imu_qos,
      std::bind(&LivoAdapter::on_imu, this, std::placeholders::_1),
      sensor_subscription_options);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, input_image_qos,
      std::bind(&LivoAdapter::on_image, this, std::placeholders::_1),
      image_subscription_options);
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, input_camera_info_qos,
      std::bind(&LivoAdapter::on_camera_info, this, std::placeholders::_1),
      sensor_subscription_options);
    vendor_odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      vendor_odometry_topic, vendor_odometry_qos,
      std::bind(&LivoAdapter::on_vendor_odometry, this, std::placeholders::_1),
      corrected_odometry_subscription_options);
    if (use_propagated_odometry_) {
      vendor_propagated_odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        vendor_propagated_odometry_topic, vendor_propagated_odometry_qos,
        std::bind(
          &LivoAdapter::on_vendor_propagated_odometry, this,
          std::placeholders::_1),
        propagated_odometry_subscription_options);
    }

    }

    const auto period = std::chrono::duration<double>(1.0 / status_rate_hz);
    status_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LivoAdapter::publish_status, this), timer_callback_group_);
    transport_diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&LivoAdapter::publish_transport_diagnostics, this),
      timer_callback_group_);
    RCLCPP_INFO(
      get_logger(),
      "LIVO adapter point/IMU QoS=%s raw image QoS=%s vendor image QoS=%s "
      "depths=4/100/4/1",
      reliable_sensor_qos_ ? "reliable" : "best_effort",
      reliable_input_image_qos_ ? "reliable" : "best_effort",
      reliable_image_qos_ ? "reliable" : "best_effort");
    RCLCPP_INFO(
      get_logger(), "LIVO adapter vendor odometry QoS reliability=%s retention=%s "
      "corrected_depth=%zu propagated_depth=%zu candidate_depth=%zu",
      reliable_odometry_qos_ ? "reliable" : "best_effort",
      retain_replay_odometry_history ? "sealed_replay" : "live_latest",
      vendor_odometry_depth, vendor_propagated_odometry_depth,
      odometry_candidate_depth);
    RCLCPP_INFO(
      get_logger(), "LIVO adapter tracking source=%s",
      use_propagated_odometry_ ? "imu_propagate_with_corrected_watchdog" :
      "corrected_livo_odometry");
    RCLCPP_INFO(
      get_logger(), "LIVO adapter vendor point topic=%s rate=%.1fHz input_topic=%s",
      vendor_points_topic.c_str(), vendor_point_rate_hz_, points_topic.c_str());
    RCLCPP_INFO(
      get_logger(), "LIVO adapter vendor RGB topic=%s rate=%.1fHz input_topic=%s",
      vendor_image_topic.c_str(), vendor_image_rate_hz_, image_topic.c_str());
    if (direct_adapter_) {
      const auto failure = [this](std::exception_ptr) {
        direct_observer_failed_.store(true);
        std::lock_guard<std::mutex> lock(mutex_);
        odometry_valid_ = false;
        odometry_error_ = "livo_direct_observer_failed";
      };
      direct_candidate_observer_ = std::make_unique<CandidateObserver>([this](auto snapshot) {
        const auto fresh = sunhorizon_livo_bringup::evaluate_measurement_freshness(
          snapshot->stamp.source_stamp_ns, now().nanoseconds(), maximum_input_age_sec_);
        const int64_t steady = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
        if (!fresh.accepted || steady < snapshot->stamp.receipt_steady_ns ||
          (steady-snapshot->stamp.receipt_steady_ns)*1e-9 > maximum_input_age_sec_) return;
        odometry_candidate_pub_->publish(*snapshot->message);
      }, failure);
      direct_status_observer_ = std::make_unique<StatusObserver>([this](auto snapshot) {
        odometry_status_pub_->publish(*snapshot);
      }, failure);
      direct_diagnostics_observer_ = std::make_unique<DiagnosticsObserver>([this](auto snapshot) {
        adapter_diagnostics_pub_->publish(*snapshot);
      }, failure);
    }
  }

  rclcpp::CallbackGroup::SharedPtr point_callback_group() const
  {
    return point_callback_group_;
  }

  rclcpp::CallbackGroup::SharedPtr sensor_callback_group() const
  {
    return sensor_callback_group_;
  }

  rclcpp::CallbackGroup::SharedPtr image_callback_group() const
  {
    return image_callback_group_;
  }

  rclcpp::CallbackGroup::SharedPtr corrected_odometry_callback_group() const
  {
    return corrected_odometry_callback_group_;
  }

  rclcpp::CallbackGroup::SharedPtr propagated_odometry_callback_group() const
  {
    return propagated_odometry_callback_group_;
  }

  rclcpp::CallbackGroup::SharedPtr timer_callback_group() const
  {
    return timer_callback_group_;
  }

  ~LivoAdapter() override {
    direct_candidate_observer_.reset();
    direct_status_observer_.reset();
    direct_diagnostics_observer_.reset();
  }

  std::shared_ptr<rclcpp::Node> node() override {return shared_from_this();}

  sunhorizon_livo_bringup::BodyOdometryGaugeOrigin body_odometry_gauge_origin() override {
    std::lock_guard<std::mutex> lock(mutex_);return body_odometry_gauge_.origin();
  }
  void reset_body_velocity(int64_t original_reset_steady_ns) override {
    std::lock_guard<std::mutex> lock(mutex_);
    body_velocity_.reset();body_odometry_gauge_.reset();
    body_reset_barrier_steady_ns_ = original_reset_steady_ns;
  }

  void receive_points(sensor_msgs::msg::PointCloud2::ConstSharedPtr input,
    const sunhorizon_livo_bringup::LivoDirectStamp& stamp) override {
    if (admit_direct(input, stamp, 0)) process_on_points(std::move(input), stamp);
  }
  void receive_imu(sensor_msgs::msg::Imu::ConstSharedPtr input,
    const sunhorizon_livo_bringup::LivoDirectStamp& stamp) override {
    if (admit_direct(input, stamp, 1)) process_on_imu(std::move(input), stamp);
  }
  void receive_image(sensor_msgs::msg::Image::ConstSharedPtr input,
    const sunhorizon_livo_bringup::LivoDirectStamp& stamp) override {
    // Disabled by the sole estimator's immutable construction contract. An
    // unused RGB input is neither a measurement nor a freshness lease.
    if (!visual_inputs_required_) return;
    if (admit_direct(input, stamp, 2)) process_on_image(std::move(input), stamp);
  }
  void receive_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr input,
    const sunhorizon_livo_bringup::LivoDirectStamp& stamp) override {
    if (!visual_inputs_required_) return;
    if (admit_direct(input, stamp, 3)) process_on_camera_info(std::move(input), stamp);
  }
  void receive_corrected_odometry(nav_msgs::msg::Odometry::ConstSharedPtr input,
    const sunhorizon_livo_bringup::LivoDirectStamp& stamp) override {
    if (admit_direct(input, stamp, 4)) process_on_vendor_odometry(std::move(input), stamp);
  }
  void receive_propagated_odometry(nav_msgs::msg::Odometry::ConstSharedPtr input,
    const sunhorizon_livo_bringup::LivoDirectStamp& stamp) override {
    if (admit_direct(input, stamp, 5)) process_on_vendor_propagated_odometry(std::move(input), stamp);
  }

private:
  using DirectStamp = sunhorizon_livo_bringup::LivoDirectStamp;
  static sunhorizon_livo_bringup::PropagationAuditStamp audit_stamp(const DirectStamp& stamp) {
    sunhorizon_livo_bringup::PropagationAuditStamp result{stamp.source_stamp_ns,
      stamp.receipt_ros_ns,stamp.receipt_steady_ns,stamp.sequence,stamp.source_identity,{}};
    if(stamp.delivery_timing){
      const auto& t=*stamp.delivery_timing;
      result.delivery_timing=std::array<int64_t,4>{{t.output_lane_enqueue_steady_ns,
        t.output_lane_dequeue_steady_ns,t.geometry_wait_begin_steady_ns,t.geometry_wait_end_steady_ns}};
    }
    return result;
  }
  struct DirectCandidate {
    nav_msgs::msg::Odometry::ConstSharedPtr message;
    DirectStamp stamp;
  };
  using CandidateObserver = sunhorizon_navigation_core::LatestObserver<DirectCandidate>;
  using StatusObserver = sunhorizon_navigation_core::LatestObserver<LocalizationStatus>;
  using DiagnosticsObserver = sunhorizon_navigation_core::LatestObserver<diagnostic_msgs::msg::DiagnosticArray>;

  rclcpp::Time receipt_time(const std::optional<DirectStamp>& stamp) const {
    return stamp ? rclcpp::Time(stamp->receipt_ros_ns, get_clock()->get_clock_type()) : now();
  }

  template <typename Message>
  bool admit_direct(const std::shared_ptr<const Message>& input,
    const DirectStamp& stamp, std::size_t channel) {
    const int64_t steady = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    const bool structure_valid = direct_adapter_ && !direct_observer_failed_.load() && input &&
      input->header.stamp.nanosec < 1000000000U &&
      stamp.source_stamp_ns > 0 && stamp.receipt_ros_ns > 0 && stamp.receipt_steady_ns > 0 &&
      stamp.sequence > 0 && !stamp.source_identity.empty() && stamp.source_identity.size() <= 256 &&
      static_cast<int64_t>(input->header.stamp.sec)*1000000000LL + input->header.stamp.nanosec ==
        stamp.source_stamp_ns;
    if (!structure_valid) {
      std::lock_guard<std::mutex> lock(mutex_);
      odometry_valid_ = false;
      odometry_error_ = "livo_direct_input_metadata_invalid";
      if(channel==4)propagation_causal_audit_.rejected_corrected(audit_stamp(stamp),odometry_error_);
      throw std::invalid_argument(odometry_error_);
    }
    if (body_velocity_enabled_ && channel>=4) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stamp.receipt_steady_ns<=body_reset_barrier_steady_ns_) {
        if(channel==4)propagation_causal_audit_.rejected_corrected(audit_stamp(stamp),"body_reset_barrier");
        return false;
      }
    }
    {
      std::lock_guard<std::mutex> lock(direct_input_mutex_);
      const auto& previous = direct_last_input_[channel];
      if (previous.sequence > 0 &&
        (stamp.sequence <= previous.sequence || stamp.source_stamp_ns <= previous.source_stamp_ns ||
        stamp.receipt_ros_ns < previous.receipt_ros_ns ||
        stamp.receipt_steady_ns < previous.receipt_steady_ns ||
        stamp.source_identity != previous.source_identity)) {
        std::lock_guard<std::mutex> state_lock(mutex_);
        odometry_valid_ = false;
        odometry_error_ = "livo_direct_input_identity_or_time_regressed";
        if(channel==4)propagation_causal_audit_.rejected_corrected(audit_stamp(stamp),odometry_error_);
      body_velocity_.reset();
        return false;
      }
      direct_last_input_[channel] = stamp;
    }
    const auto freshness = sunhorizon_livo_bringup::evaluate_measurement_freshness(
      stamp.source_stamp_ns, now().nanoseconds(), maximum_input_age_sec_);
    if (!freshness.accepted || steady < stamp.receipt_steady_ns ||
      static_cast<double>(steady-stamp.receipt_steady_ns)*1e-9 > maximum_input_age_sec_) {
      std::lock_guard<std::mutex> lock(mutex_);
      odometry_valid_ = false;
      odometry_error_ = "livo_direct_input_source_or_receipt_stale";
      if(channel==4)propagation_causal_audit_.rejected_corrected(audit_stamp(stamp),odometry_error_);
      body_velocity_.reset();
      return false;
    }
    return true;
  }

  static void require_equal(
    const std::string & actual, const std::string & expected, const std::string & name)
  {
    if (actual != expected) {
      throw std::runtime_error(name + " is a fixed Robot A contract");
    }
  }

  static bool valid_imu(const sensor_msgs::msg::Imu & input)
  {
    return !input.header.frame_id.empty() &&
      finite(input.angular_velocity.x) && finite(input.angular_velocity.y) &&
      finite(input.angular_velocity.z) && finite(input.linear_acceleration.x) &&
      finite(input.linear_acceleration.y) && finite(input.linear_acceleration.z);
  }

  static bool valid_image(const sensor_msgs::msg::Image & input)
  {
    return !input.header.frame_id.empty() && input.width > 0U && input.height > 0U &&
      !input.encoding.empty() && !input.data.empty();
  }

  static bool valid_camera_info(const sensor_msgs::msg::CameraInfo & input)
  {
    return !input.header.frame_id.empty() && input.width > 0U && input.height > 0U &&
      finite(input.k[0]) && finite(input.k[4]) && input.k[0] > 0.0 && input.k[4] > 0.0;
  }

  static bool valid_odometry(const nav_msgs::msg::Odometry & input)
  {
    const auto & position = input.pose.pose.position;
    const auto & orientation = input.pose.pose.orientation;
    const double quaternion_norm =
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w;
    const bool covariance_valid =
      finite(input.pose.covariance[0]) && input.pose.covariance[0] >= 0.0 &&
      finite(input.pose.covariance[7]) && input.pose.covariance[7] >= 0.0 &&
      finite(input.pose.covariance[14]) && input.pose.covariance[14] >= 0.0 &&
      finite(input.pose.covariance[35]) && input.pose.covariance[35] >= 0.0;
    return !input.header.frame_id.empty() && !input.child_frame_id.empty() &&
      finite(position.x) && finite(position.y) && finite(position.z) &&
      finite(orientation.x) && finite(orientation.y) && finite(orientation.z) &&
      finite(orientation.w) && std::abs(quaternion_norm - 1.0) <= 1e-3 &&
      covariance_valid;
  }

  std::optional<nav_msgs::msg::Odometry> canonical_odometry(
    const nav_msgs::msg::Odometry & input, bool selected_for_tracking,
    std::optional<sunhorizon_navigation_core::BodyVelocityEvidence>& velocity_evidence,
    sunhorizon_livo_bringup::BodyVelocityGeometryStatus* geometry_status = nullptr)
  {
    if(geometry_status)*geometry_status=sunhorizon_livo_bringup::BodyVelocityGeometryStatus::Unavailable;
    if (input.header.frame_id != expected_vendor_frame_ ||
      input.child_frame_id != expected_vendor_child_frame_)
    {
      return std::nullopt;
    }
    tf2::Transform vendor_world_to_child;
    try {
      tf2::fromMsg(input.pose.pose, vendor_world_to_child);
    } catch (const std::exception &) {
      return std::nullopt;
    }
    auto child_from_base = vendor_child_to_base_;
    if (body_velocity_enabled_) {
      const auto dynamic = direct_sinks_.source_time_vendor_child_from_base(
        rclcpp::Time(input.header.stamp).nanoseconds(), tf2::toMsg(vendor_child_to_base_));
      if(geometry_status)*geometry_status=dynamic.status;
      if (dynamic.status!=sunhorizon_livo_bringup::BodyVelocityGeometryStatus::Ready || !dynamic.transform) {
        // A malformed Ready result is a fault, never temporary unavailability.
        const auto status=dynamic.status==sunhorizon_livo_bringup::BodyVelocityGeometryStatus::Ready ?
          sunhorizon_livo_bringup::BodyVelocityGeometryStatus::Unavailable : dynamic.status;
        if(selected_for_tracking)body_velocity_reason_=body_velocity_.reject_geometry(status).reason;
        else body_velocity_reason_="body_velocity_source_time_fk_unavailable";
        return std::nullopt;
      }
      tf2::fromMsg(*dynamic.transform, child_from_base);
    }
    if(geometry_status)*geometry_status=sunhorizon_livo_bringup::BodyVelocityGeometryStatus::Ready;
    const tf2::Transform odom_to_base = vendor_world_to_child * child_from_base;
    nav_msgs::msg::Odometry output = input;
    output.header.frame_id = output_odometry_frame_;
    output.child_frame_id = output_base_frame_;
    const auto & origin = odom_to_base.getOrigin();
    const auto & rotation = odom_to_base.getRotation();
    output.pose.pose.position.x = origin.x();
    output.pose.pose.position.y = origin.y();
    output.pose.pose.position.z = body_velocity_enabled_ ?
      body_odometry_gauge_.relative_z(rclcpp::Time(input.header.stamp).nanoseconds(),origin.z()) : origin.z();
    output.pose.pose.orientation.x = rotation.x();
    output.pose.pose.orientation.y = rotation.y();
    output.pose.pose.orientation.z = rotation.z();
    output.pose.pose.orientation.w = rotation.w();
    // The fixed rollback retains unavailable twist (covariance=1000). Explicit
    // measured mode below replaces it only with a valid source-time causal fit;
    // vendor world-frame vel_end is never relabeled as body velocity.
    output.twist.twist.linear.x = 0.0;
    output.twist.twist.linear.y = 0.0;
    output.twist.twist.linear.z = 0.0;
    output.twist.twist.angular.x = 0.0;
    output.twist.twist.angular.y = 0.0;
    output.twist.twist.angular.z = 0.0;
    output.twist.covariance.fill(0.0);
    for (const std::size_t index : {0U, 7U, 14U, 21U, 28U, 35U}) {
      output.twist.covariance[index] = 1.0e3;
    }
    if (body_velocity_enabled_ && selected_for_tracking) {
      const auto estimate = body_velocity_.observe({rclcpp::Time(input.header.stamp).nanoseconds(),
        origin.x(), origin.y(), std::atan2(2.*(rotation.w()*rotation.z()+rotation.x()*rotation.y()),
          1.-2.*(rotation.y()*rotation.y()+rotation.z()*rotation.z()))});
      body_velocity_reason_=estimate.reason;
      if (!estimate.evidence.valid) return std::nullopt;
      output.twist.twist.linear.x=estimate.vx;output.twist.twist.linear.y=estimate.vy;
      output.twist.twist.angular.z=estimate.wz;
      output.twist.covariance[0]=estimate.evidence.variance_vx;
      output.twist.covariance[7]=estimate.evidence.variance_vy;
      output.twist.covariance[35]=estimate.evidence.variance_wz;
      velocity_evidence=estimate.evidence;
      velocity_evidence->measured_waist=direct_sinks_.body_transform_tf_mode=="attended_measured_direct";
      const auto gauge=body_odometry_gauge_.origin();
      velocity_evidence->odom_z_origin_source_ns=gauge.source_ns;
      velocity_evidence->odom_z_origin_epoch=gauge.epoch;
      velocity_evidence->odom_z_origin_m=gauge.raw_z_m;
    }
    return valid_odometry(output) ?
      std::optional<nav_msgs::msg::Odometry>{std::move(output)} : std::nullopt;
  }

  void on_points(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input)
  {process_on_points(input, std::nullopt);}

  void process_on_points(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input,
    const std::optional<DirectStamp>& metadata)
  {
    bool selected_for_vendor = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++point_inputs_;
      points_seen_ = true;
      points_received_ = receipt_time(metadata);
      const auto stamp_nanoseconds = rclcpp::Time(input->header.stamp).nanoseconds();
      if (stamp_nanoseconds <= 0 ||
        (last_point_input_stamp_nanoseconds_ &&
        stamp_nanoseconds <= *last_point_input_stamp_nanoseconds_))
      {
        points_valid_ = false;
        points_error_ = "point_header_timestamp_invalid_or_non_monotonic";
        ++point_conversion_rejections_;
        return;
      }
      last_point_input_stamp_nanoseconds_ = stamp_nanoseconds;
      // The owned Airy stream is already the canonical 10 Hz source. Running
      // a second exact-10-Hz period gate would interpret harmless scan jitter
      // as an early frame and drop roughly every fifth scan. Monotonicity is
      // enforced above, so 10 Hz is a true pass-through; only the explicit
      // 8 Hz comparison profile uses timestamp rate selection.
      selected_for_vendor = vendor_point_rate_hz_ >= 10.0 - 1e-9 ||
        vendor_point_rate_gate_->should_publish(stamp_nanoseconds);
      if (!selected_for_vendor) {
        ++vendor_point_throttled_;
      }
    }

    if (!selected_for_vendor) {
      return;
    }

    sensor_msgs::msg::PointCloud2 converted;
    std::string reason;
    const bool valid = sunhorizon_livo_bringup::convert_airy_points(
      *input, &converted, &reason);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      points_valid_ = valid;
      points_error_ = valid ? std::string{} : reason;
      if (valid) {
        ++vendor_point_outputs_;
      } else {
        ++point_conversion_rejections_;
      }
    }
    if (valid) {
      if (metadata) {
        direct_sinks_.vendor_points(
          std::make_shared<const sensor_msgs::msg::PointCloud2>(std::move(converted)), *metadata);
      } else {
        vendor_points_pub_->publish(std::move(converted));
      }
    }
  }

  void on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr input)
  {process_on_imu(input, std::nullopt);}

  void process_on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr input,
    const std::optional<DirectStamp>& metadata)
  {
    bool valid = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++imu_inputs_;
      imu_seen_ = true;
      imu_valid_ = valid_imu(*input);
      imu_received_ = receipt_time(metadata);
      valid = imu_valid_;
    }
    if (metadata && valid) direct_sinks_.vendor_imu(input, *metadata);
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr input)
  {process_on_image(input, std::nullopt);}

  void process_on_image(const sensor_msgs::msg::Image::ConstSharedPtr input,
    const std::optional<DirectStamp>& metadata)
  {
    bool publish_vendor_image = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++image_inputs_;
      image_seen_ = true;
      image_valid_ = valid_image(*input);
      image_received_ = receipt_time(metadata);
      if (image_valid_) {
        const rclcpp::Time stamp(input->header.stamp);
        const auto stamp_nanoseconds = stamp.nanoseconds();
        if (last_image_input_stamp_nanoseconds_ &&
          stamp_nanoseconds > *last_image_input_stamp_nanoseconds_)
        {
          maximum_image_input_gap_sec_ = std::max(
            maximum_image_input_gap_sec_,
            static_cast<double>(
              stamp_nanoseconds - *last_image_input_stamp_nanoseconds_) / 1.0e9);
        }
        last_image_input_stamp_nanoseconds_ = stamp_nanoseconds;
        publish_vendor_image = vendor_image_rate_gate_->should_publish(
          stamp_nanoseconds);
        if (publish_vendor_image) {
          if (last_vendor_image_stamp_nanoseconds_) {
            maximum_vendor_image_gap_sec_ = std::max(
              maximum_vendor_image_gap_sec_,
              static_cast<double>(
                stamp_nanoseconds - *last_vendor_image_stamp_nanoseconds_) / 1.0e9);
          }
          last_vendor_image_stamp_nanoseconds_ = stamp_nanoseconds;
          ++vendor_image_outputs_;
        }
      }
    }
    if (publish_vendor_image) {
      if (metadata) direct_sinks_.vendor_image(input, *metadata);
      else vendor_image_pub_->publish(*input);
    }
  }

  void on_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr input)
  {process_on_camera_info(input, std::nullopt);}

  void process_on_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr input,
    const std::optional<DirectStamp>& metadata)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++camera_info_inputs_;
    camera_info_seen_ = true;
    camera_info_valid_ = valid_camera_info(*input);
    camera_info_received_ = receipt_time(metadata);
  }

  void on_vendor_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr input)
  {process_on_vendor_odometry(input, std::nullopt);}

  void process_on_vendor_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr input,
    const std::optional<DirectStamp>& metadata)
  {
    std::optional<nav_msgs::msg::Odometry> candidate;
    std::optional<rclcpp::Time> candidate_received;
    std::optional<sunhorizon_navigation_core::BodyVelocityEvidence> velocity_evidence;
    {
      PendingCallbackGuard pending(corrected_odometry_callbacks_pending_);
      std::lock_guard<std::mutex> lock(mutex_);
      const rclcpp::Time stamp(input->header.stamp);
      const rclcpp::Time received = receipt_time(metadata);
      ++vendor_odometry_inputs_;
      const auto previous_seen=odometry_seen_,previous_valid=odometry_valid_;
      const auto previous_received=odometry_received_;
      const auto previous_error=odometry_error_;
      odometry_seen_ = true;
      odometry_received_ = received;
      odometry_valid_ = valid_odometry(*input);
      odometry_error_.clear();
      const auto freshness =
        sunhorizon_livo_bringup::evaluate_measurement_freshness(
          stamp.nanoseconds(), metadata ? now().nanoseconds() : received.nanoseconds(),
          maximum_input_age_sec_);
      candidate_admission_diagnostics_.observe_freshness(freshness);
      if (odometry_valid_ && !freshness.accepted) {
        odometry_valid_ = false;
        odometry_error_ = "vendor_odometry_" + freshness.reason;
        ++vendor_odometry_stale_rejections_;
      }
      if(body_velocity_enabled_ && !use_propagated_odometry_ && !odometry_valid_){
        body_velocity_.reset();body_velocity_reason_="body_velocity_invalid_original_pose";
      }
      if (odometry_valid_) {
        sunhorizon_livo_bringup::BodyVelocityGeometryStatus geometry;
        candidate = canonical_odometry(*input, !use_propagated_odometry_, velocity_evidence,&geometry);
        if(!candidate && geometry==sunhorizon_livo_bringup::BodyVelocityGeometryStatus::FuturePending){
          // No new accepted pose, lease or watchdog anchor exists. The prior
          // state can remain healthy only until its ORIGINAL receipt expires.
          odometry_seen_=previous_seen;odometry_valid_=previous_valid;
          odometry_received_=previous_received;odometry_error_=previous_error;
          if(metadata)propagation_causal_audit_.rejected_corrected(audit_stamp(*metadata),"body_geometry_future_pending");
          return;
        }
        odometry_valid_ = candidate.has_value();
      }
      if (!odometry_valid_) {
        if (odometry_error_.empty()) {
          odometry_error_ = (body_velocity_enabled_ && !body_velocity_reason_.empty()) ? body_velocity_reason_ :
            "vendor_odometry_invalid_frame_or_pose";
        }
        candidate.reset();
      } else {
        const auto continuity =
          propagation_watchdog_->observe_corrected(stamp.nanoseconds());
        // Only watchdog acceptance identifies the accepted anchor. A received
        // correction, geometry rejection or recovery-anchor rejection cannot.
        if(metadata && continuity.accepted)
          propagation_causal_audit_.accepted_corrected(audit_stamp(*metadata));
        odometry_valid_ = continuity.accepted;
        if (!odometry_valid_) {
          odometry_error_ = continuity.reason;
          candidate.reset();
        }
      }
      if(metadata && !odometry_valid_)
        propagation_causal_audit_.rejected_corrected(audit_stamp(*metadata),odometry_error_);
      if (odometry_valid_ && candidate) {
        last_vendor_stamp_ = stamp;
        last_odometry_ = *candidate;
        if (use_propagated_odometry_) {
          candidate.reset();
        } else if ((!metadata && !all_inputs_healthy(odometry_received_)) ||
          (metadata && !all_inputs_healthy(now()))) {
          const auto suppression_reason = metadata ? health_reason(now()) :
            health_reason(odometry_received_);
          candidate_admission_diagnostics_.record_aggregate_suppression(
            suppression_reason == "ok" ?
            "all_inputs_unhealthy_unspecified" : suppression_reason);
          candidate.reset();
        } else {
          candidate_admission_diagnostics_.clear_aggregate_suppression();
          candidate_received = received;
        }
      }
    }
    if (candidate) {
      if (metadata) {
        auto owner = std::make_shared<const nav_msgs::msg::Odometry>(*candidate);
        auto delivered = *metadata;delivered.body_velocity=velocity_evidence;
        direct_sinks_.candidate(owner, delivered);
        if (direct_sinks_.publish_ros_candidates) direct_candidate_observer_->enqueue(
          std::make_shared<const DirectCandidate>(DirectCandidate{std::move(owner), delivered}));
      } else odometry_candidate_pub_->publish(*candidate);
      std::lock_guard<std::mutex> lock(mutex_);
      candidate_admission_diagnostics_.record_candidate_published(
        rclcpp::Time(candidate->header.stamp).nanoseconds(),
        candidate_received->nanoseconds());
    }
  }

  void on_vendor_propagated_odometry(
    const nav_msgs::msg::Odometry::ConstSharedPtr input)
  {process_on_vendor_propagated_odometry(input, std::nullopt);}

  void process_on_vendor_propagated_odometry(
    const nav_msgs::msg::Odometry::ConstSharedPtr input,
    const std::optional<DirectStamp>& metadata)
  {
    ++vendor_propagated_odometry_inputs_;
    // A propagated sample is superseded within a few milliseconds. Never let
    // it queue behind shared work or overtake a corrected callback that is
    // already waiting to refresh the watchdog anchor.
    if (!reliable_odometry_qos_ &&
      corrected_odometry_callbacks_pending_.load(std::memory_order_acquire) != 0U)
    {
      ++vendor_propagated_admission_drops_;
      return;
    }
    std::optional<nav_msgs::msg::Odometry> candidate;
    std::optional<rclcpp::Time> candidate_received;
    std::optional<sunhorizon_navigation_core::BodyVelocityEvidence> velocity_evidence;
    {
      std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
      if (reliable_odometry_qos_) {
        lock.lock();
      } else {
        if (!lock.try_lock() ||
          corrected_odometry_callbacks_pending_.load(std::memory_order_acquire) != 0U)
        {
          ++vendor_propagated_admission_drops_;
          return;
        }
      }
      const rclcpp::Time stamp(input->header.stamp);
      const rclcpp::Time received = receipt_time(metadata);
      const auto previous_seen=propagated_odometry_seen_,previous_valid=propagated_odometry_valid_;
      const auto previous_received=propagated_odometry_received_;
      const auto previous_error=propagated_odometry_error_;
      propagated_odometry_seen_ = true;
      propagated_odometry_received_ = received;
      propagated_odometry_valid_ = valid_odometry(*input);
      propagated_odometry_error_.clear();
      const auto freshness =
        sunhorizon_livo_bringup::evaluate_measurement_freshness(
          stamp.nanoseconds(), metadata ? now().nanoseconds() : received.nanoseconds(),
          maximum_input_age_sec_);
      if (propagated_odometry_valid_ && !freshness.accepted) {
        propagated_odometry_valid_ = false;
        propagated_odometry_error_ =
          "vendor_propagated_odometry_" + freshness.reason;
        ++vendor_propagated_odometry_stale_rejections_;
      }
      if(body_velocity_enabled_ && !propagated_odometry_valid_){
        body_velocity_.reset();body_velocity_reason_="body_velocity_invalid_original_pose";
      }
      if (propagated_odometry_valid_) {
        sunhorizon_livo_bringup::BodyVelocityGeometryStatus geometry;
        candidate = canonical_odometry(*input, true, velocity_evidence,&geometry);
        if(!candidate && geometry==sunhorizon_livo_bringup::BodyVelocityGeometryStatus::FuturePending){
          propagated_odometry_seen_=previous_seen;propagated_odometry_valid_=previous_valid;
          propagated_odometry_received_=previous_received;propagated_odometry_error_=previous_error;
          ++vendor_propagated_odometry_rejections_;
          return;
        }
        propagated_odometry_valid_ = candidate.has_value();
      }
      if (!propagated_odometry_valid_) {
        if (propagated_odometry_error_.empty()) {
          propagated_odometry_error_ =
            (body_velocity_enabled_ && !body_velocity_reason_.empty()) ? body_velocity_reason_ :
            "vendor_propagated_odometry_invalid_frame_or_pose";
        }
        candidate.reset();
      } else {
        const auto decision =
          propagation_watchdog_->evaluate_propagated(stamp.nanoseconds());
        if(metadata && decision.accepted)propagation_causal_audit_.propagated_accepted();
        propagated_odometry_valid_ = decision.accepted;
        if (!propagated_odometry_valid_) {
          propagated_odometry_error_ = decision.reason;
          if (decision.reason == "propagation_unanchored") {
            if(metadata){
              // Freeze both identities under the same lock as this watchdog
              // transition. One bounded event per recovery cycle, direct only.
              const auto event=propagation_causal_audit_.unanchored(
                audit_stamp(*metadata),decision.propagation_horizon_sec,
                propagation_watchdog_->corrected_stamp_nanoseconds());
              if(event)RCLCPP_ERROR(get_logger(),"LIVO_PROPAGATION_CAUSAL %s",event->text().c_str());
            }
            RCLCPP_ERROR(
              get_logger(),
              "LIVO propagation exceeded corrected horizon at %.6f s; "
              "candidate output is stopped until a fresh stability epoch passes",
              decision.propagation_horizon_sec);
          }
          candidate.reset();
        }
      }
      if (propagated_odometry_valid_ && candidate &&
        !all_inputs_healthy(metadata ? now() : propagated_odometry_received_))
      {
        candidate.reset();
      }
      if (!candidate) {
        ++vendor_propagated_odometry_rejections_;
      } else {
        candidate_received = received;
      }
    }
    if (candidate) {
      if (metadata) {
        auto owner = std::make_shared<const nav_msgs::msg::Odometry>(*candidate);
        auto delivered = *metadata;delivered.body_velocity=velocity_evidence;
        direct_sinks_.candidate(owner, delivered);
        if (direct_sinks_.publish_ros_candidates) direct_candidate_observer_->enqueue(
          std::make_shared<const DirectCandidate>(DirectCandidate{std::move(owner), delivered}));
      } else odometry_candidate_pub_->publish(*candidate);
      std::lock_guard<std::mutex> lock(mutex_);
      candidate_admission_diagnostics_.record_candidate_published(
        rclcpp::Time(candidate->header.stamp).nanoseconds(),
        candidate_received->nanoseconds());
    }
  }

  bool fresh(bool seen, const rclcpp::Time & received, const rclcpp::Time & current) const
  {
    if (!seen) {
      return false;
    }
    const double age = (current - received).seconds();
    return age >= 0.0 && age <= maximum_input_age_sec_;
  }

  bool all_inputs_healthy(const rclcpp::Time & current) const
  {
    const bool propagated_healthy = !use_propagated_odometry_ ||
      (propagation_watchdog_->ready() && propagated_odometry_valid_ &&
      fresh(propagated_odometry_seen_, propagated_odometry_received_, current));
    const bool visual_healthy = !visual_inputs_required_ ||
      (image_valid_ && camera_info_valid_ &&
      fresh(image_seen_, image_received_, current) &&
      fresh(camera_info_seen_, camera_info_received_, current));
    return points_valid_ && imu_valid_ && visual_healthy &&
      odometry_valid_ && fresh(points_seen_, points_received_, current) &&
      fresh(imu_seen_, imu_received_, current) &&
      fresh(odometry_seen_, odometry_received_, current) && propagated_healthy;
  }

  bool propagation_initializing() const
  {
    if (!use_propagated_odometry_ || propagation_watchdog_->latched() ||
      propagation_watchdog_->ready() ||
      propagation_watchdog_->recovering_from_unanchored())
    {
      return false;
    }
    if (!propagated_odometry_seen_ || propagated_odometry_valid_ ||
      propagated_odometry_error_.empty())
    {
      return true;
    }
    return propagated_odometry_error_ == "waiting_for_corrected_odometry" ||
           propagated_odometry_error_ == "waiting_for_stable_corrected_odometry" ||
           propagated_odometry_error_ == "waiting_for_stable_propagated_odometry" ||
           propagated_odometry_error_ == "waiting_for_recent_corrected_odometry";
  }

  std::string health_reason(const rclcpp::Time & current) const
  {
    if (!points_valid_ && !points_error_.empty()) {
      return points_error_;
    }
    if (imu_seen_ && !imu_valid_) {
      return "airy_imu_invalid";
    }
    if (visual_inputs_required_ && image_seen_ && !image_valid_) {
      return "chest_image_invalid";
    }
    if (visual_inputs_required_ && camera_info_seen_ && !camera_info_valid_) {
      return "chest_camera_info_invalid";
    }
    if (odometry_seen_ && !odometry_valid_) {
      return odometry_error_.empty() ?
        "vendor_odometry_invalid_frame_or_pose" : odometry_error_;
    }
    if (use_propagated_odometry_ &&
      propagation_watchdog_->recovering_from_unanchored())
    {
      return "propagation_unanchored_recovery";
    }
    if (propagation_initializing() && !propagation_watchdog_->tracking_established())
    {
      return odometry_seen_ ? "waiting_for_stable_corrected_odometry" :
             "waiting_for_corrected_odometry";
    }
    if (propagation_initializing())
    {
      return "waiting_for_propagated_odometry_after_corrected_lock";
    }
    if (use_propagated_odometry_ && propagated_odometry_seen_ &&
      !propagated_odometry_valid_)
    {
      return propagated_odometry_error_.empty() ?
        "vendor_propagated_odometry_invalid_frame_or_pose" :
        propagated_odometry_error_;
    }

    std::vector<std::string> missing;
    if (!points_seen_) {missing.emplace_back("points");}
    if (!imu_seen_) {missing.emplace_back("imu");}
    if (visual_inputs_required_ && !image_seen_) {missing.emplace_back("image");}
    if (visual_inputs_required_ && !camera_info_seen_) {missing.emplace_back("camera_info");}
    if (!odometry_seen_) {missing.emplace_back("vendor_odometry");}
    if (use_propagated_odometry_ && !propagated_odometry_seen_) {
      missing.emplace_back("vendor_propagated_odometry");
    }
    if (!missing.empty()) {
      std::ostringstream reason;
      reason << "waiting_for";
      for (const auto & name : missing) {reason << ':' << name;}
      return reason.str();
    }

    std::vector<std::string> stale;
    if (!fresh(points_seen_, points_received_, current)) {stale.emplace_back("points");}
    if (!fresh(imu_seen_, imu_received_, current)) {stale.emplace_back("imu");}
    if (visual_inputs_required_ && !fresh(image_seen_, image_received_, current)) {stale.emplace_back("image");}
    if (visual_inputs_required_ && !fresh(camera_info_seen_, camera_info_received_, current)) {
      stale.emplace_back("camera_info");
    }
    if (!fresh(odometry_seen_, odometry_received_, current)) {
      stale.emplace_back("vendor_odometry");
    }
    if (use_propagated_odometry_ &&
      !fresh(propagated_odometry_seen_, propagated_odometry_received_, current))
    {
      stale.emplace_back("vendor_propagated_odometry");
    }
    if (!stale.empty()) {
      std::ostringstream reason;
      reason << "stale";
      for (const auto & name : stale) {reason << ':' << name;}
      return reason.str();
    }
    return "ok";
  }

  void publish_status()
  {
    LocalizationStatus output;
    sunhorizon_livo_bringup::with_locked_time(
      mutex_, [this]() {return now();}, [&](const rclcpp::Time & current) {
      const bool healthy = all_inputs_healthy(current);
      const std::string reason = health_reason(current);
      output.header.stamp = current;
      output.header.frame_id = last_odometry_ ? last_odometry_->header.frame_id : std::string{};
      output.backend = "livo_odometry";
      output.measurement_accepted = healthy;
      output.score = 0.0;
      output.iterations = 0U;
      output.position_covariance = last_odometry_ ? std::max({
        last_odometry_->pose.covariance[0], last_odometry_->pose.covariance[7],
        last_odometry_->pose.covariance[14]}) : 0.0;
      output.yaw_covariance = last_odometry_ ? last_odometry_->pose.covariance[35] : 0.0;
      output.measurement_age_sec = last_vendor_stamp_ ?
        std::max(0.0, (current - *last_vendor_stamp_).seconds()) : 0.0;
      output.reason = mode_ + ':' + reason;
      if (healthy) {
        output.state = LocalizationStatus::STATE_HEALTHY;
      } else if (propagation_initializing())
      {
        output.state = LocalizationStatus::STATE_INITIALIZING;
      } else if ((points_seen_ && !points_valid_) || (imu_seen_ && !imu_valid_) ||
        (visual_inputs_required_ && ((image_seen_ && !image_valid_) ||
        (camera_info_seen_ && !camera_info_valid_))) ||
        (odometry_seen_ && !odometry_valid_) ||
        (use_propagated_odometry_ && propagated_odometry_seen_ &&
        !propagated_odometry_valid_))
      {
        output.state = LocalizationStatus::STATE_LOST;
      } else if (!points_seen_ || !imu_seen_ ||
        (visual_inputs_required_ && (!image_seen_ || !camera_info_seen_)) ||
        !odometry_seen_ || (use_propagated_odometry_ && !propagated_odometry_seen_))
      {
        output.state = LocalizationStatus::STATE_INITIALIZING;
      } else {
        output.state = LocalizationStatus::STATE_LOST;
      }
      });
    if (direct_adapter_) {
      auto owner = std::make_shared<const LocalizationStatus>(std::move(output));
      if (direct_sinks_.status) direct_sinks_.status(owner);
      if (direct_sinks_.publish_ros_candidates || direct_sinks_.publish_ros_status_compatibility)
        direct_status_observer_->enqueue(std::move(owner));
    } else odometry_status_pub_->publish(output);
  }

  void publish_transport_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray output;
    output.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "sunhorizon_livo_adapter_transport";
    status.hardware_id = "Robot A LIVO adapter";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = std::string("sensors=") +
      (reliable_sensor_qos_ ? "reliable" : "best_effort") +
      ",raw_image=" +
      (reliable_input_image_qos_ ? "reliable" : "best_effort") +
      ",vendor_image=" +
      (reliable_image_qos_ ? "reliable" : "best_effort") +
      ",odometry=" +
      (reliable_odometry_qos_ ? "reliable" : "best_effort") +
      ",visual_inputs=" + (visual_inputs_required_ ? "required" : "disabled_by_vendor_img_en");
    status.values.resize(33);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto admission = candidate_admission_diagnostics_.snapshot();
      status.values[0].key = "point_inputs";
      status.values[0].value = std::to_string(point_inputs_);
      status.values[1].key = "point_conversion_rejections";
      status.values[1].value = std::to_string(point_conversion_rejections_);
      status.values[2].key = "vendor_point_outputs";
      status.values[2].value = std::to_string(vendor_point_outputs_);
      status.values[3].key = "vendor_point_throttled";
      status.values[3].value = std::to_string(vendor_point_throttled_);
      status.values[4].key = "vendor_point_target_rate_hz";
      status.values[4].value = std::to_string(vendor_point_rate_hz_);
      status.values[5].key = "imu_inputs";
      status.values[5].value = std::to_string(imu_inputs_);
      status.values[6].key = "image_inputs";
      status.values[6].value = std::to_string(image_inputs_);
      status.values[7].key = "camera_info_inputs";
      status.values[7].value = std::to_string(camera_info_inputs_);
      status.values[8].key = "vendor_odometry_inputs";
      status.values[8].value = std::to_string(vendor_odometry_inputs_);
      status.values[9].key = "use_propagated_odometry";
      status.values[9].value = use_propagated_odometry_ ? "true" : "false";
      status.values[10].key = "vendor_propagated_odometry_inputs";
      status.values[10].value = std::to_string(
        vendor_propagated_odometry_inputs_.load(std::memory_order_relaxed));
      status.values[11].key = "vendor_propagated_odometry_rejections";
      status.values[11].value = std::to_string(vendor_propagated_odometry_rejections_);
      status.values[12].key = "propagation_unanchored_events";
      status.values[12].value = std::to_string(
        propagation_watchdog_->unanchored_event_count());
      status.values[13].key = "propagation_maximum_observed_horizon_sec";
      status.values[13].value = std::to_string(
        propagation_watchdog_->maximum_observed_horizon_sec());
      status.values[14].key = "vendor_image_outputs";
      status.values[14].value = std::to_string(vendor_image_outputs_);
      status.values[15].key = "vendor_image_target_rate_hz";
      status.values[15].value = std::to_string(vendor_image_rate_hz_);
      status.values[16].key = "vendor_odometry_stale_rejections";
      status.values[16].value = std::to_string(vendor_odometry_stale_rejections_);
      status.values[17].key = "vendor_propagated_odometry_stale_rejections";
      status.values[17].value = std::to_string(
        vendor_propagated_odometry_stale_rejections_);
      status.values[18].key = "maximum_image_input_gap_sec";
      status.values[18].value = std::to_string(maximum_image_input_gap_sec_);
      status.values[19].key = "maximum_vendor_image_gap_sec";
      status.values[19].value = std::to_string(maximum_vendor_image_gap_sec_);
      status.values[20].key = "vendor_propagated_admission_drops";
      status.values[20].value = std::to_string(
        vendor_propagated_admission_drops_.load(std::memory_order_relaxed));
      status.values[21].key = "vendor_odometry_freshness_stale_rejections";
      status.values[21].value = std::to_string(
        admission.freshness_stale_rejections);
      status.values[22].key = "vendor_odometry_freshness_future_rejections";
      status.values[22].value = std::to_string(
        admission.freshness_future_rejections);
      status.values[23].key = "vendor_odometry_freshness_invalid_rejections";
      status.values[23].value = std::to_string(
        admission.freshness_invalid_rejections);
      status.values[24].key =
        "vendor_odometry_current_consecutive_freshness_rejections";
      status.values[24].value = std::to_string(
        admission.current_consecutive_freshness_rejections);
      status.values[25].key =
        "vendor_odometry_maximum_consecutive_freshness_rejections";
      status.values[25].value = std::to_string(
        admission.maximum_consecutive_freshness_rejections);
      status.values[26].key = "vendor_odometry_current_freshness_reason";
      status.values[26].value = admission.current_freshness_reason;
      status.values[27].key = "all_inputs_healthy_suppressions";
      status.values[27].value = std::to_string(admission.aggregate_suppressions);
      status.values[28].key = "all_inputs_healthy_current_suppression_reason";
      status.values[28].value = admission.current_aggregate_suppression_reason;
      status.values[29].key = "odometry_candidate_published";
      status.values[29].value = std::to_string(admission.candidate_published);
      status.values[30].key = "odometry_candidate_maximum_sensor_stamp_gap_sec";
      status.values[30].value = std::to_string(
        admission.maximum_candidate_sensor_stamp_gap_sec);
      status.values[31].key = "odometry_candidate_maximum_receipt_gap_sec";
      status.values[31].value = std::to_string(
        admission.maximum_candidate_receipt_gap_sec);
      status.values[32].key = "reliable_odometry_qos";
      status.values[32].value = reliable_odometry_qos_ ? "true" : "false";
    }
    output.status.push_back(std::move(status));
    if (direct_adapter_) direct_diagnostics_observer_->enqueue(
      std::make_shared<const diagnostic_msgs::msg::DiagnosticArray>(std::move(output)));
    else adapter_diagnostics_pub_->publish(output);
  }

  std::mutex mutex_;
  std::string mode_;
  std::string expected_vendor_frame_;
  std::string expected_vendor_child_frame_;
  std::string output_odometry_frame_;
  std::string output_base_frame_;
  tf2::Transform vendor_child_to_base_{tf2::Transform::getIdentity()};
  bool body_velocity_enabled_{false};
  sunhorizon_livo_bringup::CausalBodyVelocity body_velocity_;
  sunhorizon_livo_bringup::BodyOdometryGauge body_odometry_gauge_;
  std::string body_velocity_reason_;
  int64_t body_reset_barrier_steady_ns_{0};
  double maximum_input_age_sec_{0.5};
  double vendor_point_rate_hz_{10.0};
  double vendor_image_rate_hz_{15.0};
  double maximum_image_input_gap_sec_{0.0};
  double maximum_vendor_image_gap_sec_{0.0};
  bool reliable_sensor_qos_{false};
  bool reliable_input_image_qos_{false};
  bool reliable_image_qos_{true};
  bool reliable_odometry_qos_{false};
  bool use_propagated_odometry_{false};
  uint64_t point_inputs_{0};
  uint64_t point_conversion_rejections_{0};
  uint64_t vendor_point_outputs_{0};
  uint64_t vendor_point_throttled_{0};
  uint64_t imu_inputs_{0};
  uint64_t image_inputs_{0};
  uint64_t vendor_image_outputs_{0};
  uint64_t camera_info_inputs_{0};
  uint64_t vendor_odometry_inputs_{0};
  uint64_t vendor_odometry_stale_rejections_{0};
  std::atomic<std::uint64_t> vendor_propagated_odometry_inputs_{0U};
  uint64_t vendor_propagated_odometry_stale_rejections_{0};
  uint64_t vendor_propagated_odometry_rejections_{0};
  std::atomic_uint corrected_odometry_callbacks_pending_{0U};
  std::atomic<std::uint64_t> vendor_propagated_admission_drops_{0U};
  sunhorizon_livo_bringup::CandidateAdmissionDiagnostics
    candidate_admission_diagnostics_;
  bool points_seen_{false};
  bool imu_seen_{false};
  bool image_seen_{false};
  bool camera_info_seen_{false};
  bool odometry_seen_{false};
  bool propagated_odometry_seen_{false};
  bool points_valid_{false};
  bool imu_valid_{false};
  bool image_valid_{false};
  bool camera_info_valid_{false};
  bool odometry_valid_{false};
  bool propagated_odometry_valid_{false};
  std::string points_error_;
  std::string odometry_error_;
  std::string propagated_odometry_error_;
  rclcpp::Time points_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time imu_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time image_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time camera_info_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time odometry_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time propagated_odometry_received_{0, 0, RCL_ROS_TIME};
  std::optional<sunhorizon_livo_bringup::PropagationWatchdog>
    propagation_watchdog_;
  sunhorizon_livo_bringup::PropagationCausalAudit propagation_causal_audit_;
  std::optional<sunhorizon_livo_bringup::TimestampRateGate>
    vendor_point_rate_gate_;
  std::optional<sunhorizon_livo_bringup::TimestampRateGate>
    vendor_image_rate_gate_;
  bool direct_adapter_{false};
  sunhorizon_livo_bringup::LivoDirectSinks direct_sinks_;
  const bool visual_inputs_required_;
  std::mutex direct_input_mutex_;
  std::atomic<bool> direct_observer_failed_{false};
  std::unique_ptr<CandidateObserver> direct_candidate_observer_;
  std::unique_ptr<StatusObserver> direct_status_observer_;
  std::unique_ptr<DiagnosticsObserver> direct_diagnostics_observer_;
  std::array<DirectStamp, 6> direct_last_input_;
  std::optional<std::int64_t> last_point_input_stamp_nanoseconds_;
  std::optional<std::int64_t> last_image_input_stamp_nanoseconds_;
  std::optional<std::int64_t> last_vendor_image_stamp_nanoseconds_;
  std::optional<rclcpp::Time> last_vendor_stamp_;
  std::optional<nav_msgs::msg::Odometry> last_odometry_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr vendor_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr vendor_image_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_candidate_pub_;
  rclcpp::Publisher<LocalizationStatus>::SharedPtr odometry_status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    adapter_diagnostics_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vendor_odometry_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    vendor_propagated_odometry_sub_;
  rclcpp::CallbackGroup::SharedPtr point_callback_group_;
  rclcpp::CallbackGroup::SharedPtr sensor_callback_group_;
  rclcpp::CallbackGroup::SharedPtr image_callback_group_;
  rclcpp::CallbackGroup::SharedPtr corrected_odometry_callback_group_;
  rclcpp::CallbackGroup::SharedPtr propagated_odometry_callback_group_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr transport_diagnostics_timer_;
};

}  // namespace

namespace sunhorizon_livo_bringup {
std::shared_ptr<LivoDirectAdapter> make_livo_direct_adapter(
    const rclcpp::NodeOptions& options, LivoDirectSinks sinks) {
  return std::make_shared<LivoAdapter>(options, true, std::move(sinks));
}
}  // namespace sunhorizon_livo_bringup

#ifndef SUNHORIZON_LIVO_ADAPTER_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LivoAdapter>();
  const auto node_base = node->get_node_base_interface();

  // Keep high-rate transport work away from pose/status delivery. A shared
  // MultiThreadedExecutor can eagerly queue old samples while its workers are
  // busy converting point clouds or publishing images. Corrected and propagated
  // odometry use independent single-thread executors with latest-only live QoS.
  // The callback admission guard gives a waiting corrected anchor priority over
  // disposable high-rate propagation without assuming cross-topic DDS ordering.
  rclcpp::executors::MultiThreadedExecutor transport_executor(
    rclcpp::ExecutorOptions(), 3U);
  transport_executor.add_callback_group(node->point_callback_group(), node_base);
  transport_executor.add_callback_group(node->sensor_callback_group(), node_base);
  transport_executor.add_callback_group(node->image_callback_group(), node_base);

  rclcpp::executors::SingleThreadedExecutor corrected_odometry_executor;
  corrected_odometry_executor.add_callback_group(
    node->corrected_odometry_callback_group(), node_base);
  rclcpp::executors::SingleThreadedExecutor propagated_odometry_executor;
  propagated_odometry_executor.add_callback_group(
    node->propagated_odometry_callback_group(), node_base);
  rclcpp::executors::SingleThreadedExecutor status_timer_executor;
  status_timer_executor.add_callback_group(node->timer_callback_group(), node_base);

  std::thread corrected_odometry_thread(
    [&corrected_odometry_executor]() {corrected_odometry_executor.spin();});
  std::thread propagated_odometry_thread(
    [&propagated_odometry_executor]() {propagated_odometry_executor.spin();});
  std::thread status_timer_thread(
    [&status_timer_executor]() {status_timer_executor.spin();});

  transport_executor.spin();
  corrected_odometry_executor.cancel();
  propagated_odometry_executor.cancel();
  status_timer_executor.cancel();
  corrected_odometry_thread.join();
  propagated_odometry_thread.join();
  status_timer_thread.join();
  rclcpp::shutdown();
  return 0;
}

#endif
