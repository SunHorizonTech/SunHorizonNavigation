#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <map>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <filesystem>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sunhorizon_navigation_interfaces/msg/localization_status.hpp>

#include "sunhorizon_navigation_core/latest_observer.hpp"
#include "sunhorizon_3d_localizer/base_z_gate.hpp"
#include "sunhorizon_3d_localizer/ndt_direct_runtime.hpp"
#include "sunhorizon_3d_localizer/detail/accepted_ndt_snapshot.hpp"
#include "sunhorizon_3d_localizer/detail/registration_map_file.hpp"
#include "sunhorizon_3d_localizer/direct_map_tracking_logic.hpp"
#include "sunhorizon_3d_localizer/local_map_target_logic.hpp"
#include "sunhorizon_3d_localizer/ndt_backend.hpp"
#include "sunhorizon_3d_localizer/ndt_fitness_failure_gate.hpp"
#include "sunhorizon_3d_localizer/ndt_freshness.hpp"
#include "sunhorizon_3d_localizer/ndt_match_audit.hpp"
#include "sunhorizon_3d_localizer/ndt_worker_control.hpp"
#include "sunhorizon_3d_localizer/ndt_prior_drain.hpp"
#include "sunhorizon_3d_localizer/ndt_match_rate_gate.hpp"
#include "sunhorizon_3d_localizer/ndt_prior_sync_logic.hpp"
#include "sunhorizon_3d_localizer/ndt_registration_quality.hpp"
#include "sunhorizon_3d_localizer/ndt_result_step_gate.hpp"
#include "sunhorizon_3d_localizer/ndt_startup_alignment_gate.hpp"
#include "sunhorizon_3d_localizer/raw_scan_admission_gate.hpp"
#include "sunhorizon_3d_localizer/detail/ndt_pending_clouds.hpp"
#include "sunhorizon_3d_localizer/scan_range_gate.hpp"
#include "sunhorizon_3d_localizer/source_map_bounds_filter.hpp"
#include "sunhorizon_3d_localizer/tracking_odom_logic.hpp"

namespace fs = std::filesystem;

namespace {

using LocalizationStatus = sunhorizon_navigation_interfaces::msg::LocalizationStatus;
using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using OdometryCandidate = nav_msgs::msg::Odometry;
using PoseCandidate = geometry_msgs::msg::PoseWithCovarianceStamped;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

double thread_cpu_ms() noexcept {
  timespec value{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value) != 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return static_cast<double>(value.tv_sec) * 1000.0 +
      static_cast<double>(value.tv_nsec) * 1e-6;
}

constexpr float kPi = 3.14159265358979323846F;

float yaw(const Eigen::Matrix3f& rotation) {
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

float roll(const Eigen::Matrix3f& rotation) {
  return std::atan2(rotation(2, 1), rotation(2, 2));
}

float pitch(const Eigen::Matrix3f& rotation) {
  return std::asin(std::clamp(-rotation(2, 0), -1.0F, 1.0F));
}

float wrap(float angle) {
  while (angle > kPi) angle -= 2.0F * kPi;
  while (angle < -kPi) angle += 2.0F * kPi;
  return angle;
}

bool finite_transform(const Eigen::Matrix4f& transform) {
  return transform.array().isFinite().all();
}

std::int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
      static_cast<std::int64_t>(stamp.nanosec);
}

Eigen::Matrix4f planarize(const Eigen::Matrix4f& transform) {
  Eigen::Matrix4f output = Eigen::Matrix4f::Identity();
  output.block<3, 3>(0, 0) = Eigen::AngleAxisf(
      yaw(transform.block<3, 3>(0, 0)), Eigen::Vector3f::UnitZ()).toRotationMatrix();
  output(0, 3) = transform(0, 3);
  output(1, 3) = transform(1, 3);
  return output;
}

sunhorizon_3d_localizer::NdtAuditPose audit_pose(
    const Eigen::Matrix4f& transform) {
  return {
      static_cast<double>(transform(0, 3)),
      static_cast<double>(transform(1, 3)),
      static_cast<double>(transform(2, 3)),
      static_cast<double>(yaw(transform.block<3, 3>(0, 0)))};
}

Eigen::Matrix4f pose_from_xyz_rpy(const std::vector<double>& values) {
  if (values.size() != 6U || !std::all_of(values.begin(), values.end(),
      [](double value) { return std::isfinite(value); })) {
    throw std::runtime_error("initial_source_pose_xyz_rpy must contain six finite values");
  }
  Eigen::Matrix4f output = Eigen::Matrix4f::Identity();
  const Eigen::AngleAxisf roll(static_cast<float>(values[3]), Eigen::Vector3f::UnitX());
  const Eigen::AngleAxisf pitch(static_cast<float>(values[4]), Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf heading(static_cast<float>(values[5]), Eigen::Vector3f::UnitZ());
  output.block<3, 3>(0, 0) = (heading * pitch * roll).toRotationMatrix();
  output.block<3, 1>(0, 3) = Eigen::Vector3f(
      static_cast<float>(values[0]), static_cast<float>(values[1]),
      static_cast<float>(values[2]));
  return output;
}

Eigen::Matrix4f pose_from_xyz_qxyzw(const std::vector<double>& values) {
  if (values.size() != 7U || !std::all_of(values.begin(), values.end(),
      [](double value) { return std::isfinite(value); })) {
    throw std::runtime_error("source_to_base_xyz_qxyzw must contain seven finite values");
  }
  Eigen::Quaternionf quaternion(
      static_cast<float>(values[6]), static_cast<float>(values[3]),
      static_cast<float>(values[4]), static_cast<float>(values[5]));
  if (quaternion.norm() < 1e-6F) {
    throw std::runtime_error("source_to_base quaternion is invalid");
  }
  quaternion.normalize();
  Eigen::Matrix4f output = Eigen::Matrix4f::Identity();
  output.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
  output.block<3, 1>(0, 3) = Eigen::Vector3f(
      static_cast<float>(values[0]), static_cast<float>(values[1]),
      static_cast<float>(values[2]));
  return output;
}

Eigen::Matrix4f basis_from_qxyzw(const std::vector<double>& values) {
  if (values.size() != 4U || !std::all_of(values.begin(), values.end(),
      [](double value) { return std::isfinite(value); })) {
    throw std::runtime_error("prior_basis_qxyzw must contain four finite values");
  }
  Eigen::Quaternionf quaternion(
      static_cast<float>(values[3]), static_cast<float>(values[0]),
      static_cast<float>(values[1]), static_cast<float>(values[2]));
  if (quaternion.norm() < 1e-6F) {
    throw std::runtime_error("prior_basis quaternion is invalid");
  }
  quaternion.normalize();
  Eigen::Matrix4f output = Eigen::Matrix4f::Identity();
  output.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
  return output;
}

PoseCandidate pose_message(
    const Eigen::Matrix4f& transform, const builtin_interfaces::msg::Time& stamp,
    const std::string& frame, double position_covariance, double yaw_covariance) {
  PoseCandidate output;
  output.header.stamp = stamp;
  output.header.frame_id = frame;
  output.pose.pose.position.x = transform(0, 3);
  output.pose.pose.position.y = transform(1, 3);
  output.pose.pose.position.z = transform(2, 3);
  Eigen::Quaternionf quaternion(transform.block<3, 3>(0, 0));
  quaternion.normalize();
  output.pose.pose.orientation.x = quaternion.x();
  output.pose.pose.orientation.y = quaternion.y();
  output.pose.pose.orientation.z = quaternion.z();
  output.pose.pose.orientation.w = quaternion.w();
  output.pose.covariance.fill(0.0);
  output.pose.covariance[0] = position_covariance;
  output.pose.covariance[7] = position_covariance;
  output.pose.covariance[14] = position_covariance;
  output.pose.covariance[21] = yaw_covariance;
  output.pose.covariance[28] = yaw_covariance;
  output.pose.covariance[35] = yaw_covariance;
  return output;
}

Eigen::Matrix4f transform_from_pose(const geometry_msgs::msg::Pose& pose) {
  const Eigen::Quaternionf quaternion(
      static_cast<float>(pose.orientation.w), static_cast<float>(pose.orientation.x),
      static_cast<float>(pose.orientation.y), static_cast<float>(pose.orientation.z));
  if (!std::isfinite(quaternion.norm()) || std::abs(quaternion.norm() - 1.0F) > 1e-3F) {
    throw std::runtime_error("manual_initial_pose_quaternion_invalid");
  }
  Eigen::Matrix4f output = Eigen::Matrix4f::Identity();
  output.block<3, 3>(0, 0) = quaternion.normalized().toRotationMatrix();
  output.block<3, 1>(0, 3) = Eigen::Vector3f(
      static_cast<float>(pose.position.x), static_cast<float>(pose.position.y),
      static_cast<float>(pose.position.z));
  if (!finite_transform(output)) throw std::runtime_error("manual_initial_pose_non_finite");
  return output;
}

struct PriorSample {
  std::int64_t stamp_ns{0};
  Eigen::Matrix4f odom_to_source{Eigen::Matrix4f::Identity()};
  bool continuity_valid{false};
  std::string reason;
};

struct PendingPointCloud {
  std::int64_t stamp_ns{0};
  sensor_msgs::msg::PointCloud2::ConstSharedPtr message;
};

class NdtLocalizer final : public rclcpp::Node,
    public sunhorizon_3d_localizer::NdtDirectRuntime {
 public:
  explicit NdtLocalizer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
      bool direct_runtime = false,
      sunhorizon_3d_localizer::NdtDirectSinks sinks = {})
      : Node("sunhorizon_ndt_localizer", options), direct_runtime_(direct_runtime),
        direct_sinks_(std::move(sinks)) {
    rcl_interfaces::msg::ParameterDescriptor worker_descriptor;
    worker_descriptor.read_only = true;
    const auto worker_nice = declare_parameter<int64_t>("direct_worker_nice", 0, worker_descriptor);
    sunhorizon_3d_localizer::detail::validate_direct_worker_nice(worker_nice);
    if (!direct_runtime_ && worker_nice != 0)
      throw std::invalid_argument("direct_worker_nice_requires_direct_runtime");
#if !defined(__linux__)
    if (worker_nice != 0) throw std::invalid_argument("direct_worker_nice_requires_linux");
#endif
    direct_worker_nice_ = static_cast<int>(worker_nice);
    map_path_ = declare_parameter("map_path", std::string{});
    direct_map_tracking_enabled_ = declare_parameter(
        "direct_map_tracking_enabled", false);
    input_topic_ = declare_parameter("input_points_topic", "/sunhorizon/airy/points");
    reliable_input_points_qos_ = declare_parameter(
        "reliable_input_points_qos", false);
    odometry_prior_topic_ = declare_parameter(
        "odometry_prior_topic", "/sunhorizon/glim/odom_prior");
    pose_topic_ = declare_parameter(
        "map_pose_candidate_topic", "/sunhorizon/localization/lidar/map_pose_candidate");
    odometry_topic_ = declare_parameter(
        "odom_candidate_topic", "/sunhorizon/localization/lidar/odom_candidate");
    tracking_odometry_topic_ = declare_parameter(
        "tracking_odom_candidate_topic",
        "/sunhorizon/localization/lidar/tracking_odom_candidate");
    status_topic_ = declare_parameter(
        "status_topic", "/sunhorizon/localization/lidar/status");
    diagnostics_topic_ = declare_parameter(
        "diagnostics_topic", "/sunhorizon/localization/lidar/ndt_diagnostics");
    backend_name_ = declare_parameter("backend_name", "lidar");
    const std::string ndt_backend = declare_parameter("ndt_backend", "pcl");
    const auto ndt_backend_defaults =
        sunhorizon_3d_localizer::resolve_ndt_backend_options(
            ndt_backend, std::nullopt, std::nullopt);
    const int ndt_threads = declare_parameter(
        "ndt_threads", ndt_backend_defaults.threads);
    const std::string ndt_neighborhood = declare_parameter(
        "ndt_neighborhood",
        sunhorizon_3d_localizer::to_string(ndt_backend_defaults.neighborhood));
    rcl_interfaces::msg::ParameterDescriptor compute_descriptor;
    compute_descriptor.read_only = true;
    const std::string ndt_compute_backend = declare_parameter(
        "ndt_compute_backend", "cpu", compute_descriptor);
    if (ndt_compute_backend != "cpu" && !direct_runtime_)
      throw std::runtime_error("NDT CUDA requires the dedicated direct worker");
    ndt_backend_options_ =
        sunhorizon_3d_localizer::resolve_ndt_backend_options(
            ndt_backend, ndt_threads, ndt_neighborhood, ndt_compute_backend);
    initial_pose_topic_ = declare_parameter("initial_pose_topic", "/initialpose");
    map_frame_ = declare_parameter("map_frame", "map");
    odom_frame_ = declare_parameter("odom_frame", "odom");
    expected_input_frame_ = declare_parameter("expected_input_frame", "airy_lidar");
    expected_prior_frame_ = declare_parameter("expected_prior_frame", "odom");
    expected_prior_child_frame_ = declare_parameter(
        "expected_prior_child_frame", "rslidar");
    base_footprint_frame_ = declare_parameter("base_footprint_frame", "base_footprint");
    initial_pose_accepted_ = declare_parameter("initial_pose_accepted", false);
    require_runtime_initial_pose_ = declare_parameter(
        "require_runtime_initial_pose", false);
    source_to_base_accepted_ = declare_parameter(
        "source_to_base_transform_accepted", false);
    prior_basis_accepted_ = declare_parameter("prior_basis_transform_accepted", false);
    canonical_base_prior_ = declare_parameter("canonical_base_prior", false);
    use_receipt_time_ = declare_parameter("use_receipt_time_for_output", false);
    const auto initial_source_pose = declare_parameter<std::vector<double>>(
        "initial_source_pose_xyz_rpy", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    const auto source_to_base = declare_parameter<std::vector<double>>(
        "source_to_base_xyz_qxyzw", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    const auto prior_basis = declare_parameter<std::vector<double>>(
        "prior_basis_qxyzw", {-0.5, 0.5, -0.5, 0.5});
    map_leaf_ = declare_parameter("map_leaf_m", 0.5);
    local_map_target_enabled_ = declare_parameter(
        "local_map_target_enabled", false);
    local_map_target_radius_ = declare_parameter(
        "local_map_target_radius_m", 20.0);
    local_map_target_z_half_extent_ = declare_parameter(
        "local_map_target_z_half_extent_m", 4.0);
    local_map_target_refresh_distance_ = declare_parameter(
        "local_map_target_refresh_distance_m", 5.0);
    minimum_local_map_target_points_ = declare_parameter(
        "minimum_local_map_target_points", 1000);
    scan_leaf_ = declare_parameter("scan_leaf_m", 0.5);
    resolution_ = declare_parameter("resolution_m", 2.0);
    maximum_iterations_ = declare_parameter("maximum_iterations", 30);
    const double legacy_minimum_range = declare_parameter("minimum_range_m", 0.5);
    const double legacy_maximum_range = declare_parameter("maximum_range_m", 30.0);
    minimum_scan_range_ = declare_parameter(
        "minimum_scan_range_m", legacy_minimum_range);
    maximum_scan_range_ = declare_parameter(
        "maximum_scan_range_m", legacy_maximum_range);
    scan_range_gate_ = sunhorizon_3d_localizer::ScanRangeGate(
        minimum_scan_range_, maximum_scan_range_);
    minimum_raw_scan_points_ = declare_parameter("minimum_raw_scan_points", 0);
    minimum_scan_points_ = declare_parameter("minimum_scan_points", 500);
    source_map_bounds_filter_enabled_ = declare_parameter(
        "source_map_bounds_filter_enabled", false);
    source_map_bounds_filter_limits_.margin_m = declare_parameter(
        "source_map_bounds_margin_m", 1.0);
    source_map_bounds_filter_limits_.minimum_retained_fraction = declare_parameter(
        "source_map_bounds_minimum_retained_fraction", 0.6);
    source_map_bounds_minimum_retained_points_ = declare_parameter(
        "source_map_bounds_minimum_retained_points", 500);
    if (source_map_bounds_minimum_retained_points_ > 0) {
      source_map_bounds_filter_limits_.minimum_retained_points =
          static_cast<std::size_t>(source_map_bounds_minimum_retained_points_);
    }
    maximum_fitness_ = declare_parameter("maximum_fitness", 5.5);
    extended_quality_diagnostics_enabled_ = declare_parameter(
        "extended_quality_diagnostics_enabled", true);
    diagnostic_inlier_max_distance_ = declare_parameter(
        "diagnostic_inlier_max_distance_m", 1.0);
    minimum_consecutive_fitness_failures_for_lost_ = declare_parameter(
        "minimum_consecutive_fitness_failures_for_lost", 2);
    maximum_initial_translation_ = declare_parameter(
        "maximum_initial_to_result_translation_m", 0.35);
    maximum_initial_yaw_ = declare_parameter(
        "maximum_initial_to_result_yaw_rad", 0.12);
    startup_alignment_enabled_ = declare_parameter(
        "startup_alignment_enabled", false);
    startup_alignment_limits_.maximum_seed_translation_m = declare_parameter(
        "startup_alignment_maximum_seed_translation_m", 1.0);
    startup_alignment_limits_.maximum_seed_yaw_rad = declare_parameter(
        "startup_alignment_maximum_seed_yaw_rad", 0.2617993877991494);
    startup_alignment_limits_.maximum_candidate_translation_spread_m =
        declare_parameter(
            "startup_alignment_maximum_candidate_translation_spread_m", 0.08);
    startup_alignment_limits_.maximum_candidate_yaw_spread_rad =
        declare_parameter(
            "startup_alignment_maximum_candidate_yaw_spread_rad",
            0.03490658503988659);
    startup_alignment_limits_.maximum_sample_gap_sec = declare_parameter(
        "startup_alignment_maximum_sample_gap_sec", 0.30);
    const int startup_alignment_required_samples = declare_parameter(
        "startup_alignment_required_consecutive_samples", 10);
    startup_alignment_limits_.required_consecutive_samples =
        startup_alignment_required_samples > 0 ?
        static_cast<std::size_t>(startup_alignment_required_samples) : 0U;
    result_step_limits_.maximum_gap_sec = declare_parameter(
        "maximum_result_step_gap_sec", 0.50);
    rcl_interfaces::msg::ParameterDescriptor result_step_descriptor;
    result_step_descriptor.read_only = true;
    attended_result_step_profile_ = declare_parameter(
        "attended_result_step_profile", false, result_step_descriptor);
    result_step_limits_.maximum_translation_step_m = declare_parameter(
        "maximum_result_translation_step_m", 0.05, result_step_descriptor);
    result_step_limits_.maximum_yaw_step_rad = declare_parameter(
        "maximum_result_yaw_step_rad", 0.03490658503988659);
    direct_map_tracking_limits_.maximum_translation_step_m = declare_parameter(
        "maximum_direct_translation_step_m", 0.30);
    direct_map_tracking_limits_.maximum_rotation_step_rad = declare_parameter(
        "maximum_direct_rotation_step_rad", 0.2617993877991494);
    direct_map_tracking_limits_.maximum_translation_rate_mps = declare_parameter(
        "maximum_direct_translation_rate_mps", 2.0);
    direct_map_tracking_limits_.maximum_rotation_rate_radps = declare_parameter(
        "maximum_direct_rotation_rate_radps", 2.0);
    base_z_reference_ = declare_parameter(
        "base_z_reference_m",
        source_to_base.size() > 2U ? source_to_base[2] : 0.0);
    maximum_base_z_deviation_ = declare_parameter(
        "maximum_base_z_deviation_m", 0.30);
    maximum_base_roll_pitch_ = declare_parameter(
        "maximum_base_roll_pitch_rad", 0.15);
    maximum_match_rate_hz_ = declare_parameter("maximum_match_rate_hz", 1.0e9);
    ndt_match_rate_gate_ =
        sunhorizon_3d_localizer::NdtMatchRateGate(maximum_match_rate_hz_);
    position_covariance_ = declare_parameter("position_covariance", 0.04);
    yaw_covariance_ = declare_parameter("yaw_covariance", 0.01);
    maximum_input_age_sec_ = declare_parameter("maximum_input_age_sec", 0.50);
    input_future_tolerance_sec_ = declare_parameter(
        "input_future_tolerance_sec", 0.01);
    accepted_result_timeout_sec_ = declare_parameter(
        "accepted_result_timeout_sec", 0.50);
    maximum_twist_sensor_dt_sec_ = declare_parameter(
        "maximum_twist_sensor_dt_sec", 0.50);
    prior_sync_tolerance_sec_ = declare_parameter("prior_sync_tolerance_sec", 0.001);
    prior_timeout_sec_ = declare_parameter("prior_timeout_sec", 0.25);
    maximum_prior_translation_step_ = declare_parameter(
        "maximum_prior_translation_step_m", 0.30);
    maximum_prior_rotation_step_ = declare_parameter(
        "maximum_prior_rotation_step_rad", 0.2617993877991494);
    maximum_correction_translation_step_ = declare_parameter(
        "maximum_map_to_odom_correction_translation_step_m", 0.30);
    maximum_correction_rotation_step_ = declare_parameter(
        "maximum_map_to_odom_correction_rotation_step_rad", 0.2617993877991494);
    prior_buffer_size_ = declare_parameter("prior_buffer_size", 32);

    const bool attended_deadlines = declare_parameter<bool>(
        "attended_doubled_deadlines", false, worker_descriptor);
    if (attended_deadlines && (!direct_runtime_ || !direct_map_tracking_enabled_ || backend_name_ != "livo"))
      throw std::invalid_argument("attended_deadlines_require_direct_livo_map_tracking");
    validate_parameters(attended_deadlines);
    prior_continuity_gate_ =
        std::make_unique<sunhorizon_3d_localizer::PriorContinuityGate>(
            sunhorizon_3d_localizer::PriorContinuityLimits{
                prior_timeout_sec_, maximum_prior_translation_step_,
                maximum_prior_rotation_step_});
    fitness_failure_gate_ = sunhorizon_3d_localizer::NdtFitnessFailureGate(
        static_cast<std::uint32_t>(
            minimum_consecutive_fitness_failures_for_lost_));
    map_to_source_ = pose_from_xyz_rpy(initial_source_pose);
    source_to_base_ = pose_from_xyz_qxyzw(source_to_base);
    if (!direct_map_tracking_enabled_ && !canonical_base_prior_) {
      prior_basis_ = basis_from_qxyzw(prior_basis);
    }
    initialized_ = initial_pose_accepted_ && !require_runtime_initial_pose_ &&
        source_to_base_accepted_ &&
        (direct_map_tracking_enabled_ || canonical_base_prior_ ||
        prior_basis_accepted_);
    if (direct_map_tracking_enabled_ && initialized_) {
      direct_map_tracking_state_.reset(
          map_to_source_.cast<double>(), source_to_base_.cast<double>());
    }
    ndt_ = std::make_unique<sunhorizon_3d_localizer::NdtRegistration>(
        ndt_backend_options_);
    const auto team_cpus = declare_parameter<std::vector<int64_t>>(
        "ndt_team_cpu_affinity", std::vector<int64_t>{}, worker_descriptor);
    std::vector<int> native_team_cpus;
    for (const auto cpu : team_cpus) {
      if (cpu < 0 || cpu > std::numeric_limits<int>::max())
        throw std::invalid_argument("ndt_team_cpu_affinity_cpu_out_of_range");
      native_team_cpus.push_back(static_cast<int>(cpu));
    }
    ndt_->set_team_cpu_affinity(native_team_cpus);
    load_map();

    const auto candidate_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    pose_pub_ = create_publisher<PoseCandidate>(pose_topic_, candidate_qos);
    odometry_pub_ = create_publisher<OdometryCandidate>(
        odometry_topic_, candidate_qos);
    if (direct_map_tracking_enabled_) {
      const auto tracking_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
      tracking_odometry_pub_ = create_publisher<OdometryCandidate>(
          tracking_odometry_topic_, tracking_qos);
    }
    status_pub_ = create_publisher<LocalizationStatus>(
        status_topic_, rclcpp::QoS(10).transient_local());
    diagnostics_pub_ = create_publisher<DiagnosticArray>(
        diagnostics_topic_, rclcpp::QoS(10));
    rclcpp::SensorDataQoS points_qos;
    points_qos.keep_last(1);
    if (reliable_input_points_qos_) {
      points_qos.reliable();
    } else {
      points_qos.best_effort();
    }
    if (!direct_runtime_) {
      points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          input_topic_, points_qos,
          std::bind(&NdtLocalizer::on_points, this, std::placeholders::_1));
    }
    if (!direct_runtime_ && !direct_map_tracking_enabled_) {
      odometry_prior_sub_ = create_subscription<OdometryCandidate>(
          odometry_prior_topic_, rclcpp::QoS(rclcpp::KeepLast(100)).reliable(),
          std::bind(&NdtLocalizer::on_odometry_prior, this, std::placeholders::_1));
    }
    initial_pose_sub_ = create_subscription<PoseCandidate>(
        initial_pose_topic_, rclcpp::QoS(10),
        std::bind(&NdtLocalizer::on_initial_pose, this, std::placeholders::_1));
    timer_ = create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&NdtLocalizer::on_timer, this));
    RCLCPP_INFO(
        get_logger(),
        "NDT registration backend=%s threads=%d neighborhood=%s "
        "ndt_omp_compiled=%s compute_backend=%s cuda_compiled=%s",
        sunhorizon_3d_localizer::to_string(ndt_backend_options_.backend).c_str(),
        ndt_backend_options_.threads,
        sunhorizon_3d_localizer::to_string(
            ndt_backend_options_.neighborhood).c_str(),
        sunhorizon_3d_localizer::ndt_omp_compiled() ? "true" : "false",
        ndt_->compute_backend().c_str(),
        sunhorizon_3d_localizer::ndt_cuda_compiled() ? "true" : "false");
    RCLCPP_INFO(
        get_logger(), "NDT extended quality diagnostics enabled=%s",
        extended_quality_diagnostics_enabled_ ? "true" : "false");
    RCLCPP_INFO(
        get_logger(),
        "NDT sensor-time match gate configured at %.6f Hz (%ld ns minimum interval)",
        maximum_match_rate_hz_,
        static_cast<long>(ndt_match_rate_gate_.minimum_match_interval_ns()));
    RCLCPP_INFO(
        get_logger(), "NDT source scan range gate configured at [%.3f, %.3f] m",
        minimum_scan_range_, maximum_scan_range_);
    RCLCPP_INFO(
        get_logger(),
        "NDT raw scan admission minimum_points=%d enabled=%s",
        minimum_raw_scan_points_,
        minimum_raw_scan_points_ > 0 ? "true" : "false");
    RCLCPP_INFO(
        get_logger(),
        "NDT source map-bounds filter enabled=%s margin=%.3f "
        "minimum_retained_fraction=%.3f minimum_retained_points=%d",
        source_map_bounds_filter_enabled_ ? "true" : "false",
        source_map_bounds_filter_limits_.margin_m,
        source_map_bounds_filter_limits_.minimum_retained_fraction,
        source_map_bounds_minimum_retained_points_);
    RCLCPP_INFO(
        get_logger(), "NDT input point QoS reliability=%s depth=1",
        reliable_input_points_qos_ ? "reliable" : "best_effort");
    RCLCPP_INFO(
        get_logger(),
        "NDT freshness input_age_limit=%.3f s input_future_tolerance=%.3f s "
        "accepted_result_timeout=%.3f s twist_sensor_dt_limit=%.3f s",
        maximum_input_age_sec_, input_future_tolerance_sec_,
        accepted_result_timeout_sec_, maximum_twist_sensor_dt_sec_);
    RCLCPP_INFO(
        get_logger(),
        "NDT correction-step admission applicable=%s maximum_gap=%.3f s "
        "maximum_translation=%.3f m maximum_yaw=%.6f rad",
        direct_map_tracking_enabled_ ? "false" : "true",
        result_step_limits_.maximum_gap_sec,
        result_step_limits_.maximum_translation_step_m,
        result_step_limits_.maximum_yaw_step_rad);
    RCLCPP_INFO(
        get_logger(),
        "NDT startup alignment enabled=%s maximum_seed_translation=%.3f m "
        "maximum_seed_yaw=%.6f rad required_samples=%zu "
        "maximum_candidate_spread=%.3f m/%.6f rad maximum_gap=%.3f s",
        startup_alignment_enabled_ ? "true" : "false",
        startup_alignment_limits_.maximum_seed_translation_m,
        startup_alignment_limits_.maximum_seed_yaw_rad,
        startup_alignment_limits_.required_consecutive_samples,
        startup_alignment_limits_.maximum_candidate_translation_spread_m,
        startup_alignment_limits_.maximum_candidate_yaw_spread_rad,
        startup_alignment_limits_.maximum_sample_gap_sec);
    RCLCPP_INFO(
        get_logger(),
        "NDT local map target enabled=%s full_points=%zu radius=%.3f "
        "z_half_extent=%.3f refresh_distance=%.3f minimum_points=%d",
        local_map_target_enabled_ ? "true" : "false", full_target_->size(),
        local_map_target_radius_, local_map_target_z_half_extent_,
        local_map_target_refresh_distance_, minimum_local_map_target_points_);
    if (!initialized_) {
      if (initial_pose_accepted_ && require_runtime_initial_pose_ &&
          source_to_base_accepted_ &&
          (direct_map_tracking_enabled_ || canonical_base_prior_ ||
          prior_basis_accepted_)) {
        last_reason_ = "waiting_for_manual_initial_pose";
      } else {
        last_reason_ = direct_map_tracking_enabled_ ?
            "initial_pose_or_source_to_base_not_accepted" :
            "initial_pose_source_to_base_or_prior_basis_not_accepted";
      }
    } else {
      last_reason_ = direct_map_tracking_enabled_ ?
          "waiting_for_direct_map_pointcloud" :
          "waiting_for_synchronized_pointcloud_and_prior";
    }
    if (direct_runtime_) {
      if (use_receipt_time_)
        throw std::runtime_error("direct NDT preserves source time; receipt-time replay is unsupported");
      if (prior_buffer_size_ > 4096)
        throw std::runtime_error("direct NDT prior history exceeds hard bound");
      const auto failure = [this](std::exception_ptr) {
        {std::lock_guard<std::mutex> queue_lock(direct_queue_mutex_); direct_failed_ = true;}
        std::lock_guard<std::recursive_mutex> core_lock(direct_core_mutex_);
        last_accepted_ = false;
        last_reason_ = "ndt_direct_observer_failed";
      };
      direct_result_observer_ = std::make_unique<ResultObserver>([this](auto snapshot) {
        const auto fresh = sunhorizon_3d_localizer::evaluate_ndt_input_age(
          snapshot->input.source_stamp_ns, get_clock()->now().nanoseconds(),
          maximum_input_age_sec_, input_future_tolerance_sec_);
        if (!fresh.accepted) return;
        odometry_pub_->publish(*snapshot->odometry);
        pose_pub_->publish(*snapshot->map_pose);
        if (snapshot->independent_tracking_evidence)
          tracking_odometry_pub_->publish(*snapshot->odometry);
      }, failure);
      direct_status_observer_ = std::make_unique<StatusObserver>([this](auto snapshot) {
        status_pub_->publish(*snapshot);
      }, failure);
      direct_diagnostics_observer_ = std::make_unique<DiagnosticsObserver>([this](auto snapshot) {
        diagnostics_pub_->publish(*snapshot);
      }, failure);
      direct_worker_ = std::thread([this]() {run_direct();});
    }
  }

  ~NdtLocalizer() override {
    stop();
    if (direct_worker_.joinable()) direct_worker_.join();
    direct_result_observer_.reset();
    direct_status_observer_.reset();
    direct_diagnostics_observer_.reset();
  }

  std::shared_ptr<rclcpp::Node> node() override {return shared_from_this();}
  std::string registration_map_sha256() const override {return registration_map_sha256_;}

  bool submit_points(sensor_msgs::msg::PointCloud2::ConstSharedPtr input,
      const sunhorizon_3d_localizer::NdtDirectStamp& stamp) override {
    const auto submitted_ns = direct_steady_ns();
    std::lock_guard<std::mutex> lock(direct_queue_mutex_);
    if (!direct_runtime_ || direct_stopping_ || direct_failed_) return false;
    if (!valid_direct_stamp(input ? stamp_nanoseconds(input->header.stamp) : 0, stamp) ||
        input->header.stamp.nanosec >= 1000000000U ||
        stamp.source_stamp_ns <= direct_last_cloud_ns_ ||
        stamp.sequence <= direct_last_cloud_sequence_ ||
        (!direct_cloud_identity_.empty() && stamp.source_identity != direct_cloud_identity_)) {
      direct_input_error_ = "ndt_direct_cloud_stamp_or_identity_invalid";
      direct_pending_clouds_.clear();
      direct_condition_.notify_one();
      return false;
    }
    direct_last_cloud_ns_ = stamp.source_stamp_ns;
    direct_last_cloud_sequence_ = stamp.sequence;
    direct_cloud_identity_ = stamp.source_identity;
    ++direct_counts_.clouds_submitted;
    sunhorizon_3d_localizer::NdtQueueTiming timing;
    timing.submit_steady_ns = submitted_ns;
    const std::size_t raw_points = static_cast<std::size_t>(input->width) * input->height;
    timing.admitted_steady_ns = direct_steady_ns();
    if (direct_pending_clouds_.push(DirectCloud{std::move(input), stamp, timing},
        stamp.source_stamp_ns, raw_points,
        static_cast<std::size_t>(minimum_raw_scan_points_))) {
      ++direct_counts_.clouds_overwritten;
    }
    direct_condition_.notify_one();
    return true;
  }

  bool submit_prior(OdometryCandidate::ConstSharedPtr input,
      const sunhorizon_3d_localizer::NdtDirectStamp& stamp) override {
    std::lock_guard<std::mutex> lock(direct_queue_mutex_);
    if (!direct_runtime_ || direct_map_tracking_enabled_ ||
        direct_stopping_ || direct_failed_) return false;
    if (!valid_direct_stamp(input ? stamp_nanoseconds(input->header.stamp) : 0, stamp) ||
        input->header.stamp.nanosec >= 1000000000U ||
        stamp.source_stamp_ns <= direct_last_prior_ns_ ||
        stamp.sequence <= direct_last_prior_sequence_ ||
        (!direct_prior_identity_.empty() && stamp.source_identity != direct_prior_identity_)) {
      direct_input_error_ = "ndt_direct_prior_stamp_or_identity_invalid";
      direct_condition_.notify_one();
      return false;
    }
    // Priors carry continuity evidence. Overflow is a visible input failure,
    // never a hidden removal followed by fresh-looking receipt timestamps.
    if (direct_pending_priors_.size() >= 100) {
      ++direct_counts_.priors_overwritten;
      direct_pending_priors_.clear();
      direct_input_error_ = "ndt_direct_prior_mailbox_overflow";
      direct_condition_.notify_one();
      return false;
    }
    direct_last_prior_ns_ = stamp.source_stamp_ns;
    direct_last_prior_sequence_ = stamp.sequence;
    direct_prior_identity_ = stamp.source_identity;
    ++direct_counts_.priors_submitted;
    direct_pending_priors_.push_back({std::move(input), stamp});
    direct_condition_.notify_one();
    return true;
  }

  sunhorizon_3d_localizer::NdtDirectCounters counters() const override {
    std::lock_guard<std::mutex> lock(direct_queue_mutex_);
    return direct_counts_;
  }

  void stop() override {
    {
      std::lock_guard<std::mutex> lock(direct_queue_mutex_);
      direct_stopping_ = true;
      direct_pending_clouds_.clear();
      direct_pending_priors_.clear();
    }
    direct_condition_.notify_one();
    if (direct_runtime_) {
      std::lock_guard<std::recursive_mutex> core_lock(direct_core_mutex_);
      last_accepted_ = false;
      last_reason_ = "ndt_direct_runtime_stopped";
    }
  }

 private:
  using DirectStamp = sunhorizon_3d_localizer::NdtDirectStamp;
  using ResultObserver = sunhorizon_navigation_core::LatestObserver<sunhorizon_3d_localizer::NdtDirectResult>;
  using StatusObserver = sunhorizon_navigation_core::LatestObserver<LocalizationStatus>;
  using DiagnosticsObserver = sunhorizon_navigation_core::LatestObserver<DiagnosticArray>;

  void publish_diagnostics(DiagnosticArray output) {
    if (direct_runtime_) {
      // Cumulative queue counts at publication, not per-frame gate outcomes.
      // Keep them on the existing bounded observer; no extra messages/worker.
      if (!output.status.empty()) {
        const auto queue = [this]() {
          std::lock_guard<std::mutex> lock(direct_queue_mutex_);
          return direct_pending_clouds_.counters();
        }();
        auto& values = output.status.front().values;
        values.push_back(diagnostic_value("direct_queue_complete_submitted", std::to_string(queue.complete_submitted)));
        values.push_back(diagnostic_value("direct_queue_incomplete_submitted", std::to_string(queue.incomplete_submitted)));
        values.push_back(diagnostic_value("direct_queue_complete_overwritten", std::to_string(queue.complete_overwritten)));
        values.push_back(diagnostic_value("direct_queue_incomplete_overwritten", std::to_string(queue.incomplete_overwritten)));
        values.push_back(diagnostic_value("direct_queue_complete_preserved", std::to_string(queue.complete_preserved)));
      }
      direct_diagnostics_observer_->enqueue(std::make_shared<const DiagnosticArray>(std::move(output)));
    } else diagnostics_pub_->publish(output);
  }
  struct DirectCloud {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message;
    DirectStamp stamp;
    sunhorizon_3d_localizer::NdtQueueTiming timing;
  };
  struct DirectMetadata {
    DirectStamp stamp;
    sunhorizon_3d_localizer::NdtQueueTiming timing;
  };
  struct DirectPrior {
    OdometryCandidate::ConstSharedPtr message;
    DirectStamp stamp;
  };

  static int64_t direct_steady_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  static bool valid_direct_stamp(int64_t message_stamp, const DirectStamp& stamp) {
    return message_stamp > 0 && message_stamp == stamp.source_stamp_ns &&
        stamp.receipt_ros_ns > 0 && stamp.receipt_steady_ns > 0 &&
        stamp.sequence > 0 && !stamp.source_identity.empty() && stamp.source_identity.size() <= 256;
  }

  void run_direct() {
    while (true) {
      std::optional<DirectCloud> cloud;
      std::deque<DirectPrior> priors;
      std::string error;
      {
        std::unique_lock<std::mutex> lock(direct_queue_mutex_);
        direct_condition_.wait(lock, [this]() {
          return direct_stopping_ || !direct_pending_clouds_.empty() ||
              !direct_pending_priors_.empty() || !direct_input_error_.empty();
        });
        if (direct_stopping_) return;
        cloud = direct_pending_clouds_.take();
        if (cloud) cloud->timing.dequeue_steady_ns = direct_steady_ns();
        priors.swap(direct_pending_priors_);
        error.swap(direct_input_error_);
        if (!error.empty()) direct_pending_clouds_.clear();
      }
      try {
        // Startup configuration is covered by the existing failed-owner catch.
        if (!direct_worker_state_) check_direct_worker();
        if (cloud) cloud->timing.core_lock_enter_steady_ns = direct_steady_ns();
        std::lock_guard<std::recursive_mutex> core_lock(direct_core_mutex_);
        if (cloud) cloud->timing.core_lock_acquired_steady_ns = direct_steady_ns();
        if (!error.empty()) {
          reject(error, get_clock()->now());
          continue;
        }
        for (const auto& prior : priors) {
          direct_prior_metadata_[prior.stamp.source_stamp_ns] = prior.stamp;
          while (direct_prior_metadata_.size() > static_cast<std::size_t>(prior_buffer_size_)+1)
            direct_prior_metadata_.erase(direct_prior_metadata_.begin());
          on_odometry_prior(prior.message);
        }
        if (cloud) {
          direct_metadata_[cloud->stamp.source_stamp_ns] = {cloud->stamp, cloud->timing};
          while (direct_metadata_.size() > static_cast<std::size_t>(prior_buffer_size_)+1)
            direct_metadata_.erase(direct_metadata_.begin());
          on_points(cloud->message);
        }
      } catch (const std::exception& failure) {
        {
          std::lock_guard<std::mutex> lock(direct_queue_mutex_);
          direct_failed_ = true;
          direct_pending_clouds_.clear();
          direct_pending_priors_.clear();
        }
        std::lock_guard<std::recursive_mutex> core_lock(direct_core_mutex_);
        last_accepted_ = false;
        last_reason_ = "ndt_direct_worker_failed";
        RCLCPP_ERROR(get_logger(), "NDT direct worker failed: %s", failure.what());
        // The retained timer independently emits LOST; avoid reentering the
        // failing sink here. No more clouds can be accepted by this owner.
        return;
      }
    }
  }

  void check_direct_worker() {
#if defined(__linux__)
    using namespace sunhorizon_3d_localizer::detail;
    DirectWorkerSyscalls calls;
    if (!direct_worker_state_)
      direct_worker_state_ = configure_direct_worker(calls, direct_worker_nice_);
    require_direct_worker_identity(read_direct_worker(calls,false), direct_worker_state_->tid);
#endif
  }

  void validate_parameters(bool attended_deadlines) const {
    const fs::path map(map_path_);
    if (!map.is_absolute() || map.filename() != "map.pcd" ||
        !fs::is_regular_file(map) || fs::is_symlink(map)) {
      throw std::runtime_error("map_path must be an absolute regular map.pcd file");
    }
    if (input_topic_.empty() || pose_topic_.empty() || odometry_topic_.empty() ||
        status_topic_.empty() || diagnostics_topic_.empty() ||
        status_topic_ == diagnostics_topic_ || backend_name_.empty() ||
        initial_pose_topic_.empty() ||
        map_frame_.empty() || odom_frame_.empty() || expected_input_frame_.empty() ||
        base_footprint_frame_.empty() ||
        map_leaf_ <= 0.0 || scan_leaf_ <= 0.0 ||
        resolution_ <= 0.0 || maximum_iterations_ < 1 ||
        !std::isfinite(minimum_scan_range_) || minimum_scan_range_ <= 0.0 ||
        !std::isfinite(maximum_scan_range_) ||
        maximum_scan_range_ <= minimum_scan_range_ ||
        minimum_raw_scan_points_ < 0 || minimum_scan_points_ < 100 ||
        !std::isfinite(source_map_bounds_filter_limits_.margin_m) ||
        source_map_bounds_filter_limits_.margin_m < 0.0 ||
        !std::isfinite(
            source_map_bounds_filter_limits_.minimum_retained_fraction) ||
        source_map_bounds_filter_limits_.minimum_retained_fraction <= 0.0 ||
        source_map_bounds_filter_limits_.minimum_retained_fraction > 1.0 ||
        source_map_bounds_minimum_retained_points_ < 100 ||
        maximum_fitness_ <= 0.0 ||
        !std::isfinite(diagnostic_inlier_max_distance_) ||
        diagnostic_inlier_max_distance_ <= 0.0 ||
        minimum_consecutive_fitness_failures_for_lost_ < 1 ||
        maximum_initial_translation_ <= 0.0 ||
        maximum_initial_yaw_ <= 0.0 ||
        !std::isfinite(result_step_limits_.maximum_gap_sec) ||
        result_step_limits_.maximum_gap_sec <= 0.0 ||
        result_step_limits_.maximum_gap_sec > (attended_deadlines ? 1.00 : 0.80) ||
        !sunhorizon_3d_localizer::valid_result_translation_step_limit(
            result_step_limits_.maximum_translation_step_m,
            attended_result_step_profile_) ||
        !std::isfinite(result_step_limits_.maximum_yaw_step_rad) ||
        result_step_limits_.maximum_yaw_step_rad <= 0.0 ||
        result_step_limits_.maximum_yaw_step_rad > 0.03490658503988659 ||
        !std::isfinite(base_z_reference_) ||
        !std::isfinite(maximum_base_z_deviation_) ||
        maximum_base_z_deviation_ <= 0.0 ||
        maximum_base_z_deviation_ > 0.50 ||
        maximum_base_roll_pitch_ <= 0.0 ||
        !std::isfinite(maximum_match_rate_hz_) || maximum_match_rate_hz_ <= 0.0 ||
        position_covariance_ < 0.0 || yaw_covariance_ < 0.0 ||
        !std::isfinite(maximum_input_age_sec_) || maximum_input_age_sec_ <= 0.0 ||
        maximum_input_age_sec_ > (attended_deadlines ? 1.30 : 0.65) ||
        !std::isfinite(input_future_tolerance_sec_) ||
        input_future_tolerance_sec_ < 0.0 ||
        input_future_tolerance_sec_ > 0.01 ||
        !std::isfinite(accepted_result_timeout_sec_) ||
        accepted_result_timeout_sec_ <= 0.0 ||
        accepted_result_timeout_sec_ > (attended_deadlines ? 1.60 : 0.80) ||
        !std::isfinite(maximum_twist_sensor_dt_sec_) ||
        maximum_twist_sensor_dt_sec_ <= 0.0 ||
        maximum_twist_sensor_dt_sec_ > 0.65 ||
        prior_sync_tolerance_sec_ < 0.0 ||
        prior_timeout_sec_ <= 0.0 || maximum_prior_translation_step_ <= 0.0 ||
        maximum_prior_rotation_step_ <= 0.0 ||
        maximum_correction_translation_step_ <= 0.0 ||
        maximum_correction_rotation_step_ <= 0.0 || prior_buffer_size_ < 2) {
      throw std::runtime_error("invalid NDT localizer parameters");
    }
    if (attended_result_step_profile_ &&
        (backend_name_ != "livo" || !canonical_base_prior_ ||
         direct_map_tracking_enabled_ || !require_runtime_initial_pose_)) {
      throw std::runtime_error(
          "attended_result_step_profile_requires_explicit_seeded_livo_canonical_prior");
    }
    if (startup_alignment_enabled_ &&
        (!require_runtime_initial_pose_ || direct_map_tracking_enabled_ ||
         !std::isfinite(startup_alignment_limits_.maximum_seed_translation_m) ||
         startup_alignment_limits_.maximum_seed_translation_m <=
             maximum_initial_translation_ ||
         startup_alignment_limits_.maximum_seed_translation_m > 2.0 ||
         !std::isfinite(startup_alignment_limits_.maximum_seed_yaw_rad) ||
         startup_alignment_limits_.maximum_seed_yaw_rad <= maximum_initial_yaw_ ||
         startup_alignment_limits_.maximum_seed_yaw_rad > 0.7853981633974483 ||
         !std::isfinite(
             startup_alignment_limits_.maximum_candidate_translation_spread_m) ||
         startup_alignment_limits_.maximum_candidate_translation_spread_m <= 0.0 ||
         startup_alignment_limits_.maximum_candidate_translation_spread_m >
             maximum_initial_translation_ ||
         !std::isfinite(
             startup_alignment_limits_.maximum_candidate_yaw_spread_rad) ||
         startup_alignment_limits_.maximum_candidate_yaw_spread_rad <= 0.0 ||
         startup_alignment_limits_.maximum_candidate_yaw_spread_rad >
             maximum_initial_yaw_ ||
         !std::isfinite(startup_alignment_limits_.maximum_sample_gap_sec) ||
         startup_alignment_limits_.maximum_sample_gap_sec <= 0.0 ||
         startup_alignment_limits_.maximum_sample_gap_sec > 0.50 ||
         startup_alignment_limits_.required_consecutive_samples < 2U ||
         startup_alignment_limits_.required_consecutive_samples > 100U)) {
      throw std::runtime_error("invalid NDT startup alignment parameters");
    }
    if (source_map_bounds_filter_enabled_ &&
        source_map_bounds_minimum_retained_points_ < minimum_scan_points_) {
      throw std::runtime_error(
          "source map-bounds minimum retained points must cover minimum_scan_points");
    }
    if (local_map_target_enabled_ &&
        (!require_runtime_initial_pose_ ||
        !std::isfinite(local_map_target_radius_) ||
        local_map_target_radius_ <= 0.0 ||
        !std::isfinite(local_map_target_z_half_extent_) ||
        local_map_target_z_half_extent_ <= 0.0 ||
        !std::isfinite(local_map_target_refresh_distance_) ||
        local_map_target_refresh_distance_ <= 0.0 ||
        local_map_target_refresh_distance_ >= local_map_target_radius_ ||
        minimum_local_map_target_points_ < 100)) {
      throw std::runtime_error("invalid NDT local map target parameters");
    }
    if (map_frame_ == odom_frame_ || map_frame_ == base_footprint_frame_ ||
        odom_frame_ == base_footprint_frame_) {
      throw std::runtime_error(
          "map odom and base_footprint frame names must be distinct");
    }
    if (direct_map_tracking_enabled_) {
      if (tracking_odometry_topic_.empty() ||
          tracking_odometry_topic_ == odometry_topic_ ||
          tracking_odometry_topic_ == pose_topic_ ||
          tracking_odometry_topic_ == status_topic_) {
        throw std::runtime_error(
            "direct tracking odometry topic must be nonempty and distinct");
      }
      if (!std::isfinite(direct_map_tracking_limits_.maximum_translation_step_m) ||
          direct_map_tracking_limits_.maximum_translation_step_m <= 0.0 ||
          !std::isfinite(direct_map_tracking_limits_.maximum_rotation_step_rad) ||
          direct_map_tracking_limits_.maximum_rotation_step_rad <= 0.0 ||
          !std::isfinite(direct_map_tracking_limits_.maximum_translation_rate_mps) ||
          direct_map_tracking_limits_.maximum_translation_rate_mps <= 0.0 ||
          !std::isfinite(direct_map_tracking_limits_.maximum_rotation_rate_radps) ||
          direct_map_tracking_limits_.maximum_rotation_rate_radps <= 0.0) {
        throw std::runtime_error("invalid direct map tracking continuity limits");
      }
    } else if (odometry_prior_topic_.empty() || expected_prior_frame_.empty() ||
        expected_prior_child_frame_.empty() || expected_prior_frame_ != odom_frame_) {
      throw std::runtime_error(
          "legacy tracking requires an odometry prior in the odom frame");
    }
    if (canonical_base_prior_ &&
        (direct_map_tracking_enabled_ ||
        expected_prior_child_frame_ != base_footprint_frame_)) {
      throw std::runtime_error(
          "canonical base prior requires prior mode and base_footprint child frame");
    }
  }

  void load_map() {
    const sunhorizon_3d_localizer::detail::RegistrationMapFile actual(map_path_);
    auto raw = PointCloud::Ptr(new PointCloud());
    if (pcl::io::loadPCDFile(map_path_, *raw) != 0 || raw->empty()) {
      throw std::runtime_error("ndt_map_load_failed");
    }
    actual.verify(map_path_);
    registration_map_sha256_=actual.sha256;
    full_target_ = PointCloud::Ptr(new PointCloud());
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(map_leaf_, map_leaf_, map_leaf_);
    voxel.setInputCloud(raw);
    voxel.filter(*full_target_);
    if (full_target_->empty()) throw std::runtime_error("ndt_map_filter_empty");
    full_target_registration_quality_index_.set_target(full_target_);
    ndt_->set_transformation_epsilon(0.005);
    ndt_->set_step_size(0.1);
    ndt_->set_resolution(static_cast<float>(resolution_));
    ndt_->set_maximum_iterations(maximum_iterations_);
    if (!local_map_target_enabled_) {
      target_ = full_target_;
      active_target_bounds_ =
          sunhorizon_3d_localizer::compute_map_axis_aligned_bounds(*target_);
      ndt_->set_input_target(target_);
      ndt_registration_quality_index_.set_target(target_);
    }
  }

  sunhorizon_3d_localizer::LocalMapTargetCenter local_map_target_center(
      const Eigen::Matrix4f& map_to_source) const {
    return {
        static_cast<double>(map_to_source(0, 3)),
        static_cast<double>(map_to_source(1, 3)),
        static_cast<double>(map_to_source(2, 3))};
  }

  bool rebuild_local_map_target(
      const Eigen::Matrix4f& map_to_source, std::string& failure_reason) {
    const auto center = local_map_target_center(map_to_source);
    const sunhorizon_3d_localizer::LocalMapTargetCropLimits limits{
        local_map_target_radius_, local_map_target_z_half_extent_};
    auto cropped = PointCloud::Ptr(new PointCloud());
    cropped->reserve(std::min(
        full_target_->size(),
        static_cast<std::size_t>(minimum_local_map_target_points_) * 4U));
    for (const auto& point : full_target_->points) {
      if (sunhorizon_3d_localizer::local_map_target_contains(
          point.x, point.y, point.z, center, limits)) {
        cropped->push_back(point);
      }
    }
    if (!sunhorizon_3d_localizer::local_map_target_has_minimum_points(
        cropped->size(), minimum_local_map_target_points_)) {
      // Every align path checks local_map_target_ready_ first, so the NDT
      // object cannot reuse its previous target after a failed rebuild.
      target_.reset();
      active_target_bounds_.reset();
      ndt_registration_quality_index_.clear();
      local_map_target_ready_ = false;
      failure_reason = "ndt_local_map_target_too_sparse";
      RCLCPP_ERROR(
          get_logger(),
          "NDT local map target rebuild failed full_points=%zu cropped_points=%zu "
          "center=(%.3f,%.3f,%.3f) radius=%.3f z_half_extent=%.3f "
          "minimum_points=%d",
          full_target_->size(), cropped->size(), center.x, center.y, center.z,
          local_map_target_radius_, local_map_target_z_half_extent_,
          minimum_local_map_target_points_);
      return false;
    }
    target_ = std::move(cropped);
    active_target_bounds_ =
        sunhorizon_3d_localizer::compute_map_axis_aligned_bounds(*target_);
    ndt_->set_input_target(target_);
    ndt_registration_quality_index_.set_target(target_);
    local_map_target_center_ = center;
    local_map_target_ready_ = true;
    failure_reason.clear();
    RCLCPP_INFO(
        get_logger(),
        "NDT local map target rebuilt full_points=%zu cropped_points=%zu "
        "center=(%.3f,%.3f,%.3f) radius=%.3f z_half_extent=%.3f",
        full_target_->size(), target_->size(), center.x, center.y, center.z,
        local_map_target_radius_, local_map_target_z_half_extent_);
    return true;
  }

  PointCloud::Ptr convert_and_filter(const sensor_msgs::msg::PointCloud2& input) const {
    auto raw = PointCloud::Ptr(new PointCloud());
    raw->reserve(static_cast<std::size_t>(input.width) * input.height);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(input, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(input, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(input, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!scan_range_gate_.accepts(*x, *y, *z)) continue;
        raw->push_back(pcl::PointXYZ(*x, *y, *z));
      }
    } catch (const std::runtime_error&) {
      return PointCloud::Ptr(new PointCloud());
    }
    auto filtered = PointCloud::Ptr(new PointCloud());
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(scan_leaf_, scan_leaf_, scan_leaf_);
    voxel.setInputCloud(raw);
    voxel.filter(*filtered);
    return filtered;
  }

  void on_initial_pose(const PoseCandidate::SharedPtr input) {
    std::lock_guard<std::recursive_mutex> direct_core_lock(direct_core_mutex_);
    if (!initial_pose_accepted_ || !source_to_base_accepted_ ||
        (!direct_map_tracking_enabled_ && !canonical_base_prior_ &&
        !prior_basis_accepted_)) {
      last_reason_ = "manual_initial_pose_geometry_not_accepted";
      return;
    }
    if (input->header.frame_id != map_frame_) {
      last_reason_ = "manual_initial_pose_frame_mismatch";
      return;
    }
    try {
      // RViz's 2D Pose Estimate always supplies z=0.  The accepted fixed-map
      // contract, however, binds base_footprint to base_z_reference_.  Restore
      // that reference before converting the operator's base pose into the
      // NDT source frame; otherwise every initialization starts one sensor
      // mounting height away from the map and can exceed the runtime gate.
      const Eigen::Matrix4f map_to_base =
          sunhorizon_3d_localizer::planarize_with_z_reference(
              transform_from_pose(input->pose.pose).cast<double>(),
              base_z_reference_).cast<float>();
      map_to_source_ = map_to_base * source_to_base_.inverse();
      if (direct_map_tracking_enabled_) {
        direct_map_tracking_state_.reset(
            map_to_source_.cast<double>(), source_to_base_.cast<double>());
      }
      raw_prediction_correction_initialized_ = false;
      accepted_canonical_correction_valid_ = false;
      prior_continuity_gate_->reset();
      pointcloud_buffer_.clear();
      prior_buffer_.clear();
      last_pointcloud_stamp_ns_ = 0;
      ndt_match_rate_gate_.reset();
      fitness_failure_gate_.reset();
      result_step_gate_.reset();
      startup_alignment_gate_.reset();
      startup_alignment_available_ = true;
      match_throttled_count_ = 0;
      previous_odometry_valid_ = false;
      last_measurement_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      last_accepted_result_receipt_time_ =
          rclcpp::Time(0, 0, RCL_ROS_TIME);
      initialized_ = true;
      last_accepted_ = false;
      if (local_map_target_enabled_) {
        std::string failure_reason;
        if (!rebuild_local_map_target(map_to_source_, failure_reason)) {
          const auto failure_time = get_clock()->now();
          last_measurement_stamp_ = failure_time;
          last_reason_ = failure_reason;
          publish_status(failure_time, failure_time);
          return;
        }
      }
      last_reason_ = "manual_initial_pose_accepted_waiting_for_pointcloud";
    } catch (const std::exception& error) {
      last_reason_ = error.what();
    }
  }

  void reject(const std::string& reason, const rclcpp::Time& stamp) {
    last_accepted_ = false;
    last_reason_ = reason;
    last_measurement_stamp_ = stamp;
    publish_status(stamp, stamp);
  }

  sunhorizon_3d_localizer::NdtFreshnessDecision
  last_accepted_result_freshness(const rclcpp::Time& now) const {
    if (!last_accepted_) {
      return {
          false, 0.0, "ndt_accepted_result_receipt_invalid"};
    }
    return sunhorizon_3d_localizer::evaluate_ndt_result_stream_age(
        last_accepted_result_receipt_time_.nanoseconds(), now.nanoseconds(),
        accepted_result_timeout_sec_);
  }

  void finish_ndt_match_audit(
      sunhorizon_3d_localizer::NdtMatchAudit audit,
      const std::string& gate_reason, bool accepted,
      DiagnosticArray* deferred_output = nullptr) {
    prior_sync_diagnostics_.record_match_invoked();
    prior_sync_diagnostics_.record_match_result(accepted);
    audit.gate_reason = gate_reason;
    audit.accepted = accepted;
    if (accepted) {
      // Accepted matches are represented at full rate on diagnostics_topic.
      // Keep their grep-friendly text available for opt-in debugging without
      // formatting or emitting a long INFO line for every normal frame.
      RCLCPP_DEBUG(
          get_logger(), "%s",
          sunhorizon_3d_localizer::format_ndt_match_audit(audit).c_str());
    } else {
      const std::string line =
          sunhorizon_3d_localizer::format_ndt_match_audit(audit);
      RCLCPP_WARN(get_logger(), "%s", line.c_str());
    }
    publish_match_diagnostics(
        audit, rclcpp::Time(audit.sensor_stamp_ns, RCL_ROS_TIME), deferred_output);
  }

  static diagnostic_msgs::msg::KeyValue diagnostic_value(
      const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue output;
    output.key = key;
    output.value = value;
    return output;
  }

  void publish_prior_sync_diagnostics(
      std::uint8_t level, const std::string& message,
      const rclcpp::Time& output_stamp) {
    const auto& snapshot = prior_sync_diagnostics_.snapshot();
    DiagnosticArray output;
    output.header.stamp = output_stamp;
    DiagnosticStatus status;
    status.level = level;
    status.name = "sunhorizon_ndt_localizer/prior_sync";
    status.hardware_id = backend_name_;
    status.message = message;
    status.values = {
        diagnostic_value("compute_backend", ndt_->compute_backend()),
        diagnostic_value("cuda_compiled", sunhorizon_3d_localizer::ndt_cuda_compiled() ? "true" : "false"),
        diagnostic_value("attended_result_step_profile",
            attended_result_step_profile_ ? "true" : "false"),
        diagnostic_value(
            "sync_no_match_count", std::to_string(snapshot.no_match_count)),
        diagnostic_value(
            "sync_invalid_prior_count",
            std::to_string(snapshot.invalid_prior_count)),
        diagnostic_value(
            "sync_epoch_break_count", std::to_string(snapshot.epoch_break_count)),
        diagnostic_value(
            "sync_recovered_gap_count",
            std::to_string(snapshot.recovered_gap_count)),
        diagnostic_value(
            "sync_raw_scan_skip_count",
            std::to_string(snapshot.raw_scan_skip_count)),
        diagnostic_value(
            "ndt_match_invoked_count",
            std::to_string(snapshot.match_invoked_count)),
        diagnostic_value(
            "ndt_match_accepted_count",
            std::to_string(snapshot.match_accepted_count)),
        diagnostic_value(
            "ndt_match_rejected_count",
            std::to_string(snapshot.match_rejected_count)),
        diagnostic_value(
            "sync_last_match_delta_ns",
            std::to_string(snapshot.last_match_delta_ns)),
        diagnostic_value(
            "sync_maximum_match_delta_ns",
            std::to_string(snapshot.maximum_match_delta_ns)),
        diagnostic_value(
            "pointcloud_buffer_size", std::to_string(pointcloud_buffer_.size())),
        diagnostic_value(
            "prior_buffer_size", std::to_string(prior_buffer_.size()))};
    output.status.push_back(std::move(status));
    publish_diagnostics(std::move(output));
  }

  void reset_prior_epoch(
      const std::string& reason, bool invalid_prior,
      const rclcpp::Time& output_stamp) {
    prior_continuity_gate_->reset();
    pointcloud_buffer_.clear();
    prior_buffer_.clear();
    last_pointcloud_stamp_ns_ = 0;
    raw_prediction_correction_initialized_ = false;
    accepted_canonical_correction_valid_ = false;
    previous_odometry_valid_ = false;
    ndt_match_rate_gate_.reset();
    fitness_failure_gate_.reset();
    if (invalid_prior) {
      prior_sync_diagnostics_.record_invalid_prior(true);
      publish_prior_sync_diagnostics(DiagnosticStatus::ERROR, reason, output_stamp);
    }
  }

  void publish_match_diagnostics(
      const sunhorizon_3d_localizer::NdtMatchAudit& audit,
      const rclcpp::Time& output_stamp, DiagnosticArray* deferred_output = nullptr) {
    DiagnosticArray output;
    output.header.stamp = output_stamp;
    DiagnosticStatus status;
    status.level = audit.accepted ? DiagnosticStatus::OK : DiagnosticStatus::ERROR;
    status.name = "sunhorizon_ndt_localizer/ndt_match";
    status.hardware_id = backend_name_;
    status.message = audit.gate_reason;
    status.values = {
        diagnostic_value("attended_result_step_profile",
            attended_result_step_profile_ ? "true" : "false"),
        diagnostic_value("registration_backend", audit.registration_backend),
        diagnostic_value("compute_backend", audit.compute_backend),
        diagnostic_value("accepted", audit.accepted ? "true" : "false"),
        diagnostic_value("converged", audit.converged ? "true" : "false"),
        diagnostic_value("gate_reason", audit.gate_reason),
        diagnostic_value("result_z_m", std::to_string(audit.result.z)),
        diagnostic_value("raw_map_to_base_z_m", std::to_string(audit.raw_map_to_base_z_m)),
        diagnostic_value("raw_map_to_base_roll_rad", std::to_string(audit.raw_map_to_base_roll_rad)),
        diagnostic_value("raw_map_to_base_pitch_rad", std::to_string(audit.raw_map_to_base_pitch_rad)),
        diagnostic_value("raw_map_to_base_x_m", std::to_string(audit.raw_map_to_base_x_m)),
        diagnostic_value("raw_map_to_base_y_m", std::to_string(audit.raw_map_to_base_y_m)),
        diagnostic_value("raw_map_to_base_yaw_rad", std::to_string(audit.raw_map_to_base_yaw_rad)),
        diagnostic_value(
            "correction_step_applicable",
            audit.correction_step_applicable ? "true" : "false"),
        diagnostic_value("correction_step_reason", audit.correction_step_reason),
        diagnostic_value(
            "correction_step_has_baseline",
            audit.correction_step_has_baseline ? "true" : "false"),
        diagnostic_value(
            "correction_step_establishes_baseline",
            audit.correction_step_establishes_baseline ? "true" : "false"),
        diagnostic_value(
            "correction_step_gap_recovered",
            audit.correction_step_gap_recovered ? "true" : "false"),
        diagnostic_value(
            "correction_step_dt_sec",
            std::to_string(audit.correction_step_dt_sec)),
        diagnostic_value(
            "correction_translation_step_m",
            std::to_string(audit.correction_translation_step_m)),
        diagnostic_value(
            "correction_yaw_step_rad",
            std::to_string(audit.correction_yaw_step_rad)),
        diagnostic_value(
            "maximum_correction_step_gap_sec",
            std::to_string(audit.maximum_correction_step_gap_sec)),
        diagnostic_value(
            "maximum_correction_translation_step_m",
            std::to_string(audit.maximum_correction_translation_step_m)),
        diagnostic_value(
            "maximum_correction_yaw_step_rad",
            std::to_string(audit.maximum_correction_yaw_step_rad)),
        diagnostic_value(
            "source_range_voxel_points",
            std::to_string(audit.source_range_voxel_points)),
        diagnostic_value(
            "source_bounds_filter_enabled",
            audit.source_bounds_filter_enabled ? "true" : "false"),
        diagnostic_value(
            "source_bounds_retained_points",
            std::to_string(audit.source_bounds_retained_points)),
        diagnostic_value(
            "source_bounds_rejected_points",
            std::to_string(audit.source_bounds_rejected_points)),
        diagnostic_value(
            "source_bounds_retained_fraction",
            std::to_string(audit.source_bounds_retained_fraction)),
        diagnostic_value(
            "alignment_source_fitness_m2",
            std::to_string(audit.alignment_source_fitness_m2)),
        diagnostic_value(
            "alignment_source_fitness_maximum_m2",
            std::to_string(maximum_fitness_)),
        diagnostic_value(
            "pre_bounds_all_point_fitness_m2",
            std::to_string(audit.pre_bounds_all_point_fitness_m2)),
        diagnostic_value(
            "extended_quality_diagnostics_enabled",
            audit.extended_quality_diagnostics_enabled ? "true" : "false"),
        diagnostic_value(
            "inlier_max_distance_m",
            std::to_string(audit.inlier_max_distance_m)),
        diagnostic_value(
            "diagnostic_target_points",
            std::to_string(audit.diagnostic_target_points)),
        diagnostic_value(
            "diagnostic_source_points",
            std::to_string(audit.diagnostic_source_points)),
        diagnostic_value("inlier_points", std::to_string(audit.inlier_points)),
        diagnostic_value(
            "source_inlier_fraction",
            std::to_string(audit.source_inlier_fraction)),
        diagnostic_value(
            "source_to_target_inlier_rmse_m",
            std::to_string(audit.source_to_target_inlier_rmse_m)),
        diagnostic_value(
            "pre_bounds_full_target_points",
            std::to_string(audit.pre_bounds_full_target_points)),
        diagnostic_value(
            "pre_bounds_source_points",
            std::to_string(audit.pre_bounds_source_points)),
        diagnostic_value(
            "pre_bounds_inlier_points",
            std::to_string(audit.pre_bounds_inlier_points)),
        diagnostic_value(
            "pre_bounds_source_inlier_fraction",
            std::to_string(audit.pre_bounds_source_inlier_fraction)),
        diagnostic_value(
            "pre_bounds_source_to_full_target_inlier_rmse_m",
            std::to_string(
                audit.pre_bounds_source_to_full_target_inlier_rmse_m)),
        diagnostic_value("iterations", std::to_string(audit.iterations)),
        diagnostic_value(
            "alignment_runtime_ms", std::to_string(audit.alignment_runtime_ms)),
        diagnostic_value(
            "alignment_thread_cpu_ms", std::to_string(audit.alignment_thread_cpu_ms)),
        diagnostic_value(
            "fitness_thread_cpu_ms", std::to_string(audit.fitness_thread_cpu_ms)),
        diagnostic_value(
            "post_alignment_runtime_ms",
            std::to_string(audit.post_alignment_runtime_ms)),
        diagnostic_value(
            "fitness_runtime_ms", std::to_string(audit.fitness_runtime_ms)),
        diagnostic_value(
            "alignment_quality_runtime_ms",
            std::to_string(audit.alignment_quality_runtime_ms)),
        diagnostic_value(
            "pre_bounds_quality_runtime_ms",
            std::to_string(audit.pre_bounds_quality_runtime_ms)),
        diagnostic_value(
            "end_to_end_runtime_ms",
            std::to_string(audit.end_to_end_runtime_ms)),
        diagnostic_value("owner_tid", std::to_string(audit.owner_tid)),
        diagnostic_value("schedstat_available", audit.schedstat_available ? "true" : "false"),
        diagnostic_value("schedstat_status", std::to_string(audit.schedstat_status)),
        diagnostic_value("schedstat_begin_steady_ns", std::to_string(audit.schedstat_begin_steady_ns)),
        diagnostic_value("schedstat_end_steady_ns", std::to_string(audit.schedstat_end_steady_ns)),
        diagnostic_value("schedstat_sampling_wall_ns", std::to_string(audit.schedstat_sampling_wall_ns)),
        diagnostic_value("schedstat_running_ns", std::to_string(audit.schedstat_running_ns)),
        diagnostic_value("schedstat_runqueue_wait_ns", std::to_string(audit.schedstat_runqueue_wait_ns)),
        diagnostic_value(
            "sync_no_match_count",
            std::to_string(prior_sync_diagnostics_.snapshot().no_match_count)),
        diagnostic_value(
            "sync_invalid_prior_count",
            std::to_string(
                prior_sync_diagnostics_.snapshot().invalid_prior_count)),
        diagnostic_value(
            "sync_epoch_break_count",
            std::to_string(prior_sync_diagnostics_.snapshot().epoch_break_count)),
        diagnostic_value(
            "sync_recovered_gap_count",
            std::to_string(
                prior_sync_diagnostics_.snapshot().recovered_gap_count)),
        diagnostic_value(
            "sync_raw_scan_skip_count",
            std::to_string(
                prior_sync_diagnostics_.snapshot().raw_scan_skip_count)),
        diagnostic_value(
            "ndt_match_invoked_count",
            std::to_string(
                prior_sync_diagnostics_.snapshot().match_invoked_count)),
        diagnostic_value(
            "ndt_match_accepted_count",
            std::to_string(
                prior_sync_diagnostics_.snapshot().match_accepted_count)),
        diagnostic_value(
            "ndt_match_rejected_count",
            std::to_string(
                prior_sync_diagnostics_.snapshot().match_rejected_count)),
        diagnostic_value(
            "sync_last_match_delta_ns",
            std::to_string(
                prior_sync_diagnostics_.snapshot().last_match_delta_ns)),
        diagnostic_value(
            "sync_maximum_match_delta_ns",
            std::to_string(
                prior_sync_diagnostics_.snapshot().maximum_match_delta_ns))};
    const auto diagnostic_now=direct_steady_ns();
    status.values.push_back(diagnostic_value("process_enter_steady_ns",std::to_string(audit.process_enter_steady_ns)));
    status.values.push_back(diagnostic_value("audit_complete_steady_ns",std::to_string(diagnostic_now)));
    const auto add_time = [&status](const char* key, int64_t value) {
      status.values.push_back(diagnostic_value(key, std::to_string(value)));
    };
    add_time("submit_steady_ns", audit.queue_timing.submit_steady_ns);
    add_time("admitted_steady_ns", audit.queue_timing.admitted_steady_ns);
    add_time("dequeue_steady_ns", audit.queue_timing.dequeue_steady_ns);
    add_time("core_lock_enter_steady_ns", audit.queue_timing.core_lock_enter_steady_ns);
    add_time("core_lock_acquired_steady_ns", audit.queue_timing.core_lock_acquired_steady_ns);
    add_time("preprocess_done_steady_ns", audit.preprocess_done_steady_ns);
    add_time("gates_done_steady_ns", audit.gates_done_steady_ns);
    const auto add_scheduling = [&status](const std::string& prefix,
        const sunhorizon_3d_localizer::NdtStageSchedulingAudit& stage) {
      const auto add = [&status, &prefix](const char* key, auto value) {
        status.values.push_back(diagnostic_value(prefix + key, std::to_string(value)));
      };
      status.values.push_back(diagnostic_value(prefix + "available", stage.available ? "true" : "false"));
      add("status", stage.status); add("owner_tid", stage.owner_tid);
      add("begin_steady_ns", stage.begin_steady_ns); add("end_steady_ns", stage.end_steady_ns);
      add("sampling_wall_ns", stage.sampling_wall_ns); add("running_ns", stage.running_ns);
      add("runqueue_wait_ns", stage.runqueue_wait_ns);
    };
    add_scheduling("preprocess_schedstat_", audit.preprocess_scheduling);
    add_scheduling("gates_schedstat_", audit.gates_scheduling);
    const auto original=direct_metadata_.find(audit.sensor_stamp_ns);
    if(original!=direct_metadata_.end()){
      status.values.push_back(diagnostic_value("input_sequence",std::to_string(original->second.stamp.sequence)));
      status.values.push_back(diagnostic_value("input_receipt_ros_ns",std::to_string(original->second.stamp.receipt_ros_ns)));
      status.values.push_back(diagnostic_value("input_receipt_steady_ns",std::to_string(original->second.stamp.receipt_steady_ns)));
    }
    output.status.push_back(std::move(status));
    if (deferred_output) *deferred_output = std::move(output);
    else publish_diagnostics(std::move(output));
  }

  bool should_schedule_ndt_match(
      std::int64_t sensor_stamp_ns, const rclcpp::Time& sensor_stamp) {
    const auto decision = ndt_match_rate_gate_.evaluate(sensor_stamp_ns);
    if (decision == sunhorizon_3d_localizer::NdtMatchRateDecision::kMatch) {
      return true;
    }
    if (decision == sunhorizon_3d_localizer::NdtMatchRateDecision::kThrottled) {
      ++match_throttled_count_;
      if (match_throttled_count_ == 1 || match_throttled_count_ % 100 == 0) {
        RCLCPP_INFO(
            get_logger(),
            "NDT sensor-time match gate skipped frame at %ld ns; total skipped=%lu",
            static_cast<long>(sensor_stamp_ns),
            static_cast<unsigned long>(match_throttled_count_));
      }
      return false;
    }
    reject(
        decision == sunhorizon_3d_localizer::NdtMatchRateDecision::kTimestampRegressed ?
            "ndt_match_gate_sensor_stamp_regressed" :
            "ndt_match_gate_sensor_stamp_invalid",
        sensor_stamp);
    return false;
  }

  void report_incomplete_raw_scan_skip(
      const sunhorizon_3d_localizer::RawScanAdmissionDecision& decision,
      const rclcpp::Time& stamp) {
    ++incomplete_raw_scan_skipped_count_;
    prior_sync_diagnostics_.record_raw_scan_skip();
    if (incomplete_raw_scan_skipped_count_ == 1U ||
        incomplete_raw_scan_skipped_count_ % 100U == 0U) {
      RCLCPP_WARN(
          get_logger(),
          "Skipping incomplete raw scan points=%zu minimum=%zu total_skipped=%lu; "
          "last accepted localization is unchanged",
          decision.raw_points, decision.minimum_raw_points,
          static_cast<unsigned long>(incomplete_raw_scan_skipped_count_));
    }

    DiagnosticArray output;
    output.header.stamp = stamp;
    DiagnosticStatus status;
    status.level = DiagnosticStatus::WARN;
    status.name = "sunhorizon_ndt_localizer/raw_scan_admission";
    status.hardware_id = backend_name_;
    status.message = decision.reason;
    status.values = {
        diagnostic_value("raw_scan_points", std::to_string(decision.raw_points)),
        diagnostic_value(
            "minimum_raw_scan_points",
            std::to_string(decision.minimum_raw_points)),
        diagnostic_value(
            "incomplete_raw_scan_skipped_count",
            std::to_string(incomplete_raw_scan_skipped_count_)),
        diagnostic_value(
            "sync_raw_scan_skip_count",
            std::to_string(
                prior_sync_diagnostics_.snapshot().raw_scan_skip_count)),
        diagnostic_value(
            "last_accepted_before_skip", last_accepted_ ? "true" : "false"),
        diagnostic_value("localization_state_mutated", "false")};
    if (direct_runtime_) {
      const auto metadata = direct_metadata_.find(stamp.nanoseconds());
      if (metadata != direct_metadata_.end()) {
        const auto& original = metadata->second.stamp;
        status.values.push_back(diagnostic_value("input_source_stamp_ns", std::to_string(original.source_stamp_ns)));
        status.values.push_back(diagnostic_value("input_sequence", std::to_string(original.sequence)));
        status.values.push_back(diagnostic_value("input_source_identity", original.source_identity));
        status.values.push_back(diagnostic_value("input_receipt_ros_ns", std::to_string(original.receipt_ros_ns)));
        status.values.push_back(diagnostic_value("input_receipt_steady_ns", std::to_string(original.receipt_steady_ns)));
      }
    }
    output.status.push_back(std::move(status));
    publish_diagnostics(std::move(output));
  }

  void on_points(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input) {
    std::lock_guard<std::recursive_mutex> direct_core_lock(direct_core_mutex_);
    const rclcpp::Time stamp(input->header.stamp);
    if (!initialized_) {
      if (initial_pose_accepted_ && require_runtime_initial_pose_ &&
          source_to_base_accepted_ &&
          (direct_map_tracking_enabled_ || canonical_base_prior_ ||
          prior_basis_accepted_)) {
        last_accepted_ = false;
        last_reason_ = "waiting_for_manual_initial_pose";
        publish_status(stamp, stamp);
        return;
      }
      reject(direct_map_tracking_enabled_ ?
          "initial_pose_or_source_to_base_not_accepted" :
          "initial_pose_source_to_base_or_prior_basis_not_accepted", stamp);
      return;
    }
    if (input->header.frame_id != expected_input_frame_) {
      reject("pointcloud_frame_mismatch", stamp);
      return;
    }
    const std::int64_t stamp_ns = stamp_nanoseconds(input->header.stamp);
    if (stamp_ns <= 0 || stamp_ns <= last_pointcloud_stamp_ns_) {
      reject("pointcloud_stamp_not_strictly_increasing", stamp);
      return;
    }
    last_pointcloud_stamp_ns_ = stamp_ns;
    const std::size_t raw_scan_points =
        static_cast<std::size_t>(input->width) * input->height;
    const auto raw_scan_admission =
        sunhorizon_3d_localizer::evaluate_raw_scan_admission(
            raw_scan_points,
            static_cast<std::size_t>(minimum_raw_scan_points_));
    if (!raw_scan_admission.accepted) {
      report_incomplete_raw_scan_skip(raw_scan_admission, stamp);
      return;
    }
    if (local_map_target_enabled_ && !local_map_target_ready_) {
      reject("ndt_local_map_target_unavailable", stamp);
      return;
    }
    if (direct_map_tracking_enabled_) {
      if (!should_schedule_ndt_match(stamp_ns, stamp)) return;
      process_pointcloud(input, std::nullopt);
      return;
    }
    pointcloud_buffer_.push_back({stamp_ns, input});
    bool pointcloud_superseded = false;
    while (pointcloud_buffer_.size() > static_cast<std::size_t>(prior_buffer_size_)) {
      pointcloud_buffer_.pop_front();
      pointcloud_superseded = true;
    }
    if (pointcloud_superseded) {
      publish_prior_sync_diagnostics(
          DiagnosticStatus::WARN,
          "pointcloud_superseded_waiting_for_odometry_prior", stamp);
    }
    if (drain_pending_priors()) try_process_synchronized_input();
  }

  bool drain_pending_priors() {
    if (!odometry_prior_sub_) return true;
    return sunhorizon_3d_localizer::drain_prior_samples<OdometryCandidate>(
        [this](OdometryCandidate& sample) {
          rclcpp::MessageInfo info;
          return odometry_prior_sub_->take(sample, info);
        },
        [this](const OdometryCandidate& sample) {
          return cache_odometry_prior(std::make_shared<OdometryCandidate>(sample));
        }, 100);
  }

  void on_odometry_prior(const OdometryCandidate::ConstSharedPtr input) {
    std::lock_guard<std::recursive_mutex> direct_core_lock(direct_core_mutex_);
    if (!cache_odometry_prior(input) || !drain_pending_priors()) return;
    try_process_synchronized_input();
  }

  bool cache_odometry_prior(const OdometryCandidate::ConstSharedPtr input) {
    if (direct_map_tracking_enabled_) return false;
    const rclcpp::Time stamp(input->header.stamp);
    const std::int64_t stamp_ns = stamp_nanoseconds(input->header.stamp);
    if (input->header.frame_id != expected_prior_frame_ ||
        input->child_frame_id != expected_prior_child_frame_) {
      reset_prior_epoch("odometry_prior_frame_mismatch", true, stamp);
      reject("odometry_prior_frame_mismatch", stamp);
      return false;
    }

    PriorSample sample;
    sample.stamp_ns = stamp_ns;
    try {
      Eigen::Matrix4f raw_odom_to_source = transform_from_pose(input->pose.pose);
      if (expected_prior_child_frame_ == base_footprint_frame_) {
        // LIVO already publishes odom -> base_footprint.  NDT aligns an Airy
        // scan, so convert the prior back to the scan/source frame exactly
        // once before predicting the map alignment.
        raw_odom_to_source *= source_to_base_.inverse();
      }
      sample.odom_to_source =
          prior_basis_ * raw_odom_to_source * prior_basis_.inverse();
    } catch (const std::exception&) {
      reset_prior_epoch("odometry_prior_pose_invalid", true, stamp);
      reject("odometry_prior_pose_invalid", stamp);
      return false;
    }
    if (!finite_transform(sample.odom_to_source)) {
      reset_prior_epoch("odometry_prior_pose_non_finite", true, stamp);
      reject("odometry_prior_pose_non_finite", stamp);
      return false;
    }

    const auto continuity = prior_continuity_gate_->evaluate(
        stamp_ns, sample.odom_to_source.cast<double>());
    // Keep the gate's original odometry_prior_translation_jump or
    // odometry_prior_rotation_jump reason; neither may degrade into a later
    // time-gap diagnosis after the epoch is re-anchored.
    if (!continuity.matchable) {
      pointcloud_buffer_.clear();
      prior_buffer_.clear();
      last_pointcloud_stamp_ns_ = 0;
      if (continuity.epoch_break) {
        raw_prediction_correction_initialized_ = false;
        accepted_canonical_correction_valid_ = false;
        previous_odometry_valid_ = false;
        ndt_match_rate_gate_.reset();
        fitness_failure_gate_.reset();
        prior_sync_diagnostics_.record_invalid_prior(true);
        publish_prior_sync_diagnostics(
            DiagnosticStatus::ERROR, continuity.reason, stamp);
        reject(continuity.reason, stamp);
      } else {
        last_reason_ = continuity.reason;
        publish_prior_sync_diagnostics(
            DiagnosticStatus::WARN, continuity.reason, stamp);
      }
      return false;
    }
    sample.continuity_valid = true;
    sample.reason = "ok";
    if (continuity.epoch_break) {
      // A time-only gap with bounded spatial motion starts a new matching
      // epoch without discarding the last accepted map correction. Drop old
      // priors, but keep point clouds: timestamp matching below can select
      // only a scan within the configured synchronization tolerance.
      prior_buffer_.clear();
      prior_sync_diagnostics_.record_recovered_gap();
      publish_prior_sync_diagnostics(
          DiagnosticStatus::WARN, continuity.reason, stamp);
    }
    prior_buffer_.push_back(sample);
    while (prior_buffer_.size() > static_cast<std::size_t>(prior_buffer_size_)) {
      prior_buffer_.pop_front();
    }
    return true;
  }

  std::optional<PriorSample> matching_prior(
      std::int64_t point_stamp_ns, std::int64_t& matched_delta_ns) {
    if (prior_buffer_.empty()) return std::nullopt;
    const std::int64_t tolerance_ns = static_cast<std::int64_t>(
        std::llround(prior_sync_tolerance_sec_ * 1e9));
    std::vector<sunhorizon_3d_localizer::PriorSyncCandidate> candidates;
    candidates.reserve(prior_buffer_.size());
    for (const auto& sample : prior_buffer_) {
      candidates.push_back({sample.stamp_ns, sample.continuity_valid});
    }
    const auto decision =
        sunhorizon_3d_localizer::select_nearest_matchable_prior(
            point_stamp_ns, candidates, tolerance_ns);
    if (!decision.matched) return std::nullopt;
    matched_delta_ns = decision.delta_ns;
    return prior_buffer_[decision.index];
  }

  void try_process_synchronized_input() {
    if (pointcloud_buffer_.empty()) return;
    if (prior_buffer_.empty()) {
      prior_sync_diagnostics_.record_no_match();
      const auto stamp = rclcpp::Time(pointcloud_buffer_.back().message->header.stamp);
      publish_prior_sync_diagnostics(
          DiagnosticStatus::WARN,
          "waiting_for_synchronized_odometry_prior", stamp);
      return;
    }
    auto selected = pointcloud_buffer_.end();
    std::optional<PriorSample> prior;
    std::int64_t matched_delta_ns = 0;
    for (auto iterator = pointcloud_buffer_.end(); iterator != pointcloud_buffer_.begin();) {
      --iterator;
      std::int64_t candidate_delta_ns = 0;
      const auto candidate = matching_prior(iterator->stamp_ns, candidate_delta_ns);
      if (candidate) {
        selected = iterator;
        prior = candidate;
        matched_delta_ns = candidate_delta_ns;
        break;
      }
    }
    if (selected == pointcloud_buffer_.end() || !prior) {
      prior_sync_diagnostics_.record_no_match();
      const auto stamp = rclcpp::Time(pointcloud_buffer_.back().message->header.stamp);
      publish_prior_sync_diagnostics(
          DiagnosticStatus::WARN,
          "waiting_for_synchronized_odometry_prior", stamp);
      return;
    }
    const auto points = selected->message;
    const std::int64_t selected_stamp_ns = selected->stamp_ns;
    pointcloud_buffer_.erase(pointcloud_buffer_.begin(), std::next(selected));
    const std::int64_t tolerance_ns = static_cast<std::int64_t>(
        std::llround(prior_sync_tolerance_sec_ * 1e9));
    while (!prior_buffer_.empty() &&
        prior_buffer_.front().stamp_ns <= selected_stamp_ns + tolerance_ns) {
      prior_buffer_.pop_front();
    }
    prior_sync_diagnostics_.record_match(matched_delta_ns);
    const rclcpp::Time sensor_stamp(points->header.stamp);
    if (!should_schedule_ndt_match(selected_stamp_ns, sensor_stamp)) return;
    process_pointcloud(points, prior);
  }

  void process_pointcloud(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input,
      const std::optional<PriorSample>& prior) {
    sunhorizon_3d_localizer::NdtMatchAudit audit;
    audit.process_enter_steady_ns = direct_steady_ns();
    const auto timing_metadata = direct_metadata_.find(stamp_nanoseconds(input->header.stamp));
    if (timing_metadata != direct_metadata_.end()) audit.queue_timing = timing_metadata->second.timing;
    auto& scheduling = sunhorizon_3d_localizer::detail::current_ndt_schedstat();
    const auto preprocessing_before = direct_runtime_ ? scheduling.sample() :
        sunhorizon_3d_localizer::detail::NdtSchedstatSample{};
    if (direct_runtime_) check_direct_worker();
    const rclcpp::Time sensor_stamp(input->header.stamp);
    const std::size_t source_raw_points =
        static_cast<std::size_t>(input->width) * input->height;
    const auto range_voxel_source = convert_and_filter(*input);
    if (range_voxel_source->size() <
        static_cast<std::size_t>(minimum_scan_points_)) {
      const auto rejection_time = use_receipt_time_ ? get_clock()->now() : sensor_stamp;
      reject("pointcloud_too_sparse_or_invalid", rejection_time);
      return;
    }

    if (!direct_map_tracking_enabled_ && !prior) {
      reject("odometry_prior_missing_for_legacy_tracking", sensor_stamp);
      return;
    }
    if (!direct_map_tracking_enabled_ &&
        !raw_prediction_correction_initialized_) {
      raw_prediction_map_to_odom_ =
          map_to_source_ * prior->odom_to_source.inverse();
      raw_prediction_correction_initialized_ = true;
    }
    Eigen::Matrix4f initial;
    if (direct_map_tracking_enabled_) {
      initial = direct_map_tracking_state_.next_initial_guess().cast<float>();
    } else {
      initial = raw_prediction_map_to_odom_ * prior->odom_to_source;
    }

    const rclcpp::Time pre_alignment_output_time =
        use_receipt_time_ ? get_clock()->now() : sensor_stamp;
    audit.registration_backend =
        sunhorizon_3d_localizer::to_string(ndt_backend_options_.backend);
    audit.compute_backend = ndt_->compute_backend();
    audit.registration_threads =
        static_cast<std::uint32_t>(ndt_backend_options_.threads);
    audit.registration_neighborhood =
        sunhorizon_3d_localizer::to_string(ndt_backend_options_.neighborhood);
    audit.sensor_stamp_ns = stamp_nanoseconds(input->header.stamp);
    audit.source_raw_points = source_raw_points;
    audit.source_range_voxel_points = range_voxel_source->size();
    audit.source_bounds_filter_enabled = source_map_bounds_filter_enabled_;
    audit.initial = audit_pose(initial);
    audit.extended_quality_diagnostics_enabled =
        extended_quality_diagnostics_enabled_;
    audit.inlier_max_distance_m = diagnostic_inlier_max_distance_;
    audit.correction_step_applicable = !direct_map_tracking_enabled_;
    audit.correction_step_reason = direct_map_tracking_enabled_ ?
        "not_applicable_direct_map_tracking_has_independent_pose_continuity" :
        "not_evaluated";
    audit.correction_step_has_baseline =
        !direct_map_tracking_enabled_ && result_step_gate_.has_baseline();
    audit.maximum_correction_step_gap_sec = result_step_limits_.maximum_gap_sec;
    audit.maximum_correction_translation_step_m =
        result_step_limits_.maximum_translation_step_m;
    audit.maximum_correction_yaw_step_rad =
        result_step_limits_.maximum_yaw_step_rad;

    PointCloud::Ptr alignment_source = range_voxel_source;
    if (source_map_bounds_filter_enabled_) {
      if (!active_target_bounds_) {
        audit.source_bounds_rejected_points = range_voxel_source->size();
        audit.source_bounds_retained_fraction = 0.0;
        finish_ndt_match_audit(
            audit, "source_map_bounds_target_unavailable", false);
        fitness_failure_gate_.reset();
        reject("source_map_bounds_target_unavailable", pre_alignment_output_time);
        return;
      }
      try {
        const auto bounds_result =
            sunhorizon_3d_localizer::filter_source_by_map_bounds(
                *range_voxel_source, initial, *active_target_bounds_,
                source_map_bounds_filter_limits_);
        audit.source_bounds_retained_points = bounds_result.retained_points;
        audit.source_bounds_rejected_points = bounds_result.rejected_points;
        audit.source_bounds_retained_fraction = bounds_result.retained_fraction;
        audit.source_filtered_points = bounds_result.retained_points;
        if (!bounds_result.accepted) {
          finish_ndt_match_audit(audit, bounds_result.reason, false);
          fitness_failure_gate_.reset();
          reject(bounds_result.reason, pre_alignment_output_time);
          return;
        }
        alignment_source = bounds_result.retained;
      } catch (const std::exception& error) {
        const std::string reason = error.what();
        finish_ndt_match_audit(audit, reason, false);
        fitness_failure_gate_.reset();
        reject(reason, pre_alignment_output_time);
        return;
      }
    } else {
      audit.source_bounds_retained_points = range_voxel_source->size();
      audit.source_bounds_rejected_points = 0U;
      audit.source_bounds_retained_fraction = 1.0;
      audit.source_filtered_points = range_voxel_source->size();
    }

    ndt_->set_input_source(alignment_source);
    PointCloud aligned;
    audit.preprocess_done_steady_ns = direct_steady_ns();
    const auto scheduling_before = scheduling.sample();
    if (direct_runtime_)
      audit.preprocess_scheduling = sunhorizon_3d_localizer::detail::ndt_stage_scheduling_audit(
          preprocessing_before, scheduling_before);
    const auto started = std::chrono::steady_clock::now();
    // PCL-default compatibility contract: ndt_.align(aligned, initial)
    const double alignment_cpu_started = thread_cpu_ms();
    ndt_->align(aligned, initial);
    const double alignment_thread_cpu_ms = thread_cpu_ms() - alignment_cpu_started;
    const auto alignment_finished = std::chrono::steady_clock::now();
    const double alignment_runtime_ms =
        std::chrono::duration<double, std::milli>(
            alignment_finished - started).count();
    const Eigen::Matrix4f result = ndt_->final_transformation();
    const Eigen::Matrix4f delta = initial.inverse() * result;
    const double initial_translation = delta.block<3, 1>(0, 3).norm();
    const double initial_yaw = std::abs(wrap(yaw(delta.block<3, 3>(0, 0))));
    const auto fitness_started = std::chrono::steady_clock::now();
    const double fitness_cpu_started = thread_cpu_ms();
    const double fitness = ndt_->fitness_score();
    const double fitness_thread_cpu_ms = thread_cpu_ms() - fitness_cpu_started;
    const auto fitness_finished = std::chrono::steady_clock::now();
    const double score =
        std::isfinite(fitness) ? 1.0 / (1.0 + fitness) : 0.0;
    const auto iterations =
        static_cast<std::uint32_t>(ndt_->final_iteration_count());
    sunhorizon_3d_localizer::NdtRegistrationQuality registration_quality;
    sunhorizon_3d_localizer::NdtRegistrationQuality
        pre_bounds_registration_quality;
    double alignment_quality_runtime_ms = 0.0;
    double pre_bounds_quality_runtime_ms = 0.0;
    if (extended_quality_diagnostics_enabled_) {
      const auto alignment_quality_started = std::chrono::steady_clock::now();
      registration_quality = ndt_registration_quality_index_.compute(
          aligned, diagnostic_inlier_max_distance_);
      const auto alignment_quality_finished = std::chrono::steady_clock::now();
      alignment_quality_runtime_ms =
          std::chrono::duration<double, std::milli>(
              alignment_quality_finished - alignment_quality_started).count();
      const auto pre_bounds_quality_started = std::chrono::steady_clock::now();
      PointCloud pre_bounds_aligned;
      pcl::transformPointCloud(
          *range_voxel_source, pre_bounds_aligned, result);
      pre_bounds_registration_quality =
          full_target_registration_quality_index_.compute(
              pre_bounds_aligned, diagnostic_inlier_max_distance_);
      pre_bounds_quality_runtime_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() -
              pre_bounds_quality_started).count();
    }
    const auto post_alignment_finished = std::chrono::steady_clock::now();
    const auto scheduling_after = scheduling.sample();
    const auto scheduling_delta = sunhorizon_3d_localizer::detail::ndt_schedstat_delta(
        scheduling_before, scheduling_after);
    audit.owner_tid = scheduling_delta.tid;
    audit.schedstat_available = scheduling_delta.available;
    audit.schedstat_status = static_cast<int>(scheduling_delta.status);
    audit.schedstat_begin_steady_ns = scheduling_delta.begin_ns;
    audit.schedstat_end_steady_ns = scheduling_delta.end_ns;
    audit.schedstat_sampling_wall_ns = scheduling_delta.sampling_ns;
    audit.schedstat_running_ns = scheduling_delta.running_ns;
    audit.schedstat_runqueue_wait_ns = scheduling_delta.runqueue_ns;
    const double post_alignment_runtime_ms =
        std::chrono::duration<double, std::milli>(
            post_alignment_finished - alignment_finished).count();
    const double fitness_runtime_ms =
        std::chrono::duration<double, std::milli>(
            fitness_finished - fitness_started).count();
    const double end_to_end_runtime_ms =
        std::chrono::duration<double, std::milli>(
            post_alignment_finished - started).count();
    // Recorded sensor headers use the device's boot-time clock. In shadow
    // replay, stamp the result after all synchronous match work completes so
    // freshness covers the complete causal operation. Live
    // mode always preserves the sensor measurement stamp.
    const rclcpp::Time output_time =
        use_receipt_time_ ? get_clock()->now() : sensor_stamp;

    audit.result = audit_pose(result);
    audit.initial_to_result_translation_m = initial_translation;
    audit.initial_to_result_yaw_rad = initial_yaw;
    audit.alignment_source_fitness_m2 = fitness;
    audit.pre_bounds_all_point_fitness_m2 =
        pre_bounds_registration_quality.all_point_mean_squared_nearest_neighbor_m2;
    audit.diagnostic_target_points = registration_quality.target_points;
    audit.diagnostic_source_points = registration_quality.source_points;
    audit.inlier_points = registration_quality.inlier_points;
    audit.source_inlier_fraction = registration_quality.source_inlier_fraction;
    audit.source_to_target_inlier_rmse_m =
        registration_quality.source_to_target_inlier_rmse_m;
    audit.pre_bounds_full_target_points =
        pre_bounds_registration_quality.target_points;
    audit.pre_bounds_source_points =
        pre_bounds_registration_quality.source_points;
    audit.pre_bounds_inlier_points =
        pre_bounds_registration_quality.inlier_points;
    audit.pre_bounds_source_inlier_fraction =
        pre_bounds_registration_quality.source_inlier_fraction;
    audit.pre_bounds_source_to_full_target_inlier_rmse_m =
        pre_bounds_registration_quality.source_to_target_inlier_rmse_m;
    audit.iterations = iterations;
    audit.alignment_runtime_ms = alignment_runtime_ms;
    audit.alignment_thread_cpu_ms = alignment_thread_cpu_ms;
    audit.fitness_thread_cpu_ms = fitness_thread_cpu_ms;
    audit.post_alignment_runtime_ms = post_alignment_runtime_ms;
    audit.fitness_runtime_ms = fitness_runtime_ms;
    audit.alignment_quality_runtime_ms = alignment_quality_runtime_ms;
    audit.pre_bounds_quality_runtime_ms = pre_bounds_quality_runtime_ms;
    audit.end_to_end_runtime_ms = end_to_end_runtime_ms;
    audit.converged = ndt_->has_converged();
    const auto finish_gates_timing = [&]() {
      audit.gates_done_steady_ns = direct_steady_ns();
      if (direct_runtime_)
        audit.gates_scheduling = sunhorizon_3d_localizer::detail::ndt_stage_scheduling_audit(
            scheduling_after, scheduling.sample());
    };
    const auto audit_rejected_ndt_match = [&](const std::string& reason) {
      finish_gates_timing();
      finish_ndt_match_audit(audit, reason, false);
    };
    const auto record_match_diagnostics = [&]() {
      last_score_ = score;
      last_iterations_ = iterations;
      last_runtime_ms_ = end_to_end_runtime_ms;
    };
    const auto reject_ndt_match = [&](const std::string& reason) {
      audit_rejected_ndt_match(reason);
      record_match_diagnostics();
      fitness_failure_gate_.reset();
      reject(reason, output_time);
    };

    const rclcpp::Time accepted_result_receipt_time = get_clock()->now();
    // Recorded shadow replay intentionally substitutes its local receipt clock
    // for boot-time sensor stamps, and is forbidden when motion output is
    // enabled. The live path validates and preserves the original sensor stamp.
    // Check before any acceptance gate mutates continuity, fitness, map-target,
    // or published localization state.
    const rclcpp::Time freshness_input_stamp =
        use_receipt_time_ ? accepted_result_receipt_time : sensor_stamp;
    const auto input_freshness =
        sunhorizon_3d_localizer::evaluate_ndt_input_age(
            freshness_input_stamp.nanoseconds(),
            accepted_result_receipt_time.nanoseconds(),
            maximum_input_age_sec_, input_future_tolerance_sec_);
    if (!input_freshness.accepted) {
      return reject_ndt_match(input_freshness.reason);
    }

    if (!audit.converged) return reject_ndt_match("ndt_not_converged");
    if (!finite_transform(result) || !std::isfinite(fitness)) {
      return reject_ndt_match("ndt_result_non_finite");
    }
    if (fitness > maximum_fitness_) {
      audit_rejected_ndt_match("ndt_fitness_limit");
      const auto decision = fitness_failure_gate_.observe_failure();
      if (decision ==
          sunhorizon_3d_localizer::NdtFitnessFailureDecision::kHoldLastAccepted) {
        RCLCPP_WARN(
            get_logger(),
            "Holding last accepted NDT correction after isolated fitness "
            "failure (%u/%u)",
            fitness_failure_gate_.consecutive_failures(),
            fitness_failure_gate_.minimum_consecutive_failures_for_lost());
        return;
      }
      record_match_diagnostics();
      reject("ndt_fitness_limit", output_time);
      return;
    }
    if (iterations > static_cast<uint32_t>(maximum_iterations_)) {
      return reject_ndt_match("ndt_iteration_limit");
    }
    const bool initial_translation_outside_tracking_limit =
        initial_translation > maximum_initial_translation_;
    const bool initial_yaw_outside_tracking_limit =
        initial_yaw > maximum_initial_yaw_;
    const bool startup_alignment_candidate = startup_alignment_enabled_ &&
        startup_alignment_available_ &&
        (initial_translation_outside_tracking_limit ||
         initial_yaw_outside_tracking_limit);
    if (!startup_alignment_candidate &&
        initial_translation_outside_tracking_limit) {
      startup_alignment_gate_.reset();
      return reject_ndt_match("ndt_initial_translation_limit");
    }
    if (!startup_alignment_candidate && initial_yaw_outside_tracking_limit) {
      startup_alignment_gate_.reset();
      return reject_ndt_match("ndt_initial_yaw_limit");
    }

    const Eigen::Matrix4f raw_map_to_base = result * source_to_base_;
    const Eigen::Matrix3f raw_base_rotation = raw_map_to_base.block<3, 3>(0, 0);
    audit.raw_map_to_base_z_m = raw_map_to_base(2, 3);
    audit.raw_map_to_base_roll_rad = roll(raw_base_rotation);
    audit.raw_map_to_base_pitch_rad = pitch(raw_base_rotation);
    audit.raw_map_to_base_x_m = raw_map_to_base(0, 3);
    audit.raw_map_to_base_y_m = raw_map_to_base(1, 3);
    audit.raw_map_to_base_yaw_rad = yaw(raw_base_rotation);
    if (!sunhorizon_3d_localizer::base_z_within_reference(
            raw_map_to_base(2, 3), base_z_reference_,
            maximum_base_z_deviation_)) {
      return reject_ndt_match("ndt_base_z_limit");
    }
    if (std::abs(roll(raw_base_rotation)) > maximum_base_roll_pitch_ ||
        std::abs(pitch(raw_base_rotation)) > maximum_base_roll_pitch_) {
      return reject_ndt_match("ndt_base_roll_pitch_limit");
    }
    if (startup_alignment_candidate) {
      const auto decision = startup_alignment_gate_.observe(
          sunhorizon_3d_localizer::NdtStartupAlignmentSample{
              stamp_nanoseconds(input->header.stamp),
              static_cast<double>(result(0, 3)),
              static_cast<double>(result(1, 3)),
              static_cast<double>(yaw(result.block<3, 3>(0, 0))),
              initial_translation, initial_yaw},
          startup_alignment_limits_);
      if (!decision.accepted) {
        return reject_ndt_match(decision.reason);
      }
      RCLCPP_INFO(
          get_logger(),
          "NDT startup alignment stable after %zu samples "
          "seed_delta=%.3f m/%.6f rad result=(%.3f,%.3f,%.6f)",
          decision.consecutive_samples, initial_translation, initial_yaw,
          result(0, 3), result(1, 3), yaw(result.block<3, 3>(0, 0)));
    } else {
      startup_alignment_gate_.reset();
    }
    Eigen::Matrix4f map_to_base = planarize(raw_map_to_base);
    Eigen::Matrix4f odom_to_base = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f candidate_raw_prediction_map_to_odom =
        Eigen::Matrix4f::Identity();
    Eigen::Matrix4f candidate_canonical_map_to_odom =
        Eigen::Matrix4f::Identity();
    std::optional<sunhorizon_3d_localizer::DirectMapTrackingState>
        next_direct_map_tracking_state;
    std::optional<sunhorizon_3d_localizer::NdtResultStepGate>
        next_result_step_gate;
    if (direct_map_tracking_enabled_) {
      sunhorizon_3d_localizer::DirectMapTrackingDecision direct_decision;
      if (local_map_target_enabled_) {
        auto proposed_state = direct_map_tracking_state_;
        direct_decision = proposed_state.evaluate(
            stamp_nanoseconds(input->header.stamp), result.cast<double>(),
            direct_map_tracking_limits_);
        if (direct_decision.accepted) {
          next_direct_map_tracking_state = std::move(proposed_state);
        }
      } else {
        direct_decision = direct_map_tracking_state_.evaluate(
            stamp_nanoseconds(input->header.stamp), result.cast<double>(),
            direct_map_tracking_limits_);
      }
      if (!direct_decision.accepted) {
        return reject_ndt_match(direct_decision.reason);
      }
      map_to_base = direct_decision.output.planar_map_to_base.cast<float>();
      odom_to_base =
          direct_decision.output.map_equivalent_odom_to_base.cast<float>();
    } else {
      odom_to_base = planarize(prior->odom_to_source * source_to_base_);
      // Keep the raw 3-D source-frame correction only for predicting the next
      // NDT registration.  Admission must use the same planar base-pose
      // composition that the localization manager uses for canonical TF.
      candidate_raw_prediction_map_to_odom =
          result * prior->odom_to_source.inverse();
      candidate_canonical_map_to_odom =
          sunhorizon_3d_localizer::
              canonical_map_to_odom_from_planar_base_poses(
                  map_to_base.cast<double>(), odom_to_base.cast<double>())
              .cast<float>();
      if (accepted_canonical_correction_valid_) {
        const auto correction_step =
            sunhorizon_3d_localizer::evaluate_transform_step(
                canonical_map_to_odom_.cast<double>(),
                candidate_canonical_map_to_odom.cast<double>(),
                maximum_correction_translation_step_,
                maximum_correction_rotation_step_);
        if (!correction_step.accepted) {
          if (correction_step.reason == "translation_step_limit") {
            return reject_ndt_match("map_to_odom_correction_translation_jump");
          }
          if (correction_step.reason == "rotation_step_limit") {
            return reject_ndt_match("map_to_odom_correction_rotation_jump");
          }
          return reject_ndt_match("map_to_odom_correction_invalid");
        }
      }
      auto proposed_result_step_gate = result_step_gate_;
      const auto result_step_decision = proposed_result_step_gate.evaluate(
          stamp_nanoseconds(input->header.stamp),
          candidate_canonical_map_to_odom.cast<double>(), result_step_limits_);
      audit.correction_step_reason = result_step_decision.reason;
      audit.correction_step_has_baseline = result_step_decision.has_baseline;
      audit.correction_step_establishes_baseline =
          result_step_decision.establishes_baseline;
      audit.correction_step_gap_recovered =
          result_step_decision.gap_recovered;
      audit.correction_step_dt_sec = result_step_decision.dt_sec;
      audit.correction_translation_step_m =
          result_step_decision.translation_step_m;
      audit.correction_yaw_step_rad = result_step_decision.yaw_step_rad;
      if (!result_step_decision.accepted) {
        // A gait impulse can produce one outlying NDT correction even while
        // the previous accepted correction is still fresh.  Keep the strict
        // selected translation / 2 degree admission envelope and never advance the baseline,
        // but do not turn that isolated rejected measurement into an
        // immediate LOST pulse.  The independent accepted-result timer owns
        // liveness: repeated outliers still become LOST once the last accepted
        // correction exceeds accepted_result_timeout_sec_.  Invalid stamps or
        // transforms remain immediate failures because they are not bounded
        // geometric outliers.
        audit_rejected_ndt_match(result_step_decision.reason);
        record_match_diagnostics();
        fitness_failure_gate_.reset();
        const bool bounded_step_outlier =
            result_step_decision.reason ==
                "ndt_correction_translation_step_limit" ||
            result_step_decision.reason == "ndt_correction_yaw_step_limit";
        if (bounded_step_outlier &&
            last_accepted_result_freshness(get_clock()->now()).accepted) {
          RCLCPP_WARN(
              get_logger(),
              "Holding last accepted NDT correction after transient result "
              "step rejection (reason=%s translation=%.6f m yaw=%.6f rad)",
              result_step_decision.reason.c_str(),
              result_step_decision.translation_step_m,
              result_step_decision.yaw_step_rad);
          return;
        }
        reject(result_step_decision.reason, output_time);
        return;
      }
      next_result_step_gate = std::move(proposed_result_step_gate);
    }
    if (local_map_target_enabled_ &&
        sunhorizon_3d_localizer::local_map_target_refresh_required(
            local_map_target_center_, local_map_target_center(result),
            local_map_target_refresh_distance_)) {
      std::string failure_reason;
      if (!rebuild_local_map_target(result, failure_reason)) {
        return reject_ndt_match(failure_reason);
      }
    }
    finish_gates_timing();
    if (next_direct_map_tracking_state) {
      direct_map_tracking_state_ = std::move(*next_direct_map_tracking_state);
    }
    if (next_result_step_gate) {
      result_step_gate_ = std::move(*next_result_step_gate);
    }
    map_to_source_ = result;
    if (!direct_map_tracking_enabled_) {
      raw_prediction_map_to_odom_ = candidate_raw_prediction_map_to_odom;
      canonical_map_to_odom_ = candidate_canonical_map_to_odom;
      accepted_canonical_correction_valid_ = true;
    }
    const rclcpp::Time accepted_output_time =
        use_receipt_time_ ? accepted_result_receipt_time : sensor_stamp;
    const builtin_interfaces::msg::Time output_stamp = accepted_output_time;
    OdometryCandidate odometry;
    odometry.header.stamp = output_stamp;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_footprint_frame_;
    odometry.pose = pose_message(
        odom_to_base, output_stamp, odom_frame_,
        position_covariance_, yaw_covariance_).pose;
    odometry.twist.covariance.fill(0.0);
    odometry.twist.covariance[0] = position_covariance_;
    odometry.twist.covariance[7] = position_covariance_;
    odometry.twist.covariance[35] = yaw_covariance_;
    if (previous_odometry_valid_) {
      const double dt =
          (sensor_stamp - previous_odometry_sensor_stamp_).seconds();
      if (dt > 1e-4 && dt <= maximum_twist_sensor_dt_sec_) {
        const Eigen::Matrix4f change = previous_odom_to_base_.inverse() * odom_to_base;
        odometry.twist.twist.linear.x = change(0, 3) / dt;
        odometry.twist.twist.linear.y = change(1, 3) / dt;
        odometry.twist.twist.linear.z = change(2, 3) / dt;
        odometry.twist.twist.angular.z = yaw(change.block<3, 3>(0, 0)) / dt;
      }
    }
    const auto accepted_pose = std::make_shared<const PoseCandidate>(pose_message(
        map_to_base, output_stamp, map_frame_, position_covariance_, yaw_covariance_));
    if (!direct_runtime_) {
      odometry_pub_->publish(odometry);
      pose_pub_->publish(*accepted_pose);
    }
    if (direct_map_tracking_enabled_ && !direct_runtime_) {
      // The manager requires the same accepted NDT result on its independent
      // tracking evidence channel before it can publish canonical odometry.
      tracking_odometry_pub_->publish(odometry);
    }
    previous_odom_to_base_ = odom_to_base;
    previous_odometry_sensor_stamp_ = sensor_stamp;
    previous_odometry_valid_ = true;
    record_match_diagnostics();
    last_measurement_stamp_ = accepted_output_time;
    last_accepted_result_receipt_time_ = accepted_result_receipt_time;
    fitness_failure_gate_.observe_success();
    last_accepted_ = true;
    startup_alignment_gate_.reset();
    startup_alignment_available_ = false;
    last_reason_ = direct_map_tracking_enabled_ ?
        "ok_direct_map_tracking_map_equivalent_odom" :
        (audit.correction_step_gap_recovered ?
            audit.correction_step_reason : "ok");
    DiagnosticArray deferred_diagnostics;
    finish_ndt_match_audit(audit, last_reason_, true,
        direct_runtime_ ? &deferred_diagnostics : nullptr);
    publish_status(accepted_output_time, accepted_result_receipt_time);
    if (direct_runtime_) {
      const auto metadata = direct_metadata_.find(stamp_nanoseconds(input->header.stamp));
      if (metadata == direct_metadata_.end())
        throw std::runtime_error("ndt_direct_source_metadata_missing");
      sunhorizon_3d_localizer::NdtDirectResult accepted;
      accepted.input = metadata->second.stamp;
      if (prior) {
        const auto prior_metadata = direct_prior_metadata_.find(prior->stamp_ns);
        if (prior_metadata == direct_prior_metadata_.end())
          throw std::runtime_error("ndt_direct_prior_metadata_missing");
        accepted.prior_input = prior_metadata->second;
      }
      accepted.result_receipt_ros_ns = accepted_result_receipt_time.nanoseconds();
      accepted.result_receipt_steady_ns = direct_steady_ns();
      accepted.map_pose = accepted_pose;
      accepted.odometry = std::make_shared<const OdometryCandidate>(std::move(odometry));
      accepted.independent_tracking_evidence = direct_map_tracking_enabled_;
      auto snapshot = sunhorizon_3d_localizer::detail::make_accepted_ndt_snapshot(
          std::move(accepted), result, input->header, map_frame_, map_path_);
      const auto handoff_enter_ns = direct_steady_ns();
      if (direct_sinks_.accepted) direct_sinks_.accepted(snapshot);
      const auto handoff_exit_ns = direct_steady_ns();
      // Publish the existing one-per-match diagnostic only after the direct
      // handoff. Original result clocks and all acceptance/counter ordering
      // remain unchanged; these added clocks never enter freshness decisions.
      auto& values = deferred_diagnostics.status.front().values;
      values.push_back(diagnostic_value("result_receipt_ros_ns", std::to_string(snapshot->result_receipt_ros_ns)));
      values.push_back(diagnostic_value("result_receipt_steady_ns", std::to_string(snapshot->result_receipt_steady_ns)));
      values.push_back(diagnostic_value("result_handoff_enter_steady_ns", std::to_string(handoff_enter_ns)));
      values.push_back(diagnostic_value("result_handoff_exit_steady_ns", std::to_string(handoff_exit_ns)));
      publish_diagnostics(std::move(deferred_diagnostics));
      if (direct_sinks_.publish_ros_candidates) direct_result_observer_->enqueue(std::move(snapshot));
    }
  }

  void publish_status(const rclcpp::Time& measurement_stamp, const rclcpp::Time& now) {
    LocalizationStatus output;
    output.header.stamp = measurement_stamp;
    output.header.frame_id = map_frame_;
    output.backend = backend_name_;
    output.measurement_accepted = last_accepted_;
    output.score = last_score_;
    output.iterations = last_iterations_;
    output.position_covariance = position_covariance_;
    output.yaw_covariance = yaw_covariance_;
    output.measurement_age_sec = last_measurement_stamp_.nanoseconds() > 0
        ? std::max(0.0, (now - last_measurement_stamp_).seconds()) : 0.0;
    output.reason = last_reason_;
    if (!initialized_) {
      output.state = LocalizationStatus::STATE_INITIALIZING;
    } else if (last_accepted_) {
      output.state = LocalizationStatus::STATE_HEALTHY;
    } else if (last_measurement_stamp_.nanoseconds() == 0) {
      output.state = LocalizationStatus::STATE_INITIALIZING;
    } else {
      output.state = LocalizationStatus::STATE_LOST;
    }
    if (direct_runtime_) {
      auto snapshot = std::make_shared<const LocalizationStatus>(std::move(output));
      if (direct_sinks_.status) direct_sinks_.status(snapshot);
      if (direct_sinks_.publish_ros_candidates || direct_sinks_.publish_ros_status_compatibility)
        direct_status_observer_->enqueue(std::move(snapshot));
    } else status_pub_->publish(output);
  }

  void on_timer() {
    std::lock_guard<std::recursive_mutex> direct_core_lock(direct_core_mutex_);
    const auto now = get_clock()->now();
    if (last_accepted_) {
      const auto freshness = last_accepted_result_freshness(now);
      if (!freshness.accepted) {
        last_accepted_ = false;
        last_reason_ = freshness.reason;
      }
    }
    const auto measurement_stamp = last_measurement_stamp_.nanoseconds() > 0
        ? last_measurement_stamp_ : now;
    publish_status(measurement_stamp, now);
  }

  bool direct_runtime_{false};
  int direct_worker_nice_{0};
  std::optional<sunhorizon_3d_localizer::detail::DirectWorkerState> direct_worker_state_;
  sunhorizon_3d_localizer::NdtDirectSinks direct_sinks_;
  mutable std::mutex direct_queue_mutex_;
  std::recursive_mutex direct_core_mutex_;
  std::condition_variable direct_condition_;
  std::thread direct_worker_;
  std::unique_ptr<ResultObserver> direct_result_observer_;
  std::unique_ptr<StatusObserver> direct_status_observer_;
  std::unique_ptr<DiagnosticsObserver> direct_diagnostics_observer_;
  bool direct_stopping_{false}, direct_failed_{false};
  int64_t direct_last_cloud_ns_{0}, direct_last_prior_ns_{0};
  uint64_t direct_last_cloud_sequence_{0}, direct_last_prior_sequence_{0};
  std::string direct_cloud_identity_, direct_prior_identity_;
  sunhorizon_3d_localizer::detail::NdtPendingClouds<DirectCloud> direct_pending_clouds_;
  std::deque<DirectPrior> direct_pending_priors_;
  std::string direct_input_error_;
  std::map<int64_t, DirectMetadata> direct_metadata_;
  std::map<int64_t, DirectStamp> direct_prior_metadata_;
  sunhorizon_3d_localizer::NdtDirectCounters direct_counts_;

  std::string map_path_;
  std::string input_topic_;
  std::string odometry_prior_topic_;
  std::string pose_topic_;
  std::string odometry_topic_;
  std::string tracking_odometry_topic_;
  std::string status_topic_;
  std::string diagnostics_topic_;
  std::string backend_name_;
  std::string initial_pose_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string expected_input_frame_;
  std::string expected_prior_frame_;
  std::string expected_prior_child_frame_;
  std::string base_footprint_frame_;
  std::string last_reason_{"initializing"};
  bool reliable_input_points_qos_{false};
  double map_leaf_{0.5};
  double local_map_target_radius_{20.0};
  double local_map_target_z_half_extent_{4.0};
  double local_map_target_refresh_distance_{5.0};
  double scan_leaf_{0.5};
  double resolution_{2.0};
  double minimum_scan_range_{0.5};
  double maximum_scan_range_{30.0};
  double maximum_fitness_{5.5};
  double diagnostic_inlier_max_distance_{1.0};
  double maximum_initial_translation_{0.35};
  double maximum_initial_yaw_{0.12};
  double base_z_reference_{0.0};
  double maximum_base_z_deviation_{0.30};
  double maximum_base_roll_pitch_{0.15};
  double maximum_match_rate_hz_{1.0e9};
  double position_covariance_{0.04};
  double yaw_covariance_{0.01};
  double maximum_input_age_sec_{0.50};
  double input_future_tolerance_sec_{0.01};
  double accepted_result_timeout_sec_{0.50};
  double maximum_twist_sensor_dt_sec_{0.50};
  double prior_sync_tolerance_sec_{0.001};
  double prior_timeout_sec_{0.25};
  double maximum_prior_translation_step_{0.30};
  double maximum_prior_rotation_step_{0.2617993877991494};
  double maximum_correction_translation_step_{0.30};
  double maximum_correction_rotation_step_{0.2617993877991494};
  double last_score_{0.0};
  double last_runtime_ms_{0.0};
  int maximum_iterations_{30};
  int minimum_raw_scan_points_{0};
  int minimum_scan_points_{500};
  int source_map_bounds_minimum_retained_points_{500};
  int minimum_consecutive_fitness_failures_for_lost_{2};
  int minimum_local_map_target_points_{1000};
  int prior_buffer_size_{32};
  uint32_t last_iterations_{0};
  std::uint64_t match_throttled_count_{0};
  std::uint64_t incomplete_raw_scan_skipped_count_{0};
  sunhorizon_3d_localizer::NdtFitnessFailureGate fitness_failure_gate_{2};
  sunhorizon_3d_localizer::NdtResultStepLimits result_step_limits_;
  bool attended_result_step_profile_{false};
  sunhorizon_3d_localizer::NdtResultStepGate result_step_gate_;
  bool startup_alignment_enabled_{false};
  bool startup_alignment_available_{false};
  sunhorizon_3d_localizer::NdtStartupAlignmentLimits startup_alignment_limits_;
  sunhorizon_3d_localizer::NdtStartupAlignmentGate startup_alignment_gate_;
  bool initial_pose_accepted_{false};
  bool require_runtime_initial_pose_{false};
  bool source_to_base_accepted_{false};
  bool prior_basis_accepted_{false};
  bool canonical_base_prior_{false};
  bool use_receipt_time_{false};
  bool direct_map_tracking_enabled_{false};
  bool local_map_target_enabled_{false};
  bool local_map_target_ready_{false};
  bool source_map_bounds_filter_enabled_{false};
  bool extended_quality_diagnostics_enabled_{true};
  bool initialized_{false};
  bool last_accepted_{false};
  bool previous_odometry_valid_{false};
  bool raw_prediction_correction_initialized_{false};
  bool accepted_canonical_correction_valid_{false};
  std::int64_t last_pointcloud_stamp_ns_{0};
  Eigen::Matrix4f map_to_source_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f source_to_base_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f prior_basis_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f raw_prediction_map_to_odom_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f canonical_map_to_odom_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f previous_odom_to_base_{Eigen::Matrix4f::Identity()};
  sunhorizon_3d_localizer::DirectMapTrackingLimits direct_map_tracking_limits_;
  sunhorizon_3d_localizer::DirectMapTrackingState direct_map_tracking_state_;
  sunhorizon_3d_localizer::NdtMatchRateGate ndt_match_rate_gate_{1.0e9};
  sunhorizon_3d_localizer::NdtPriorSyncDiagnostics prior_sync_diagnostics_;
  sunhorizon_3d_localizer::ScanRangeGate scan_range_gate_{0.5, 30.0};
  sunhorizon_3d_localizer::SourceMapBoundsFilterLimits
      source_map_bounds_filter_limits_;
  sunhorizon_3d_localizer::NdtBackendOptions ndt_backend_options_;
  sunhorizon_3d_localizer::LocalMapTargetCenter local_map_target_center_;
  std::optional<sunhorizon_3d_localizer::MapAxisAlignedBounds>
      active_target_bounds_;
  std::deque<PriorSample> prior_buffer_;
  std::deque<PendingPointCloud> pointcloud_buffer_;
  PointCloud::Ptr full_target_;
  std::string registration_map_sha256_;
  PointCloud::Ptr target_;
  sunhorizon_3d_localizer::NdtRegistrationQualityIndex
      ndt_registration_quality_index_;
  sunhorizon_3d_localizer::NdtRegistrationQualityIndex
      full_target_registration_quality_index_;
  std::unique_ptr<sunhorizon_3d_localizer::NdtRegistration> ndt_;
  std::unique_ptr<sunhorizon_3d_localizer::PriorContinuityGate>
      prior_continuity_gate_;
  rclcpp::Time last_measurement_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_accepted_result_receipt_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time previous_odometry_sensor_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<PoseCandidate>::SharedPtr pose_pub_;
  rclcpp::Publisher<OdometryCandidate>::SharedPtr odometry_pub_;
  rclcpp::Publisher<OdometryCandidate>::SharedPtr tracking_odometry_pub_;
  rclcpp::Publisher<LocalizationStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<OdometryCandidate>::SharedPtr odometry_prior_sub_;
  rclcpp::Subscription<PoseCandidate>::SharedPtr initial_pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

namespace sunhorizon_3d_localizer {
std::shared_ptr<NdtDirectRuntime> make_ndt_direct_runtime(
    const rclcpp::NodeOptions& options, NdtDirectSinks sinks) {
  return std::make_shared<NdtLocalizer>(options, true, std::move(sinks));
}
}  // namespace sunhorizon_3d_localizer

#ifndef SUNHORIZON_NDT_NO_MAIN
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NdtLocalizer>());
  rclcpp::shutdown();
  return 0;
}

#endif
