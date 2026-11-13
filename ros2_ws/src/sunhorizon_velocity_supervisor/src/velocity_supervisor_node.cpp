#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <unistd.h>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>
#include <std_srvs/srv/trigger.hpp>
#include <sunhorizon_navigation_core/safety_gate.hpp>
#include <sunhorizon_navigation_core/geometry_recovery.hpp>
#include <sunhorizon_velocity_supervisor/coverage_decision_tracker.hpp>
#include <sunhorizon_velocity_supervisor/spatial_coverage_decision_tracker.hpp>
#include <sunhorizon_obstacle_coverage/spatial_evidence_message.hpp>
#include <sunhorizon_navigation_interfaces/msg/obstacle_coverage_spatial_decision.hpp>
#include <sunhorizon_navigation_interfaces/msg/obstacle_coverage_decision.hpp>
#include <sunhorizon_navigation_interfaces/msg/velocity_command_evidence.hpp>
#include "velocity_command_evidence.hpp"
#include "velocity_command_evidence_linux.hpp"
#include <sunhorizon_navigation_interfaces/msg/localization_status.hpp>
#include <sunhorizon_navigation_interfaces/msg/navigation_status.hpp>
#include <sunhorizon_navigation_interfaces/msg/obstacle_coverage_status.hpp>
#include <sunhorizon_navigation_interfaces/msg/platform_safety_state.hpp>
#include <sunhorizon_navigation_interfaces/msg/velocity_supervisor_status.hpp>
#include <sunhorizon_velocity_supervisor/motion_authority_contract.hpp>
#include <sunhorizon_velocity_supervisor/attended_travel_guard.hpp>
#include <sunhorizon_velocity_supervisor/parameter_contract.hpp>
#include <sunhorizon_velocity_supervisor/tf_continuity.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace {

#ifndef SUNHORIZON_RELEASE_SHA
#define SUNHORIZON_RELEASE_SHA "development"
#endif

using LocalizationStatus = sunhorizon_navigation_interfaces::msg::LocalizationStatus;
using ObstacleCoverageStatus =
    sunhorizon_navigation_interfaces::msg::ObstacleCoverageStatus;
using CoverageDecision = sunhorizon_navigation_interfaces::msg::ObstacleCoverageDecision;
using SpatialDecision = sunhorizon_navigation_interfaces::msg::ObstacleCoverageSpatialDecision;
using CommandEvidence = sunhorizon_navigation_interfaces::msg::VelocityCommandEvidence;
using NavigationStatus = sunhorizon_navigation_interfaces::msg::NavigationStatus;
using PlatformSafetyState = sunhorizon_navigation_interfaces::msg::PlatformSafetyState;
using SupervisorStatus = sunhorizon_navigation_interfaces::msg::VelocitySupervisorStatus;
using sunhorizon_navigation_core::CollisionAuditClass;
using sunhorizon_navigation_core::SafetyGate;
using sunhorizon_navigation_core::SafetyInputs;
using sunhorizon_navigation_core::SafetyLimits;
using sunhorizon_navigation_core::Twist2D;
using sunhorizon_navigation_core::collision_audit_requires_immediate_latch;
using sunhorizon_navigation_core::parse_collision_audit_frame_id;
using sunhorizon_velocity_supervisor::AttendedQualificationNonzeroBudget;
using sunhorizon_velocity_supervisor::MotionAuthorityContract;
using sunhorizon_velocity_supervisor::kAttendedThreeMeterNavigationMode;
using sunhorizon_velocity_supervisor::kAttendedFiveMeterNavigationMode;
using sunhorizon_velocity_supervisor::PlanarTfObservation;
using sunhorizon_velocity_supervisor::TfContinuityTracker;
using sunhorizon_velocity_supervisor::TfProgressTracker;
using sunhorizon_velocity_supervisor::kAttendedFiveCmQualificationMode;
using sunhorizon_velocity_supervisor::kAttendedForwardCalibrationBudgetExhaustedReason;
using sunhorizon_velocity_supervisor::kAttendedForwardCalibrationMode;
using sunhorizon_velocity_supervisor::kAttendedForwardPulseMode;
using sunhorizon_velocity_supervisor::kAttendedForwardPulseBudgetExhaustedReason;
using sunhorizon_velocity_supervisor::kAttendedFirstClickNavigationBudgetExhaustedReason;
using sunhorizon_velocity_supervisor::kAttendedFirstClickNavigationMode;
using sunhorizon_velocity_supervisor::kAttendedHeadingCorrectionNavigationMode;
using sunhorizon_velocity_supervisor::kAttendedHeadingCorrectionNavigationBudgetExhaustedReason;
using sunhorizon_velocity_supervisor::kAttendedQualificationBudgetExhaustedReason;
using sunhorizon_velocity_supervisor::kAttendedYawQualificationMode;
using sunhorizon_velocity_supervisor::kAttendedYawQualificationBudgetExhaustedReason;
using sunhorizon_velocity_supervisor::motion_authority_contract_failure;
using sunhorizon_velocity_supervisor::read_only_parameter;

double stamp_seconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

geometry_msgs::msg::Twist to_message(const Twist2D& input) {
  geometry_msgs::msg::Twist output;
  output.linear.x = input.vx;
  output.linear.y = input.vy;
  output.angular.z = input.wz;
  return output;
}

bool near_zero(const Twist2D& command) {
  return std::abs(command.vx) < 1e-6 && std::abs(command.vy) < 1e-6 &&
         std::abs(command.wz) < 1e-6;
}

std::string gid_bytes_hex(const uint8_t* data, const std::size_t size) {
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (std::size_t index = 0; index < size; ++index) {
    output << std::setw(2) << static_cast<unsigned int>(data[index]);
  }
  return output.str();
}

std::string endpoint_gid_hex(const rmw_gid_t& gid) {
  return gid_bytes_hex(gid.data, RMW_GID_STORAGE_SIZE);
}

template <std::size_t Size>
std::string endpoint_gid_hex(const std::array<uint8_t, Size>& gid) {
  return gid_bytes_hex(gid.data(), gid.size());
}

struct CollisionEndpointEpoch {
  std::string candidate_gid;
  std::string audit_gid;
  std::string coverage_gid, controller_gid, command_gid, navigation_gid;

  bool empty() const { return candidate_gid.empty() && audit_gid.empty(); }
  bool operator==(const CollisionEndpointEpoch& other) const {
    return candidate_gid == other.candidate_gid && audit_gid == other.audit_gid &&
        coverage_gid == other.coverage_gid && controller_gid == other.controller_gid &&
        command_gid == other.command_gid && navigation_gid == other.navigation_gid;
  }
  bool operator!=(const CollisionEndpointEpoch& other) const {
    return !(*this == other);
  }
};

enum class SupervisorState : uint8_t {
  kDisarmed = SupervisorStatus::STATE_DISARMED,
  kArmedIdle = SupervisorStatus::STATE_ARMED_IDLE,
  kRunning = SupervisorStatus::STATE_RUNNING,
  kFaultLatched = SupervisorStatus::STATE_FAULT_LATCHED,
};

class VelocitySupervisor final : public rclcpp::Node {
 public:
  VelocitySupervisor()
      : Node("sunhorizon_velocity_supervisor"),
        gate_(declare_limits()),
        tf_buffer_(get_clock()),
        tf_listener_(tf_buffer_) {
    tf_watchdog_timeout_sec_ = get_parameter("tf_timeout_sec").as_double();
    candidate_topic_ = declare_parameter(
        "candidate_topic", "/sunhorizon/navigation/cmd_vel_candidate");
    collision_audit_topic_ = declare_parameter(
        "collision_audit_topic", "/sunhorizon/navigation/collision_audit");
    final_topic_ = declare_parameter("final_topic", "/navigation/cmd_vel");
    map_frame_ = declare_parameter("map_frame", "map");
    base_frame_ = declare_parameter("base_frame", "base_footprint");
    const auto release_sha = declare_parameter<std::string>(
        "release_sha", SUNHORIZON_RELEASE_SHA,
        read_only_parameter("Must equal the release SHA compiled into this binary."));
    if (release_sha != SUNHORIZON_RELEASE_SHA) {
      throw std::runtime_error("release_sha does not match compiled binary identity");
    }
    declare_parameter<int64_t>(
        "process_id", static_cast<int64_t>(::getpid()),
        read_only_parameter(
            "Linux PID of this exact supervisor process for attended runtime binding."));
    control_rate_hz_ = declare_parameter<double>(
        "control_rate_hz", 50.0,
        read_only_parameter(
            "Immutable after construction: the fixed control timer is created from this value."));
    motion_output_requested_ = declare_parameter("motion_output_enabled", false);
    motion_authority_mode_ = declare_parameter<std::string>(
        "motion_authority_mode",
        sunhorizon_velocity_supervisor::kReceiptGatedNavigationMode,
        read_only_parameter(
            "receipt-gated navigation or a dedicated attended qualification."));
    qualification_profile_acknowledgement_ = declare_parameter<std::string>(
        "qualification_profile_acknowledgement", "",
        read_only_parameter(
            "Exact acknowledgement accepted only by the dedicated qualification profile."));
    velocity_calibration_accepted_ = declare_parameter(
        "velocity_calibration_accepted", false,
        read_only_parameter("True only after validating the attended calibration receipt."));
    vendor_watchdog_accepted_ = declare_parameter(
        "vendor_watchdog_accepted", false,
        read_only_parameter("True only after three <=0.25 s publisher-loss stop trials."));
    vendor_vx_gain_ = declare_parameter<double>(
        "vendor_vx_gain", 1.0,
        read_only_parameter("Receipt-derived final adapter gain; candidate remains SI."));
    vendor_vx_deadzone_command_ = declare_parameter<double>(
        "vendor_vx_deadzone_command", 0.0,
        read_only_parameter("Measured upper bound; recorded but never used to force motion."));
    if (!std::isfinite(vendor_vx_gain_) || vendor_vx_gain_ < 0.5 ||
        vendor_vx_gain_ > 2.0 || !std::isfinite(vendor_vx_deadzone_command_) ||
        vendor_vx_deadzone_command_ < 0.0 || vendor_vx_deadzone_command_ > 0.01) {
      throw std::runtime_error("vendor_velocity_calibration_parameters_invalid");
    }
    final_output_lock_path_ = declare_parameter<std::string>(
        "final_output_lock_path",
        "/run/lock/sunhorizon_velocity_supervisor_final.lock",
        read_only_parameter(
            "Host-local singleton lock held by the final velocity publisher."));
    publisher_discovery_grace_sec_ = declare_parameter("publisher_discovery_grace_sec", 2.0);
    max_tf_translation_jump_m_ = declare_parameter("max_tf_translation_jump_m", 0.30);
    max_tf_yaw_jump_rad_ = declare_parameter("max_tf_yaw_jump_rad", 0.25);
    endpoint_stability_sec_ = declare_parameter<double>(
        "endpoint_stability_sec", 0.50,
        read_only_parameter("Publisher ownership must remain exact for this long before arm."));
    lease_acquisition_grace_sec_ = declare_parameter<double>(
        "lease_acquisition_grace_sec", 0.10,
        read_only_parameter(
            "Maximum zero-output ordering grace for the first navigation lease."));
    expected_candidate_publisher_fqn_ = declare_parameter<std::string>(
        "expected_candidate_publisher_fqn", "/collision_monitor",
        read_only_parameter("Only this node may publish collision-filtered candidates."));
    if (endpoint_stability_sec_ <= 0.0 || lease_acquisition_grace_sec_ <= 0.0) {
      throw std::runtime_error("supervisor_timing_parameters_must_be_positive");
    }

    const auto platform_topic = declare_parameter(
        "platform_state_topic", "/sunhorizon/platform/safety_state");
    const auto localization_topic = declare_parameter(
        "localization_status_topic", "/sunhorizon/localization/status");
    const auto obstacle_coverage_topic = declare_parameter(
        "obstacle_coverage_topic", "/sunhorizon/obstacle_coverage/status");
    const auto navigation_status_topic = declare_parameter(
        "navigation_status_topic", "/sunhorizon/navigation/status");
    if (geometry_hold_enabled_ && navigation_status_topic != "/sunhorizon/navigation/status") {
      throw std::runtime_error("geometry_navigation_status_topic_must_remain_fixed");
    }

    const MotionAuthorityContract authority_contract{
        motion_authority_mode_,
        qualification_profile_acknowledgement_,
        motion_output_requested_,
        velocity_calibration_accepted_,
        vendor_watchdog_accepted_,
        vendor_vx_gain_,
        vendor_vx_deadzone_command_,
        candidate_topic_,
        collision_audit_topic_,
        final_topic_,
        map_frame_,
        base_frame_,
        platform_topic,
        localization_topic,
        obstacle_coverage_topic,
        expected_candidate_publisher_fqn_,
        candidate_timeout_sec_,
        control_rate_hz_,
        max_vx_,
        max_vy_,
        max_wz_,
        max_ax_,
        max_ay_,
        max_aw_,
    };
    const auto authority_failure =
        motion_authority_contract_failure(authority_contract);
    if (!authority_failure.empty()) {
      throw std::runtime_error(authority_failure);
    }
    direct_qualification_candidate_mode_ =
        motion_authority_mode_ == kAttendedFiveCmQualificationMode ||
        motion_authority_mode_ == kAttendedYawQualificationMode ||
        motion_authority_mode_ == kAttendedForwardCalibrationMode ||
        motion_authority_mode_ == kAttendedForwardPulseMode;
    spatial_visibility_enabled_ = declare_parameter<bool>(
        "spatial_visibility_enabled", false,
        read_only_parameter("Accept the separately typed spatial decision only; requires geometry hold."));
    spatial_obstacle_source_mode_=declare_parameter<std::string>("spatial_obstacle_source_mode",
      "lidar-rgbd-v1",read_only_parameter("Exact upstream profile/source inventory; no physical coverage qualification."));
    if((spatial_obstacle_source_mode_!="lidar-rgbd-v1" && spatial_obstacle_source_mode_!="rgbd-only-v1") ||
        (spatial_obstacle_source_mode_=="rgbd-only-v1" && !spatial_visibility_enabled_))
      throw std::runtime_error("spatial_obstacle_source_mode_invalid");
    if (spatial_visibility_enabled_ && !geometry_hold_enabled_)
      throw std::runtime_error("spatial_visibility_requires_geometry_hold");
    if (command_generation_timeout_sec_ != candidate_timeout_sec_) {
      if (!spatial_visibility_enabled_ || spatial_obstacle_source_mode_ != "rgbd-only-v1" ||
          motion_authority_mode_ != "attended_heading_correction_navigation_v1") {
        throw std::invalid_argument("extended_generation_timeout_requires_attended_navigation");
      }
      sunhorizon_obstacle_coverage::validate_low_speed_camera_limits(max_vx_, max_vy_, max_wz_);
    }
    if (geometry_hold_enabled_) {
      if (direct_qualification_candidate_mode_) {
        throw std::runtime_error("geometry_hold_requires_navigation_task_authority");
      }
      geometry_contract_ = declare_parameter<std::string>(
          "geometry_hold_region_contract", "",
          read_only_parameter("Exact accepted typed region contract; no runtime substitution."));
      if (geometry_contract_.empty()) throw std::runtime_error("geometry_hold_contract_missing");
      geometry_boot_id_ = sunhorizon_command_evidence::linux_boot_id();
      if (spatial_visibility_enabled_) {
        spatial_config_.maximum_age_ns=sunhorizon_obstacle_coverage::camera_freshness_age_ns(
          declare_parameter<std::string>("camera_freshness_profile",
            sunhorizon_obstacle_coverage::kStrictCameraFreshness,
            read_only_parameter("Original camera sample deadline for the attended low-speed profile.")));
        if(spatial_config_.maximum_age_ns>sunhorizon_obstacle_coverage::kDefaultCameraAgeNs) {
          sunhorizon_obstacle_coverage::validate_low_speed_camera_limits(max_vx_,max_vy_,max_wz_);
          if(motion_authority_mode_!="attended_heading_correction_navigation_v1" ||
              spatial_obstacle_source_mode_!="rgbd-only-v1")
            throw std::invalid_argument("extended_camera_age_requires_attended_navigation_authority");
        }
        spatial_config_.contract = geometry_contract_;
        spatial_config_.frame_id = base_frame_;
        spatial_config_.clock_boot_id = geometry_boot_id_;
        spatial_config_.query_digest = declare_parameter<std::string>(
            "spatial_query_digest", "", read_only_parameter("Exact accepted query identity."));
        spatial_config_.producer_epoch = declare_parameter<std::string>(
            "spatial_producer_epoch", "", read_only_parameter("Pinned spatial producer instance."));
        const auto ids = declare_parameter<std::vector<int64_t>>(
            "spatial_source_ids", {}, read_only_parameter("Pinned source IDs, belly required."));
        const auto epochs = declare_parameter<std::vector<std::string>>(
            "spatial_source_epochs", {}, read_only_parameter("Pinned source producer instances."));
        const auto frames = declare_parameter<std::vector<std::string>>(
            "spatial_source_frames", {}, read_only_parameter("Pinned source optical frames."));
        if (ids.size() != epochs.size() || ids.size() != frames.size())
          throw std::runtime_error("spatial_source_pin_layout_invalid");
        for (size_t i = 0; i < ids.size(); ++i) {
          if (ids[i] < 0 || ids[i] > 2) throw std::runtime_error("spatial_source_id_invalid");
          spatial_config_.source_identities.push_back(
              {static_cast<uint8_t>(ids[i]), epochs[i], frames[i]});
        }
        auto validate = spatial_config_;
        validate.owner_epoch = "construction_only_no_motion_authority";
        sunhorizon_obstacle_coverage::SpatialEvidenceTracker check(validate);
      }
    }
    const bool five_meter_navigation =
        sunhorizon_velocity_supervisor::is_five_meter_navigation_mode(motion_authority_mode_);
    bounded_distance_navigation_ = motion_authority_mode_ == kAttendedThreeMeterNavigationMode || five_meter_navigation;
    attended_distance_prefix_ = five_meter_navigation ? "attended_five_meter" : "attended_three_meter";
    const auto cross_track_stop = declare_parameter<double>(
        "attended_cross_track_stop_m", .50,
        read_only_parameter("Attended site boundary; changing it requires a stopped restart."));
    const auto cross_track_observation = declare_parameter<double>(
        "attended_cross_track_observation_m", .50,
        read_only_parameter("Deviation reporting threshold, not a stop condition."));
    if (!std::isfinite(cross_track_observation) || cross_track_observation <= 0. ||
        cross_track_observation > cross_track_stop)
      throw std::runtime_error("attended_cross_track_observation_m_invalid");
    travel_guard_ = sunhorizon_velocity_supervisor::AttendedTravelGuard(
        five_meter_navigation, cross_track_stop);
    const bool heading_correction_navigation =
        motion_authority_mode_ == kAttendedHeadingCorrectionNavigationMode || bounded_distance_navigation_;
    bounded_attended_output_mode_ = direct_qualification_candidate_mode_ ||
        motion_authority_mode_ == kAttendedFirstClickNavigationMode ||
        heading_correction_navigation;
    qualification_budget_latch_reason_ =
        heading_correction_navigation
            ? kAttendedHeadingCorrectionNavigationBudgetExhaustedReason :
        motion_authority_mode_ == kAttendedForwardPulseMode
            ? kAttendedForwardPulseBudgetExhaustedReason :
        motion_authority_mode_ == kAttendedFirstClickNavigationMode
            ? kAttendedFirstClickNavigationBudgetExhaustedReason
            : motion_authority_mode_ == kAttendedYawQualificationMode
            ? kAttendedYawQualificationBudgetExhaustedReason
            : (motion_authority_mode_ == kAttendedForwardCalibrationMode
                   ? kAttendedForwardCalibrationBudgetExhaustedReason
                   : kAttendedQualificationBudgetExhaustedReason);
    qualification_budget_spent_reason_ =
        heading_correction_navigation
            ? "attended_heading_correction_navigation_nonzero_budget_spent" :
        motion_authority_mode_ == kAttendedForwardPulseMode
            ? "attended_forward_pulse_nonzero_budget_spent" :
        motion_authority_mode_ == kAttendedFirstClickNavigationMode
            ? "attended_first_click_navigation_nonzero_budget_spent"
            : motion_authority_mode_ == kAttendedYawQualificationMode
            ? "attended_yaw_qualification_nonzero_budget_spent"
            : (motion_authority_mode_ == kAttendedForwardCalibrationMode
                   ? "attended_forward_calibration_nonzero_budget_spent"
                   : "attended_five_cm_qualification_nonzero_budget_spent");
    if (motion_authority_mode_ == kAttendedForwardPulseMode) {
      qualification_nonzero_budget_ = AttendedQualificationNonzeroBudget(
          AttendedQualificationNonzeroBudget::kForwardPulseHardLimit);
    } else if (motion_authority_mode_ == kAttendedForwardCalibrationMode) {
      qualification_nonzero_budget_ = AttendedQualificationNonzeroBudget(
          AttendedQualificationNonzeroBudget::kForwardCalibrationHardLimit);
    } else if (motion_authority_mode_ == kAttendedFirstClickNavigationMode ||
               heading_correction_navigation) {
      qualification_nonzero_budget_ = AttendedQualificationNonzeroBudget(
          AttendedQualificationNonzeroBudget::kFirstClickNavigationHardLimit);
    }
    if (bounded_distance_navigation_) {
      qualification_nonzero_budget_ = AttendedQualificationNonzeroBudget(
          five_meter_navigation ? AttendedQualificationNonzeroBudget::kFiveMeterHardLimit
                                : AttendedQualificationNonzeroBudget::kThreeMeterHardLimit);
      qualification_budget_latch_reason_ = attended_distance_prefix_ + "_navigation_nonzero_budget_exhausted";
      qualification_budget_spent_reason_ = attended_distance_prefix_ + "_navigation_nonzero_budget_spent";
    }
    if (motion_output_requested_) acquire_final_output_lock();

    status_pub_ = create_publisher<SupervisorStatus>(
        "/sunhorizon/navigation/velocity_supervisor_status",
        rclcpp::QoS(10).transient_local());
    if (direct_qualification_candidate_mode_) {
      qualification_candidate_sub_ = create_subscription<geometry_msgs::msg::Twist>(
          candidate_topic_, 10,
          std::bind(
              &VelocitySupervisor::on_qualification_candidate, this,
              std::placeholders::_1));
    } else if (geometry_hold_enabled_) {
      command_evidence_sub_ = create_subscription<CommandEvidence>(
          kCommandTopic, rclcpp::QoS(1).reliable().durability_volatile(),
          std::bind(&VelocitySupervisor::on_command_evidence, this,
                    std::placeholders::_1, std::placeholders::_2));
      if (std::string(command_evidence_sub_->get_topic_name()) != kCommandTopic) {
        throw std::runtime_error("geometry_command_topic_must_remain_fixed");
      }
    } else {
      collision_audit_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
          collision_audit_topic_, 10,
          std::bind(
              &VelocitySupervisor::on_collision_audit, this,
              std::placeholders::_1, std::placeholders::_2));
    }
    platform_sub_ = create_subscription<PlatformSafetyState>(
        platform_topic, 10,
        std::bind(&VelocitySupervisor::on_platform, this, std::placeholders::_1));
    localization_sub_ = create_subscription<LocalizationStatus>(
        localization_topic, 10,
        std::bind(&VelocitySupervisor::on_localization, this, std::placeholders::_1));
    if (spatial_visibility_enabled_) {
      spatial_decision_sub_ = create_subscription<SpatialDecision>(
          kSpatialDecisionTopic, rclcpp::QoS(1).reliable().durability_volatile(),
          std::bind(&VelocitySupervisor::on_spatial_decision, this,
                    std::placeholders::_1, std::placeholders::_2));
      if (std::string(spatial_decision_sub_->get_topic_name()) != kSpatialDecisionTopic)
        throw std::runtime_error("spatial_coverage_topic_must_remain_fixed");
    } else if (geometry_hold_enabled_) {
      coverage_decision_sub_ = create_subscription<CoverageDecision>(
          kCoverageDecisionTopic, rclcpp::QoS(1).reliable().durability_volatile(),
          std::bind(&VelocitySupervisor::on_coverage_decision, this,
                    std::placeholders::_1, std::placeholders::_2));
      if (std::string(coverage_decision_sub_->get_topic_name()) != kCoverageDecisionTopic) {
        throw std::runtime_error("geometry_coverage_topic_must_remain_fixed");
      }
    } else {
      obstacle_coverage_sub_ = create_subscription<ObstacleCoverageStatus>(
          obstacle_coverage_topic, rclcpp::QoS(1).reliable().transient_local(),
          std::bind(&VelocitySupervisor::on_obstacle_coverage, this, std::placeholders::_1));
    }
    navigation_status_sub_ = create_subscription<NavigationStatus>(
        navigation_status_topic, rclcpp::QoS(10).reliable().transient_local(),
        std::bind(&VelocitySupervisor::on_navigation_status, this,
                  std::placeholders::_1, std::placeholders::_2));
    reset_service_ = create_service<std_srvs::srv::Trigger>(
        "/sunhorizon/navigation/reset_velocity_latch",
        std::bind(&VelocitySupervisor::on_reset, this,
          std::placeholders::_1, std::placeholders::_2));
    stop_service_ = create_service<std_srvs::srv::Trigger>(
        "/sunhorizon/navigation/latch_zero_velocity",
        std::bind(&VelocitySupervisor::on_stop, this,
          std::placeholders::_1, std::placeholders::_2));

    if (control_rate_hz_ <= 0.0) throw std::runtime_error("control_rate_hz must be positive");
    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&VelocitySupervisor::on_control, this));
    startup_time_ = get_clock()->now();
    gate_.force_latch("startup_disarmed");
    if (!motion_output_requested_) {
      RCLCPP_WARN(get_logger(),
          "Shadow mode: no publisher will be created for %s", final_topic_.c_str());
    }
  }

  ~VelocitySupervisor() override {
    if (final_output_lock_fd_ >= 0) {
      ::flock(final_output_lock_fd_, LOCK_UN);
      ::close(final_output_lock_fd_);
    }
  }

 private:
  void acquire_final_output_lock() {
    final_output_lock_fd_ = ::open(
        final_output_lock_path_.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0660);
    if (final_output_lock_fd_ < 0) {
      throw std::runtime_error(
          "final_output_lock_open_failed:" +
          std::string(std::strerror(errno)));
    }
    if (::flock(final_output_lock_fd_, LOCK_EX | LOCK_NB) != 0) {
      const std::string error = std::strerror(errno);
      ::close(final_output_lock_fd_);
      final_output_lock_fd_ = -1;
      throw std::runtime_error("final_output_lock_already_held:" + error);
    }
  }

  SafetyLimits declare_limits() {
    SafetyLimits limits;
    limits.geometry_hold_enabled = declare_parameter<bool>(
        "geometry_hold_enabled", false,
        read_only_parameter("Requires authenticated typed coverage and generated command evidence."));
    geometry_hold_enabled_ = limits.geometry_hold_enabled;
    limits.candidate_timeout_sec = declare_parameter<double>(
        "candidate_timeout_sec", 0.25,
        read_only_parameter(
            "Immutable after construction: the safety gate copies this timeout at startup."));
    candidate_timeout_sec_ = limits.candidate_timeout_sec;
    command_generation_timeout_sec_ = declare_parameter<double>(
        "command_generation_timeout_sec", candidate_timeout_sec_,
        read_only_parameter("Maximum age from typed command computation to final output."));
    if (command_generation_timeout_sec_ != candidate_timeout_sec_ &&
        !(geometry_hold_enabled_ && candidate_timeout_sec_ == 0.25 &&
          command_generation_timeout_sec_ == 0.50)) {
      throw std::invalid_argument("unsupported_command_generation_timeout");
    }
    // SafetyGate receives the original compute stamp. The independent steady
    // receipt watchdog continues to use candidate_timeout_sec_.
    limits.candidate_timeout_sec = command_generation_timeout_sec_;
    limits.platform_timeout_sec = declare_parameter<double>(
        "platform_timeout_sec", 0.25,
        read_only_parameter("Immutable safety-evidence freshness limit."));
    limits.obstacle_coverage_timeout_sec = declare_parameter<double>(
        "obstacle_coverage_timeout_sec", 0.30,
        read_only_parameter("Immutable obstacle-coverage freshness limit."));
    limits.localization_timeout_sec = declare_parameter<double>(
        "localization_timeout_sec", 0.50,
        read_only_parameter("Immutable localization freshness limit."));
    limits.tf_timeout_sec = declare_parameter<double>(
        "tf_timeout_sec", 0.25,
        read_only_parameter("Immutable TF freshness limit."));
    limits.navigation_lease_timeout_sec = declare_parameter<double>(
        "navigation_lease_timeout_sec", 0.50,
        read_only_parameter(
            "Maximum age of the navigation-server execution lease for non-zero motion."));
    navigation_lease_timeout_sec_ = limits.navigation_lease_timeout_sec;
    if (limits.obstacle_coverage_timeout_sec == .60)
      coverage_report_age_ns_ = 600000000;
    limits.max_vx = declare_parameter<double>(
        "max_vx", 0.20, read_only_parameter("Immutable forward velocity limit."));
    max_vx_ = limits.max_vx;
    limits.max_vy = declare_parameter<double>(
        "max_vy", 0.10, read_only_parameter("Immutable lateral velocity limit."));
    max_vy_ = limits.max_vy;
    limits.max_wz = declare_parameter<double>(
        "max_wz", 0.30, read_only_parameter("Immutable yaw velocity limit."));
    max_wz_ = limits.max_wz;
    limits.max_ax = declare_parameter<double>(
        "max_ax", 0.20, read_only_parameter("Immutable forward slew limit."));
    max_ax_ = limits.max_ax;
    limits.max_ay = declare_parameter<double>(
        "max_ay", 0.20, read_only_parameter("Immutable lateral slew limit."));
    max_ay_ = limits.max_ay;
    limits.max_aw = declare_parameter<double>(
        "max_aw", 0.40, read_only_parameter("Immutable yaw slew limit."));
    max_aw_ = limits.max_aw;
    limits.max_waist_yaw_rad = declare_parameter<double>(
        "max_waist_yaw_rad", 0.10,
        read_only_parameter("Immutable articulated-waist safety limit."));
    const bool timeouts_valid =
        std::isfinite(limits.candidate_timeout_sec) &&
        limits.candidate_timeout_sec > 0.0 &&
        std::isfinite(limits.platform_timeout_sec) &&
        limits.platform_timeout_sec > 0.0 &&
        std::isfinite(limits.obstacle_coverage_timeout_sec) &&
        limits.obstacle_coverage_timeout_sec > 0.0 &&
        std::isfinite(limits.localization_timeout_sec) &&
        limits.localization_timeout_sec > 0.0 &&
        std::isfinite(limits.tf_timeout_sec) && limits.tf_timeout_sec > 0.0 &&
        std::isfinite(limits.navigation_lease_timeout_sec) &&
        limits.navigation_lease_timeout_sec > 0.0;
    const auto axis_limits_valid = [](double velocity, double acceleration) {
      return std::isfinite(velocity) && velocity >= 0.0 &&
             std::isfinite(acceleration) && acceleration >= 0.0 &&
             (velocity == 0.0 || acceleration > 0.0);
    };
    const bool motion_limits_valid =
        axis_limits_valid(limits.max_vx, limits.max_ax) &&
        axis_limits_valid(limits.max_vy, limits.max_ay) &&
        axis_limits_valid(limits.max_wz, limits.max_aw) &&
        std::isfinite(limits.max_waist_yaw_rad) &&
        limits.max_waist_yaw_rad >= 0.0;
    if (!timeouts_valid || !motion_limits_valid) {
      throw std::runtime_error("safety_limits_must_be_finite_and_bounded");
    }
    if (declare_parameter("autonomous_reverse_enabled", false)) {
      throw std::runtime_error(
          "autonomous reverse remains disabled until rear coverage is accepted");
    }
    return limits;
  }

  void observe_candidate(
      const geometry_msgs::msg::Twist& input, double stamp_sec) {
    const Twist2D candidate{input.linear.x, input.linear.y, input.angular.z};
    if (bounded_attended_output_mode_ && !near_zero(candidate)) {
      if (bounded_distance_navigation_ && !qualification_nonzero_budget_.spent()) {
        const auto origin = latest_tf_observation();
        if (!origin || stale_stamp(get_clock()->now().seconds(), origin->stamp_sec, .85)) {
          enter_fault(attended_distance_prefix_ + "_origin_unavailable");
        } else {
          travel_guard_.begin(origin->x, origin->y, origin->yaw);
        }
      }
      qualification_nonzero_budget_.observe_first_nonzero(
          AttendedQualificationNonzeroBudget::Clock::now());
    }
    candidate_message_ = input;
    candidate_ = candidate;
    inputs_.candidate_seen = true;
    inputs_.candidate_stamp_sec = stamp_sec;
    last_candidate_steady_ = std::chrono::steady_clock::now();
    have_candidate_steady_ = true;
    have_candidate_ = true;
  }

  void clear_candidate_history() {
    candidate_ = {};
    candidate_message_ = geometry_msgs::msg::Twist{};
    inputs_.candidate_seen = false;
    inputs_.candidate_stamp_sec = 0.0;
    have_candidate_ = false;
    have_candidate_steady_ = false;
    last_candidate_steady_ = {};
    lease_wait_started_ = {};
  }

  void invalidate_collision_endpoint_epoch(const std::string& reason) {
    active_collision_endpoint_epoch_ = {};
    pending_collision_endpoint_epoch_ = {};
    endpoint_contract_stable_ = false;
    endpoint_contract_valid_since_ = {};
    endpoint_contract_reason_ = reason;
    clear_candidate_history();
    enter_fault(reason);
    publish_zero(false, reason);
  }

  bool typed_sender_valid(const rclcpp::MessageInfo& info,
                          const std::string& expected, const std::string& label) {
    if (!endpoint_contract_stable_ || expected.empty()) return false;
    const auto actual = endpoint_gid_hex(info.get_rmw_message_info().publisher_gid);
    if (actual != expected) {
      const auto reason = label + "_publisher_gid_changed";
      enter_fault(reason);
      publish_zero(false, reason);
      return false;
    }
    return true;
  }

  struct IngressTiming {
    std::int64_t ros_ns{0}, steady_ns{0}, rmw_source_ns{0}, rmw_received_ns{0};
    std::int64_t source_ns{0};
    std::uint64_t sequence{0}, secondary_sequence{0};
    unsigned kind{0};
    const char* outcome{"entered"};
  };
  static constexpr std::size_t kIngressHistory = 64;
  using IngressHistory = std::array<IngressTiming, kIngressHistory>;

  IngressTiming& record_ingress(IngressHistory& history, std::size_t& count,
                                const rclcpp::MessageInfo& info) {
    auto& sample = history[count++ % kIngressHistory];
    sample = IngressTiming{};
    sample.ros_ns = now().nanoseconds();
    sample.steady_ns = sunhorizon_command_evidence::monotonic_now_ns();
    sample.rmw_source_ns = info.get_rmw_message_info().source_timestamp;
    sample.rmw_received_ns = info.get_rmw_message_info().received_timestamp;
    return sample;
  }

  void log_ingress_history(const char* label, const IngressHistory& history,
                           std::size_t count) {
    // Fixed-size memory only during callbacks. Emit after the first fault's
    // zero command. RMW timestamps may be zero (unsupported), never age proof.
    std::ostringstream trace;
    trace << "geometry_ingress_first_fault channel=" << label
          << " total=" << count
          << " columns=seq,secondary,kind,source,entry_ros,entry_steady,rmw_source,rmw_received,outcome rows=";
    const auto first = count > kIngressHistory ? count - kIngressHistory : 0;
    for (std::size_t i = first; i < count; ++i) {
      const auto& s = history[i % kIngressHistory];
      trace << '[' << s.sequence << ',' << s.secondary_sequence << ',' << s.kind
            << ',' << s.source_ns << ',' << s.ros_ns << ',' << s.steady_ns
            << ',' << s.rmw_source_ns << ',' << s.rmw_received_ns << ',' << s.outcome << ']';
    }
    RCLCPP_ERROR(get_logger(), "%s", trace.str().c_str());
  }

  void on_spatial_decision(const SpatialDecision::ConstSharedPtr msg,
                           const rclcpp::MessageInfo& info) {
    auto& timing = record_ingress(coverage_ingress_, coverage_ingress_count_, info);
    const auto& decision = msg->decision;
    timing.sequence = msg->spatial.anchor_frame_sequence;
    timing.secondary_sequence = msg->spatial.evaluation_sequence;
    timing.kind = decision.disposition;
    timing.source_ns = msg->spatial.anchor_source_ros_ns;
    if (!typed_sender_valid(info, active_collision_endpoint_epoch_.coverage_gid,
                            "spatial_coverage_decision")) {
      timing.outcome = "sender_rejected"; return;
    }
    const auto& old = decision.coverage;
    if (old.header.stamp.sec < 0 || old.header.stamp.nanosec >= 1000000000U) {
      timing.outcome = "stamp_rejected";
      enter_fault("coverage_decision_stamp_encoding_invalid");
      publish_zero(false, "coverage_decision_stamp_encoding_invalid");
      return;
    }
    sunhorizon_velocity_supervisor::CombinedCoverageEvidence e;
    e.schema = decision.schema_version;
    e.disposition = decision.disposition;
    e.region_code = decision.region_evaluation_code;
    e.region_disposition = decision.region_disposition;
    e.non_region_healthy = decision.non_region_healthy;
    e.complete = decision.region_evaluation_complete;
    e.healthy = old.healthy; e.forward = old.forward_coverage;
    e.near = old.near_field_coverage; e.rear = old.rear_coverage;
    e.header_ros_ns = static_cast<std::int64_t>(old.header.stamp.sec) * 1000000000 +
        old.header.stamp.nanosec;
    e.source_ros_ns = decision.region_source_stamp_ns;
    e.frame_sequence = decision.region_frame_sequence;
    e.evaluation_sequence = decision.region_evaluation_sequence;
    e.frame_id = old.header.frame_id; e.profile = old.profile;
    e.contract = decision.region_contract;
    e.required_sources = old.required_sources; e.healthy_sources = old.healthy_sources;
    latest_coverage_ = std::move(e);
    latest_spatial_ = sunhorizon_obstacle_coverage::spatial_evidence_observation(msg->spatial);
    latest_coverage_owner_ = active_collision_endpoint_epoch_.coverage_gid;
    latest_coverage_receipt_ros_ = now().nanoseconds();
    latest_coverage_receipt_steady_ = sunhorizon_command_evidence::monotonic_now_ns();
    have_typed_coverage_ = true;
    timing.outcome = "spatial_cache_updated";
    on_obstacle_coverage(std::make_shared<ObstacleCoverageStatus>(old));
    if (spatial_tracker_ && !spatial_tracker_->observe(
        latest_coverage_, latest_spatial_, latest_coverage_owner_,
        latest_coverage_receipt_ros_, latest_coverage_receipt_steady_)) {
      const auto result = spatial_tracker_->evaluate(
          latest_coverage_receipt_ros_, latest_coverage_receipt_steady_);
      timing.outcome = "spatial_tracker_rejected";
      enter_fault(result.reason); publish_zero(false, result.reason);
    }
  }

  void on_coverage_decision(const CoverageDecision::ConstSharedPtr msg,
                            const rclcpp::MessageInfo& info) {
    auto& timing = record_ingress(coverage_ingress_, coverage_ingress_count_, info);
    timing.sequence = msg->region_frame_sequence;
    timing.secondary_sequence = msg->region_evaluation_sequence;
    timing.kind = msg->disposition;
    timing.source_ns = msg->region_source_stamp_ns;
    if (!typed_sender_valid(info, active_collision_endpoint_epoch_.coverage_gid,
                            "coverage_decision")) {
      timing.outcome = "sender_rejected"; return;
    }
    const auto& old = msg->coverage;
    if (old.header.stamp.sec < 0 || old.header.stamp.nanosec >= 1000000000U) {
      timing.outcome = "stamp_rejected";
      enter_fault("coverage_decision_stamp_encoding_invalid");
      publish_zero(false, "coverage_decision_stamp_encoding_invalid");
      return;
    }
    sunhorizon_velocity_supervisor::CombinedCoverageEvidence e;
    e.schema = msg->schema_version;
    e.disposition = msg->disposition;
    e.region_code = msg->region_evaluation_code;
    e.region_disposition = msg->region_disposition;
    e.non_region_healthy = msg->non_region_healthy;
    e.complete = msg->region_evaluation_complete;
    e.healthy = old.healthy; e.forward = old.forward_coverage;
    e.near = old.near_field_coverage; e.rear = old.rear_coverage;
    e.header_ros_ns = static_cast<std::int64_t>(old.header.stamp.sec) * 1000000000 +
        old.header.stamp.nanosec;
    e.source_ros_ns = msg->region_source_stamp_ns;
    e.frame_sequence = msg->region_frame_sequence;
    e.evaluation_sequence = msg->region_evaluation_sequence;
    e.frame_id = old.header.frame_id; e.profile = old.profile;
    e.contract = msg->region_contract;
    e.required_sources = old.required_sources; e.healthy_sources = old.healthy_sources;
    latest_coverage_ = std::move(e);
    latest_coverage_owner_ = active_collision_endpoint_epoch_.coverage_gid;
    latest_coverage_receipt_ros_ = now().nanoseconds();
    latest_coverage_receipt_steady_ = sunhorizon_command_evidence::monotonic_now_ns();
    have_typed_coverage_ = true;
    timing.outcome = "cache_updated";
    on_obstacle_coverage(std::make_shared<ObstacleCoverageStatus>(old));
    if (coverage_tracker_ && !coverage_tracker_->observe(
            latest_coverage_, latest_coverage_owner_, latest_coverage_receipt_ros_,
            latest_coverage_receipt_steady_)) {
      timing.outcome = "cache_updated_tracker_rejected";
      const auto result = coverage_tracker_->evaluate(
          latest_coverage_receipt_ros_, latest_coverage_receipt_steady_);
      enter_fault(result.reason);
      publish_zero(false, result.reason);
    }
  }

  void on_command_evidence(const CommandEvidence::ConstSharedPtr msg,
                           const rclcpp::MessageInfo& info) {
    auto& timing = record_ingress(command_ingress_, command_ingress_count_, info);
    timing.sequence = msg->command_sequence;
    timing.secondary_sequence = msg->collision_sequence;
    timing.kind = msg->generation_state;
    timing.source_ns = msg->compute_begin_ros_ns;
    if (!typed_sender_valid(info, active_collision_endpoint_epoch_.command_gid,
                            "collision_command")) {
      timing.outcome = "sender_rejected"; return;
    }
    const auto ros = now().nanoseconds();
    const auto steady = sunhorizon_command_evidence::monotonic_now_ns();
    const auto fail = [this](const std::string& reason) {
      enter_fault(reason); publish_zero(false, reason);
    };
    if (const char* reason = sunhorizon_command_evidence::validate_collision_result(
            *msg, ros, steady, static_cast<std::int64_t>(command_generation_timeout_sec_ * 1e9),
            geometry_boot_id_, base_frame_)) {
      timing.outcome = "payload_rejected";
      fail(reason); return;
    }
    if (typed_command_sequence_ != 0 &&
        (msg->controller_epoch_uuid != controller_epoch_ ||
         msg->collision_epoch_uuid != collision_epoch_ ||
         msg->command_sequence <= typed_command_sequence_ ||
         msg->collision_sequence <= typed_collision_sequence_)) {
      timing.outcome = "sequence_or_epoch_rejected";
      fail("typed_command_epoch_or_sequence_invalid"); return;
    }
    controller_epoch_ = msg->controller_epoch_uuid;
    collision_epoch_ = msg->collision_epoch_uuid;
    typed_command_sequence_ = msg->command_sequence;
    typed_collision_sequence_ = msg->collision_sequence;
    command_owner_epoch_ = geometry_boot_id_ + ":" +
        active_collision_endpoint_epoch_.controller_gid + ":" +
        active_collision_endpoint_epoch_.command_gid + ":" +
        gid_bytes_hex(controller_epoch_.data(), controller_epoch_.size()) + ":" +
        gid_bytes_hex(collision_epoch_.data(), collision_epoch_.size());
    if (msg->generation_state == sunhorizon_command_evidence::kComputed &&
        (msg->compute_begin_ros_ns <= geometry_arm_ros_ ||
         msg->compute_begin_steady_ns <= geometry_arm_steady_ ||
         !inputs_.navigation_task_active || geometry_task_id_.empty() ||
         msg->compute_begin_ros_ns <= geometry_task_ros_ ||
         msg->compute_begin_steady_ns <= geometry_task_steady_)) {
      // A current lease cannot relabel a delayed command from a previous task.
      // Keep sequence high-water marks but never turn it into a candidate.
      clear_candidate_history();
      have_typed_command_ = false;
      timing.outcome = "task_barrier_rejected";
      publish_zero(false, "waiting_for_post_task_command");
      return;
    }
    using Kind = sunhorizon_navigation_core::GeometryCommandKind;
    geometry_command_ = {
        msg->generation_state == sunhorizon_command_evidence::kComputed ?
          Kind::kController : Kind::kSyntheticZero,
        msg->command_sequence, command_owner_epoch_,
        msg->compute_begin_ros_ns, msg->compute_end_ros_ns,
        msg->compute_begin_steady_ns, msg->compute_end_steady_ns, steady};
    have_typed_command_ = true;
    geometry_command_receipt_ros_ns_ = ros;
    timing.outcome = "cache_updated";
    // The safety timeout starts at generation, never collision processing.
    observe_candidate(msg->velocity, static_cast<double>(msg->compute_begin_ros_ns) * 1e-9);
  }

  bool update_geometry_recovery(double ros_seconds) {
    const auto fail = [this](const std::string& reason) {
      enter_fault(reason); publish_zero(false, reason); return false;
    };
    if ((!coverage_tracker_ && !spatial_tracker_) || !recovery_gate_)
      return fail("geometry_tracker_not_armed");
    const auto ros = now().nanoseconds();
    const auto steady = sunhorizon_command_evidence::monotonic_now_ns();
    const auto coverage = spatial_visibility_enabled_ ? spatial_tracker_->evaluate(ros, steady) :
        coverage_tracker_->evaluate(ros, steady);
    if (!coverage.valid) return fail(coverage.reason);
    // A held task must retain its exact active lease. Normal terminal idle may
    // publish zero; a subsequent task needs the normal explicit idle reset.
    if (!inputs_.navigation_task_active || navigation_task_id_.empty()) {
      if (geometry_holding_ || coverage.disposition == 2) {
        return fail("geometry_hold_execution_lease_inactive");
      }
      inputs_.geometry_hold_requested = false;
      return true;
    }
    if (!geometry_task_id_.empty() && geometry_task_id_ != navigation_task_id_) {
      return fail("geometry_navigation_task_changed");
    }
    if (geometry_task_id_.empty()) geometry_task_id_ = navigation_task_id_;
    if (!have_typed_command_ || !have_candidate_) {
      if (geometry_holding_ || coverage.disposition == 2) {
        return fail("candidate_missing_during_geometry_hold");
      }
      return true;  // Existing startup grace, still zero; no recovery authority.
    }
    sunhorizon_navigation_core::GeometryRecoveryInputs in;
    in.now_ros_ns = ros; in.now_steady_ns = steady;
    in.hard_fault = gate_.latched();
    in.task_budget_valid = inputs_.navigation_status_seen &&
        !stale_stamp(ros_seconds, inputs_.navigation_status_stamp_sec,
                     navigation_lease_timeout_sec_);
    in.task_id = navigation_task_id_;
    in.producer_epoch = coverage.owner_epoch;
    in.coverage_contract = coverage.contract;
    in.command_owner_epoch = command_owner_epoch_;
    in.coverage_seen = true;
    in.coverage = {
        coverage.disposition == 1 ? sunhorizon_navigation_core::GeometryCoverageClass::kClear :
                                    sunhorizon_navigation_core::GeometryCoverageClass::kGeometryHold,
        coverage.source_ros_ns, coverage.receipt_steady_ns,
        coverage.frame_sequence, coverage.evaluation_sequence};
    in.command_seen = true; in.command = geometry_command_;
    const auto recovery = recovery_gate_->evaluate(in);
    if (recovery.fault) {
      // Emit a first-fault snapshot only AFTER latching and publishing zero.
      // This preserves the exact evidence evaluated, not later queued inputs.
      const auto previous_state = state_;
      fail("geometry_recovery:" + recovery.reason);
      if (recovery.reason == "candidate_stale_or_future") {
        std::ostringstream snapshot;
        snapshot << "geometry_candidate_first_fault"
                 << " now_ros_ns=" << in.now_ros_ns
                 << " now_steady_ns=" << in.now_steady_ns
                 << " timeout_ns=" << static_cast<std::int64_t>(command_generation_timeout_sec_ * 1e9)
                 << " begin_ros_ns=" << in.command.compute_begin_ros_ns
                 << " end_ros_ns=" << in.command.compute_end_ros_ns
                 << " begin_steady_ns=" << in.command.compute_begin_steady_ns
                 << " end_steady_ns=" << in.command.compute_end_steady_ns
                 << " receipt_steady_ns=" << in.command.receipt_steady_ns
                 << " receipt_ros_ns=" << geometry_command_receipt_ros_ns_
                 << " sequence=" << in.command.sequence
                 << " kind=" << static_cast<int>(in.command.kind)
                 << " previous_state=" << static_cast<int>(previous_state)
                 << " holding=" << geometry_holding_
                 << " task=" << in.task_id;
        RCLCPP_ERROR(get_logger(), "%s", snapshot.str().c_str());
        log_ingress_history("command", command_ingress_, command_ingress_count_);
        log_ingress_history("coverage", coverage_ingress_, coverage_ingress_count_);
      }
      return false;
    }
    geometry_holding_ = recovery.hold;
    inputs_.geometry_hold_requested = recovery.hold;
    return true;
  }

  void on_collision_audit(
      const geometry_msgs::msg::TwistStamped::ConstSharedPtr input,
      const rclcpp::MessageInfo& message_info) {
    if (!endpoint_contract_stable_ ||
        active_collision_endpoint_epoch_.audit_gid.empty()) {
      clear_candidate_history();
      return;
    }
    const auto message_gid = endpoint_gid_hex(
        message_info.get_rmw_message_info().publisher_gid);
    if (message_gid != active_collision_endpoint_epoch_.audit_gid) {
      invalidate_collision_endpoint_epoch("collision_audit_message_gid_changed");
      return;
    }

    const auto audit_class = parse_collision_audit_frame_id(input->header.frame_id);
    if (audit_class == CollisionAuditClass::kInvalid) {
      const std::string reason = "collision_audit_state_invalid";
      enter_fault(reason);
      publish_zero(false, reason);
      return;
    }
    observe_candidate(input->twist, stamp_seconds(input->header.stamp));
    if (collision_audit_requires_immediate_latch(audit_class)) {
      const std::string reason = audit_class == CollisionAuditClass::kStop
          ? "nav2_collision_stop"
          : "nav2_collision_source_invalid";
      enter_fault(reason);
      publish_zero(false, reason);
    }
  }

  void on_qualification_candidate(
      const geometry_msgs::msg::Twist::SharedPtr input) {
    // The dedicated attended probe publishes an unstamped Twist directly. Its
    // identity is enforced continuously by endpoint_contract_failure(), while
    // steady-clock receipt freshness provides the command timeout backstop.
    observe_candidate(*input, get_clock()->now().seconds());
  }

  bool candidate_stale_by_steady_clock() const {
    if (!have_candidate_steady_) return true;
    const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_candidate_steady_).count();
    return !std::isfinite(elapsed) || elapsed > candidate_timeout_sec_;
  }

  void on_platform(const PlatformSafetyState::SharedPtr input) {
    inputs_.platform_stamp_sec = stamp_seconds(input->header.stamp);
    inputs_.mode_allowed = input->mode_allowed;
    inputs_.ethercat_op = input->ethercat_op;
    inputs_.wkc_normal = input->wkc_normal;
    inputs_.joint_feedback_fresh = input->joint_feedback_fresh;
    inputs_.leg_joints_error_free = input->leg_joints_error_free;
    inputs_.leg_joints_operation_enabled = input->leg_joints_operation_enabled;
    inputs_.motion_ready = input->motion_ready;
    inputs_.navigation_mode_discovered = input->navigation_mode_discovered;
    inputs_.platform_fault = input->platform_fault;
    inputs_.waist_yaw_rad = input->waist_yaw_rad;
    have_platform_ = true;
  }

  void on_localization(const LocalizationStatus::SharedPtr input) {
    inputs_.localization_stamp_sec = stamp_seconds(input->header.stamp);
    inputs_.localization_healthy = input->state == LocalizationStatus::STATE_HEALTHY;
    have_localization_ = true;
  }

  void on_obstacle_coverage(const ObstacleCoverageStatus::SharedPtr input) {
    inputs_.obstacle_coverage_seen = true;
    inputs_.obstacle_coverage_stamp_sec = stamp_seconds(input->header.stamp);
    inputs_.obstacle_coverage_healthy = input->healthy;
    inputs_.forward_coverage = input->forward_coverage;
    inputs_.near_field_coverage = input->near_field_coverage;
    inputs_.rear_coverage = input->rear_coverage;
    have_obstacle_coverage_ = true;
  }

  void on_navigation_status(const NavigationStatus::SharedPtr input,
                            const rclcpp::MessageInfo& info) {
    if (geometry_hold_enabled_ && !typed_sender_valid(
            info, active_collision_endpoint_epoch_.navigation_gid, "navigation_status")) return;
    navigation_task_id_ = input->task_id;
    inputs_.navigation_status_seen = true;
    inputs_.navigation_status_stamp_sec = stamp_seconds(input->header.stamp);
    inputs_.navigation_task_active =
        input->state == NavigationStatus::STATE_NAVIGATING && !input->task_id.empty();
    if (geometry_hold_enabled_ && inputs_.navigation_task_active) {
      if (!geometry_task_id_.empty() && geometry_task_id_ != input->task_id) {
        enter_fault("geometry_navigation_task_changed");
        publish_zero(false, "geometry_navigation_task_changed");
      } else if (geometry_task_id_.empty()) {
        geometry_task_id_ = input->task_id;
        geometry_task_ros_ = now().nanoseconds();
        geometry_task_steady_ = sunhorizon_command_evidence::monotonic_now_ns();
        clear_candidate_history();
        have_typed_command_ = false;
      }
    }
    navigation_resettable_without_task_ = input->task_id.empty() &&
        (input->state == NavigationStatus::STATE_READY ||
         input->state == NavigationStatus::STATE_STOPPED ||
         input->state == NavigationStatus::STATE_FAULT);
    have_navigation_status_ = true;
  }

  std::optional<PlanarTfObservation> latest_tf_observation() {
    try {
      const auto transform = tf_buffer_.lookupTransform(
          map_frame_, base_frame_, tf2::TimePointZero);
      const double stamp_sec = stamp_seconds(transform.header.stamp);
      const double x = transform.transform.translation.x;
      const double y = transform.transform.translation.y;
      const double z = transform.transform.translation.z;
      const auto& rotation = transform.transform.rotation;
      if (!std::isfinite(stamp_sec) || !std::isfinite(x) ||
          !std::isfinite(y) || !std::isfinite(z) ||
          !std::isfinite(rotation.x) || !std::isfinite(rotation.y) ||
          !std::isfinite(rotation.z) || !std::isfinite(rotation.w)) {
        return std::nullopt;
      }
      tf2::Quaternion quaternion;
      tf2::fromMsg(rotation, quaternion);
      const double quaternion_norm_squared = quaternion.length2();
      if (!std::isfinite(quaternion_norm_squared) ||
          quaternion_norm_squared <= 0.0) {
        return std::nullopt;
      }
      quaternion.normalize();
      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
      if (!std::isfinite(roll) || !std::isfinite(pitch) ||
          !std::isfinite(yaw)) {
        return std::nullopt;
      }
      return PlanarTfObservation{stamp_sec, x, y, yaw};
    } catch (const tf2::TransformException&) {
      return std::nullopt;
    }
  }

  void update_tf(double now) {
    const auto observation = latest_tf_observation();
    if (observation) {
      const auto steady = double(sunhorizon_command_evidence::monotonic_now_ns())*1e-9;
      const bool progressing = tf_progress_.observe(*observation, now, steady) &&
          tf_progress_.current(now, steady, tf_watchdog_timeout_sec_);
      // SafetyGate receives the original observation-of-progress time, never
      // a fresh timestamp for polling the same cached TF. Source stays intact.
      inputs_.tf_stamp_sec = tf_progress_.receipt_ros_sec();
      const bool continuous = tf_continuity_.observe(
          *observation, now, max_tf_translation_jump_m_, max_tf_yaw_jump_rad_);
      inputs_.tf_continuous = progressing && continuous;
    } else {
      inputs_.tf_stamp_sec = 0.0;
      inputs_.tf_continuous = false;
    }
  }

  void try_connect_output() {
    if (!motion_output_requested_ || motion_output_connected_ || output_connection_refused_) return;
    if ((get_clock()->now() - startup_time_).seconds() < publisher_discovery_grace_sec_) return;
    const auto publishers = count_publishers(final_topic_);
    if (publishers != 0U) {
      output_connection_refused_ = true;
      output_connection_reason_ = "existing_final_velocity_publisher:" + std::to_string(publishers);
      gate_.force_latch(output_connection_reason_);
      RCLCPP_FATAL(get_logger(), "%s", output_connection_reason_.c_str());
      return;
    }
    final_pub_ = create_publisher<geometry_msgs::msg::Twist>(final_topic_, 10);
    motion_output_connected_ = true;
    output_connection_reason_ = "connected";
    RCLCPP_WARN(get_logger(), "Final velocity output connected to %s", final_topic_.c_str());
  }

  static std::string endpoint_fqn(const rclcpp::TopicEndpointInfo& endpoint) {
    const auto name = endpoint.node_name();
    auto name_space = endpoint.node_namespace();
    if (name_space.empty() || name_space == "/") return "/" + name;
    if (name_space.front() != '/') name_space.insert(name_space.begin(), '/');
    if (name_space.back() == '/') name_space.pop_back();
    return name_space + "/" + name;
  }

  std::string endpoint_contract_failure(CollisionEndpointEpoch* observed_epoch) {
    *observed_epoch = {};
    const auto candidates = get_publishers_info_by_topic(candidate_topic_);
    if (candidates.size() != 1U) {
      return "candidate_publisher_count:" + std::to_string(candidates.size());
    }
    const auto candidate_owner = endpoint_fqn(candidates.front());
    if (candidate_owner != expected_candidate_publisher_fqn_) {
      return "candidate_publisher_owner:" + candidate_owner;
    }
    observed_epoch->candidate_gid = endpoint_gid_hex(
        candidates.front().endpoint_gid());
    if (observed_epoch->candidate_gid.find_first_not_of('0') == std::string::npos) {
      return "candidate_publisher_gid_invalid";
    }
    const auto audits = get_publishers_info_by_topic(collision_audit_topic_);
    if (direct_qualification_candidate_mode_) {
      if (!audits.empty()) {
        return "qualification_collision_audit_publisher_count:" +
            std::to_string(audits.size());
      }
    } else {
      if (audits.size() != 1U) {
        return "collision_audit_publisher_count:" + std::to_string(audits.size());
      }
      const auto audit_owner = endpoint_fqn(audits.front());
      if (audit_owner != expected_candidate_publisher_fqn_) {
        return "collision_audit_publisher_owner:" + audit_owner;
      }
      observed_epoch->audit_gid = endpoint_gid_hex(audits.front().endpoint_gid());
      if (observed_epoch->audit_gid.find_first_not_of('0') == std::string::npos) {
        return "collision_audit_publisher_gid_invalid";
      }
    }

    if (geometry_hold_enabled_) {
      const auto require_owner = [this](const std::string& topic, const std::string& owner,
                                       std::string& gid) -> std::string {
        const auto peers = get_publishers_info_by_topic(topic);
        if (peers.size() != 1 || endpoint_fqn(peers.front()) != owner ||
            peers.front().topic_type() != (topic == "/sunhorizon/navigation/status" ?
              "sunhorizon_navigation_interfaces/msg/NavigationStatus" :
              topic == kSpatialDecisionTopic ?
              "sunhorizon_navigation_interfaces/msg/ObstacleCoverageSpatialDecision" :
              topic == kCoverageDecisionTopic ?
              "sunhorizon_navigation_interfaces/msg/ObstacleCoverageDecision" :
              "sunhorizon_navigation_interfaces/msg/VelocityCommandEvidence")) {
          return "typed_publisher_contract:" + topic;
        }
        gid = endpoint_gid_hex(peers.front().endpoint_gid());
        if (!sunhorizon_command_evidence::valid_gid(gid)) return "typed_publisher_gid_invalid";
        return {};
      };
      for (const auto& failure : {
          require_owner(spatial_visibility_enabled_ ? kSpatialDecisionTopic : kCoverageDecisionTopic,
              "/sunhorizon_obstacle_coverage", observed_epoch->coverage_gid),
          require_owner(kControllerTopic, "/controller_server", observed_epoch->controller_gid),
          require_owner(kCommandTopic, "/collision_monitor", observed_epoch->command_gid),
          require_owner("/sunhorizon/navigation/status", "/sunhorizon_navigation_server", observed_epoch->navigation_gid)}) {
        if (!failure.empty()) return failure;
      }
    }
    const auto finals = get_publishers_info_by_topic(final_topic_);
    if (!motion_output_requested_) {
      if (!finals.empty()) {
        return "shadow_final_publisher_count:" + std::to_string(finals.size());
      }
      return {};
    }
    if (!motion_output_connected_) return "final_output_not_connected";
    if (finals.size() != 1U) {
      return "final_publisher_count:" + std::to_string(finals.size());
    }
    const auto final_owner = endpoint_fqn(finals.front());
    if (final_owner != get_fully_qualified_name()) {
      return "final_publisher_owner:" + final_owner;
    }
    return {};
  }

  void update_endpoint_contract() {
    CollisionEndpointEpoch observed_epoch;
    const auto failure = endpoint_contract_failure(&observed_epoch);
    const auto now = std::chrono::steady_clock::now();
    if (!failure.empty()) {
      if (!active_collision_endpoint_epoch_.empty()) {
        invalidate_collision_endpoint_epoch(
            "publisher_endpoint_epoch_invalid:" + failure);
        return;
      }
      endpoint_contract_stable_ = false;
      pending_collision_endpoint_epoch_ = {};
      endpoint_contract_valid_since_ = {};
      endpoint_contract_reason_ = failure;
      return;
    }
    if (!active_collision_endpoint_epoch_.empty()) {
      if (observed_epoch != active_collision_endpoint_epoch_) {
        invalidate_collision_endpoint_epoch("publisher_endpoint_epoch_changed");
        pending_collision_endpoint_epoch_ = observed_epoch;
        endpoint_contract_valid_since_ = now;
        return;
      }
      endpoint_contract_stable_ = true;
      endpoint_contract_reason_ = "publisher_ownership_stable";
      return;
    }
    if (pending_collision_endpoint_epoch_ != observed_epoch ||
        endpoint_contract_valid_since_.time_since_epoch().count() == 0) {
      pending_collision_endpoint_epoch_ = observed_epoch;
      endpoint_contract_valid_since_ = now;
    }
    const auto elapsed = std::chrono::duration<double>(
        now - endpoint_contract_valid_since_).count();
    endpoint_contract_stable_ = elapsed >= endpoint_stability_sec_;
    if (endpoint_contract_stable_) {
      active_collision_endpoint_epoch_ = observed_epoch;
    }
    endpoint_contract_reason_ = endpoint_contract_stable_ ?
        "publisher_ownership_stable" : "publisher_ownership_stabilizing";
  }

  void publish_status(
      bool allowed, const std::string& reason, const geometry_msgs::msg::Twist& supervised) {
    SupervisorStatus status;
    status.header.stamp = get_clock()->now();
    status.header.frame_id = base_frame_;
    status.state = static_cast<uint8_t>(state_);
    status.motion_output_connected = motion_output_connected_;
    status.candidate_seen = have_candidate_;
    status.command_allowed = allowed;
    status.latched = gate_.latched();
    status.reason = reason;
    status.candidate = candidate_message_;
    status.supervised = supervised;
    status_pub_->publish(status);
  }

  void enter_fault(const std::string& reason) {
    state_ = SupervisorState::kFaultLatched;
    gate_.force_latch(reason);
  }

  void publish_zero(bool allowed, const std::string& reason) {
    const geometry_msgs::msg::Twist zero;
    if (motion_output_connected_) final_pub_->publish(zero);
    publish_status(allowed, reason, zero);
  }

  geometry_msgs::msg::Twist adapt_for_vendor(
      const geometry_msgs::msg::Twist& command) const {
    auto output = command;
    output.linear.x *= vendor_vx_gain_;
    return output;
  }

  SafetyInputs safety_inputs_for_authority(double now) const {
    SafetyInputs result = inputs_;
    if (direct_qualification_candidate_mode_) {
      // The exclusive, receipt-producing attended probe is the execution
      // authority in this mode. It replaces only the Nav2 task lease; platform,
      // localization, obstacle, TF, candidate freshness, and command bounds
      // remain enforced by the same SafetyGate.
      result.navigation_status_seen = true;
      result.navigation_status_stamp_sec = now;
      result.navigation_task_active = true;
    }
    return result;
  }

  void on_control() {
    try_connect_output();
    update_endpoint_contract();
    const double now = get_clock()->now().seconds();
    inputs_.now_sec = now;
    update_tf(now);

    if (bounded_distance_navigation_ && qualification_nonzero_budget_.spent()) {
      const auto current = latest_tf_observation();
      const auto failure = current ? travel_guard_.observe(current->x, current->y)
                                   : attended_distance_prefix_ + "_travel_pose_unavailable";
      if (!failure.empty()) {
        enter_fault(failure);
        publish_zero(false, failure);
        return;
      }
    }
    if (bounded_attended_output_mode_ &&
        qualification_nonzero_budget_.expired(
            AttendedQualificationNonzeroBudget::Clock::now())) {
      const std::string reason = qualification_budget_latch_reason_;
      enter_fault(reason);
      publish_zero(false, reason);
      return;
    }

    if (output_connection_refused_) {
      enter_fault(output_connection_reason_);
      publish_zero(false, output_connection_reason_);
      return;
    }

    if (state_ == SupervisorState::kFaultLatched) {
      publish_zero(false, gate_.latch_reason());
      return;
    }

    if (state_ == SupervisorState::kDisarmed) {
      publish_zero(false, endpoint_contract_stable_ ? "disarmed" : endpoint_contract_reason_);
      return;
    }

    if (!endpoint_contract_stable_) {
      const std::string reason = "publisher_contract:" + endpoint_contract_reason_;
      enter_fault(reason);
      publish_zero(false, reason);
      return;
    }

    // This wall/steady-clock backstop is deliberately independent of ROS time.
    // SafetyGate still validates stamped platform/localization/TF evidence, while
    // this branch guarantees that a frozen or rewound ROS clock cannot keep a
    // stale candidate command alive.
    if (have_candidate_ && candidate_stale_by_steady_clock()) {
      if (state_ == SupervisorState::kRunning || geometry_holding_ || !near_zero(candidate_)) {
        const std::string reason = "candidate_stale_steady_clock";
        enter_fault(reason);
        publish_zero(false, reason);
        return;
      }
      // Nav2 and collision_monitor normally stop publishing after a bounded
      // zero window.  A stale *zero* while already idle is not a motion fault.
      have_candidate_ = false;
      have_candidate_steady_ = false;
      inputs_.candidate_seen = false;
    }

    if (!have_platform_ || !have_localization_ || !have_obstacle_coverage_) {
      std::string reason = "waiting_for:";
      if (!have_platform_) reason += " platform";
      if (!have_localization_) reason += " localization";
      if (!have_obstacle_coverage_) reason += " obstacle_coverage";
      enter_fault(reason);
      publish_zero(false, reason);
      return;
    }

    if (geometry_hold_enabled_ && !update_geometry_recovery(now)) return;

    if (!have_candidate_) {
      lease_wait_started_ = {};
      SafetyInputs idle_inputs = safety_inputs_for_authority(now);
      idle_inputs.candidate_seen = true;
      idle_inputs.candidate_stamp_sec = now;
      const auto idle_decision = gate_.evaluate(idle_inputs, {});
      if (idle_decision.latched) {
        state_ = SupervisorState::kFaultLatched;
        publish_zero(false, idle_decision.reason);
        return;
      }
      state_ = SupervisorState::kArmedIdle;
      publish_zero(false, "armed_idle");
      return;
    }

    const bool navigation_lease_valid = direct_qualification_candidate_mode_ ||
        (inputs_.navigation_status_seen && inputs_.navigation_task_active &&
         !stale_stamp(
             now, inputs_.navigation_status_stamp_sec,
             navigation_lease_timeout_sec_));
    if (!near_zero(candidate_) && !navigation_lease_valid &&
        state_ == SupervisorState::kArmedIdle && !geometry_holding_) {
      const auto steady_now = std::chrono::steady_clock::now();
      if (lease_wait_started_.time_since_epoch().count() == 0) {
        lease_wait_started_ = steady_now;
      }
      const auto elapsed = std::chrono::duration<double>(
          steady_now - lease_wait_started_).count();
      if (elapsed <= lease_acquisition_grace_sec_) {
        publish_zero(false, "waiting_for_navigation_execution_lease");
        return;
      }
    } else {
      lease_wait_started_ = {};
    }

    const auto decision = gate_.evaluate(
        safety_inputs_for_authority(now), candidate_);
    const auto output = adapt_for_vendor(to_message(decision.output));
    if (decision.latched) {
      state_ = SupervisorState::kFaultLatched;
    } else {
      state_ = (decision.geometry_hold || !near_zero(candidate_)) ?
          SupervisorState::kRunning : SupervisorState::kArmedIdle;
    }
    if (motion_output_connected_) final_pub_->publish(output);
    publish_status(decision.allowed, decision.reason, output);
  }

  void on_reset(
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
    try_connect_output();
    update_endpoint_contract();
    if (motion_output_requested_ && !motion_output_connected_) {
      response->success = false;
      response->message = "final_output_must_be_connected_before_reset";
      return;
    }
    if (!endpoint_contract_stable_) {
      response->success = false;
      response->message = "publisher_contract_not_stable:" + endpoint_contract_reason_;
      return;
    }
    if (bounded_attended_output_mode_ &&
        !qualification_nonzero_budget_.reset_allowed()) {
      response->success = false;
      response->message = qualification_budget_spent_reason_;
      return;
    }
    const double reset_now = get_clock()->now().seconds();
    if (!direct_qualification_candidate_mode_) {
      if (!have_navigation_status_) {
        response->success = false;
        response->message = "navigation_status_missing_before_reset";
        return;
      }
      if (stale_stamp(
              reset_now, inputs_.navigation_status_stamp_sec,
              navigation_lease_timeout_sec_)) {
        response->success = false;
        response->message = "navigation_status_stale_before_reset";
        return;
      }
      if (!navigation_resettable_without_task_) {
        response->success = false;
        response->message = "navigation_must_be_idle_without_task_before_reset";
        return;
      }
    }
    if (have_candidate_ && candidate_stale_by_steady_clock()) {
      candidate_ = {};
      candidate_message_ = geometry_msgs::msg::Twist{};
      inputs_.candidate_seen = false;
      have_candidate_ = false;
      have_candidate_steady_ = false;
    }
    if (have_candidate_ && !near_zero(candidate_)) {
      response->success = false;
      response->message = "candidate_must_be_zero_before_reset";
      return;
    }
    std::unique_ptr<sunhorizon_velocity_supervisor::CoverageDecisionTracker> proposed_coverage;
    std::unique_ptr<sunhorizon_velocity_supervisor::SpatialCoverageDecisionTracker> proposed_spatial;
    if (geometry_hold_enabled_) {
      if (!have_typed_coverage_) {
        response->success = false;
        response->message = "geometry_coverage_missing_before_reset";
        return;
      }
      const auto steady = sunhorizon_command_evidence::monotonic_now_ns();
      bool valid_clear = false;
      if (spatial_visibility_enabled_) {
        auto config = spatial_config_;
        config.owner_epoch = active_collision_endpoint_epoch_.coverage_gid;
        proposed_spatial = std::make_unique<sunhorizon_velocity_supervisor::SpatialCoverageDecisionTracker>(
            config, spatial_obstacle_source_mode_, coverage_report_age_ns_);
        valid_clear = proposed_spatial->observe(latest_coverage_, latest_spatial_, latest_coverage_owner_,
            latest_coverage_receipt_ros_, latest_coverage_receipt_steady_) &&
            proposed_spatial->evaluate(now().nanoseconds(), steady).valid &&
            latest_coverage_.disposition == 1;
      } else {
        proposed_coverage = std::make_unique<sunhorizon_velocity_supervisor::CoverageDecisionTracker>(
            active_collision_endpoint_epoch_.coverage_gid, geometry_contract_, base_frame_);
        valid_clear = proposed_coverage->observe(latest_coverage_, latest_coverage_owner_,
            latest_coverage_receipt_ros_, latest_coverage_receipt_steady_) &&
            proposed_coverage->evaluate(now().nanoseconds(), steady).valid &&
            latest_coverage_.disposition == 1;
      }
      if (!valid_clear) {
        response->success = false;
        response->message = "geometry_coverage_must_be_fresh_clear_before_reset";
        return;
      }
    }
    inputs_.now_sec = get_clock()->now().seconds();
    const auto reset_tf_observation = latest_tf_observation();
    TfContinuityTracker proposed_tf_continuity = tf_continuity_;
    TfProgressTracker proposed_tf_progress = tf_progress_;
    const auto reset_steady = double(sunhorizon_command_evidence::monotonic_now_ns())*1e-9;
    const bool reset_tf_valid = reset_tf_observation &&
        proposed_tf_progress.observe(*reset_tf_observation, inputs_.now_sec, reset_steady) &&
        proposed_tf_progress.current(inputs_.now_sec, reset_steady, tf_watchdog_timeout_sec_) &&
        proposed_tf_continuity.rebaseline(
            *reset_tf_observation, inputs_.now_sec);
    // Arming is intentionally permitted before Nav2 has produced its first
    // candidate.  All non-candidate safety evidence must nevertheless be
    // present and fresh. An explicit reset may establish a new TF continuity
    // epoch after relocalization. Missing/invalid TF or a stopped original
    // source still fails the progress watchdog; resetting cannot renew it. Candidate
    // history is cleared so an old command can never become effective.
    SafetyInputs reset_inputs = safety_inputs_for_authority(inputs_.now_sec);
    if (proposed_coverage || proposed_spatial) reset_inputs.geometry_hold_requested = false;
    reset_inputs.candidate_seen = false;
    reset_inputs.tf_stamp_sec = reset_tf_valid ?
        proposed_tf_progress.receipt_ros_sec() : 0.0;
    reset_inputs.tf_continuous = reset_tf_valid;
    response->success = gate_.reset(reset_inputs, {}, &response->message);
    if (!response->success) return;
    if (geometry_hold_enabled_) {
      coverage_tracker_ = std::move(proposed_coverage);
      spatial_tracker_ = std::move(proposed_spatial);
      sunhorizon_navigation_core::GeometryRecoveryConfig config;
      config.command_timeout_ns = static_cast<std::int64_t>(command_generation_timeout_sec_ * 1e9);
      recovery_gate_ = std::make_unique<sunhorizon_navigation_core::GeometryRecoveryGate>(config);
      geometry_holding_ = false;
      inputs_.geometry_hold_requested = false;
      geometry_task_id_.clear();
      geometry_arm_ros_ = now().nanoseconds();
      geometry_arm_steady_ = sunhorizon_command_evidence::monotonic_now_ns();
      geometry_task_ros_ = geometry_task_steady_ = 0;
      typed_command_sequence_ = typed_collision_sequence_ = 0;
      controller_epoch_ = {}; collision_epoch_ = {};
      command_owner_epoch_.clear();
      have_typed_command_ = false;
    }
    tf_continuity_ = proposed_tf_continuity;
    tf_progress_ = proposed_tf_progress;
    inputs_.tf_stamp_sec = tf_progress_.receipt_ros_sec();
    inputs_.tf_continuous = true;
    candidate_ = {};
    candidate_message_ = geometry_msgs::msg::Twist{};
    inputs_.candidate_seen = false;
    have_candidate_ = false;
    have_candidate_steady_ = false;
    state_ = SupervisorState::kArmedIdle;
    response->message = "armed_idle";
  }

  void on_stop(
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
    enter_fault("operator_stop");
    if (motion_output_connected_) final_pub_->publish(geometry_msgs::msg::Twist{});
    response->success = true;
    response->message = "zero_velocity_latched";
  }

  static bool stale_stamp(double now, double stamp, double timeout) {
    return stamp <= 0.0 || now < stamp || now - stamp > timeout;
  }
  bool bounded_distance_navigation_{false};
  std::string attended_distance_prefix_;
  sunhorizon_velocity_supervisor::AttendedTravelGuard travel_guard_;
  std::string candidate_topic_;
  std::string collision_audit_topic_;
  std::string final_topic_;
  std::string map_frame_;
  std::string base_frame_;
  std::string final_output_lock_path_;
  int final_output_lock_fd_{-1};
  double control_rate_hz_{50.0};
  double candidate_timeout_sec_{0.25};
  double command_generation_timeout_sec_{0.25};
  double publisher_discovery_grace_sec_{2.0};
  double max_tf_translation_jump_m_{0.30};
  double max_tf_yaw_jump_rad_{0.25};
  std::int64_t coverage_report_age_ns_{300000000};
  double navigation_lease_timeout_sec_{0.50};
  double endpoint_stability_sec_{0.50};
  double lease_acquisition_grace_sec_{0.10};
  double max_vx_{0.20};
  double max_vy_{0.10};
  double max_wz_{0.30};
  double max_ax_{0.20};
  double max_ay_{0.20};
  double max_aw_{0.40};
  std::string expected_candidate_publisher_fqn_{"/collision_monitor"};
  std::string motion_authority_mode_{
      sunhorizon_velocity_supervisor::kReceiptGatedNavigationMode};
  std::string qualification_profile_acknowledgement_;
  bool direct_qualification_candidate_mode_{false};
  bool bounded_attended_output_mode_{false};
  std::string qualification_budget_latch_reason_{
      kAttendedQualificationBudgetExhaustedReason};
  std::string qualification_budget_spent_reason_{
      "attended_five_cm_qualification_nonzero_budget_spent"};
  AttendedQualificationNonzeroBudget qualification_nonzero_budget_;
  bool motion_output_requested_{false};
  bool velocity_calibration_accepted_{false};
  bool vendor_watchdog_accepted_{false};
  double vendor_vx_gain_{1.0};
  double vendor_vx_deadzone_command_{0.0};
  bool motion_output_connected_{false};
  bool output_connection_refused_{false};
  std::string output_connection_reason_{"shadow_mode"};
  SupervisorState state_{SupervisorState::kDisarmed};
  bool geometry_hold_enabled_{false};
  bool spatial_visibility_enabled_{false};
  std::string spatial_obstacle_source_mode_;
  sunhorizon_obstacle_coverage::SpatialEvidenceTrackerConfig spatial_config_;
  sunhorizon_obstacle_coverage::SpatialCoverageObservation latest_spatial_;
  std::unique_ptr<sunhorizon_velocity_supervisor::SpatialCoverageDecisionTracker> spatial_tracker_;
  IngressHistory command_ingress_{}, coverage_ingress_{};
  std::size_t command_ingress_count_{0}, coverage_ingress_count_{0};
  bool geometry_holding_{false}, have_typed_coverage_{false}, have_typed_command_{false};
  std::int64_t geometry_command_receipt_ros_ns_{0};
  std::string geometry_contract_, geometry_boot_id_, geometry_task_id_, navigation_task_id_;
  std::int64_t geometry_arm_ros_{0}, geometry_arm_steady_{0};
  std::int64_t geometry_task_ros_{0}, geometry_task_steady_{0};
  std::string latest_coverage_owner_, command_owner_epoch_;
  sunhorizon_velocity_supervisor::CombinedCoverageEvidence latest_coverage_;
  std::int64_t latest_coverage_receipt_ros_{0}, latest_coverage_receipt_steady_{0};
  sunhorizon_navigation_core::GeometryCommandEvidence geometry_command_;
  sunhorizon_command_evidence::Uuid controller_epoch_{}, collision_epoch_{};
  std::uint64_t typed_command_sequence_{0}, typed_collision_sequence_{0};
  std::unique_ptr<sunhorizon_velocity_supervisor::CoverageDecisionTracker> coverage_tracker_;
  std::unique_ptr<sunhorizon_navigation_core::GeometryRecoveryGate> recovery_gate_;
  static constexpr const char* kSpatialDecisionTopic = "/sunhorizon/obstacle_coverage/spatial_decision_v2";
  static constexpr const char* kCoverageDecisionTopic = "/sunhorizon/obstacle_coverage/decision_v1";
  static constexpr const char* kControllerTopic = "/sunhorizon/navigation/controller_command_v1";
  static constexpr const char* kCommandTopic = "/sunhorizon/navigation/collision_command_v1";
  SafetyGate gate_;
  SafetyInputs inputs_;
  CollisionEndpointEpoch active_collision_endpoint_epoch_;
  CollisionEndpointEpoch pending_collision_endpoint_epoch_;
  Twist2D candidate_;
  geometry_msgs::msg::Twist candidate_message_;
  bool have_candidate_{false};
  bool have_candidate_steady_{false};
  bool have_platform_{false};
  bool have_localization_{false};
  bool have_obstacle_coverage_{false};
  bool have_navigation_status_{false};
  bool navigation_resettable_without_task_{false};
  bool endpoint_contract_stable_{false};
  std::string endpoint_contract_reason_{"publisher_ownership_unchecked"};
  std::chrono::steady_clock::time_point endpoint_contract_valid_since_{};
  std::chrono::steady_clock::time_point lease_wait_started_{};
  TfContinuityTracker tf_continuity_;
  TfProgressTracker tf_progress_;
  double tf_watchdog_timeout_sec_{0.25};
  std::chrono::steady_clock::time_point last_candidate_steady_{};
  rclcpp::Time startup_time_{0, 0, RCL_ROS_TIME};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr final_pub_;
  rclcpp::Publisher<SupervisorStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr collision_audit_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr qualification_candidate_sub_;
  rclcpp::Subscription<PlatformSafetyState>::SharedPtr platform_sub_;
  rclcpp::Subscription<LocalizationStatus>::SharedPtr localization_sub_;
  rclcpp::Subscription<ObstacleCoverageStatus>::SharedPtr obstacle_coverage_sub_;
  rclcpp::Subscription<NavigationStatus>::SharedPtr navigation_status_sub_;
  rclcpp::Subscription<CoverageDecision>::SharedPtr coverage_decision_sub_;
  rclcpp::Subscription<SpatialDecision>::SharedPtr spatial_decision_sub_;
  rclcpp::Subscription<CommandEvidence>::SharedPtr command_evidence_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<VelocitySupervisor>());
  } catch (const rclcpp::exceptions::RCLError&) {
    // SIGINT can invalidate the context between a timer's graph query and
    // rcl execution. Only shutdown-context errors are expected here; active
    // runtime errors must still propagate. This does not claim physical stop.
    if (rclcpp::ok()) throw;
  }
  rclcpp::shutdown();
  return 0;
}
