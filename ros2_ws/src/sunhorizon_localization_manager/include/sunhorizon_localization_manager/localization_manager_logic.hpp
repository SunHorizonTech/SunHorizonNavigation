#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sunhorizon_navigation_interfaces/msg/localization_status.hpp>

namespace sunhorizon_localization_manager {

struct FrameContract {
  std::string map{"map"};
  std::string odom{"odom"};
  std::string base_footprint{"base_footprint"};
  // Preserve the registration map datum and body-origin Z. In this opt-in
  // mode base_footprint is a level frame at the body origin, not a foot/ground
  // datum; its residual to the full body pose has rotation only. Legacy
  // consumers retain the original strictly planar map alignment and Z=0.
  bool preserve_6d_map_alignment{false};
};

struct EvidenceLimits {
  // Stream timeouts use local ROS receipt time. They detect a publisher or
  // transport stall without confusing bounded algorithm latency for dropout.
  // Generic defaults retain the existing lidar/visual budgets. Robot A LIVO
  // overrides correction and raw-tracking budgets independently at bringup.
  double odometry_timeout_sec{0.50};
  double tracking_odometry_timeout_sec{0.25};
  double map_pose_timeout_sec{0.50};
  double status_timeout_sec{0.50};
  // Header time remains an independent sensor-measurement age gate. A freshly
  // received replay of old data must never pass merely because the stream is live.
  // This age is measured at receipt, not repeatedly against timer time: holding
  // an accepted correction is bounded separately by the receipt timeout above.
  double maximum_measurement_age_sec{0.50};
  double evidence_sync_tolerance_sec{0.001};
  double future_tolerance_sec{0.01};
  double quaternion_norm_tolerance{1e-3};
  double maximum_correction_translation_step_m{0.30};
  double maximum_correction_rotation_step_rad{0.2617993877991494};
  double maximum_comparison_translation_m{0.50};
  double maximum_comparison_yaw_rad{0.35};
};

struct BackendEvidence {
  std::optional<nav_msgs::msg::Odometry> odometry;
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> map_pose;
  std::optional<sunhorizon_navigation_interfaces::msg::LocalizationStatus> status;
  std::optional<double> odometry_received_sec;
  std::optional<double> map_pose_received_sec;
  std::optional<double> status_received_sec;
};

// Structural and original acquisition qualification. Live tracking, temporal
// alignment and correction-step gates remain separate requirements.
bool healthy_correction_pair_input_valid(
    const std::string& backend,
    const geometry_msgs::msg::PoseWithCovarianceStamped& pose,
    const sunhorizon_navigation_interfaces::msg::LocalizationStatus& status,
    const FrameContract& frames, const EvidenceLimits& limits, double now_sec,
    double pose_received_sec, double status_received_sec);

class HealthyPoseStatusPairCache {
 public:
  explicit HealthyPoseStatusPairCache(std::size_t maximum_pending_epochs = 16);

  // Returns true only when a strictly newer healthy pose/status epoch commits.
  // Duplicate or delayed messages never refresh the epoch's receipt lifetime.
  bool update_map_pose(
      const geometry_msgs::msg::PoseWithCovarianceStamped& map_pose);
  bool update_status(
      const sunhorizon_navigation_interfaces::msg::LocalizationStatus& status);
  void reset();

  const std::optional<geometry_msgs::msg::PoseWithCovarianceStamped>&
  committed_map_pose() const;
  const std::optional<sunhorizon_navigation_interfaces::msg::LocalizationStatus>&
  effective_status() const;
  std::int64_t committed_stamp_ns() const;

 private:
  bool commit_latest_matching_epoch();
  void trim_pending_epochs();

  std::size_t maximum_pending_epochs_;
  std::map<std::int64_t, geometry_msgs::msg::PoseWithCovarianceStamped>
      pending_map_poses_;
  std::map<
      std::int64_t,
      sunhorizon_navigation_interfaces::msg::LocalizationStatus>
      pending_healthy_statuses_;
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped>
      committed_map_pose_;
  std::optional<sunhorizon_navigation_interfaces::msg::LocalizationStatus>
      committed_status_;
  std::optional<sunhorizon_navigation_interfaces::msg::LocalizationStatus>
      fail_closed_status_;
  std::optional<std::int64_t> committed_stamp_ns_;
  std::optional<std::int64_t> fail_closed_stamp_ns_;
};

struct BackendDecision {
  bool valid{false};
  std::string reason;
  geometry_msgs::msg::Transform map_to_odom;
  geometry_msgs::msg::Transform odom_to_base_footprint;
};

struct TrackingDecision {
  bool valid{false};
  std::string reason;
  // Canonical orientation is level. Z remains zero unless the frame contract
  // explicitly preserves the 6D map alignment and body-origin height.
  // The upstream sample remains untouched for registration/history consumers.
  nav_msgs::msg::Odometry canonical_odometry;
  geometry_msgs::msg::Transform odom_to_base_footprint;
};

struct CorrectionTransitionDecision {
  bool valid{false};
  std::string reason;
  double translation_delta_m{0.0};
  double rotation_delta_rad{0.0};
};

struct ComparisonDecision {
  bool available{false};
  bool lidar_healthy{false};
  bool visual_healthy{false};
  double translation_delta_m{0.0};
  double yaw_delta_rad{0.0};
  bool within_limits{false};
  std::string reason;
};

// Exact-source-time full SE(3) transport, independent of navigation's planar
// correction. E is T_Airy<-nominalBase, L is the original LIVO candidate.
geometry_msgs::msg::Transform transport_pcd_from_airy(
    const geometry_msgs::msg::Transform& pcd_from_anchor_airy,
    const geometry_msgs::msg::Pose& anchor_livo,const geometry_msgs::msg::Pose& query_livo,
    const geometry_msgs::msg::Transform& source_from_nominal_base);

struct OdometryHistoryConfig {
  double duration_sec{2.0};
  double maximum_time_delta_sec{0.05};
  std::size_t maximum_samples{4096};
  double quaternion_norm_tolerance{1e-3};
};

struct OdometryHistoryUpdate {
  bool accepted{false};
  bool requires_reinitialization{false};
  std::string reason;
};

struct OdometryLookupDecision {
  bool valid{false};
  bool interpolated{false};
  std::string reason;
  nav_msgs::msg::Odometry odometry;
  // Receipt time belongs to the exact source epoch. For interpolation this is
  // the later arrival of the two samples needed to make the result available.
  std::optional<double> received_sec;
};

class OdometryHistory {
 public:
  explicit OdometryHistory(const OdometryHistoryConfig& config = {});

  OdometryHistoryUpdate insert(
      const nav_msgs::msg::Odometry& odometry,
      std::optional<double> received_sec = std::nullopt);
  OdometryLookupDecision lookup(std::int64_t target_stamp_ns) const;
  std::optional<nav_msgs::msg::Odometry> latest() const;
  void reset();

  bool requires_reinitialization() const;
  const std::string& failure_reason() const;
  std::size_t size() const;
  std::int64_t oldest_stamp_ns() const;
  std::int64_t newest_stamp_ns() const;

 private:
  OdometryHistoryUpdate latch_failure(const std::string& reason);

  OdometryHistoryConfig config_;
  std::deque<nav_msgs::msg::Odometry> samples_;
  std::deque<std::optional<double>> received_times_;
  std::string failure_reason_;
};

struct CorrectionGateDecision {
  bool valid{false};
  bool accepted_new_correction{false};
  bool requires_reinitialization{false};
  std::string reason;
};

class CorrectionGate {
 public:
  explicit CorrectionGate(
      const EvidenceLimits& limits = {},
      double filter_time_constant_sec = 0.0);

  CorrectionGateDecision consider(
      std::int64_t candidate_stamp_ns,
      const geometry_msgs::msg::Transform& candidate);
  void reinitialize();

  bool requires_reinitialization() const;
  const std::string& failure_reason() const;
  const std::optional<geometry_msgs::msg::Transform>& accepted_correction() const;
  std::int64_t accepted_stamp_ns() const;

 private:
  CorrectionGateDecision latch_failure(const std::string& reason);

  EvidenceLimits limits_;
  double filter_time_constant_sec_{0.0};
  std::optional<geometry_msgs::msg::Transform> last_raw_correction_;
  std::optional<geometry_msgs::msg::Transform> accepted_correction_;
  std::int64_t accepted_stamp_ns_{0};
  std::string failure_reason_;
};

bool should_publish_tracking_stamp(
    std::int64_t candidate_stamp_ns,
    std::int64_t last_published_stamp_ns);

std::optional<std::string> reported_backend_failure(
    const std::string& backend,
    const BackendEvidence& evidence,
    const FrameContract& frames = {});

BackendDecision evaluate_backend(
    const std::string& backend,
    const BackendEvidence& evidence,
    double now_sec,
    const FrameContract& frames = {},
    const EvidenceLimits& limits = {});

BackendDecision evaluate_active_backend(
    const std::string& active_backend,
    const BackendEvidence& lidar,
    const BackendEvidence& visual,
    const BackendEvidence& livo,
    double now_sec,
    const FrameContract& frames = {},
    const EvidenceLimits& limits = {});

TrackingDecision evaluate_tracking_odometry(
    const std::optional<nav_msgs::msg::Odometry>& odometry,
    std::optional<double> received_sec,
    double now_sec,
    const FrameContract& frames = {},
    const EvidenceLimits& limits = {});

CorrectionTransitionDecision evaluate_correction_transition(
    const std::optional<geometry_msgs::msg::Transform>& previous,
    const geometry_msgs::msg::Transform& candidate,
    const EvidenceLimits& limits = {});

ComparisonDecision compare_backends(
    const BackendEvidence& lidar,
    const BackendEvidence& visual,
    double now_sec,
    const FrameContract& frames = {},
    const EvidenceLimits& limits = {});

}  // namespace sunhorizon_localization_manager
