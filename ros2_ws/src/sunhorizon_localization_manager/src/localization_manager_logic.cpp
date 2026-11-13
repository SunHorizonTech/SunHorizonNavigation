#include "sunhorizon_localization_manager/localization_manager_logic.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <stdexcept>

#include <builtin_interfaces/msg/time.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace sunhorizon_localization_manager {
namespace {

using LocalizationStatus = sunhorizon_navigation_interfaces::msg::LocalizationStatus;

constexpr double kPi = 3.14159265358979323846;

double stamp_seconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

std::int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
      static_cast<std::int64_t>(stamp.nanosec);
}

builtin_interfaces::msg::Time stamp_from_nanoseconds(std::int64_t stamp_ns) {
  builtin_interfaces::msg::Time output;
  output.sec = static_cast<std::int32_t>(stamp_ns / 1000000000LL);
  output.nanosec = static_cast<std::uint32_t>(stamp_ns % 1000000000LL);
  return output;
}

bool finite(double value) { return std::isfinite(value); }

template<std::size_t Size>
bool finite_array(const std::array<double, Size>& values) {
  return std::all_of(values.begin(), values.end(),
      [](double value) { return finite(value); });
}

bool finite_quaternion(const geometry_msgs::msg::Quaternion& quaternion, double tolerance) {
  if (!finite(quaternion.x) || !finite(quaternion.y) ||
      !finite(quaternion.z) || !finite(quaternion.w)) {
    return false;
  }
  const double norm = std::sqrt(
      quaternion.x * quaternion.x + quaternion.y * quaternion.y +
      quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  return finite(norm) && norm > std::numeric_limits<double>::epsilon() &&
      std::abs(norm - 1.0) <= tolerance;
}

bool finite_pose(const geometry_msgs::msg::Pose& pose, double quaternion_tolerance) {
  return finite(pose.position.x) && finite(pose.position.y) && finite(pose.position.z) &&
      finite_quaternion(pose.orientation, quaternion_tolerance);
}

bool finite_transform_message(
    const geometry_msgs::msg::Transform& transform, double quaternion_tolerance) {
  return finite(transform.translation.x) && finite(transform.translation.y) &&
      finite(transform.translation.z) &&
      finite_quaternion(transform.rotation, quaternion_tolerance);
}

bool finite_odometry(const nav_msgs::msg::Odometry& odometry, double quaternion_tolerance) {
  const auto& linear = odometry.twist.twist.linear;
  const auto& angular = odometry.twist.twist.angular;
  return finite_pose(odometry.pose.pose, quaternion_tolerance) &&
      finite_array(odometry.pose.covariance) &&
      finite(linear.x) && finite(linear.y) && finite(linear.z) &&
      finite(angular.x) && finite(angular.y) && finite(angular.z) &&
      finite_array(odometry.twist.covariance);
}

nav_msgs::msg::Odometry interpolate_odometry(
    const nav_msgs::msg::Odometry& lower,
    const nav_msgs::msg::Odometry& upper,
    std::int64_t target_stamp_ns) {
  const std::int64_t lower_ns = stamp_nanoseconds(lower.header.stamp);
  const std::int64_t upper_ns = stamp_nanoseconds(upper.header.stamp);
  const double alpha = static_cast<double>(target_stamp_ns - lower_ns) /
      static_cast<double>(upper_ns - lower_ns);
  nav_msgs::msg::Odometry output = lower;
  output.header.stamp = stamp_from_nanoseconds(target_stamp_ns);

  const auto lerp = [alpha](double from, double to) {
    return from + alpha * (to - from);
  };
  output.pose.pose.position.x = lerp(
      lower.pose.pose.position.x, upper.pose.pose.position.x);
  output.pose.pose.position.y = lerp(
      lower.pose.pose.position.y, upper.pose.pose.position.y);
  output.pose.pose.position.z = lerp(
      lower.pose.pose.position.z, upper.pose.pose.position.z);
  tf2::Quaternion lower_rotation;
  tf2::Quaternion upper_rotation;
  tf2::fromMsg(lower.pose.pose.orientation, lower_rotation);
  tf2::fromMsg(upper.pose.pose.orientation, upper_rotation);
  lower_rotation.normalize();
  upper_rotation.normalize();
  output.pose.pose.orientation = tf2::toMsg(
      lower_rotation.slerp(upper_rotation, alpha).normalized());
  for (std::size_t index = 0; index < output.pose.covariance.size(); ++index) {
    output.pose.covariance[index] = lerp(
        lower.pose.covariance[index], upper.pose.covariance[index]);
    output.twist.covariance[index] = lerp(
        lower.twist.covariance[index], upper.twist.covariance[index]);
  }
  output.twist.twist.linear.x = lerp(
      lower.twist.twist.linear.x, upper.twist.twist.linear.x);
  output.twist.twist.linear.y = lerp(
      lower.twist.twist.linear.y, upper.twist.twist.linear.y);
  output.twist.twist.linear.z = lerp(
      lower.twist.twist.linear.z, upper.twist.twist.linear.z);
  output.twist.twist.angular.x = lerp(
      lower.twist.twist.angular.x, upper.twist.twist.angular.x);
  output.twist.twist.angular.y = lerp(
      lower.twist.twist.angular.y, upper.twist.twist.angular.y);
  output.twist.twist.angular.z = lerp(
      lower.twist.twist.angular.z, upper.twist.twist.angular.z);
  return output;
}

bool finite_status(const LocalizationStatus& status) {
  return finite(status.score) && finite(status.position_covariance) &&
      finite(status.yaw_covariance) && finite(status.measurement_age_sec) &&
      status.measurement_age_sec >= 0.0;
}

std::string measurement_stamp_failure(
    const char* name, double stamp, double received_sec, double now_sec,
    double future_tolerance) {
  if (!finite(stamp) || stamp <= 0.0) return std::string(name) + "_stamp_invalid";
  const double age = received_sec - stamp;
  if (age < -future_tolerance || now_sec - stamp < -future_tolerance) {
    return std::string(name) + "_from_future";
  }
  // Delivery latency is recorded separately, not a localization failure.
  // Receipt liveness and monotonic input/sequence ownership remain required.
  return {};
}

std::string receipt_liveness_failure(
    const char* name, const std::optional<double>& received_sec, double now,
    double timeout, double future_tolerance) {
  if (!received_sec) return std::string(name) + "_receipt_missing";
  if (!finite(*received_sec) || *received_sec <= 0.0) {
    return std::string(name) + "_receipt_invalid";
  }
  const double age = now - *received_sec;
  if (age < -future_tolerance) {
    return std::string(name) + "_receipt_from_future";
  }
  if (age > timeout) return std::string(name) + "_stream_stale";
  return {};
}

tf2::Transform pose_transform(const geometry_msgs::msg::Pose& pose) {
  tf2::Quaternion rotation(
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  rotation.normalize();
  return tf2::Transform(rotation, tf2::Vector3(
      pose.position.x, pose.position.y, pose.position.z));
}

tf2::Transform transform_value(const geometry_msgs::msg::Transform& transform) {
  tf2::Quaternion rotation(
      transform.rotation.x, transform.rotation.y,
      transform.rotation.z, transform.rotation.w);
  rotation.normalize();
  return tf2::Transform(rotation, tf2::Vector3(
      transform.translation.x, transform.translation.y, transform.translation.z));
}

geometry_msgs::msg::Transform transform_message(const tf2::Transform& transform) {
  geometry_msgs::msg::Transform output;
  output.translation.x = transform.getOrigin().x();
  output.translation.y = transform.getOrigin().y();
  output.translation.z = transform.getOrigin().z();
  output.rotation = tf2::toMsg(transform.getRotation());
  return output;
}

tf2::Transform planar_base_transform(
    const tf2::Transform& transform, bool preserve_body_z = false) {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, yaw);
  rotation.normalize();
  return tf2::Transform(
      rotation,
      tf2::Vector3(transform.getOrigin().x(), transform.getOrigin().y(),
          preserve_body_z ? transform.getOrigin().z() : 0.0));
}

nav_msgs::msg::Odometry planar_canonical_odometry(
    const nav_msgs::msg::Odometry& odometry, bool preserve_body_z = false) {
  nav_msgs::msg::Odometry output = odometry;
  const auto planar = planar_base_transform(pose_transform(odometry.pose.pose), preserve_body_z);
  output.pose.pose.position.x = planar.getOrigin().x();
  output.pose.pose.position.y = planar.getOrigin().y();
  output.pose.pose.position.z = preserve_body_z ? planar.getOrigin().z() : 0.0;
  output.pose.pose.orientation = tf2::toMsg(planar.getRotation());
  output.twist.twist.linear.z = 0.0;
  output.twist.twist.angular.x = 0.0;
  output.twist.twist.angular.y = 0.0;
  return output;
}

double wrapped_absolute_yaw(const tf2::Transform& transform) {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
  return std::abs(std::remainder(yaw, 2.0 * kPi));
}

std::string comparison_pose_failure(
    const std::string& backend,
    const BackendEvidence& evidence,
    double now_sec,
    const FrameContract& frames,
    const EvidenceLimits& limits) {
  if (!evidence.map_pose) return backend + "_map_pose_missing";
  if (!evidence.status) return backend + "_status_missing";
  if (evidence.map_pose->header.frame_id != frames.map) {
    return backend + "_map_pose_frame_mismatch";
  }
  if (evidence.status->header.frame_id != frames.map) {
    return backend + "_status_frame_mismatch";
  }
  if (evidence.status->backend != backend) return backend + "_status_backend_mismatch";
  if (!finite_pose(evidence.map_pose->pose.pose, limits.quaternion_norm_tolerance) ||
      !finite_array(evidence.map_pose->pose.covariance)) {
    return backend + "_map_pose_non_finite";
  }
  auto reason = receipt_liveness_failure(
      "map_pose", evidence.map_pose_received_sec, now_sec,
      limits.map_pose_timeout_sec, limits.future_tolerance_sec);
  if (!reason.empty()) return backend + "_" + reason;
  reason = receipt_liveness_failure(
      "status", evidence.status_received_sec, now_sec,
      limits.status_timeout_sec, limits.future_tolerance_sec);
  if (!reason.empty()) return backend + "_" + reason;
  reason = measurement_stamp_failure(
      "map_pose", stamp_seconds(evidence.map_pose->header.stamp),
      *evidence.map_pose_received_sec, now_sec,
      limits.future_tolerance_sec);
  if (!reason.empty()) return backend + "_" + reason;
  reason = measurement_stamp_failure(
      "status", stamp_seconds(evidence.status->header.stamp),
      *evidence.status_received_sec, now_sec,
      limits.future_tolerance_sec);
  if (!reason.empty()) return backend + "_" + reason;
  return {};
}

}  // namespace

bool should_publish_tracking_stamp(
    std::int64_t candidate_stamp_ns,
    std::int64_t last_published_stamp_ns) {
  return candidate_stamp_ns > 0 && candidate_stamp_ns > last_published_stamp_ns;
}

std::optional<std::string> reported_backend_failure(
    const std::string& backend,
    const BackendEvidence& evidence,
    const FrameContract& frames) {
  if (!evidence.status) return std::nullopt;
  const auto& status = *evidence.status;
  if (status.header.frame_id != frames.map || status.backend != backend) {
    return std::nullopt;
  }
  if (status.state == LocalizationStatus::STATE_HEALTHY &&
      status.measurement_accepted) {
    return std::nullopt;
  }
  return "backend_unhealthy:" + status.reason;
}

bool healthy_correction_pair_input_valid(
    const std::string& backend,
    const geometry_msgs::msg::PoseWithCovarianceStamped& pose,
    const LocalizationStatus& status,
    const FrameContract& frames, const EvidenceLimits& limits, double now_sec,
    double pose_received_sec, double status_received_sec) {
  const auto source = stamp_nanoseconds(pose.header.stamp);
  return finite(now_sec) && now_sec > 0.0 && source > 0 &&
      source == stamp_nanoseconds(status.header.stamp) &&
      pose.header.frame_id == frames.map && status.header.frame_id == frames.map &&
      status.backend == backend && status.state == LocalizationStatus::STATE_HEALTHY &&
      status.measurement_accepted && finite_status(status) &&
      finite_pose(pose.pose.pose, limits.quaternion_norm_tolerance) &&
      finite_array(pose.pose.covariance) &&
      receipt_liveness_failure("map_pose", pose_received_sec, now_sec,
        limits.map_pose_timeout_sec, limits.future_tolerance_sec).empty() &&
      receipt_liveness_failure("status", status_received_sec, now_sec,
        limits.status_timeout_sec, limits.future_tolerance_sec).empty() &&
      measurement_stamp_failure("map_pose", stamp_seconds(pose.header.stamp),
        pose_received_sec, now_sec, limits.future_tolerance_sec).empty() &&
      measurement_stamp_failure("status", stamp_seconds(status.header.stamp),
        status_received_sec, now_sec, limits.future_tolerance_sec).empty();
}

HealthyPoseStatusPairCache::HealthyPoseStatusPairCache(
    std::size_t maximum_pending_epochs)
    : maximum_pending_epochs_(maximum_pending_epochs) {
  if (maximum_pending_epochs_ == 0) {
    throw std::invalid_argument(
        "healthy pose/status cache must retain at least one pending epoch");
  }
}

bool HealthyPoseStatusPairCache::update_map_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped& map_pose) {
  pending_map_poses_[stamp_nanoseconds(map_pose.header.stamp)] = map_pose;
  trim_pending_epochs();
  return commit_latest_matching_epoch();
}

bool HealthyPoseStatusPairCache::update_status(
    const LocalizationStatus& status) {
  const auto status_stamp_ns = stamp_nanoseconds(status.header.stamp);
  if (status.state != LocalizationStatus::STATE_HEALTHY ||
      !status.measurement_accepted) {
    // A health report received before this failure cannot prove recovery later.
    // Keep poses so a subsequent healthy status can still form an exact pair.
    pending_healthy_statuses_.clear();
    fail_closed_status_ = status;
    fail_closed_stamp_ns_ = status_stamp_ns;
    return false;
  }

  pending_healthy_statuses_[status_stamp_ns] = status;
  trim_pending_epochs();
  return commit_latest_matching_epoch();
}

void HealthyPoseStatusPairCache::reset() {
  pending_map_poses_.clear();
  pending_healthy_statuses_.clear();
  committed_map_pose_.reset();
  committed_status_.reset();
  fail_closed_status_.reset();
  committed_stamp_ns_.reset();
  fail_closed_stamp_ns_.reset();
}

const std::optional<geometry_msgs::msg::PoseWithCovarianceStamped>&
HealthyPoseStatusPairCache::committed_map_pose() const {
  return committed_map_pose_;
}

const std::optional<LocalizationStatus>&
HealthyPoseStatusPairCache::effective_status() const {
  return fail_closed_status_ ? fail_closed_status_ : committed_status_;
}

std::int64_t HealthyPoseStatusPairCache::committed_stamp_ns() const {
  return committed_stamp_ns_.value_or(0);
}

bool HealthyPoseStatusPairCache::commit_latest_matching_epoch() {
  for (auto status = pending_healthy_statuses_.rbegin();
       status != pending_healthy_statuses_.rend(); ++status) {
    const auto pose = pending_map_poses_.find(status->first);
    if (pose == pending_map_poses_.end()) continue;
    // Repeated status/map messages for the committed measurement epoch must
    // not make old evidence look newly received.
    if (committed_stamp_ns_ && status->first <= *committed_stamp_ns_) continue;

    committed_map_pose_ = pose->second;
    committed_status_ = status->second;
    committed_stamp_ns_ = status->first;
    if (fail_closed_stamp_ns_ && status->first >= *fail_closed_stamp_ns_) {
      fail_closed_status_.reset();
      fail_closed_stamp_ns_.reset();
    }

    pending_map_poses_.erase(
        pending_map_poses_.begin(), pending_map_poses_.lower_bound(status->first));
    pending_healthy_statuses_.erase(
        pending_healthy_statuses_.begin(),
        pending_healthy_statuses_.lower_bound(status->first));
    return true;
  }
  return false;
}

void HealthyPoseStatusPairCache::trim_pending_epochs() {
  while (pending_map_poses_.size() > maximum_pending_epochs_) {
    pending_map_poses_.erase(pending_map_poses_.begin());
  }
  while (pending_healthy_statuses_.size() > maximum_pending_epochs_) {
    pending_healthy_statuses_.erase(pending_healthy_statuses_.begin());
  }
}

OdometryHistory::OdometryHistory(const OdometryHistoryConfig& config)
    : config_(config) {
  if (!finite(config_.duration_sec) || config_.duration_sec < 2.0 ||
      !finite(config_.maximum_time_delta_sec) ||
      config_.maximum_time_delta_sec <= 0.0 ||
      config_.maximum_samples < 2 ||
      !finite(config_.quaternion_norm_tolerance) ||
      config_.quaternion_norm_tolerance <= 0.0) {
    throw std::invalid_argument("invalid odometry history configuration");
  }
}

geometry_msgs::msg::Transform transport_pcd_from_airy(
    const geometry_msgs::msg::Transform& pcd_from_anchor_airy,
    const geometry_msgs::msg::Pose& anchor_livo,const geometry_msgs::msg::Pose& query_livo,
    const geometry_msgs::msg::Transform& source_from_nominal_base) {
  const auto base_from_source=transform_value(source_from_nominal_base).inverse();
  const auto anchor_world_from_airy=pose_transform(anchor_livo)*base_from_source;
  const auto query_world_from_airy=pose_transform(query_livo)*base_from_source;
  return transform_message(transform_value(pcd_from_anchor_airy)*
    anchor_world_from_airy.inverse()*query_world_from_airy);
}

OdometryHistoryUpdate OdometryHistory::latch_failure(const std::string& reason) {
  if (failure_reason_.empty()) failure_reason_ = reason;
  return {false, true, failure_reason_};
}

OdometryHistoryUpdate OdometryHistory::insert(
    const nav_msgs::msg::Odometry& odometry,
    std::optional<double> received_sec) {
  if (!failure_reason_.empty()) return {false, true, failure_reason_};
  const std::int64_t candidate_ns = stamp_nanoseconds(odometry.header.stamp);
  if (candidate_ns <= 0) return latch_failure("odometry_history_stamp_invalid");
  if (!finite_odometry(odometry, config_.quaternion_norm_tolerance)) {
    return latch_failure("odometry_history_non_finite");
  }
  if (!samples_.empty()) {
    const std::int64_t newest_ns = stamp_nanoseconds(samples_.back().header.stamp);
    if (candidate_ns < newest_ns) {
      return latch_failure("odometry_history_stamp_reversed");
    }
    if (candidate_ns == newest_ns) {
      return {false, false, "odometry_history_stamp_duplicate"};
    }
    if (odometry.header.frame_id != samples_.back().header.frame_id ||
        odometry.child_frame_id != samples_.back().child_frame_id) {
      return latch_failure("odometry_history_frame_discontinuity");
    }
  }

  samples_.push_back(odometry);
  received_times_.push_back(received_sec);
  const auto duration_ns = static_cast<std::int64_t>(config_.duration_sec * 1e9);
  const std::int64_t cutoff_ns = candidate_ns - duration_ns;
  while (samples_.size() > 1 &&
      stamp_nanoseconds(samples_[1].header.stamp) <= cutoff_ns) {
    samples_.pop_front();
    received_times_.pop_front();
  }
  while (samples_.size() > config_.maximum_samples) {
    samples_.pop_front();
    received_times_.pop_front();
  }
  return {true, false, "ok"};
}

OdometryLookupDecision OdometryHistory::lookup(
    std::int64_t target_stamp_ns) const {
  OdometryLookupDecision output;
  if (!failure_reason_.empty()) {
    output.reason = failure_reason_;
    return output;
  }
  if (target_stamp_ns <= 0) {
    output.reason = "odometry_history_lookup_stamp_invalid";
    return output;
  }
  if (samples_.empty()) {
    output.reason = "odometry_history_empty";
    return output;
  }
  const std::int64_t oldest_ns = stamp_nanoseconds(samples_.front().header.stamp);
  const std::int64_t newest_ns = stamp_nanoseconds(samples_.back().header.stamp);
  if (target_stamp_ns < oldest_ns) {
    output.reason = "odometry_history_target_too_old";
    return output;
  }
  if (target_stamp_ns > newest_ns) {
    output.reason = "odometry_history_target_after_newest";
    return output;
  }
  const auto upper = std::lower_bound(
      samples_.begin(), samples_.end(), target_stamp_ns,
      [](const nav_msgs::msg::Odometry& sample, std::int64_t target) {
        return stamp_nanoseconds(sample.header.stamp) < target;
      });
  if (upper != samples_.end() &&
      stamp_nanoseconds(upper->header.stamp) == target_stamp_ns) {
    const auto index = static_cast<std::size_t>(
        std::distance(samples_.begin(), upper));
    output.valid = true;
    output.reason = "ok";
    output.odometry = *upper;
    output.received_sec = received_times_[index];
    return output;
  }
  if (upper == samples_.begin() || upper == samples_.end()) {
    output.reason = "odometry_history_target_out_of_bounds";
    return output;
  }
  const auto lower = std::prev(upper);
  const auto upper_index = static_cast<std::size_t>(
      std::distance(samples_.begin(), upper));
  const auto lower_index = upper_index - 1U;
  const std::int64_t lower_ns = stamp_nanoseconds(lower->header.stamp);
  const std::int64_t upper_ns = stamp_nanoseconds(upper->header.stamp);
  const auto maximum_delta_ns = static_cast<std::int64_t>(
      config_.maximum_time_delta_sec * 1e9);
  if (target_stamp_ns - lower_ns > maximum_delta_ns ||
      upper_ns - target_stamp_ns > maximum_delta_ns) {
    output.reason = "odometry_history_gap_exceeds_tolerance";
    return output;
  }
  output.odometry = interpolate_odometry(*lower, *upper, target_stamp_ns);
  if (!finite_odometry(output.odometry, config_.quaternion_norm_tolerance)) {
    output.reason = "odometry_history_interpolation_non_finite";
    return output;
  }
  output.valid = true;
  output.interpolated = true;
  output.reason = "ok";
  if (received_times_[lower_index] && received_times_[upper_index]) {
    output.received_sec = std::max(
        *received_times_[lower_index], *received_times_[upper_index]);
  }
  return output;
}

std::optional<nav_msgs::msg::Odometry> OdometryHistory::latest() const {
  if (samples_.empty() || !failure_reason_.empty()) return std::nullopt;
  return samples_.back();
}

void OdometryHistory::reset() {
  samples_.clear();
  received_times_.clear();
  failure_reason_.clear();
}

bool OdometryHistory::requires_reinitialization() const {
  return !failure_reason_.empty();
}

const std::string& OdometryHistory::failure_reason() const {
  return failure_reason_;
}

std::size_t OdometryHistory::size() const { return samples_.size(); }

std::int64_t OdometryHistory::oldest_stamp_ns() const {
  return samples_.empty() ? 0 : stamp_nanoseconds(samples_.front().header.stamp);
}

std::int64_t OdometryHistory::newest_stamp_ns() const {
  return samples_.empty() ? 0 : stamp_nanoseconds(samples_.back().header.stamp);
}

CorrectionGate::CorrectionGate(
    const EvidenceLimits& limits, double filter_time_constant_sec)
    : limits_(limits), filter_time_constant_sec_(filter_time_constant_sec) {
  if (!finite(filter_time_constant_sec_) || filter_time_constant_sec_ < 0.0) {
    throw std::invalid_argument(
        "correction filter time constant must be finite and nonnegative");
  }
}

CorrectionGateDecision CorrectionGate::latch_failure(const std::string& reason) {
  if (failure_reason_.empty()) failure_reason_ = reason;
  return {false, false, true, failure_reason_};
}

CorrectionGateDecision CorrectionGate::consider(
    std::int64_t candidate_stamp_ns,
    const geometry_msgs::msg::Transform& candidate) {
  if (!failure_reason_.empty()) {
    return {false, false, true, failure_reason_};
  }
  if (candidate_stamp_ns <= 0) {
    return latch_failure("correction_stamp_invalid");
  }
  if (accepted_stamp_ns_ > 0 && candidate_stamp_ns < accepted_stamp_ns_) {
    return latch_failure("correction_stamp_not_monotonic");
  }
  if (candidate_stamp_ns == accepted_stamp_ns_) {
    return {true, false, false, "correction_already_accepted"};
  }
  const auto transition = evaluate_correction_transition(
      last_raw_correction_, candidate, limits_);
  if (!transition.valid) return latch_failure(transition.reason);
  if (!accepted_correction_ || filter_time_constant_sec_ == 0.0) {
    accepted_correction_ = candidate;
  } else {
    const double delta_sec = static_cast<double>(
        candidate_stamp_ns - accepted_stamp_ns_) * 1e-9;
    if (!finite(delta_sec) || delta_sec <= 0.0) {
      return latch_failure("correction_filter_delta_invalid");
    }
    const double alpha = 1.0 - std::exp(
        -delta_sec / filter_time_constant_sec_);
    const tf2::Transform previous = transform_value(*accepted_correction_);
    const tf2::Transform raw = transform_value(candidate);
    tf2::Transform filtered;
    filtered.setOrigin(previous.getOrigin().lerp(raw.getOrigin(), alpha));
    tf2::Quaternion rotation = previous.getRotation().slerp(
        raw.getRotation(), alpha);
    rotation.normalize();
    filtered.setRotation(rotation);
    accepted_correction_ = transform_message(filtered);
  }
  last_raw_correction_ = candidate;
  accepted_stamp_ns_ = candidate_stamp_ns;
  return {true, true, false, "ok"};
}

void CorrectionGate::reinitialize() {
  last_raw_correction_.reset();
  accepted_correction_.reset();
  accepted_stamp_ns_ = 0;
  failure_reason_.clear();
}

bool CorrectionGate::requires_reinitialization() const {
  return !failure_reason_.empty();
}

const std::string& CorrectionGate::failure_reason() const {
  return failure_reason_;
}

const std::optional<geometry_msgs::msg::Transform>&
CorrectionGate::accepted_correction() const {
  return accepted_correction_;
}

std::int64_t CorrectionGate::accepted_stamp_ns() const {
  return accepted_stamp_ns_;
}

BackendDecision evaluate_backend(
    const std::string& backend,
    const BackendEvidence& evidence,
    double now_sec,
    const FrameContract& frames,
    const EvidenceLimits& limits) {
  BackendDecision output;
  if (backend != "lidar" && backend != "visual" && backend != "livo") {
    output.reason = "unsupported_backend";
    return output;
  }
  if (!finite(now_sec) || now_sec <= 0.0) {
    output.reason = "manager_time_invalid";
    return output;
  }
  if (!evidence.odometry) {
    output.reason = "odometry_candidate_missing";
    return output;
  }
  if (!evidence.map_pose) {
    output.reason = "map_pose_candidate_missing";
    return output;
  }
  if (!evidence.status) {
    output.reason = "backend_status_missing";
    return output;
  }
  const auto& odometry = *evidence.odometry;
  const auto& map_pose = *evidence.map_pose;
  const auto& status = *evidence.status;
  if (odometry.header.frame_id != frames.odom ||
      odometry.child_frame_id != frames.base_footprint) {
    output.reason = "odometry_frame_mismatch";
    return output;
  }
  if (map_pose.header.frame_id != frames.map) {
    output.reason = "map_pose_frame_mismatch";
    return output;
  }
  if (status.header.frame_id != frames.map) {
    output.reason = "status_frame_mismatch";
    return output;
  }
  if (status.backend != backend) {
    output.reason = "status_backend_mismatch";
    return output;
  }
  if (!finite_odometry(odometry, limits.quaternion_norm_tolerance)) {
    output.reason = "odometry_non_finite";
    return output;
  }
  if (!finite_pose(map_pose.pose.pose, limits.quaternion_norm_tolerance) ||
      !finite_array(map_pose.pose.covariance)) {
    output.reason = "map_pose_non_finite";
    return output;
  }
  const double odometry_stamp = stamp_seconds(odometry.header.stamp);
  const double map_pose_stamp = stamp_seconds(map_pose.header.stamp);
  const double status_stamp = stamp_seconds(status.header.stamp);
  if (!finite_status(status)) {
    output.reason = "backend_status_non_finite";
    return output;
  }
  if (status.state != LocalizationStatus::STATE_HEALTHY ||
      !status.measurement_accepted) {
    output.reason = "backend_unhealthy:" + status.reason;
    return output;
  }
  auto reason = receipt_liveness_failure(
      "odometry", evidence.odometry_received_sec, now_sec,
      limits.odometry_timeout_sec, limits.future_tolerance_sec);
  if (!reason.empty()) {
    output.reason = reason;
    return output;
  }
  reason = receipt_liveness_failure(
      "map_pose", evidence.map_pose_received_sec, now_sec,
      limits.map_pose_timeout_sec, limits.future_tolerance_sec);
  if (!reason.empty()) {
    output.reason = reason;
    return output;
  }
  reason = receipt_liveness_failure(
      "status", evidence.status_received_sec, now_sec,
      limits.status_timeout_sec, limits.future_tolerance_sec);
  if (!reason.empty()) {
    output.reason = reason;
    return output;
  }
  reason = measurement_stamp_failure(
      "odometry", odometry_stamp, *evidence.odometry_received_sec,
      now_sec, limits.future_tolerance_sec);
  if (!reason.empty()) {
    output.reason = reason;
    return output;
  }
  reason = measurement_stamp_failure(
      "map_pose", map_pose_stamp, *evidence.map_pose_received_sec,
      now_sec, limits.future_tolerance_sec);
  if (!reason.empty()) {
    output.reason = reason;
    return output;
  }
  reason = measurement_stamp_failure(
      "status", status_stamp, *evidence.status_received_sec,
      now_sec, limits.future_tolerance_sec);
  if (!reason.empty()) {
    output.reason = reason;
    return output;
  }
  if (std::abs(map_pose_stamp - odometry_stamp) > limits.evidence_sync_tolerance_sec) {
    output.reason = "candidate_stamp_mismatch";
    return output;
  }
  if (std::abs(status_stamp - map_pose_stamp) > limits.evidence_sync_tolerance_sec) {
    output.reason = "candidate_status_stamp_mismatch";
    return output;
  }
  const tf2::Transform map_to_base = planar_base_transform(
      pose_transform(map_pose.pose.pose));
  const tf2::Transform odom_to_base = planar_base_transform(
      pose_transform(odometry.pose.pose), frames.preserve_6d_map_alignment);
  const tf2::Transform map_to_odom = frames.preserve_6d_map_alignment ?
      pose_transform(map_pose.pose.pose) * pose_transform(odometry.pose.pose).inverse() :
      map_to_base * odom_to_base.inverse();
  output.map_to_odom = transform_message(map_to_odom);
  output.odom_to_base_footprint = transform_message(odom_to_base);
  output.valid = true;
  output.reason = "ok";
  return output;
}

BackendDecision evaluate_active_backend(
    const std::string& active_backend,
    const BackendEvidence& lidar,
    const BackendEvidence& visual,
    const BackendEvidence& livo,
    double now_sec,
    const FrameContract& frames,
    const EvidenceLimits& limits) {
  if (active_backend == "lidar") {
    return evaluate_backend("lidar", lidar, now_sec, frames, limits);
  }
  if (active_backend == "visual") {
    return evaluate_backend("visual", visual, now_sec, frames, limits);
  }
  if (active_backend == "livo") {
    return evaluate_backend("livo", livo, now_sec, frames, limits);
  }
  BackendDecision output;
  output.reason = "unsupported_active_backend";
  return output;
}

TrackingDecision evaluate_tracking_odometry(
    const std::optional<nav_msgs::msg::Odometry>& odometry,
    std::optional<double> received_sec,
    double now_sec,
    const FrameContract& frames,
    const EvidenceLimits& limits) {
  TrackingDecision output;
  if (!finite(now_sec) || now_sec <= 0.0) {
    output.reason = "manager_time_invalid";
    return output;
  }
  if (!odometry) {
    output.reason = "tracking_odometry_candidate_missing";
    return output;
  }
  if (odometry->header.frame_id != frames.odom ||
      odometry->child_frame_id != frames.base_footprint) {
    output.reason = "tracking_odometry_frame_mismatch";
    return output;
  }
  if (!finite_odometry(*odometry, limits.quaternion_norm_tolerance)) {
    output.reason = "tracking_odometry_non_finite";
    return output;
  }
  auto reason = receipt_liveness_failure(
      "tracking_odometry", received_sec, now_sec,
      limits.tracking_odometry_timeout_sec, limits.future_tolerance_sec);
  if (!reason.empty()) {
    output.reason = reason;
    return output;
  }
  reason = measurement_stamp_failure(
      "tracking_odometry", stamp_seconds(odometry->header.stamp), *received_sec,
      now_sec,
      limits.future_tolerance_sec);
  if (!reason.empty()) {
    output.reason = reason;
    return output;
  }
  output.canonical_odometry = planar_canonical_odometry(
      *odometry, frames.preserve_6d_map_alignment);
  output.odom_to_base_footprint = transform_message(
      pose_transform(output.canonical_odometry.pose.pose));
  output.valid = true;
  output.reason = "ok";
  return output;
}

CorrectionTransitionDecision evaluate_correction_transition(
    const std::optional<geometry_msgs::msg::Transform>& previous,
    const geometry_msgs::msg::Transform& candidate,
    const EvidenceLimits& limits) {
  CorrectionTransitionDecision output;
  if (limits.maximum_correction_translation_step_m <= 0.0 ||
      limits.maximum_correction_rotation_step_rad <= 0.0 ||
      !finite_transform_message(candidate, limits.quaternion_norm_tolerance)) {
    output.reason = "correction_invalid";
    return output;
  }
  if (!previous) {
    output.valid = true;
    output.reason = "ok";
    return output;
  }
  if (!finite_transform_message(*previous, limits.quaternion_norm_tolerance)) {
    output.reason = "previous_correction_invalid";
    return output;
  }
  const tf2::Transform delta = transform_value(*previous).inverse() *
      transform_value(candidate);
  output.translation_delta_m = delta.getOrigin().length();
  tf2::Quaternion rotation = delta.getRotation();
  rotation.normalize();
  output.rotation_delta_rad = 2.0 * std::acos(std::clamp(
      std::abs(rotation.w()), 0.0, 1.0));
  if (output.translation_delta_m > limits.maximum_correction_translation_step_m) {
    output.reason = "map_to_odom_correction_translation_jump";
    return output;
  }
  if (output.rotation_delta_rad > limits.maximum_correction_rotation_step_rad) {
    output.reason = "map_to_odom_correction_rotation_jump";
    return output;
  }
  output.valid = true;
  output.reason = "ok";
  return output;
}

ComparisonDecision compare_backends(
    const BackendEvidence& lidar,
    const BackendEvidence& visual,
    double now_sec,
    const FrameContract& frames,
    const EvidenceLimits& limits) {
  ComparisonDecision output;
  if (!finite(now_sec) || now_sec <= 0.0) {
    output.reason = "manager_time_invalid";
    return output;
  }
  const auto lidar_failure = comparison_pose_failure(
      "lidar", lidar, now_sec, frames, limits);
  if (!lidar_failure.empty()) {
    output.reason = lidar_failure;
    return output;
  }
  const auto visual_failure = comparison_pose_failure(
      "visual", visual, now_sec, frames, limits);
  if (!visual_failure.empty()) {
    output.reason = visual_failure;
    return output;
  }

  output.available = true;
  output.lidar_healthy = evaluate_backend(
      "lidar", lidar, now_sec, frames, limits).valid;
  output.visual_healthy = evaluate_backend(
      "visual", visual, now_sec, frames, limits).valid;
  const tf2::Transform lidar_map_to_base = pose_transform(lidar.map_pose->pose.pose);
  const tf2::Transform visual_map_to_base = pose_transform(visual.map_pose->pose.pose);
  const tf2::Transform delta = lidar_map_to_base.inverse() * visual_map_to_base;
  output.translation_delta_m = delta.getOrigin().length();
  output.yaw_delta_rad = wrapped_absolute_yaw(delta);
  output.within_limits = output.lidar_healthy && output.visual_healthy &&
      output.translation_delta_m <= limits.maximum_comparison_translation_m &&
      output.yaw_delta_rad <= limits.maximum_comparison_yaw_rad;
  if (!output.lidar_healthy || !output.visual_healthy) {
    output.reason = "backend_unhealthy";
  } else if (!output.within_limits) {
    output.reason = "comparison_limits_exceeded";
  } else {
    output.reason = "ok";
  }
  return output;
}

}  // namespace sunhorizon_localization_manager
