#include "sunhorizon_3d_localizer/ndt_prior_sync_logic.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>

#include <Eigen/Geometry>

namespace sunhorizon_3d_localizer {
namespace {

bool valid_positive_limit(double value) {
  return std::isfinite(value) && value > 0.0;
}

bool finite_transform(const Eigen::Matrix4d& transform) {
  return transform.array().isFinite().all();
}

double rotation_distance(
    const Eigen::Matrix4d& first, const Eigen::Matrix4d& second) {
  const Eigen::Matrix3d delta =
      first.block<3, 3>(0, 0).transpose() * second.block<3, 3>(0, 0);
  return std::abs(Eigen::AngleAxisd(delta).angle());
}

}  // namespace

PriorContinuityGate::PriorContinuityGate(PriorContinuityLimits limits)
    : limits_(limits) {
  if (!valid_positive_limit(limits_.maximum_gap_sec) ||
      !valid_positive_limit(limits_.maximum_translation_step_m) ||
      !valid_positive_limit(limits_.maximum_rotation_step_rad)) {
    throw std::invalid_argument("invalid_prior_continuity_configuration");
  }
}

void PriorContinuityGate::reset() {
  anchor_valid_ = false;
  anchor_stamp_ns_ = 0;
  anchor_ = Eigen::Matrix4d::Identity();
}

PriorContinuityDecision PriorContinuityGate::evaluate(
    std::int64_t stamp_ns, const Eigen::Matrix4d& odom_to_source) {
  PriorContinuityDecision output;
  if (stamp_ns <= 0) {
    reset();
    output.epoch_break = true;
    output.reason = "odometry_prior_stamp_invalid";
    return output;
  }
  if (!finite_transform(odom_to_source)) {
    reset();
    output.epoch_break = true;
    output.reason = "odometry_prior_pose_non_finite";
    return output;
  }
  if (!anchor_valid_) {
    anchor_valid_ = true;
    anchor_stamp_ns_ = stamp_ns;
    anchor_ = odom_to_source;
    output.reason = "odometry_prior_anchor_waiting_for_continuity";
    return output;
  }

  output.dt_sec = static_cast<double>(stamp_ns - anchor_stamp_ns_) * 1e-9;
  if (!std::isfinite(output.dt_sec) || output.dt_sec <= 0.0) {
    reset();
    output.epoch_break = true;
    output.reason = "odometry_prior_stamp_not_strictly_increasing";
    return output;
  }

  const Eigen::Matrix4d delta = anchor_.inverse() * odom_to_source;
  output.translation_step_m = delta.block<3, 1>(0, 3).norm();
  output.rotation_step_rad = rotation_distance(anchor_, odom_to_source);

  // A valid transform with a time or spatial discontinuity is itself the
  // anchor of the next epoch.  Updating both fields atomically prevents a
  // spatial jump from being compared with the old epoch and later mislabeled
  // as a time gap.
  anchor_stamp_ns_ = stamp_ns;
  anchor_ = odom_to_source;
  // Spatial continuity is the safety property needed by the NDT prediction.
  // Check it before classifying a time gap so a transport pause cannot hide a
  // real pose jump. Conversely, a fresh pose that is still spatially bounded
  // is usable immediately after a time-only gap. Exact timestamp matching and
  // the unchanged point-cloud input-age gate still reject an old replayed
  // prior/scan pair before it can become an accepted localization result.
  if (output.translation_step_m > limits_.maximum_translation_step_m) {
    output.epoch_break = true;
    output.reason = "odometry_prior_translation_jump";
    return output;
  }
  if (output.rotation_step_rad > limits_.maximum_rotation_step_rad) {
    output.epoch_break = true;
    output.reason = "odometry_prior_rotation_jump";
    return output;
  }
  if (output.dt_sec > limits_.maximum_gap_sec) {
    output.matchable = true;
    output.epoch_break = true;
    output.reason = "odometry_prior_gap_recovered_spatially_continuous";
    return output;
  }

  output.matchable = true;
  output.reason = "ok";
  return output;
}

PriorMatchDecision select_nearest_matchable_prior(
    std::int64_t point_stamp_ns,
    const std::vector<PriorSyncCandidate>& candidates,
    std::int64_t tolerance_ns) {
  PriorMatchDecision output;
  output.reason = "no_synchronized_odometry_prior";
  if (point_stamp_ns <= 0 || tolerance_ns < 0) {
    output.reason = "invalid_sync_input";
    return output;
  }

  std::int64_t best_delta = std::numeric_limits<std::int64_t>::max();
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    const auto& candidate = candidates[index];
    if (candidate.stamp_ns <= 0) continue;
    const std::int64_t delta = std::llabs(candidate.stamp_ns - point_stamp_ns);
    if (!candidate.matchable) {
      if (delta <= tolerance_ns) output.invalid_prior_within_tolerance = true;
      continue;
    }
    if (delta < best_delta) {
      best_delta = delta;
      output.index = index;
    }
  }

  if (best_delta <= tolerance_ns) {
    output.matched = true;
    output.delta_ns = best_delta;
    output.reason = "ok";
  } else if (output.invalid_prior_within_tolerance) {
    output.reason = "invalid_odometry_prior_not_matchable";
  }
  return output;
}

void NdtPriorSyncDiagnostics::record_no_match() {
  ++snapshot_.no_match_count;
}

void NdtPriorSyncDiagnostics::record_invalid_prior(bool epoch_break) {
  ++snapshot_.invalid_prior_count;
  if (epoch_break) ++snapshot_.epoch_break_count;
}

void NdtPriorSyncDiagnostics::record_recovered_gap() {
  ++snapshot_.epoch_break_count;
  ++snapshot_.recovered_gap_count;
}

void NdtPriorSyncDiagnostics::record_raw_scan_skip() {
  ++snapshot_.raw_scan_skip_count;
}

void NdtPriorSyncDiagnostics::record_match(std::int64_t delta_ns) {
  const std::int64_t absolute_delta = std::llabs(delta_ns);
  snapshot_.last_match_delta_ns = absolute_delta;
  snapshot_.maximum_match_delta_ns = std::max(
      snapshot_.maximum_match_delta_ns, absolute_delta);
}

void NdtPriorSyncDiagnostics::record_match_invoked() {
  ++snapshot_.match_invoked_count;
}

void NdtPriorSyncDiagnostics::record_match_result(bool accepted) {
  if (accepted) {
    ++snapshot_.match_accepted_count;
  } else {
    ++snapshot_.match_rejected_count;
  }
}

void NdtPriorSyncDiagnostics::reset() {
  snapshot_ = {};
}

}  // namespace sunhorizon_3d_localizer
