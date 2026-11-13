#include "sunhorizon_3d_localizer/ndt_startup_alignment_gate.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace sunhorizon_3d_localizer {
namespace {

constexpr double kPi = 3.14159265358979323846;

double wrapped_yaw_delta(double left, double right) {
  double delta = left - right;
  while (delta > kPi) delta -= 2.0 * kPi;
  while (delta < -kPi) delta += 2.0 * kPi;
  return std::abs(delta);
}

bool finite_sample(const NdtStartupAlignmentSample& sample) {
  return sample.stamp_ns > 0 && std::isfinite(sample.x_m) &&
      std::isfinite(sample.y_m) && std::isfinite(sample.yaw_rad) &&
      std::isfinite(sample.seed_translation_m) &&
      std::isfinite(sample.seed_yaw_rad);
}

void validate_limits(const NdtStartupAlignmentLimits& limits) {
  if (!std::isfinite(limits.maximum_seed_translation_m) ||
      limits.maximum_seed_translation_m <= 0.0 ||
      !std::isfinite(limits.maximum_seed_yaw_rad) ||
      limits.maximum_seed_yaw_rad <= 0.0 ||
      !std::isfinite(limits.maximum_candidate_translation_spread_m) ||
      limits.maximum_candidate_translation_spread_m <= 0.0 ||
      !std::isfinite(limits.maximum_candidate_yaw_spread_rad) ||
      limits.maximum_candidate_yaw_spread_rad <= 0.0 ||
      !std::isfinite(limits.maximum_sample_gap_sec) ||
      limits.maximum_sample_gap_sec <= 0.0 ||
      limits.required_consecutive_samples < 2U) {
    throw std::invalid_argument("invalid NDT startup alignment limits");
  }
}

}  // namespace

NdtStartupAlignmentDecision NdtStartupAlignmentGate::observe(
    const NdtStartupAlignmentSample& sample,
    const NdtStartupAlignmentLimits& limits) {
  validate_limits(limits);
  if (!finite_sample(sample)) {
    reset();
    return {false, 0U, "ndt_startup_alignment_invalid"};
  }
  if (sample.seed_translation_m > limits.maximum_seed_translation_m) {
    reset();
    return {false, 0U, "ndt_initial_translation_limit"};
  }
  if (sample.seed_yaw_rad > limits.maximum_seed_yaw_rad) {
    reset();
    return {false, 0U, "ndt_initial_yaw_limit"};
  }

  const double gap_sec = previous_stamp_ns_ > 0 ?
      static_cast<double>(sample.stamp_ns - previous_stamp_ns_) / 1.0e9 : 0.0;
  const double translation_spread = has_anchor_ ?
      std::hypot(sample.x_m - anchor_x_m_, sample.y_m - anchor_y_m_) : 0.0;
  const double yaw_spread = has_anchor_ ?
      wrapped_yaw_delta(sample.yaw_rad, anchor_yaw_rad_) : 0.0;
  const bool sequence_broken = has_anchor_ &&
      (gap_sec <= 0.0 || gap_sec > limits.maximum_sample_gap_sec ||
       translation_spread > limits.maximum_candidate_translation_spread_m ||
       yaw_spread > limits.maximum_candidate_yaw_spread_rad);
  if (!has_anchor_ || sequence_broken) {
    has_anchor_ = true;
    anchor_x_m_ = sample.x_m;
    anchor_y_m_ = sample.y_m;
    anchor_yaw_rad_ = sample.yaw_rad;
    consecutive_samples_ = 1U;
  } else {
    ++consecutive_samples_;
  }
  previous_stamp_ns_ = sample.stamp_ns;

  if (consecutive_samples_ < limits.required_consecutive_samples) {
    return {
        false, consecutive_samples_,
        sequence_broken ? "ndt_startup_alignment_inconsistent" :
                          "ndt_startup_alignment_pending"};
  }
  return {true, consecutive_samples_, "ndt_startup_alignment_stable"};
}

void NdtStartupAlignmentGate::reset() {
  has_anchor_ = false;
  previous_stamp_ns_ = 0;
  anchor_x_m_ = 0.0;
  anchor_y_m_ = 0.0;
  anchor_yaw_rad_ = 0.0;
  consecutive_samples_ = 0U;
}

}  // namespace sunhorizon_3d_localizer
