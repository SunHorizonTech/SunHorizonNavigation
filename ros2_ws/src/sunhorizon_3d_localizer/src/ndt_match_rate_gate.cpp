#include "sunhorizon_3d_localizer/ndt_match_rate_gate.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace sunhorizon_3d_localizer {

NdtMatchRateGate::NdtMatchRateGate(double maximum_match_rate_hz) {
  if (!std::isfinite(maximum_match_rate_hz) || maximum_match_rate_hz <= 0.0) {
    throw std::invalid_argument("maximum_match_rate_hz_must_be_finite_and_positive");
  }
  const double interval_ns = std::ceil(1.0e9 / maximum_match_rate_hz);
  if (!std::isfinite(interval_ns) ||
      interval_ns > static_cast<double>(std::numeric_limits<std::int64_t>::max())) {
    throw std::invalid_argument("maximum_match_rate_hz_interval_out_of_range");
  }
  minimum_match_interval_ns_ = std::max<std::int64_t>(
      1, static_cast<std::int64_t>(interval_ns));
}

NdtMatchRateDecision NdtMatchRateGate::evaluate(std::int64_t sensor_stamp_ns) {
  if (sensor_stamp_ns <= 0) return NdtMatchRateDecision::kInvalidTimestamp;
  if (observed_stamp_valid_ && sensor_stamp_ns < last_observed_stamp_ns_) {
    return NdtMatchRateDecision::kTimestampRegressed;
  }
  last_observed_stamp_ns_ = sensor_stamp_ns;
  observed_stamp_valid_ = true;

  if (!scheduled_match_valid_) {
    last_scheduled_match_stamp_ns_ = sensor_stamp_ns;
    scheduled_match_valid_ = true;
    return NdtMatchRateDecision::kMatch;
  }
  if (sensor_stamp_ns - last_scheduled_match_stamp_ns_ < minimum_match_interval_ns_) {
    return NdtMatchRateDecision::kThrottled;
  }
  last_scheduled_match_stamp_ns_ = sensor_stamp_ns;
  return NdtMatchRateDecision::kMatch;
}

void NdtMatchRateGate::reset() {
  last_observed_stamp_ns_ = 0;
  last_scheduled_match_stamp_ns_ = 0;
  observed_stamp_valid_ = false;
  scheduled_match_valid_ = false;
}

std::int64_t NdtMatchRateGate::minimum_match_interval_ns() const {
  return minimum_match_interval_ns_;
}

}  // namespace sunhorizon_3d_localizer
