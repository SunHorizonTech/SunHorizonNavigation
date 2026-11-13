#pragma once

#include <cstdint>

namespace sunhorizon_3d_localizer {

enum class NdtMatchRateDecision {
  kMatch,
  kThrottled,
  kInvalidTimestamp,
  kTimestampRegressed,
};

// Limits expensive NDT attempts using sensor time rather than callback or wall time.
// A stream timestamp regression is reported without changing the current epoch;
// callers must explicitly reset the gate when they accept a new sensor-time epoch.
class NdtMatchRateGate {
 public:
  explicit NdtMatchRateGate(double maximum_match_rate_hz);

  NdtMatchRateDecision evaluate(std::int64_t sensor_stamp_ns);
  void reset();

  std::int64_t minimum_match_interval_ns() const;

 private:
  std::int64_t minimum_match_interval_ns_{1};
  std::int64_t last_observed_stamp_ns_{0};
  std::int64_t last_scheduled_match_stamp_ns_{0};
  bool observed_stamp_valid_{false};
  bool scheduled_match_valid_{false};
};

}  // namespace sunhorizon_3d_localizer
