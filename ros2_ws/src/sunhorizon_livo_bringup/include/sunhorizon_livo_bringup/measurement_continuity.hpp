#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace sunhorizon_livo_bringup
{

struct MeasurementContinuityDecision
{
  bool accepted{false};
  std::string reason;
  double gap_sec{0.0};
};

class MeasurementContinuityGate
{
public:
  explicit MeasurementContinuityGate(double maximum_gap_sec)
  : maximum_gap_sec_(maximum_gap_sec)
  {
    if (!std::isfinite(maximum_gap_sec_) || maximum_gap_sec_ <= 0.0) {
      throw std::invalid_argument("maximum measurement gap must be finite and positive");
    }
  }

  MeasurementContinuityDecision evaluate(std::int64_t stamp_nanoseconds)
  {
    if (latched_) {
      return {false, latch_reason_, last_rejected_gap_sec_};
    }
    if (stamp_nanoseconds <= 0) {
      return latch("vendor_odometry_measurement_stamp_invalid", 0.0);
    }
    if (!has_accepted_stamp_) {
      accepted_stamp_nanoseconds_ = stamp_nanoseconds;
      has_accepted_stamp_ = true;
      return {true, "ok", 0.0};
    }

    const double gap_sec = static_cast<double>(
      stamp_nanoseconds - accepted_stamp_nanoseconds_) * 1e-9;
    if (!std::isfinite(gap_sec) || gap_sec <= 0.0) {
      return latch("vendor_odometry_measurement_non_monotonic", gap_sec);
    }
    if (gap_sec > maximum_gap_sec_) {
      return latch("vendor_odometry_measurement_gap", gap_sec);
    }

    accepted_stamp_nanoseconds_ = stamp_nanoseconds;
    return {true, "ok", gap_sec};
  }

  void reset()
  {
    has_accepted_stamp_ = false;
    accepted_stamp_nanoseconds_ = 0;
    latched_ = false;
    latch_reason_.clear();
    last_rejected_gap_sec_ = 0.0;
  }

  bool latched() const {return latched_;}
  std::int64_t accepted_stamp_nanoseconds() const {return accepted_stamp_nanoseconds_;}

private:
  MeasurementContinuityDecision latch(const std::string & reason, double gap_sec)
  {
    latched_ = true;
    latch_reason_ = reason;
    last_rejected_gap_sec_ = gap_sec;
    return {false, latch_reason_, last_rejected_gap_sec_};
  }

  double maximum_gap_sec_;
  bool has_accepted_stamp_{false};
  std::int64_t accepted_stamp_nanoseconds_{0};
  bool latched_{false};
  std::string latch_reason_;
  double last_rejected_gap_sec_{0.0};
};

}  // namespace sunhorizon_livo_bringup
