#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace sunhorizon_livo_bringup
{

class TimestampRateGate
{
public:
  explicit TimestampRateGate(double output_rate_hz)
  {
    if (!std::isfinite(output_rate_hz) || output_rate_hz <= 0.0) {
      throw std::invalid_argument("output rate must be finite and positive");
    }
    period_nanoseconds_ = static_cast<std::int64_t>(
      std::llround(1.0e9 / output_rate_hz));
    if (period_nanoseconds_ <= 0) {
      throw std::invalid_argument("output rate period is invalid");
    }
  }

  bool should_publish(std::int64_t stamp_nanoseconds)
  {
    if (stamp_nanoseconds <= 0) {
      return false;
    }
    if (!initialized_) {
      initialized_ = true;
      last_input_stamp_nanoseconds_ = stamp_nanoseconds;
      next_publish_stamp_nanoseconds_ = stamp_nanoseconds + period_nanoseconds_;
      return true;
    }
    if (stamp_nanoseconds <= last_input_stamp_nanoseconds_) {
      return false;
    }
    last_input_stamp_nanoseconds_ = stamp_nanoseconds;
    if (stamp_nanoseconds < next_publish_stamp_nanoseconds_) {
      return false;
    }
    do {
      next_publish_stamp_nanoseconds_ += period_nanoseconds_;
    } while (next_publish_stamp_nanoseconds_ <= stamp_nanoseconds);
    return true;
  }

  void reset()
  {
    initialized_ = false;
    last_input_stamp_nanoseconds_ = 0;
    next_publish_stamp_nanoseconds_ = 0;
  }

private:
  std::int64_t period_nanoseconds_{0};
  std::int64_t last_input_stamp_nanoseconds_{0};
  std::int64_t next_publish_stamp_nanoseconds_{0};
  bool initialized_{false};
};

}  // namespace sunhorizon_livo_bringup
