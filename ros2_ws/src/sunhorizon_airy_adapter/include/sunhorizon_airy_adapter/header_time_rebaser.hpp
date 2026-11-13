#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace sunhorizon_airy_adapter {

enum class HeaderTimeMode {
  kPreserve,
  kFirstImuToRos,
  kFixedOffset,
  kTrackingImuToRos,
};

inline HeaderTimeMode parse_header_time_mode(const std::string& value) {
  if (value == "preserve") {
    return HeaderTimeMode::kPreserve;
  }
  if (value == "first_imu_to_ros") {
    return HeaderTimeMode::kFirstImuToRos;
  }
  if (value == "fixed_offset") {
    return HeaderTimeMode::kFixedOffset;
  }
  if (value == "tracking_imu_to_ros") {
    return HeaderTimeMode::kTrackingImuToRos;
  }
  throw std::invalid_argument("unsupported_header_time_mode");
}

struct HeaderTimeRebaseResult {
  bool accepted{false};
  bool initialized_now{false};
  double shifted_stamp_sec{0.0};
  std::string reason{"header_time_rebaser_uninitialized"};
};

// Translates the Airy device epoch into the ROS clock without changing the
// relative point timestamps within a scan. Live tracking uses only IMU samples
// to update one shared point/IMU offset. The bounded slew rate is strictly less
// than one second per second, so a monotonic device clock remains monotonic
// after correction.
class HeaderTimeRebaser {
 public:
  HeaderTimeRebaser(
      HeaderTimeMode mode, double fixed_offset_sec,
      double maximum_future_stamp_sec, double tracking_target_delay_sec,
      double maximum_clock_offset_slew_rate,
      double tracking_initialization_warmup_sec)
      : mode_(mode),
        maximum_future_stamp_sec_(maximum_future_stamp_sec),
        tracking_target_delay_sec_(tracking_target_delay_sec),
        maximum_clock_offset_slew_rate_(maximum_clock_offset_slew_rate),
        tracking_initialization_warmup_sec_(tracking_initialization_warmup_sec) {
    if (!std::isfinite(fixed_offset_sec) ||
        !std::isfinite(maximum_future_stamp_sec_) ||
        maximum_future_stamp_sec_ < 0.0 ||
        !std::isfinite(tracking_target_delay_sec_) ||
        tracking_target_delay_sec_ < 0.0 ||
        !std::isfinite(maximum_clock_offset_slew_rate_) ||
        maximum_clock_offset_slew_rate_ <= 0.0 ||
        maximum_clock_offset_slew_rate_ >= 1.0 ||
        !std::isfinite(tracking_initialization_warmup_sec_) ||
        tracking_initialization_warmup_sec_ < 0.0) {
      throw std::invalid_argument("invalid_header_time_rebaser_config");
    }
    if (mode_ == HeaderTimeMode::kPreserve) {
      offset_sec_ = 0.0;
      initialized_ = true;
    } else if (mode_ == HeaderTimeMode::kFixedOffset) {
      offset_sec_ = fixed_offset_sec;
      initialized_ = true;
    }
  }

  HeaderTimeRebaseResult rebase(
      double raw_stamp_sec, double now_sec, bool imu_tracking_sample) {
    if (!std::isfinite(raw_stamp_sec) || raw_stamp_sec <= 0.0) {
      return rejected("raw_header_stamp_invalid");
    }
    if (!std::isfinite(now_sec) || now_sec <= 0.0) {
      return rejected("ros_clock_not_ready_for_header_rebase");
    }

    HeaderTimeRebaseResult output;
    if (!initialized_) {
      if (!imu_tracking_sample ||
          (mode_ != HeaderTimeMode::kFirstImuToRos &&
           mode_ != HeaderTimeMode::kTrackingImuToRos)) {
        return rejected("clock_offset_waiting_for_first_imu");
      }
      const double target_delay =
          mode_ == HeaderTimeMode::kTrackingImuToRos
              ? tracking_target_delay_sec_
              : 0.0;
      if (mode_ == HeaderTimeMode::kTrackingImuToRos) {
        if (!tracking_warmup_started_) {
          tracking_warmup_started_ = true;
          tracking_warmup_start_sec_ = now_sec;
        }
        const double warmup_age_sec = now_sec - tracking_warmup_start_sec_;
        if (!std::isfinite(warmup_age_sec) || warmup_age_sec < 0.0) {
          return rejected("ros_clock_regressed_during_header_rebase_warmup");
        }
        last_observed_offset_sec_ = now_sec - raw_stamp_sec;
        if (warmup_age_sec < tracking_initialization_warmup_sec_) {
          return rejected("clock_offset_warming_up");
        }
      }
      offset_sec_ = now_sec - raw_stamp_sec - target_delay;
      initialized_ = true;
      last_tracking_raw_stamp_sec_ = raw_stamp_sec;
      last_observed_offset_sec_ = now_sec - raw_stamp_sec;
      last_offset_adjustment_sec_ = 0.0;
      output.initialized_now = true;
    } else if (mode_ == HeaderTimeMode::kTrackingImuToRos &&
               imu_tracking_sample) {
      if (raw_stamp_sec <= last_tracking_raw_stamp_sec_) {
        return rejected("tracking_imu_header_not_strictly_increasing");
      }
      const double raw_delta_sec = raw_stamp_sec - last_tracking_raw_stamp_sec_;
      last_observed_offset_sec_ = now_sec - raw_stamp_sec;
      const double target_offset_sec =
          last_observed_offset_sec_ - tracking_target_delay_sec_;
      const double maximum_adjustment_sec =
          maximum_clock_offset_slew_rate_ * raw_delta_sec;
      last_offset_adjustment_sec_ = std::clamp(
          target_offset_sec - offset_sec_, -maximum_adjustment_sec,
          maximum_adjustment_sec);
      offset_sec_ += last_offset_adjustment_sec_;
      last_tracking_raw_stamp_sec_ = raw_stamp_sec;
    }

    const double shifted_stamp_sec = raw_stamp_sec + offset_sec_;
    if (!std::isfinite(shifted_stamp_sec) || shifted_stamp_sec <= 0.0) {
      return rejected("rebased_header_stamp_non_positive");
    }
    if (shifted_stamp_sec > now_sec + maximum_future_stamp_sec_) {
      return rejected("rebased_header_in_future");
    }
    output.accepted = true;
    output.shifted_stamp_sec = shifted_stamp_sec;
    output.reason = "ok";
    return output;
  }

  bool initialized() const { return initialized_; }
  double offset_sec() const { return offset_sec_; }
  double last_observed_offset_sec() const { return last_observed_offset_sec_; }
  double last_offset_adjustment_sec() const { return last_offset_adjustment_sec_; }

 private:
  static HeaderTimeRebaseResult rejected(const std::string& reason) {
    HeaderTimeRebaseResult output;
    output.reason = reason;
    return output;
  }

  HeaderTimeMode mode_;
  double maximum_future_stamp_sec_{0.1};
  double tracking_target_delay_sec_{0.02};
  double maximum_clock_offset_slew_rate_{0.002};
  double tracking_initialization_warmup_sec_{0.5};
  double tracking_warmup_start_sec_{0.0};
  double offset_sec_{0.0};
  double last_tracking_raw_stamp_sec_{0.0};
  double last_observed_offset_sec_{0.0};
  double last_offset_adjustment_sec_{0.0};
  bool tracking_warmup_started_{false};
  bool initialized_{false};
};

}  // namespace sunhorizon_airy_adapter
