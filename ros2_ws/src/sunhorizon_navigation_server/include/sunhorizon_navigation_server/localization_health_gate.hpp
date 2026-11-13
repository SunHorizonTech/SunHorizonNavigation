#pragma once

#include <cstdint>
#include <stdexcept>

namespace sunhorizon_navigation_server {

class LocalizationHealthGate {
 public:
  explicit LocalizationHealthGate(
      std::int64_t required_stable_duration_ns = 1000000000LL)
      : required_stable_duration_ns_(required_stable_duration_ns) {
    if (required_stable_duration_ns_ <= 0) {
      throw std::invalid_argument(
          "localization healthy stability duration must be positive");
    }
  }

  void arm(std::uint64_t receipt_epoch, std::int64_t stamp_ns) {
    active_ = stamp_ns > 0;
    minimum_receipt_epoch_ = receipt_epoch;
    minimum_stamp_ns_ = stamp_ns;
    reset_window();
  }

  void disarm() {
    active_ = false;
    minimum_receipt_epoch_ = 0U;
    minimum_stamp_ns_ = 0;
    reset_window();
  }

  bool newer_than_arm(
      std::uint64_t receipt_epoch, std::int64_t stamp_ns) const {
    return active_ && receipt_epoch > minimum_receipt_epoch_ &&
           stamp_ns > minimum_stamp_ns_;
  }

  // Observe one post-seed status. Only an uninterrupted sequence of accepted
  // HEALTHY evidence spanning required_stable_duration_ns_ completes the gate.
  // LOST, unaccepted/non-healthy evidence, and non-increasing evidence stamps
  // reset the window while leaving the gate armed for a later recovery.
  bool observe(
      std::uint64_t receipt_epoch, std::int64_t stamp_ns,
      bool measurement_accepted, bool state_healthy) {
    if (!newer_than_arm(receipt_epoch, stamp_ns)) return false;
    if (last_evidence_stamp_ns_ > 0 && stamp_ns <= last_evidence_stamp_ns_) {
      healthy_since_stamp_ns_ = 0;
      return false;
    }
    last_evidence_stamp_ns_ = stamp_ns;
    if (!measurement_accepted || !state_healthy) {
      healthy_since_stamp_ns_ = 0;
      return false;
    }
    if (healthy_since_stamp_ns_ == 0) {
      healthy_since_stamp_ns_ = stamp_ns;
      return false;
    }
    return stamp_ns - healthy_since_stamp_ns_ >= required_stable_duration_ns_;
  }

 private:
  void reset_window() {
    healthy_since_stamp_ns_ = 0;
    last_evidence_stamp_ns_ = 0;
  }

  std::int64_t required_stable_duration_ns_{1000000000LL};
  bool active_{false};
  std::uint64_t minimum_receipt_epoch_{0U};
  std::int64_t minimum_stamp_ns_{0};
  std::int64_t healthy_since_stamp_ns_{0};
  std::int64_t last_evidence_stamp_ns_{0};
};

}  // namespace sunhorizon_navigation_server
