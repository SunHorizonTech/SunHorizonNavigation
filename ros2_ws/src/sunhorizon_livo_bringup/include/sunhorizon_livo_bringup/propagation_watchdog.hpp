#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "sunhorizon_livo_bringup/measurement_continuity.hpp"

namespace sunhorizon_livo_bringup
{

struct PropagationWatchdogDecision
{
  bool accepted{false};
  std::string reason;
  double propagation_horizon_sec{0.0};
};

class PropagationWatchdog
{
public:
  PropagationWatchdog(
    double maximum_corrected_gap_sec,
    double maximum_propagated_gap_sec,
    double maximum_propagation_horizon_sec)
  : corrected_continuity_(maximum_corrected_gap_sec),
    propagated_continuity_(maximum_propagated_gap_sec),
    maximum_propagation_horizon_sec_(maximum_propagation_horizon_sec),
    startup_stability_window_sec_(maximum_corrected_gap_sec)
  {
    if (!std::isfinite(maximum_propagation_horizon_sec_) ||
      maximum_propagation_horizon_sec_ <= 0.0)
    {
      throw std::invalid_argument(
              "maximum propagation horizon must be finite and positive");
    }
  }

  PropagationWatchdogDecision observe_corrected(std::int64_t stamp_nanoseconds)
  {
    if (latched_) {
      return {false, latch_reason_, last_horizon_sec_};
    }
    auto continuity = corrected_continuity_.evaluate(stamp_nanoseconds);
    bool reject_recovery_anchor = false;
    double rejected_gap_sec = 0.0;
    if (!continuity.accepted &&
      continuity.reason == "vendor_odometry_measurement_gap")
    {
      // FAST-LIVO2 may pause corrected output during initialization or a
      // transient scheduling stall. Fail closed immediately, then require a
      // complete corrected-and-propagated stability epoch before output can
      // resume. Invalid or non-monotonic stamps still take the latched path
      // below because they cannot be proven safe by waiting for fresh data.
      const bool healthy_epoch_was_established =
        tracking_established_ || propagated_ready_ || recovering_from_unanchored_;
      if (healthy_epoch_was_established) {
        if (!recovering_from_unanchored_) {
          ++unanchored_event_count_;
        }
        recovering_from_unanchored_ = true;
        reject_recovery_anchor = true;
        rejected_gap_sec = continuity.gap_sec;
      }
      reset_startup_epoch();
      continuity = corrected_continuity_.evaluate(stamp_nanoseconds);
    }
    if (!continuity.accepted) {
      return latch(continuity.reason, continuity.gap_sec);
    }
    corrected_stamp_nanoseconds_ = stamp_nanoseconds;
    corrected_seen_ = true;
    ++corrected_anchor_count_;
    tracking_established_ = corrected_anchor_count_ >= kRequiredCorrectedAnchors;
    if (reject_recovery_anchor) {
      return {
        false, "vendor_odometry_measurement_gap", rejected_gap_sec};
    }
    return {true, "ok", 0.0};
  }

  PropagationWatchdogDecision evaluate_propagated(std::int64_t stamp_nanoseconds)
  {
    if (latched_) {
      return {false, latch_reason_, last_horizon_sec_};
    }
    if (!corrected_seen_) {
      return {false, "waiting_for_corrected_odometry", 0.0};
    }
    if (!tracking_established_) {
      return {false, "waiting_for_stable_corrected_odometry", 0.0};
    }
    auto continuity = propagated_continuity_.evaluate(stamp_nanoseconds);
    if (!continuity.accepted) {
      if (continuity.reason == "vendor_odometry_measurement_gap")
      {
        const double rejected_gap_sec = continuity.gap_sec;
        const bool healthy_epoch_was_established =
          propagated_ready_ || recovering_from_unanchored_;
        if (healthy_epoch_was_established) {
          if (!recovering_from_unanchored_) {
            ++unanchored_event_count_;
          }
          propagated_ready_ = false;
          recovering_from_unanchored_ = true;
        }
        propagated_continuity_.reset();
        propagation_epoch_seen_ = false;
        continuity = propagated_continuity_.evaluate(stamp_nanoseconds);
        if (!continuity.accepted) {
          return latch(continuity.reason, continuity.gap_sec);
        }
        propagation_epoch_stamp_nanoseconds_ = stamp_nanoseconds;
        propagation_epoch_seen_ = true;
        return {
          false,
          healthy_epoch_was_established ?
          "vendor_odometry_measurement_gap" :
          "waiting_for_stable_propagated_odometry",
          rejected_gap_sec};
      }
      return latch(continuity.reason, continuity.gap_sec);
    }
    const double horizon_sec = static_cast<double>(
      stamp_nanoseconds - corrected_stamp_nanoseconds_) * 1e-9;
    if (!std::isfinite(horizon_sec)) {
      return latch("propagation_horizon_non_finite", horizon_sec);
    }
    if (horizon_sec < 0.0) {
      // Cross-topic delivery order can briefly expose an older propagated
      // sample after a correction. Reject that sample without poisoning the
      // next strictly newer propagated measurement.
      return {false, "propagation_precedes_latest_correction", horizon_sec};
    }
    maximum_observed_horizon_sec_ = std::max(
      maximum_observed_horizon_sec_, horizon_sec);
    if (horizon_sec > maximum_propagation_horizon_sec_) {
      if (!propagated_ready_) {
        propagated_continuity_.reset();
        propagation_epoch_seen_ = false;
        return {false, "waiting_for_recent_corrected_odometry", horizon_sec};
      }
      // Stop propagated output immediately, but let the local odometry source
      // prove a fresh stability epoch instead of requiring a process restart.
      // The velocity supervisor remains responsible for latching final zero on
      // the resulting LOST status.
      propagated_ready_ = false;
      recovering_from_unanchored_ = true;
      ++unanchored_event_count_;
      propagated_continuity_.reset();
      propagation_epoch_seen_ = false;
      return {false, "propagation_unanchored", horizon_sec};
    }
    if (!propagation_epoch_seen_) {
      propagation_epoch_stamp_nanoseconds_ = stamp_nanoseconds;
      propagation_epoch_seen_ = true;
      return {false, "waiting_for_stable_propagated_odometry", horizon_sec};
    }
    const double stable_duration_sec = static_cast<double>(
      stamp_nanoseconds - propagation_epoch_stamp_nanoseconds_) * 1e-9;
    if (stable_duration_sec < startup_stability_window_sec_) {
      return {false, "waiting_for_stable_propagated_odometry", horizon_sec};
    }
    propagated_ready_ = true;
    recovering_from_unanchored_ = false;
    return {true, "ok", horizon_sec};
  }

  void reset()
  {
    corrected_continuity_.reset();
    propagated_continuity_.reset();
    corrected_seen_ = false;
    corrected_anchor_count_ = 0U;
    tracking_established_ = false;
    propagated_ready_ = false;
    recovering_from_unanchored_ = false;
    propagation_epoch_seen_ = false;
    propagation_epoch_stamp_nanoseconds_ = 0;
    corrected_stamp_nanoseconds_ = 0;
    latched_ = false;
    latch_reason_.clear();
    last_horizon_sec_ = 0.0;
    maximum_observed_horizon_sec_ = 0.0;
    unanchored_event_count_ = 0U;
  }

  bool latched() const {return latched_;}
  bool tracking_established() const {return tracking_established_;}
  bool ready() const {return propagated_ready_ && !latched_;}
  bool recovering_from_unanchored() const {return recovering_from_unanchored_;}
  std::size_t unanchored_event_count() const {return unanchored_event_count_;}
  double maximum_observed_horizon_sec() const
  {
    return maximum_observed_horizon_sec_;
  }
  const std::string & latch_reason() const {return latch_reason_;}
  std::int64_t corrected_stamp_nanoseconds() const
  {
    return corrected_stamp_nanoseconds_;
  }

private:
  static constexpr std::size_t kRequiredCorrectedAnchors = 2U;

  void reset_startup_epoch()
  {
    corrected_continuity_.reset();
    propagated_continuity_.reset();
    corrected_seen_ = false;
    corrected_anchor_count_ = 0U;
    tracking_established_ = false;
    propagated_ready_ = false;
    propagation_epoch_seen_ = false;
    propagation_epoch_stamp_nanoseconds_ = 0;
    corrected_stamp_nanoseconds_ = 0;
  }

  PropagationWatchdogDecision latch(const std::string & reason, double horizon_sec)
  {
    latched_ = true;
    latch_reason_ = reason;
    last_horizon_sec_ = horizon_sec;
    return {false, latch_reason_, last_horizon_sec_};
  }

  MeasurementContinuityGate corrected_continuity_;
  MeasurementContinuityGate propagated_continuity_;
  double maximum_propagation_horizon_sec_;
  double startup_stability_window_sec_;
  bool corrected_seen_{false};
  std::size_t corrected_anchor_count_{0U};
  bool tracking_established_{false};
  bool propagated_ready_{false};
  bool recovering_from_unanchored_{false};
  bool propagation_epoch_seen_{false};
  std::int64_t propagation_epoch_stamp_nanoseconds_{0};
  std::int64_t corrected_stamp_nanoseconds_{0};
  bool latched_{false};
  std::string latch_reason_;
  double last_horizon_sec_{0.0};
  double maximum_observed_horizon_sec_{0.0};
  std::size_t unanchored_event_count_{0U};
};

}  // namespace sunhorizon_livo_bringup
