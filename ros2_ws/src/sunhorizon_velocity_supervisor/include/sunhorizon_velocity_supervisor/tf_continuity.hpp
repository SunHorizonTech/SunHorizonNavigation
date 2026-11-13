#pragma once

#include <cmath>

namespace sunhorizon_velocity_supervisor {

struct PlanarTfObservation {
  double stamp_sec{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline bool finite_tf_observation(
    const PlanarTfObservation& observation, double seen_sec) {
  return std::isfinite(observation.stamp_sec) &&
      observation.stamp_sec > 0.0 && std::isfinite(observation.x) &&
      std::isfinite(observation.y) && std::isfinite(observation.yaw) &&
      std::isfinite(seen_sec) && seen_sec > 0.0;
}

class TfContinuityTracker {
 public:
  bool observe(
      const PlanarTfObservation& observation, double seen_sec,
      double maximum_translation_jump_m, double maximum_yaw_jump_rad) {
    if (!finite_tf_observation(observation, seen_sec) ||
        !std::isfinite(maximum_translation_jump_m) ||
        maximum_translation_jump_m <= 0.0 ||
        !std::isfinite(maximum_yaw_jump_rad) ||
        maximum_yaw_jump_rad <= 0.0) {
      return false;
    }

    bool continuous = true;
    if (have_baseline_) {
      const double distance = std::hypot(
          observation.x - baseline_.x, observation.y - baseline_.y);
      const double yaw_delta = std::abs(std::remainder(
          observation.yaw - baseline_.yaw, 2.0 * kPi));
      const double elapsed = seen_sec - baseline_seen_sec_;
      if (elapsed >= 0.0 && elapsed < 0.5 &&
          (distance > maximum_translation_jump_m ||
           yaw_delta > maximum_yaw_jump_rad)) {
        continuous = false;
      }
    }
    commit(observation, seen_sec);
    return continuous;
  }

  // An operator reset after explicit relocalization starts a new map-frame
  // continuity epoch. This method validates only the finite observation; the
  // caller must still pass its timestamp through the normal freshness gate
  // before committing the staged tracker.
  bool rebaseline(const PlanarTfObservation& observation, double seen_sec) {
    if (!finite_tf_observation(observation, seen_sec)) return false;
    commit(observation, seen_sec);
    return true;
  }

 private:
  void commit(const PlanarTfObservation& observation, double seen_sec) {
    baseline_ = observation;
    baseline_seen_sec_ = seen_sec;
    have_baseline_ = true;
  }

  static constexpr double kPi = 3.14159265358979323846;
  bool have_baseline_{false};
  PlanarTfObservation baseline_;
  double baseline_seen_sec_{0.0};
};

// Observe progress of the original TF stamp. Polling the same cached transform
// never refreshes the stream watchdog, even across an operator reset.
class TfProgressTracker {
 public:
  bool observe(const PlanarTfObservation& value, double ros, double steady) {
    if (fault_ || !finite_tf_observation(value, ros) || !std::isfinite(steady) ||
        steady <= 0 || value.stamp_sec > ros ||
        (seen_ && (ros < last_ros_ || steady < last_steady_))) {
      fault_ = true; return false;
    }
    last_ros_ = ros; last_steady_ = steady;
    if (seen_ && value.stamp_sec < previous_.stamp_sec) {
      fault_ = true; return false;
    }
    if (!seen_ || value.stamp_sec > previous_.stamp_sec) {
      advancing_ = seen_;
      seen_ = true; previous_ = value;
      progress_ros_ = ros; progress_steady_ = steady;
    }
    return true;
  }
  bool current(double ros, double steady, double timeout) const {
    return !fault_ && advancing_ && std::isfinite(ros) && std::isfinite(steady) &&
      std::isfinite(timeout) && timeout > 0 && ros >= last_ros_ && steady >= last_steady_ &&
      ros-progress_ros_ <= timeout && steady-progress_steady_ <= timeout;
  }
  double receipt_ros_sec() const {return progress_ros_;}
  double source_sec() const {return previous_.stamp_sec;}
 private:
  bool seen_{false}, advancing_{false}, fault_{false};
  PlanarTfObservation previous_;
  double last_ros_{0}, last_steady_{0}, progress_ros_{0}, progress_steady_{0};
};

}  // namespace sunhorizon_velocity_supervisor
