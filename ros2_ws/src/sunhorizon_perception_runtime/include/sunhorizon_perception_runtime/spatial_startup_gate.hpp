#pragma once

#include <cstdint>
#include <mutex>
#include <stdexcept>

namespace sunhorizon_perception_runtime {
struct SpatialStartupStatus {
  bool deferred{false}, owner_ready{false}, opened{false};
  std::int64_t opened_ros_ns{0}, opened_steady_ns{0};
  std::int64_t deadline_steady_ns{0};
  std::int64_t first_published_steady_ns{0};
  std::uint64_t first_published_sequence{0};
};
// This gate delays FIRST evidence only. It never owns or resets source expiry,
// failure state, or a downstream tracker. Legacy standalone owners bypass it.
// The coordinator owns startup timeout policy. Expired first qualification is
// permanently unavailable evidence; it must not destroy acquisition owners.
class SpatialStartupGate {
 public:
  void defer() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (input_seen_ || state_.deferred) throw std::logic_error("spatial_startup_defer_requires_unstarted_owner");
    state_.deferred = true;
  }
  void note_input() {std::lock_guard<std::mutex> lock(mutex_); input_seen_ = true;}
  void owner_ready(std::int64_t deadline_steady) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_.deferred || state_.owner_ready) throw std::logic_error("spatial_startup_owner_ready_once");
    if (deadline_steady <= 0) throw std::invalid_argument("spatial_startup_requires_original_deadline");
    state_.owner_ready = true;
    state_.deadline_steady_ns = deadline_steady;
  }
  bool qualify(bool healthy, std::int64_t ros, std::int64_t steady) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_.deferred || !state_.owner_ready || state_.opened || !healthy) return false;
    if (ros <= 0 || steady <= 0) throw std::runtime_error("spatial_startup_invalid_clock");
    if (steady >= state_.deadline_steady_ns) return false;
    state_.opened = true; state_.opened_ros_ns = ros; state_.opened_steady_ns = steady;
    return true;
  }
  bool allows(std::int64_t source, std::int64_t receipt_ros, std::int64_t receipt_steady) const {
    std::lock_guard<std::mutex> lock(mutex_); return allows_locked(source, receipt_ros, receipt_steady);
  }
  bool publication_allowed(std::int64_t steady) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return !state_.deferred || state_.first_published_steady_ns ||
      (state_.opened && steady < state_.deadline_steady_ns);
  }
  bool published(std::int64_t source, std::int64_t receipt_ros, std::int64_t receipt_steady,
    std::uint64_t sequence, std::int64_t publication_steady) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_.deferred || state_.first_published_steady_ns || !sequence ||
        publication_steady <= state_.opened_steady_ns || !allows_locked(source, receipt_ros, receipt_steady)) return false;
    if (publication_steady >= state_.deadline_steady_ns) return false;
    state_.first_published_sequence = sequence; state_.first_published_steady_ns = publication_steady;
    return true;
  }
  SpatialStartupStatus snapshot() const {std::lock_guard<std::mutex> lock(mutex_); return state_;}
 private:
  bool allows_locked(std::int64_t source, std::int64_t receipt_ros, std::int64_t receipt_steady) const {
    return !state_.deferred || (state_.opened && source > state_.opened_ros_ns &&
      receipt_ros > state_.opened_ros_ns && receipt_steady > state_.opened_steady_ns);
  }
  mutable std::mutex mutex_;
  bool input_seen_{false};
  SpatialStartupStatus state_;
};
}  // namespace sunhorizon_perception_runtime
