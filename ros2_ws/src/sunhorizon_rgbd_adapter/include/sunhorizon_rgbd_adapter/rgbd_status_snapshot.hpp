#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <utility>

namespace sunhorizon_rgbd_adapter {

struct RgbdStatusState {
  double chest_pair_stamp{0.0};
  double chest_points_stamp{0.0};
  double belly_depth_stamp{0.0};
  double belly_points_stamp{0.0};
  // Diagnostic identities share the exact business-state update/snapshot lock.
  int64_t belly_depth_source_ns{0}, belly_points_source_ns{0};
  uint64_t belly_depth_callback_sequence{0}, belly_points_callback_sequence{0};
  double last_pair_delta_sec{0.0};
  bool chest_info_valid{false};
  bool belly_info_valid{false};
  std::string last_rejection_reason;
  std::string belly_reason;
  uint64_t accepted_pairs{0};
  uint64_t rejected_pairs{0};
  std::size_t window_accepted{0};
  std::size_t window_rejected{0};
  bool statistics_overflow{false};
};

// Only scalar/small-string state is shared between the three callback groups.
// Image ownership, validation, projection and all publications stay outside this
// lock. Window queues are bounded; overflow remains unhealthy for a full window.
class RgbdStatusSnapshot {
 public:
  static constexpr std::size_t kMaxWindowEvents = 4096;

  template<class Update>
  void update(Update&& update) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::forward<Update>(update)(state_);
  }

  void accepted(double now, double source_stamp, double window) {
    std::lock_guard<std::mutex> lock(mutex_);
    expire(now, window);
    state_.last_rejection_reason.clear();
    state_.chest_pair_stamp = source_stamp;
    ++state_.accepted_pairs;
    append(acceptance_stamps_, now);
  }

  void rejected(double now, double window) {
    std::lock_guard<std::mutex> lock(mutex_);
    expire(now, window);
    ++state_.rejected_pairs;
    append(rejection_stamps_, now);
  }

  RgbdStatusState snapshot(double now, double window) {
    std::lock_guard<std::mutex> lock(mutex_);
    expire(now, window);
    state_.window_accepted = acceptance_stamps_.size();
    state_.window_rejected = rejection_stamps_.size();
    return state_;
  }

 private:
  void append(std::deque<double>& events, double now) {
    if (events.size() == kMaxWindowEvents) {
      events.pop_front();
      overflow_stamp_ = now;
      state_.statistics_overflow = true;
    }
    events.push_back(now);
  }

  void expire(double now, double window) {
    for (auto* events : {&acceptance_stamps_, &rejection_stamps_}) {
      while (!events->empty() && now - events->front() > window) events->pop_front();
    }
    if (state_.statistics_overflow && now - overflow_stamp_ > window) {
      state_.statistics_overflow = false;
    }
  }

  std::mutex mutex_;
  RgbdStatusState state_;
  std::deque<double> acceptance_stamps_;
  std::deque<double> rejection_stamps_;
  double overflow_stamp_{0.0};
};

}  // namespace sunhorizon_rgbd_adapter
