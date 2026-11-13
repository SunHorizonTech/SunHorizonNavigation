#pragma once

#include <cstdint>
#include <cmath>
#include <exception>
#include <mutex>
#include <sstream>
#include <string>

namespace sunhorizon_rgbd_adapter {

enum class BellyExit { kRunning, kPublished, kTimestampRejected, kInfoMissing,
  kInvalidDepthOrInfo, kCalibrationRejected, kException, kAborted };

inline const char* belly_exit_name(BellyExit value) {
  switch (value) {
    case BellyExit::kRunning: return "running";
    case BellyExit::kPublished: return "published";
    case BellyExit::kTimestampRejected: return "timestamp_rejected";
    case BellyExit::kInfoMissing: return "info_missing";
    case BellyExit::kInvalidDepthOrInfo: return "invalid_depth_or_info";
    case BellyExit::kCalibrationRejected: return "calibration_rejected";
    case BellyExit::kException: return "exception";
    case BellyExit::kAborted: return "aborted";
  }
  return "unknown";
}

// Two fixed-size snapshots only, not a complete callback/event history.
// All absolute steady values belong to this process's steady clock.
struct BellyStageFrame {
  uint64_t sequence{0};
  int64_t source_ns{0}, entry_ros_ns{0}, entry_steady_ns{0};
  int64_t projection_begin_ns{0}, projection_end_ns{0};
  int64_t publish_begin_ns{0}, publish_return_ns{0}, finish_ns{0};
  BellyExit outcome{BellyExit::kRunning};
};
struct BellyStageSnapshot {
  BellyStageFrame inflight, last_finished;
};

class BellyStageDiagnostics {
 public:
  uint64_t begin(int64_t source, int64_t ros, int64_t steady) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.inflight = {};
    state_.inflight.sequence = ++sequence_;
    state_.inflight.source_ns = source;
    state_.inflight.entry_ros_ns = ros;
    state_.inflight.entry_steady_ns = steady;
    return sequence_;
  }
  void stage(uint64_t sequence, int64_t BellyStageFrame::*field, int64_t steady) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (sequence == state_.inflight.sequence) state_.inflight.*field = steady;
  }
  void finish(uint64_t sequence, BellyExit outcome, int64_t steady) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (sequence != state_.inflight.sequence) return;
    state_.inflight.outcome = outcome;
    state_.inflight.finish_ns = steady;
    state_.last_finished = state_.inflight;
    state_.inflight = {};
  }
  BellyStageSnapshot snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }
 private:
  mutable std::mutex mutex_;
  uint64_t sequence_{0};
  BellyStageSnapshot state_;
};

// Clock injection permits deterministic stage and exception tests. Constructed
// only when enabled; destruction never masks the original callback exception.
template<class Clock> class BellyStageScope {
 public:
  BellyStageScope(BellyStageDiagnostics& owner, int64_t source, int64_t ros, Clock clock)
      : owner_(owner), clock_(clock), exceptions_(std::uncaught_exceptions()),
        sequence_(owner_.begin(source, ros, clock_())) {}
  BellyStageScope(const BellyStageScope&) = delete;
  BellyStageScope& operator=(const BellyStageScope&) = delete;
  ~BellyStageScope() noexcept {
    if (!finished_) {
      try { finish(std::uncaught_exceptions() > exceptions_ ?
          BellyExit::kException : BellyExit::kAborted); } catch (...) {}
    }
  }
  void stage(int64_t BellyStageFrame::*field) { owner_.stage(sequence_, field, clock_()); }
  void finish(BellyExit outcome) {
    owner_.finish(sequence_, outcome, clock_());
    finished_ = true;
  }
 private:
  BellyStageDiagnostics& owner_;
  Clock clock_;
  int exceptions_;
  uint64_t sequence_;
  bool finished_{false};
};

constexpr int64_t kBellyDiagnosticThresholdNs = 150000000;  // Not a safety gate.
inline bool belly_stage_abnormal(const BellyStageFrame& frame, int64_t now) {
  if (!frame.sequence) return false;
  const auto end = frame.finish_ns ? frame.finish_ns : now;
  return frame.source_ns <= 0 || frame.entry_ros_ns < frame.source_ns ||
      frame.entry_ros_ns - frame.source_ns > kBellyDiagnosticThresholdNs ||
      end < frame.entry_steady_ns || end - frame.entry_steady_ns > kBellyDiagnosticThresholdNs ||
      (frame.outcome != BellyExit::kRunning && frame.outcome != BellyExit::kPublished);
}
inline bool belly_stage_abnormal(const BellyStageSnapshot& snapshot, int64_t now) {
  return belly_stage_abnormal(snapshot.inflight, now) ||
      belly_stage_abnormal(snapshot.last_finished, now);
}

inline bool belly_diagnostic_needed(const BellyStageSnapshot& snapshot, int64_t now,
    double status_depth_age_sec, double status_points_age_sec) {
  const auto old_or_missing = [](double age) {
    return !std::isfinite(age) || age < 0. || age > .150;
  };
  // Also report absence of new callbacks: the last completed frame may itself
  // have been fast/fresh, with no inflight callback to exceed a wall threshold.
  return old_or_missing(status_depth_age_sec) || old_or_missing(status_points_age_sec) ||
      belly_stage_abnormal(snapshot, now);
}

inline std::string format_belly_stages(const BellyStageSnapshot& snapshot,
    int64_t observed_steady_ns, int64_t status_header_ns,
    double status_depth_source_sec, double status_points_source_sec) {
  std::ostringstream out;
  out.precision(9); out << std::fixed;
  out << "status_header_ns=" << status_header_ns
      << ",status_snapshot_depth_source_sec=" << status_depth_source_sec
      << ",status_snapshot_points_source_sec=" << status_points_source_sec
      << ",diagnostic_snapshot_steady_after_ns=" << observed_steady_ns;
  const auto write = [&](const char* label, const BellyStageFrame& frame) {
    out << ';' << label << "(seq=" << frame.sequence;
    if (frame.sequence) {
      out << ",source_ns=" << frame.source_ns << ",entry_ros_ns=" << frame.entry_ros_ns
          << ",entry_source_age_ms=" << (static_cast<double>(frame.entry_ros_ns) -
              static_cast<double>(frame.source_ns)) / 1e6
          << ",entry_steady_ns=" << frame.entry_steady_ns
          << ",projection_begin_ns=" << frame.projection_begin_ns
          << ",projection_end_ns=" << frame.projection_end_ns
          << ",publish_begin_ns=" << frame.publish_begin_ns
          << ",publish_return_ns=" << frame.publish_return_ns
          << ",finish_ns=" << frame.finish_ns << ",outcome=" << belly_exit_name(frame.outcome);
    }
    out << ')';
  };
  write("inflight", snapshot.inflight);
  write("last_finished", snapshot.last_finished);
  return out.str();
}

}  // namespace sunhorizon_rgbd_adapter
