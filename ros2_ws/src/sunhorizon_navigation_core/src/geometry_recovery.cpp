#include "sunhorizon_navigation_core/geometry_recovery.hpp"

#include <limits>
#include <stdexcept>

namespace sunhorizon_navigation_core {
namespace {

bool fresh(std::int64_t now, std::int64_t stamp, std::int64_t timeout) {
  return stamp > 0 && now >= stamp && now - stamp <= timeout;
}

bool progress_matches(std::int64_t ros_delta, std::int64_t steady_delta,
                      std::int64_t tolerance) {
  // Both deltas are nonnegative, so subtracting the smaller cannot overflow.
  return (ros_delta >= steady_delta ? ros_delta - steady_delta :
          steady_delta - ros_delta) <= tolerance;
}

bool same_command(const GeometryCommandEvidence& a, const GeometryCommandEvidence& b) {
  return a.kind == b.kind && a.sequence == b.sequence &&
      a.owner_epoch == b.owner_epoch &&
      a.compute_begin_ros_ns == b.compute_begin_ros_ns &&
      a.compute_end_ros_ns == b.compute_end_ros_ns &&
      a.compute_begin_steady_ns == b.compute_begin_steady_ns &&
      a.compute_end_steady_ns == b.compute_end_steady_ns &&
      a.receipt_steady_ns == b.receipt_steady_ns;
}

}  // namespace

GeometryRecoveryGate::GeometryRecoveryGate(GeometryRecoveryConfig config) : config_(config) {
  // Avoid an overflowing multiplication even for hostile configuration input.
  const auto gaps = config_.clear_frames > 0U ? config_.clear_frames - 1U : 0U;
  const auto max_count = static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max());
  const bool positive = config_.clear_frames >= 2U && config_.clear_span_ns > 0 &&
      config_.max_frame_gap_ns > 0 && config_.coverage_timeout_ns > 0 &&
      config_.command_timeout_ns > 0 && config_.max_clock_progress_difference_ns > 0;
  bool attainable = false;
  if (positive) {
    const auto gap = static_cast<std::uint64_t>(config_.max_frame_gap_ns);
    attainable = gaps > max_count / gap ||
        static_cast<std::uint64_t>(config_.clear_span_ns) <= gaps * gap;
  }
  if (!positive || !attainable) {
    throw std::invalid_argument("geometry_recovery_config_invalid");
  }
}

GeometryRecoveryDecision GeometryRecoveryGate::decision(
    const std::string& reason, bool recovered) const {
  const bool hold = state_ == GeometryRecoveryState::kGeometryHold ||
      state_ == GeometryRecoveryState::kQualifyingClear ||
      state_ == GeometryRecoveryState::kAwaitingCommand;
  return {state_, hold, state_ == GeometryRecoveryState::kFault,
          state_ == GeometryRecoveryState::kRunning, recovered, reason,
          clear_frames_, barrier_ros_ns_, barrier_steady_ns_};
}

void GeometryRecoveryGate::clear_qualification() {
  clear_frames_ = 0U;
  first_clear_source_ns_ = 0;
  last_clear_source_ns_ = 0;
  last_clear_receipt_ns_ = 0;
  barrier_ros_ns_ = 0;
  barrier_steady_ns_ = 0;
  barrier_command_sequence_ = 0U;
}

GeometryRecoveryDecision GeometryRecoveryGate::fail(const std::string& reason) {
  state_ = GeometryRecoveryState::kFault;
  fault_reason_ = reason;
  clear_qualification();
  return decision(fault_reason_);
}

GeometryRecoveryDecision GeometryRecoveryGate::evaluate(const GeometryRecoveryInputs& in) {
  if (state_ == GeometryRecoveryState::kFault) {
    return decision(fault_reason_);
  }
  if (in.hard_fault || (in.coverage_seen &&
      in.coverage.classification == GeometryCoverageClass::kHardFault)) {
    return fail("hard_fault");
  }
  if (in.now_ros_ns <= 0 || in.now_steady_ns <= 0) {
    return fail("invalid_clock");
  }
  if (clocks_seen_) {
    if (in.now_ros_ns < last_ros_ns_ || in.now_steady_ns < last_steady_ns_) {
      return fail("clock_rollback");
    }
    if (!progress_matches(in.now_ros_ns - clock_anchor_ros_ns_,
                          in.now_steady_ns - clock_anchor_steady_ns_,
                          config_.max_clock_progress_difference_ns)) {
      return fail("clock_progress_mismatch");
    }
  } else {
    clocks_seen_ = true;
    clock_anchor_ros_ns_ = in.now_ros_ns;
    clock_anchor_steady_ns_ = in.now_steady_ns;
  }
  last_ros_ns_ = in.now_ros_ns;
  last_steady_ns_ = in.now_steady_ns;

  if (identity_seen_) {
    if (in.task_id != task_id_ || in.producer_epoch != producer_epoch_ ||
        in.coverage_contract != coverage_contract_ ||
        in.command_owner_epoch != command_owner_epoch_) {
      return fail("identity_changed");
    }
  } else {
    if (in.task_id.empty()) {
      if (in.command_seen) {
        return fail("command_without_task");
      }
      return decision("idle_without_task");
    }
    if (in.producer_epoch.empty() || in.coverage_contract.empty() ||
        in.command_owner_epoch.empty()) {
      return fail("identity_missing");
    }
    identity_seen_ = true;
    task_id_ = in.task_id;
    producer_epoch_ = in.producer_epoch;
    coverage_contract_ = in.coverage_contract;
    command_owner_epoch_ = in.command_owner_epoch;
  }
  if (!in.task_budget_valid) {
    return fail("task_budget_invalid");
  }
  if (!in.coverage_seen) {
    return fail("coverage_missing");
  }
  const auto& cov = in.coverage;
  if (cov.classification != GeometryCoverageClass::kClear &&
      cov.classification != GeometryCoverageClass::kGeometryHold) {
    return fail("coverage_class_invalid");
  }
  if (!fresh(in.now_ros_ns, cov.source_ros_ns, config_.coverage_timeout_ns) ||
      !fresh(in.now_steady_ns, cov.receipt_steady_ns, config_.coverage_timeout_ns)) {
    return fail("coverage_stale_or_future");
  }
  if (cov.frame_sequence == 0U || cov.evaluation_sequence == 0U) {
    return fail("coverage_sequence_invalid");
  }
  bool new_source = !coverage_seen_;
  if (coverage_seen_) {
    const auto& last = last_coverage_;
    if (cov.frame_sequence < last.frame_sequence ||
        cov.evaluation_sequence < last.evaluation_sequence ||
        cov.source_ros_ns < last.source_ros_ns ||
        cov.receipt_steady_ns < last.receipt_steady_ns) {
      return fail("coverage_out_of_order");
    }
    new_source = cov.frame_sequence > last.frame_sequence;
    if (new_source && (cov.source_ros_ns <= last.source_ros_ns ||
        cov.evaluation_sequence <= last.evaluation_sequence)) {
      return fail("coverage_sequence_inconsistent");
    }
    if (!new_source && (cov.source_ros_ns != last.source_ros_ns ||
        cov.classification != last.classification ||
        cov.evaluation_sequence != last.evaluation_sequence)) {
      return fail("coverage_same_frame_changed");
    }
    if (cov.evaluation_sequence == last.evaluation_sequence &&
        (new_source || cov.receipt_steady_ns != last.receipt_steady_ns)) {
      return fail("coverage_replay_changed");
    }
  }
  coverage_seen_ = true;
  last_coverage_ = cov;

  if (!in.command_seen) {
    if (held_ || cov.classification == GeometryCoverageClass::kGeometryHold) {
      return fail("candidate_missing_during_geometry_hold");
    }
    // The caller retains its existing initial-controller grace policy. Once
    // any command was observed, losing it is no longer an initial idle state.
    if (command_seen_) {
      return fail("candidate_missing");
    }
    return decision("waiting_for_candidate");
  }
  const auto& cmd = in.command;
  if (cmd.owner_epoch != command_owner_epoch_ || cmd.sequence == 0U) {
    return fail("command_identity_invalid");
  }
  if (cmd.kind != GeometryCommandKind::kController &&
      cmd.kind != GeometryCommandKind::kSyntheticZero && cmd.kind != GeometryCommandKind::kOther) {
    return fail("command_kind_invalid");
  }
  if (!fresh(in.now_ros_ns, cmd.compute_begin_ros_ns, config_.command_timeout_ns) ||
      !fresh(in.now_ros_ns, cmd.compute_end_ros_ns, config_.command_timeout_ns) ||
      !fresh(in.now_steady_ns, cmd.compute_begin_steady_ns, config_.command_timeout_ns) ||
      !fresh(in.now_steady_ns, cmd.compute_end_steady_ns, config_.command_timeout_ns) ||
      !fresh(in.now_steady_ns, cmd.receipt_steady_ns, config_.command_timeout_ns)) {
    return fail("candidate_stale_or_future");
  }
  if (cmd.compute_end_ros_ns < cmd.compute_begin_ros_ns ||
      cmd.compute_end_steady_ns < cmd.compute_begin_steady_ns ||
      cmd.receipt_steady_ns < cmd.compute_end_steady_ns ||
      !progress_matches(cmd.compute_end_ros_ns - cmd.compute_begin_ros_ns,
                        cmd.compute_end_steady_ns - cmd.compute_begin_steady_ns,
                        config_.max_clock_progress_difference_ns)) {
    return fail("command_compute_interval_invalid");
  }
  if (command_seen_) {
    if (cmd.sequence < last_command_.sequence ||
        cmd.compute_begin_ros_ns < last_command_.compute_begin_ros_ns ||
        cmd.compute_end_ros_ns < last_command_.compute_end_ros_ns ||
        cmd.compute_begin_steady_ns < last_command_.compute_begin_steady_ns ||
        cmd.compute_end_steady_ns < last_command_.compute_end_steady_ns ||
        cmd.receipt_steady_ns < last_command_.receipt_steady_ns) {
      return fail("command_out_of_order");
    }
    if (cmd.sequence == last_command_.sequence && !same_command(cmd, last_command_)) {
      return fail("command_replay_changed");
    }
  }
  command_seen_ = true;
  last_command_ = cmd;

  if (cov.classification == GeometryCoverageClass::kGeometryHold) {
    held_ = true;
    state_ = GeometryRecoveryState::kGeometryHold;
    clear_qualification();
    return decision("geometry_hold");
  }
  if (!held_) {
    if (cmd.kind != GeometryCommandKind::kController) {
      state_ = GeometryRecoveryState::kIdle;
      return decision("waiting_for_controller_command");
    }
    state_ = GeometryRecoveryState::kRunning;
    return decision("running");
  }

  // A stopped Clear stream loses continuity even while the broader coverage
  // freshness budget remains valid. Timer ticks/new command callbacks cannot
  // preserve a barrier waiting for the next distinct source to expose the gap.
  if (clear_frames_ > 0U &&
      in.now_steady_ns - last_clear_receipt_ns_ > config_.max_frame_gap_ns) {
    clear_qualification();
  }
  if (new_source) {
    if (clear_frames_ > 0U &&
        (cov.source_ros_ns - last_clear_source_ns_ > config_.max_frame_gap_ns ||
         cov.receipt_steady_ns - last_clear_receipt_ns_ > config_.max_frame_gap_ns)) {
      clear_qualification();
    }
    if (clear_frames_ == 0U) {
      first_clear_source_ns_ = cov.source_ros_ns;
    }
    if (clear_frames_ < config_.clear_frames) {
      ++clear_frames_;
    }
    last_clear_source_ns_ = cov.source_ros_ns;
    last_clear_receipt_ns_ = cov.receipt_steady_ns;
  }
  if (clear_frames_ < config_.clear_frames ||
      cov.source_ros_ns - first_clear_source_ns_ < config_.clear_span_ns) {
    state_ = GeometryRecoveryState::kQualifyingClear;
    return decision("qualifying_clear");
  }
  if (barrier_ros_ns_ == 0) {
    barrier_ros_ns_ = in.now_ros_ns;
    barrier_steady_ns_ = in.now_steady_ns;
    barrier_command_sequence_ = cmd.sequence;
    state_ = GeometryRecoveryState::kAwaitingCommand;
    return decision("waiting_for_post_clear_command");
  }
  if (cmd.kind != GeometryCommandKind::kController ||
      cmd.sequence <= barrier_command_sequence_ ||
      cmd.compute_begin_ros_ns <= barrier_ros_ns_ ||
      cmd.compute_begin_steady_ns <= barrier_steady_ns_) {
    state_ = GeometryRecoveryState::kAwaitingCommand;
    return decision("waiting_for_post_clear_command");
  }
  held_ = false;
  state_ = GeometryRecoveryState::kRunning;
  clear_qualification();
  return decision("recovered", true);
}

}  // namespace sunhorizon_navigation_core
