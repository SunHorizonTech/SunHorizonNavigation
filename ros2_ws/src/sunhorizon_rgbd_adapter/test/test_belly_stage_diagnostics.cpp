#include "sunhorizon_rgbd_adapter/belly_stage_diagnostics.hpp"

#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>

using namespace sunhorizon_rgbd_adapter;

namespace {
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
constexpr int64_t ms = 1000000;
struct Clock {
  int64_t* time;
  int* calls;
  int64_t operator()() { ++*calls; return *time; }
};
using Scope = BellyStageScope<Clock>;
}

int main() {
  try {
    BellyStageDiagnostics state;
    int calls = 0;
    int64_t now = 1000 * ms;
    Clock clock{&now, &calls};
    // Disabled diagnostic scope samples no clock or diagnostic state.
    { std::optional<Scope> disabled; }
    check(calls == 0 && state.snapshot().inflight.sequence == 0, "disabled_clock");

    // Old on arrival, but validation/projection/publication all fast.
    {
      Scope scope(state, 800 * ms, 1000 * ms, clock);
      now += ms; scope.stage(&BellyStageFrame::projection_begin_ns);
      now += ms; scope.stage(&BellyStageFrame::projection_end_ns);
      now += ms; scope.stage(&BellyStageFrame::publish_begin_ns);
      now += ms; scope.stage(&BellyStageFrame::publish_return_ns);
      scope.finish(BellyExit::kPublished);
    }
    auto snap = state.snapshot();
    check(!snap.inflight.sequence && snap.last_finished.outcome == BellyExit::kPublished,
          "finished_not_inflight");
    check(belly_stage_abnormal(snap, now), "input_old_detected");
    check(snap.last_finished.finish_ns - snap.last_finished.entry_steady_ns == 4 * ms,
          "input_delay_not_compute");
    const auto old = snap.last_finished;

    // A previously fast and fresh callback followed by silence must still log.
    BellyStageDiagnostics quiet;
    {
      Scope scope(quiet, 1000 * ms, 1001 * ms, clock);
      scope.stage(&BellyStageFrame::projection_begin_ns);
      scope.stage(&BellyStageFrame::projection_end_ns);
      scope.stage(&BellyStageFrame::publish_begin_ns);
      scope.stage(&BellyStageFrame::publish_return_ns);
      scope.finish(BellyExit::kPublished);
    }
    const auto quiet_snapshot = quiet.snapshot();
    check(!quiet_snapshot.inflight.sequence && !belly_stage_abnormal(quiet_snapshot, now),
          "quiet_frame_was_fast_and_fresh");
    check(!belly_diagnostic_needed(quiet_snapshot, now, .1, .1), "fresh_quiet_no_log");
    check(belly_diagnostic_needed(quiet_snapshot, now + 300 * ms, .301, .301),
          "no_new_callback_gap_detected");
    check(belly_diagnostic_needed({}, now, std::numeric_limits<double>::infinity(), .1),
          "missing_source_detected");
    check(belly_diagnostic_needed(quiet_snapshot, now, .1, .2), "points_only_old_detected");

    // New frame remains distinct from that old finished frame while projection stalls.
    {
      Scope scope(state, 1000 * ms, 1001 * ms, clock);
      now += ms; scope.stage(&BellyStageFrame::projection_begin_ns);
      now += 160 * ms;
      snap = state.snapshot();
      check(snap.last_finished.sequence == old.sequence &&
          snap.inflight.sequence != old.sequence && !snap.inflight.projection_end_ns &&
          !snap.inflight.publish_begin_ns, "projection_phase_distinct");
      check(belly_stage_abnormal(snap.inflight, now), "inflight_projection_delay");
      scope.stage(&BellyStageFrame::projection_end_ns);
      scope.stage(&BellyStageFrame::publish_begin_ns);
      now += ms; scope.stage(&BellyStageFrame::publish_return_ns);
      scope.finish(BellyExit::kPublished);
    }
    snap = state.snapshot();
    check(snap.last_finished.projection_end_ns - snap.last_finished.projection_begin_ns ==
        160 * ms, "projection_duration");
    check(snap.last_finished.publish_return_ns - snap.last_finished.publish_begin_ns == ms,
          "projection_not_publish");

    // Fresh on arrival and projection fast, but publication has not returned.
    {
      Scope scope(state, 1000 * ms, 1001 * ms, clock);
      scope.stage(&BellyStageFrame::projection_begin_ns);
      now += ms; scope.stage(&BellyStageFrame::projection_end_ns);
      scope.stage(&BellyStageFrame::publish_begin_ns);
      now += 170 * ms;
      snap = state.snapshot();
      check(snap.inflight.projection_end_ns && snap.inflight.publish_begin_ns &&
          !snap.inflight.publish_return_ns && !snap.inflight.finish_ns,
          "publish_inflight_not_completed");
      scope.stage(&BellyStageFrame::publish_return_ns);
      scope.finish(BellyExit::kPublished);
    }
    snap = state.snapshot();
    check(snap.last_finished.publish_return_ns - snap.last_finished.publish_begin_ns ==
        170 * ms, "publish_duration");

    // All explicit early exits remain early; no fabricated projection/publication.
    for (auto outcome : {BellyExit::kTimestampRejected, BellyExit::kInfoMissing,
                        BellyExit::kInvalidDepthOrInfo, BellyExit::kCalibrationRejected}) {
      { Scope scope(state, 1000 * ms, 1001 * ms, clock); scope.finish(outcome); }
      snap = state.snapshot();
      check(snap.last_finished.outcome == outcome && !snap.last_finished.publish_return_ns &&
          !snap.last_finished.projection_begin_ns, "early_exit_labeled");
      check(belly_stage_abnormal(snap, now), "early_exit_diagnostic");
    }
    try {
      Scope scope(state, 1000 * ms, 1001 * ms, clock);
      scope.stage(&BellyStageFrame::projection_begin_ns);
      throw std::runtime_error("projection failed");
    } catch (const std::runtime_error&) {}
    snap = state.snapshot();
    check(snap.last_finished.outcome == BellyExit::kException &&
        !snap.last_finished.projection_end_ns && !snap.last_finished.publish_return_ns &&
        !snap.inflight.sequence, "exception_never_published");
    { Scope scope(state, 1000 * ms, 1001 * ms, clock); }
    check(state.snapshot().last_finished.outcome == BellyExit::kAborted, "unfinished_aborted");

    const auto sequence = state.begin(2000 * ms, 2001 * ms, now);
    state.finish(sequence - 1, BellyExit::kPublished, now);
    check(state.snapshot().inflight.sequence == sequence, "old_scope_cannot_finish_new_frame");
    const auto text = format_belly_stages(state.snapshot(), now, 3000 * ms, 1.1, 1.0);
    check(text.find("status_header_ns=3000000000") != std::string::npos &&
        text.find("inflight(seq=") != std::string::npos &&
        text.find("last_finished(seq=") != std::string::npos &&
        text.find("status_snapshot_points_source_sec=1.000000000") != std::string::npos,
        "status_and_diagnostic_sources_labeled");
    std::cout << "belly stage diagnostics deterministic scenarios passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}
