#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <stdexcept>

#include "sunhorizon_navigation_core/geometry_recovery.hpp"

namespace sunhorizon_navigation_core {
namespace {

constexpr std::int64_t kMs = 1000000;

class GeometryRecoveryTest : public ::testing::Test {
 protected:
  GeometryRecoveryInputs in;
  GeometryRecoveryGate gate;

  void SetUp() override {
    in.now_ros_ns = 10000 * kMs;
    in.now_steady_ns = 20000 * kMs;
    in.task_budget_valid = true;
    in.task_id = "task-1";
    in.producer_epoch = "coverage-producer-boot-1";
    in.coverage_contract = "tested-region-contract";
    in.command_owner_epoch = "controller-activation-1";
    in.coverage_seen = true;
    in.coverage = {GeometryCoverageClass::kClear, in.now_ros_ns,
                   in.now_steady_ns, 1U, 1U};
    in.command_seen = true;
    fresh_command();
  }

  void advance(std::int64_t ns) {
    in.now_ros_ns += ns;
    in.now_steady_ns += ns;
  }

  void fresh_command(GeometryCommandKind kind = GeometryCommandKind::kController) {
    in.command = {kind, in.command.sequence + 1U, in.command_owner_epoch,
                  in.now_ros_ns - 2, in.now_ros_ns - 1,
                  in.now_steady_ns - 2, in.now_steady_ns - 1,
                  in.now_steady_ns};
  }

  GeometryRecoveryDecision source(GeometryCoverageClass cls, std::int64_t gap = 50 * kMs) {
    advance(gap);
    in.coverage = {cls, in.now_ros_ns, in.now_steady_ns,
                   in.coverage.frame_sequence + 1U,
                   in.coverage.evaluation_sequence + 1U};
    fresh_command();
    return gate.evaluate(in);
  }

  void hold() {
    in.coverage.classification = GeometryCoverageClass::kGeometryHold;
    const auto result = gate.evaluate(in);
    ASSERT_TRUE(result.hold);
    ASSERT_FALSE(result.fault);
  }

  GeometryRecoveryDecision qualify() {
    hold();
    EXPECT_EQ(source(GeometryCoverageClass::kClear).clear_frames, 1U);
    EXPECT_EQ(source(GeometryCoverageClass::kClear).clear_frames, 2U);
    return source(GeometryCoverageClass::kClear);
  }
};

TEST(GeometryRecoveryConfig, RejectsInvalidAndImpossibleQualification) {
  auto cfg = GeometryRecoveryConfig{};
  cfg.clear_frames = 1U;
  EXPECT_THROW(GeometryRecoveryGate{cfg}, std::invalid_argument);
  cfg = {};
  cfg.clear_span_ns = 201 * kMs;
  EXPECT_THROW(GeometryRecoveryGate{cfg}, std::invalid_argument);
  cfg = {};
  cfg.max_frame_gap_ns = 0;
  EXPECT_THROW(GeometryRecoveryGate{cfg}, std::invalid_argument);
  cfg = {};
  cfg.coverage_timeout_ns = -1;
  EXPECT_THROW(GeometryRecoveryGate{cfg}, std::invalid_argument);
  cfg = {};
  cfg.command_timeout_ns = 0;
  EXPECT_THROW(GeometryRecoveryGate{cfg}, std::invalid_argument);
  cfg = {};
  cfg.max_clock_progress_difference_ns = 0;
  EXPECT_THROW(GeometryRecoveryGate{cfg}, std::invalid_argument);
  cfg = {};
  cfg.clear_frames = std::numeric_limits<std::size_t>::max();
  cfg.max_frame_gap_ns = std::numeric_limits<std::int64_t>::max();
  EXPECT_NO_THROW(GeometryRecoveryGate{cfg});
}

TEST_F(GeometryRecoveryTest, MissingCandidateInitiallyIsIdleButMissingDuringHoldFaults) {
  in.command_seen = false;
  EXPECT_EQ(gate.evaluate(in).state, GeometryRecoveryState::kIdle);
  in.coverage.classification = GeometryCoverageClass::kGeometryHold;
  ++in.coverage.frame_sequence;
  ++in.coverage.evaluation_sequence;
  ++in.coverage.source_ros_ns;
  advance(1);
  EXPECT_EQ(gate.evaluate(in).reason, "candidate_missing_during_geometry_hold");
}

TEST_F(GeometryRecoveryTest, EmptyTaskIsIdleAndDefaultBudgetFailsClosedForActiveTask) {
  auto no_task = in;
  no_task.task_id.clear();
  no_task.command_seen = false;
  no_task.task_budget_valid = false;
  EXPECT_EQ(gate.evaluate(no_task).reason, "idle_without_task");
  in.task_budget_valid = GeometryRecoveryInputs{}.task_budget_valid;
  EXPECT_EQ(gate.evaluate(in).reason, "task_budget_invalid");
}

TEST_F(GeometryRecoveryTest, CandidateLostAfterRunningIsFault) {
  ASSERT_TRUE(gate.evaluate(in).ready);
  in.command_seen = false;
  EXPECT_EQ(gate.evaluate(in).reason, "candidate_missing");
}

TEST_F(GeometryRecoveryTest, SyntheticZeroCannotStartRunningOrReportReady) {
  fresh_command(GeometryCommandKind::kSyntheticZero);
  EXPECT_FALSE(gate.evaluate(in).ready);
  fresh_command();
  ASSERT_TRUE(gate.evaluate(in).ready);
  fresh_command(GeometryCommandKind::kSyntheticZero);
  EXPECT_FALSE(gate.evaluate(in).ready);
}

TEST_F(GeometryRecoveryTest, RepeatTicksDoNotQualifyClear) {
  hold();
  ASSERT_EQ(source(GeometryCoverageClass::kClear).clear_frames, 1U);
  for (int tick = 0; tick < 10; ++tick) {
    advance(kMs);
    fresh_command();
    const auto result = gate.evaluate(in);
    EXPECT_EQ(result.clear_frames, 1U);
    EXPECT_TRUE(result.hold);
  }
}

TEST_F(GeometryRecoveryTest, CompletedFrameCannotChangeEvaluationIdentity) {
  hold();
  source(GeometryCoverageClass::kClear);
  ++in.coverage.evaluation_sequence;
  EXPECT_EQ(gate.evaluate(in).reason, "coverage_same_frame_changed");
}

TEST_F(GeometryRecoveryTest, SameFrameClassificationOrSourceMutationFaults) {
  hold();
  in.coverage.classification = GeometryCoverageClass::kClear;
  ++in.coverage.evaluation_sequence;
  EXPECT_EQ(gate.evaluate(in).reason, "coverage_same_frame_changed");
  GeometryRecoveryGate other;
  in.coverage.classification = GeometryCoverageClass::kClear;
  ASSERT_TRUE(other.evaluate(in).ready);
  --in.coverage.source_ros_ns;
  EXPECT_TRUE(other.evaluate(in).fault);
}

TEST_F(GeometryRecoveryTest, OutOfOrderSourceAndReplayWithNewReceiptFault) {
  ASSERT_TRUE(gate.evaluate(in).ready);
  ++in.coverage.evaluation_sequence;
  --in.coverage.frame_sequence;
  EXPECT_TRUE(gate.evaluate(in).fault);
  GeometryRecoveryGate other;
  SetUp();
  ASSERT_TRUE(other.evaluate(in).ready);
  advance(kMs);
  in.coverage.receipt_steady_ns = in.now_steady_ns;
  EXPECT_EQ(other.evaluate(in).reason, "coverage_replay_changed");
}

TEST_F(GeometryRecoveryTest, FrameAndEvaluationMustAdvanceWithSourceTime) {
  hold();
  ++in.coverage.frame_sequence;
  ++in.coverage.evaluation_sequence;
  EXPECT_EQ(gate.evaluate(in).reason, "coverage_sequence_inconsistent");
}

TEST_F(GeometryRecoveryTest, TooLittleSourceSpanCannotQualifyEvenWithThreeFrames) {
  hold();
  source(GeometryCoverageClass::kClear, kMs);
  source(GeometryCoverageClass::kClear, kMs);
  EXPECT_EQ(source(GeometryCoverageClass::kClear, kMs).state,
            GeometryRecoveryState::kQualifyingClear);
}

TEST_F(GeometryRecoveryTest, LongSourceGapResetsQualificationAndBarrier) {
  ASSERT_EQ(qualify().state, GeometryRecoveryState::kAwaitingCommand);
  const auto result = source(GeometryCoverageClass::kClear, 101 * kMs);
  EXPECT_EQ(result.clear_frames, 1U);
  EXPECT_EQ(result.barrier_ros_ns, 0);
  EXPECT_TRUE(result.hold);
}

TEST_F(GeometryRecoveryTest, NoFurtherClearFrameExpiresBarrierBeforeCoverageTimeout) {
  qualify();
  advance(101 * kMs);
  fresh_command();
  const auto result = gate.evaluate(in);
  EXPECT_FALSE(result.fault);
  EXPECT_TRUE(result.hold);
  EXPECT_FALSE(result.ready);
  EXPECT_EQ(result.clear_frames, 0U);
  EXPECT_EQ(result.barrier_ros_ns, 0);
  EXPECT_EQ(result.state, GeometryRecoveryState::kQualifyingClear);
}

TEST_F(GeometryRecoveryTest, QualificationContinuityExpiresWithoutNewSource) {
  hold();
  source(GeometryCoverageClass::kClear);
  advance(101 * kMs);
  fresh_command();
  EXPECT_EQ(gate.evaluate(in).clear_frames, 0U);
  const auto restarted = source(GeometryCoverageClass::kClear, kMs);
  EXPECT_EQ(restarted.clear_frames, 1U);
  EXPECT_EQ(restarted.barrier_ros_ns, 0);
}

TEST_F(GeometryRecoveryTest, LongReceiptGapResetsQualificationWithoutFalseClockFault) {
  hold();
  source(GeometryCoverageClass::kClear);
  advance(101 * kMs);
  in.coverage.source_ros_ns += 50 * kMs;
  in.coverage.receipt_steady_ns = in.now_steady_ns;
  ++in.coverage.frame_sequence;
  ++in.coverage.evaluation_sequence;
  fresh_command();
  const auto result = gate.evaluate(in);
  EXPECT_FALSE(result.fault);
  EXPECT_EQ(result.clear_frames, 1U);
}

TEST_F(GeometryRecoveryTest, NewClearThenNewComputationRecoversOnce) {
  const auto barrier = qualify();
  ASSERT_EQ(barrier.state, GeometryRecoveryState::kAwaitingCommand);
  EXPECT_EQ(barrier.barrier_ros_ns, in.now_ros_ns);
  EXPECT_EQ(barrier.barrier_steady_ns, in.now_steady_ns);
  EXPECT_FALSE(gate.evaluate(in).ready);
  advance(kMs);
  fresh_command();
  const auto result = gate.evaluate(in);
  EXPECT_TRUE(result.ready);
  EXPECT_TRUE(result.recovered);
  EXPECT_FALSE(result.hold);
  EXPECT_FALSE(gate.evaluate(in).recovered);
}

TEST_F(GeometryRecoveryTest, GeneratedBeforeClearOrAcrossBarrierCannotRecover) {
  const auto barrier = qualify();
  advance(kMs);
  fresh_command();
  in.command.compute_begin_ros_ns = barrier.barrier_ros_ns - 1;
  in.command.compute_begin_steady_ns = barrier.barrier_steady_ns - 1;
  auto result = gate.evaluate(in);
  EXPECT_TRUE(result.hold);
  EXPECT_FALSE(result.ready);
  advance(kMs);
  fresh_command();
  in.command.compute_begin_ros_ns = barrier.barrier_ros_ns;
  in.command.compute_begin_steady_ns = barrier.barrier_steady_ns;
  result = gate.evaluate(in);
  EXPECT_TRUE(result.hold);
  EXPECT_FALSE(result.fault);
}

TEST_F(GeometryRecoveryTest, BothComputeBeginClocksMustBeAfterBarrier) {
  const auto barrier = qualify();
  advance(kMs);
  fresh_command();
  in.command.compute_begin_steady_ns = barrier.barrier_steady_ns;
  EXPECT_TRUE(gate.evaluate(in).hold);
}

TEST_F(GeometryRecoveryTest, SyntheticZeroAndOtherCommandNeverRecover) {
  qualify();
  advance(kMs);
  fresh_command(GeometryCommandKind::kSyntheticZero);
  EXPECT_TRUE(gate.evaluate(in).hold);
  advance(kMs);
  fresh_command(GeometryCommandKind::kOther);
  EXPECT_TRUE(gate.evaluate(in).hold);
  advance(kMs);
  fresh_command();
  EXPECT_TRUE(gate.evaluate(in).recovered);
}

TEST_F(GeometryRecoveryTest, RepeatedCommandSequenceCannotAcquireNewGenerationTimes) {
  qualify();
  const auto sequence = in.command.sequence;
  advance(kMs);
  fresh_command();
  in.command.sequence = sequence;
  EXPECT_EQ(gate.evaluate(in).reason, "command_replay_changed");
}

TEST_F(GeometryRecoveryTest, ReenteredHoldClearsPreviousBarrier) {
  qualify();
  advance(kMs);
  fresh_command();
  ASSERT_TRUE(gate.evaluate(in).recovered);
  const auto held = source(GeometryCoverageClass::kGeometryHold);
  ASSERT_TRUE(held.hold);
  EXPECT_EQ(held.clear_frames, 0U);
  EXPECT_EQ(held.barrier_ros_ns, 0);
  source(GeometryCoverageClass::kClear);
  source(GeometryCoverageClass::kClear);
  ASSERT_EQ(source(GeometryCoverageClass::kClear).state,
            GeometryRecoveryState::kAwaitingCommand);
  EXPECT_FALSE(gate.evaluate(in).ready);
}

TEST_F(GeometryRecoveryTest, HardFaultAlwaysWinsOverHoldAndClearAndStaysLatched) {
  hold();
  in.hard_fault = true;
  EXPECT_EQ(gate.evaluate(in).reason, "hard_fault");
  in.hard_fault = false;
  EXPECT_TRUE(source(GeometryCoverageClass::kClear).fault);
  EXPECT_FALSE(gate.evaluate(in).hold);
  GeometryRecoveryGate other;
  in.coverage.classification = GeometryCoverageClass::kHardFault;
  EXPECT_EQ(other.evaluate(in).reason, "hard_fault");
}

TEST_F(GeometryRecoveryTest, TargetProducerContractAndCommandOwnerChangesFault) {
  for (int field = 0; field < 4; ++field) {
    GeometryRecoveryGate other;
    auto changed = in;
    ASSERT_TRUE(other.evaluate(changed).ready);
    if (field == 0) changed.task_id = "task-2";
    if (field == 1) changed.producer_epoch = "producer-2";
    if (field == 2) changed.coverage_contract = "contract-2";
    if (field == 3) changed.command_owner_epoch = "controller-2";
    EXPECT_EQ(other.evaluate(changed).reason, "identity_changed");
  }
  hold();
  in.command.owner_epoch = "forged-owner";
  EXPECT_EQ(gate.evaluate(in).reason, "command_identity_invalid");
}

TEST_F(GeometryRecoveryTest, RosOrSteadyRollbackFaults) {
  for (int clock = 0; clock < 2; ++clock) {
    GeometryRecoveryGate other;
    ASSERT_TRUE(other.evaluate(in).ready);
    auto changed = in;
    if (clock == 0) --changed.now_ros_ns;
    else --changed.now_steady_ns;
    EXPECT_EQ(other.evaluate(changed).reason, "clock_rollback");
  }
}

TEST_F(GeometryRecoveryTest, FrozenClockUsesCumulativeAnchorNotPerTickTolerance) {
  hold();
  for (int tick = 1; tick <= 10; ++tick) {
    in.now_steady_ns += 10 * kMs;
    EXPECT_FALSE(gate.evaluate(in).fault);
  }
  ++in.now_steady_ns;
  EXPECT_EQ(gate.evaluate(in).reason, "clock_progress_mismatch");
}

TEST_F(GeometryRecoveryTest, ClockForwardJumpAlsoFaults) {
  hold();
  in.now_ros_ns += 101 * kMs;
  EXPECT_EQ(gate.evaluate(in).reason, "clock_progress_mismatch");
}

TEST_F(GeometryRecoveryTest, FrozenSteadyClockAlsoUsesCumulativeAnchor) {
  hold();
  for (int tick = 1; tick <= 10; ++tick) {
    in.now_ros_ns += 10 * kMs;
    EXPECT_FALSE(gate.evaluate(in).fault);
  }
  ++in.now_ros_ns;
  EXPECT_EQ(gate.evaluate(in).reason, "clock_progress_mismatch");
}

TEST_F(GeometryRecoveryTest, RepeatedUnchangedDualClockTicksDoNotAddEvidence) {
  hold();
  source(GeometryCoverageClass::kClear);
  for (int tick = 0; tick < 10; ++tick) {
    const auto result = gate.evaluate(in);
    EXPECT_EQ(result.clear_frames, 1U);
    EXPECT_FALSE(result.fault);
    EXPECT_FALSE(result.ready);
  }
}

TEST_F(GeometryRecoveryTest, WaitingForCommandCannotMaskCommandTimeout) {
  qualify();
  advance(251 * kMs);
  EXPECT_EQ(gate.evaluate(in).reason, "candidate_stale_or_future");
}

TEST_F(GeometryRecoveryTest, StaleOrFutureCoverageInEitherClockFaultsDuringHold) {
  for (int stamp = 0; stamp < 4; ++stamp) {
    GeometryRecoveryGate other;
    auto changed = in;
    changed.coverage.classification = GeometryCoverageClass::kGeometryHold;
    ASSERT_TRUE(other.evaluate(changed).hold);
    if (stamp == 0) changed.coverage.source_ros_ns -= 301 * kMs;
    if (stamp == 1) changed.coverage.source_ros_ns += 1;
    if (stamp == 2) changed.coverage.receipt_steady_ns -= 301 * kMs;
    if (stamp == 3) changed.coverage.receipt_steady_ns += 1;
    EXPECT_EQ(other.evaluate(changed).reason, "coverage_stale_or_future");
  }
}

TEST_F(GeometryRecoveryTest, StaleCandidateCannotWaitForRecoveryOrRenewFromReceipt) {
  qualify();
  in.command.compute_begin_ros_ns = in.now_ros_ns - 251 * kMs;
  in.command.receipt_steady_ns = in.now_steady_ns;
  EXPECT_EQ(gate.evaluate(in).reason, "candidate_stale_or_future");
}

TEST_F(GeometryRecoveryTest, EveryComputeAndReceiptTimestampMustBeFresh) {
  for (int field = 0; field < 5; ++field) {
    GeometryRecoveryGate other;
    auto changed = in;
    auto& cmd = changed.command;
    if (field == 0) cmd.compute_begin_ros_ns -= 251 * kMs;
    if (field == 1) cmd.compute_end_ros_ns += kMs;
    if (field == 2) cmd.compute_begin_steady_ns -= 251 * kMs;
    if (field == 3) cmd.compute_end_steady_ns += kMs;
    if (field == 4) cmd.receipt_steady_ns += kMs;
    EXPECT_EQ(other.evaluate(changed).reason, "candidate_stale_or_future");
  }
}

TEST_F(GeometryRecoveryTest, ComputeEndBeforeBeginOrReceiptBeforeEndFaults) {
  in.command.compute_end_ros_ns = in.command.compute_begin_ros_ns - 1;
  EXPECT_EQ(gate.evaluate(in).reason, "command_compute_interval_invalid");
  GeometryRecoveryGate other;
  SetUp();
  in.command.receipt_steady_ns = in.command.compute_end_steady_ns - 1;
  EXPECT_EQ(other.evaluate(in).reason, "command_compute_interval_invalid");
}

TEST_F(GeometryRecoveryTest, ExistingTaskBudgetExpiryFaultsWithoutReset) {
  qualify();
  in.task_budget_valid = false;
  EXPECT_EQ(gate.evaluate(in).reason, "task_budget_invalid");
  in.task_budget_valid = true;
  advance(kMs);
  fresh_command();
  EXPECT_TRUE(gate.evaluate(in).fault);
}

TEST_F(GeometryRecoveryTest, MissingCoverageAndCandidateInAwaitingCommandFault) {
  qualify();
  in.command_seen = false;
  EXPECT_EQ(gate.evaluate(in).reason, "candidate_missing_during_geometry_hold");
  GeometryRecoveryGate other;
  in.command_seen = true;
  in.coverage_seen = false;
  EXPECT_EQ(other.evaluate(in).reason, "coverage_missing");
}

TEST_F(GeometryRecoveryTest, UnknownClassificationAndInvalidClockFailClosed) {
  in.coverage.classification = static_cast<GeometryCoverageClass>(99);
  EXPECT_EQ(gate.evaluate(in).reason, "coverage_class_invalid");
  GeometryRecoveryGate other;
  in.now_ros_ns = -1;
  EXPECT_EQ(other.evaluate(in).reason, "invalid_clock");
}

}  // namespace
}  // namespace sunhorizon_navigation_core
