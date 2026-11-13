#include <limits>
#include <deque>
#include "sunhorizon_3d_localizer/ndt_prior_drain.hpp"
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "sunhorizon_3d_localizer/ndt_prior_sync_logic.hpp"

namespace {

using sunhorizon_3d_localizer::NdtPriorSyncDiagnostics;
using sunhorizon_3d_localizer::PriorContinuityGate;
using sunhorizon_3d_localizer::PriorContinuityLimits;
using sunhorizon_3d_localizer::PriorSyncCandidate;

Eigen::Matrix4d pose(double x, double yaw = 0.0) {
  Eigen::Matrix4d output = Eigen::Matrix4d::Identity();
  output.block<3, 3>(0, 0) = Eigen::AngleAxisd(
      yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  output(0, 3) = x;
  return output;
}

TEST(PriorContinuityGate, FirstSampleAnchorsAndSecondContinuousSampleRecovers) {
  PriorContinuityGate gate(PriorContinuityLimits{});
  const auto anchor = gate.evaluate(1'000'000'000LL, pose(0.0));
  EXPECT_FALSE(anchor.matchable);
  EXPECT_FALSE(anchor.epoch_break);
  EXPECT_EQ(anchor.reason, "odometry_prior_anchor_waiting_for_continuity");

  const auto recovered = gate.evaluate(1'100'000'000LL, pose(0.02));
  EXPECT_TRUE(recovered.matchable) << recovered.reason;
  EXPECT_FALSE(recovered.epoch_break);
  EXPECT_EQ(recovered.reason, "ok");
}

TEST(PriorContinuityGate, TranslationJumpKeepsOriginalReasonAndReanchors) {
  PriorContinuityGate gate(PriorContinuityLimits{});
  ASSERT_FALSE(gate.evaluate(1'000'000'000LL, pose(0.0)).matchable);
  const auto jump = gate.evaluate(1'100'000'000LL, pose(0.50));
  EXPECT_FALSE(jump.matchable);
  EXPECT_TRUE(jump.epoch_break);
  EXPECT_EQ(jump.reason, "odometry_prior_translation_jump");

  // This sample is continuous with the jump sample.  It must recover instead
  // of being compared with the old epoch and eventually reported as a gap.
  const auto recovered = gate.evaluate(1'200'000'000LL, pose(0.51));
  EXPECT_TRUE(recovered.matchable) << recovered.reason;
  EXPECT_EQ(recovered.reason, "ok");
}

TEST(PriorContinuityGate, RotationJumpKeepsOriginalReasonAndReanchors) {
  PriorContinuityGate gate(PriorContinuityLimits{});
  ASSERT_FALSE(gate.evaluate(1'000'000'000LL, pose(0.0)).matchable);
  const auto jump = gate.evaluate(1'100'000'000LL, pose(0.0, 0.40));
  EXPECT_TRUE(jump.epoch_break);
  EXPECT_EQ(jump.reason, "odometry_prior_rotation_jump");
  EXPECT_TRUE(gate.evaluate(1'200'000'000LL, pose(0.0, 0.41)).matchable);
}

TEST(PriorContinuityGate, TimeOnlyGapRecoversOnFirstSpatiallyContinuousSample) {
  PriorContinuityGate gate(PriorContinuityLimits{});
  ASSERT_FALSE(gate.evaluate(1'000'000'000LL, pose(0.0)).matchable);
  const auto gap = gate.evaluate(1'300'000'001LL, pose(0.01));
  EXPECT_TRUE(gap.matchable);
  EXPECT_TRUE(gap.epoch_break);
  EXPECT_EQ(
      gap.reason, "odometry_prior_gap_recovered_spatially_continuous");
  EXPECT_TRUE(gate.evaluate(1'400'000'001LL, pose(0.02)).matchable);
}

TEST(PriorContinuityGate, BagSizedGapCannotHideTranslationDiscontinuity) {
  PriorContinuityGate gate(PriorContinuityLimits{0.60, 0.30, 0.2617993877991494});
  ASSERT_FALSE(gate.evaluate(1'000'000'000LL, pose(0.0)).matchable);
  ASSERT_TRUE(gate.evaluate(1'180'000'000LL, pose(0.01)).matchable);

  // The Robot A bag exposed a 6.8 s corrected-prior gap. A resumed pose may
  // recover immediately only if it is also inside the unchanged hard spatial
  // gate; elapsed time must never mask a jump.
  const auto jump = gate.evaluate(7'988'000'000LL, pose(0.32));
  EXPECT_FALSE(jump.matchable);
  EXPECT_TRUE(jump.epoch_break);
  EXPECT_EQ(jump.reason, "odometry_prior_translation_jump");
  EXPECT_TRUE(gate.evaluate(8'168'000'000LL, pose(0.33)).matchable);
}

TEST(PriorContinuityGate, BagSizedStaticGapRecoversWithoutSecondPrior) {
  PriorContinuityGate gate(PriorContinuityLimits{0.60, 0.30, 0.2617993877991494});
  ASSERT_FALSE(gate.evaluate(1'000'000'000LL, pose(0.0)).matchable);
  ASSERT_TRUE(gate.evaluate(1'180'000'000LL, pose(0.01)).matchable);

  const auto recovered = gate.evaluate(7'988'000'000LL, pose(0.02));
  EXPECT_TRUE(recovered.matchable) << recovered.reason;
  EXPECT_TRUE(recovered.epoch_break);
  EXPECT_NEAR(recovered.dt_sec, 6.808, 1.0e-12);
  EXPECT_EQ(
      recovered.reason,
      "odometry_prior_gap_recovered_spatially_continuous");
}

TEST(PriorContinuityGate, InvalidSampleCannotBecomeMatchable) {
  PriorContinuityGate gate(PriorContinuityLimits{});
  ASSERT_FALSE(gate.evaluate(1'000'000'000LL, pose(0.0)).matchable);
  auto invalid = pose(0.0);
  invalid(0, 3) = std::numeric_limits<double>::quiet_NaN();
  const auto decision = gate.evaluate(1'100'000'000LL, invalid);
  EXPECT_FALSE(decision.matchable);
  EXPECT_TRUE(decision.epoch_break);
  EXPECT_EQ(decision.reason, "odometry_prior_pose_non_finite");
  EXPECT_FALSE(gate.evaluate(1'200'000'000LL, pose(0.0)).matchable);
  EXPECT_TRUE(gate.evaluate(1'300'000'000LL, pose(0.01)).matchable);
}

TEST(PriorContinuityGate, RegressedStampBreaksEpochAndRequiresTwoFreshSamples) {
  PriorContinuityGate gate(PriorContinuityLimits{});
  ASSERT_FALSE(gate.evaluate(1'000'000'000LL, pose(0.0)).matchable);
  ASSERT_TRUE(gate.evaluate(1'100'000'000LL, pose(0.01)).matchable);

  const auto regression = gate.evaluate(1'050'000'000LL, pose(0.02));
  EXPECT_FALSE(regression.matchable);
  EXPECT_TRUE(regression.epoch_break);
  EXPECT_EQ(regression.reason, "odometry_prior_stamp_not_strictly_increasing");

  EXPECT_FALSE(gate.evaluate(1'200'000'000LL, pose(0.03)).matchable);
  EXPECT_TRUE(gate.evaluate(1'300'000'000LL, pose(0.04)).matchable);
}

TEST(PriorMatching, IgnoresInvalidPriorEvenWhenItIsClosest) {
  const std::vector<PriorSyncCandidate> candidates{
      {1'000'000'000LL, true},
      {1'045'000'000LL, false},
      {1'070'000'000LL, true}};
  const auto decision = sunhorizon_3d_localizer::select_nearest_matchable_prior(
      1'050'000'000LL, candidates, 50'000'000LL);
  ASSERT_TRUE(decision.matched) << decision.reason;
  EXPECT_EQ(decision.index, 2U);
  EXPECT_EQ(decision.delta_ns, 20'000'000LL);
  EXPECT_TRUE(decision.invalid_prior_within_tolerance);
}

TEST(PriorMatching, ReportsInvalidPriorWithoutMatchingIt) {
  const std::vector<PriorSyncCandidate> candidates{
      {1'045'000'000LL, false}};
  const auto decision = sunhorizon_3d_localizer::select_nearest_matchable_prior(
      1'050'000'000LL, candidates, 50'000'000LL);
  EXPECT_FALSE(decision.matched);
  EXPECT_TRUE(decision.invalid_prior_within_tolerance);
  EXPECT_EQ(decision.reason, "invalid_odometry_prior_not_matchable");
}

TEST(PriorSyncDiagnostics, SeparatesSynchronizationAndRegistrationOutcomes) {
  NdtPriorSyncDiagnostics diagnostics;
  diagnostics.record_no_match();
  diagnostics.record_invalid_prior(true);
  diagnostics.record_recovered_gap();
  diagnostics.record_raw_scan_skip();
  diagnostics.record_match(-40'000'000LL);
  diagnostics.record_match(15'000'000LL);
  diagnostics.record_match_invoked();
  diagnostics.record_match_result(false);
  diagnostics.record_match_invoked();
  diagnostics.record_match_result(true);

  const auto& snapshot = diagnostics.snapshot();
  EXPECT_EQ(snapshot.no_match_count, 1U);
  EXPECT_EQ(snapshot.invalid_prior_count, 1U);
  EXPECT_EQ(snapshot.epoch_break_count, 2U);
  EXPECT_EQ(snapshot.recovered_gap_count, 1U);
  EXPECT_EQ(snapshot.raw_scan_skip_count, 1U);
  EXPECT_EQ(snapshot.match_invoked_count, 2U);
  EXPECT_EQ(snapshot.match_accepted_count, 1U);
  EXPECT_EQ(snapshot.match_rejected_count, 1U);
  EXPECT_EQ(snapshot.last_match_delta_ns, 15'000'000LL);
  EXPECT_EQ(snapshot.maximum_match_delta_ns, 40'000'000LL);
}

}  // namespace


TEST(PriorDrain, BurstReachesLatestMatchWithoutRegisteringEachOldPrior) {
  std::deque<sunhorizon_3d_localizer::PriorSyncCandidate> queue;
  for (int i = 0; i < 99; ++i) queue.push_back({1'000'000'000LL + i*5'000'000LL, true});
  std::vector<sunhorizon_3d_localizer::PriorSyncCandidate> history;
  const bool ready = sunhorizon_3d_localizer::drain_prior_samples<sunhorizon_3d_localizer::PriorSyncCandidate>(
      [&](auto& value) { if (queue.empty()) return false; value=queue.front(); queue.pop_front(); return true; },
      [&](const auto& value) { history.push_back(value); if (history.size()>32) history.erase(history.begin()); return true; }, 100);
  ASSERT_TRUE(ready);
  ASSERT_TRUE(queue.empty());
  const auto match = sunhorizon_3d_localizer::select_nearest_matchable_prior(1'490'000'000LL, history, 60'000'000LL);
  ASSERT_TRUE(match.matched);
  EXPECT_EQ(history[match.index].stamp_ns, 1'490'000'000LL);
  EXPECT_EQ(match.delta_ns, 0);
}

TEST(PriorDrain, DiscontinuityStopsBatchBeforeLaterHealthySample) {
  sunhorizon_3d_localizer::PriorContinuityGate gate({});
  gate.evaluate(1'000'000'000LL, pose(0.0));
  std::deque<double> queue{0.01, 0.50, 0.51};
  std::int64_t stamp=1'000'000'000LL;
  std::string reason;
  const bool ready = sunhorizon_3d_localizer::drain_prior_samples<double>(
      [&](double& value) { if(queue.empty()) return false; value=queue.front();queue.pop_front();return true; },
      [&](double value) { stamp+=5'000'000LL;const auto d=gate.evaluate(stamp,pose(value));reason=d.reason;return d.matchable; },100);
  EXPECT_FALSE(ready);
  EXPECT_EQ(reason,"odometry_prior_translation_jump");
  ASSERT_EQ(queue.size(),1U);
  EXPECT_DOUBLE_EQ(queue.front(),.51);
}

TEST(PriorDrain, ContinuousArrivalHasBoundedWorkAndDefersMatching) {
  int count=0;
  const bool ready = sunhorizon_3d_localizer::drain_prior_samples<int>(
      [&](int& value) {value=++count;return true;},[](int){return true;},100);
  EXPECT_FALSE(ready);
  EXPECT_EQ(count,100);
}

TEST(PriorDrain, EmptyQueueDoesNotInventAContinuitySample) {
  int ingested=0;
  EXPECT_TRUE(sunhorizon_3d_localizer::drain_prior_samples<int>(
      [](int&){return false;},[&](int){++ingested;return true;},100));
  EXPECT_EQ(ingested,0);
}
