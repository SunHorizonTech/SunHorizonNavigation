#include <gtest/gtest.h>

#include <stdexcept>

#include "sunhorizon_obstacle_coverage/coverage_evaluator.hpp"

using namespace sunhorizon_obstacle_coverage;

namespace {

SourceObservation healthy(double stamp_sec) {
  return {true, true, stamp_sec, stamp_sec};
}

TEST(CoverageEvaluator, ProcessedOutputCannotReplaceMissingStaleOrUnacceptedInputs) {
  CoverageEvaluator evaluator(CoverageProfile::kVisual);
  CoverageInputs inputs; inputs.now_sec = 10.; inputs.chest = healthy(9.95);
  for (const auto& pair : std::vector<std::pair<SourceObservation, SourceObservation>>{
      {healthy(9.95), {}}, {healthy(9.95), healthy(9.5)},
      {healthy(9.5), healthy(9.95)}, {{true, false, 9.95, 9.95}, healthy(9.95)},
      {healthy(9.95), {true, false, 9.95, 9.95}},
      {healthy(9.95), healthy(10.5)}, {healthy(10.5), healthy(9.95)}}) {
    inputs.belly = require_processed_observation(pair.first, pair.second, 10., .3, .3);
    EXPECT_FALSE(evaluator.evaluate(inputs).healthy);
  }
  inputs.belly = require_processed_observation(healthy(9.95), healthy(9.9), 10., .3, .3);
  EXPECT_TRUE(evaluator.evaluate(inputs).healthy);
  EXPECT_DOUBLE_EQ(inputs.belly.stamp_sec, 9.9);
  CoverageEvaluator slower_camera(CoverageProfile::kVisual, {.3, .3, 1.});
  inputs.belly = require_processed_observation(healthy(9.95), healthy(9.5), 10., 1., .3);
  EXPECT_FALSE(slower_camera.evaluate(inputs).healthy);
}

TEST(CoverageProfile, ParsesOnlySupportedProfiles) {
  EXPECT_EQ(parse_profile("lidar"), CoverageProfile::kLidar);
  EXPECT_EQ(parse_profile("visual"), CoverageProfile::kVisual);
  EXPECT_EQ(parse_profile("fusion_shadow"), CoverageProfile::kFusionShadow);
  EXPECT_THROW(parse_profile("fusion"), std::invalid_argument);
}

TEST(CoverageEvaluator, LidarRequiresFreshAiryAndNeverClaimsRear) {
  CoverageEvaluator evaluator(CoverageProfile::kLidar, {0.30, 0.20, 0.20});
  CoverageInputs inputs;
  inputs.now_sec = 10.0;

  auto decision = evaluator.evaluate(inputs);
  EXPECT_FALSE(decision.healthy);
  EXPECT_EQ(decision.required_sources, std::vector<std::string>({"airy"}));
  EXPECT_EQ(decision.reason, "airy_missing");
  EXPECT_FALSE(decision.rear_coverage);

  inputs.airy = healthy(9.71);
  decision = evaluator.evaluate(inputs);
  EXPECT_TRUE(decision.healthy);
  EXPECT_TRUE(decision.forward_coverage);
  EXPECT_TRUE(decision.near_field_coverage);
  EXPECT_FALSE(decision.rear_coverage);

  inputs.airy.stamp_sec = 9.69;
  inputs.airy.receipt_stamp_sec = 9.69;
  decision = evaluator.evaluate(inputs);
  EXPECT_FALSE(decision.healthy);
  EXPECT_EQ(decision.reason, "airy_stale");
}

TEST(CoverageEvaluator, DistinguishesHeaderDelayFromReceiptStaleness) {
  CoverageEvaluator evaluator(CoverageProfile::kLidar, {0.30, 0.20, 0.20});
  CoverageInputs inputs;
  inputs.now_sec = 10.0;

  inputs.airy = {true, true, 9.60, 9.95};
  auto decision = evaluator.evaluate(inputs);
  EXPECT_FALSE(decision.healthy);
  EXPECT_EQ(decision.reason, "airy_queue_delay");

  inputs.airy = {true, true, 9.95, 9.60};
  decision = evaluator.evaluate(inputs);
  EXPECT_FALSE(decision.healthy);
  EXPECT_EQ(decision.reason, "airy_receipt_stale");

  inputs.airy = {true, true, 9.60, 9.60};
  decision = evaluator.evaluate(inputs);
  EXPECT_FALSE(decision.healthy);
  EXPECT_EQ(decision.reason, "airy_stale");
}

TEST(CoverageEvaluator, AcceptsFreshnessBoundaryAndRejectsPastIt) {
  CoverageEvaluator evaluator(CoverageProfile::kLidar, {0.25, 0.20, 0.20});
  CoverageInputs inputs;
  inputs.now_sec = 1.0;
  inputs.airy = {true, true, 0.75, 0.75};

  auto decision = evaluator.evaluate(inputs);
  EXPECT_TRUE(decision.healthy);
  EXPECT_EQ(decision.reason, "ok");

  inputs.airy = {true, true, 0.749, 0.749};
  decision = evaluator.evaluate(inputs);
  EXPECT_FALSE(decision.healthy);
  EXPECT_EQ(decision.reason, "airy_stale");
}

TEST(CoverageEvaluator, FreshLatestRgbdStatusImmediatelyClearsQueueDelay) {
  CoverageEvaluator evaluator(CoverageProfile::kVisual, {0.30, 0.30, 0.30});
  CoverageInputs inputs;
  inputs.now_sec = 10.0;
  inputs.chest = {true, true, 9.0, 9.90};
  inputs.belly = {true, true, 9.92, 9.93};

  auto decision = evaluator.evaluate(inputs);
  EXPECT_FALSE(decision.healthy);
  EXPECT_EQ(decision.reason, "chest_queue_delay");

  inputs.chest = {true, true, 9.92, 9.93};
  decision = evaluator.evaluate(inputs);
  EXPECT_TRUE(decision.healthy);
  EXPECT_EQ(decision.reason, "ok");
}

TEST(CoverageEvaluator, RejectsInvalidReceiptTimestamp) {
  CoverageEvaluator evaluator(CoverageProfile::kLidar);
  CoverageInputs inputs;
  inputs.now_sec = 10.0;
  inputs.airy = {true, true, 9.9, 0.0};

  const auto decision = evaluator.evaluate(inputs);
  EXPECT_FALSE(decision.healthy);
  EXPECT_EQ(decision.reason, "airy_invalid_receipt_stamp");
}

TEST(RgbdCoverageHealth, EvaluatesChestAndBellyIndependently) {
  RgbdCoverageEvidence evidence{
      true, true, true, true, true,
      true, true, true, true};

  auto health = evaluate_rgbd_coverage_health(evidence);
  EXPECT_TRUE(health.chest);
  EXPECT_TRUE(health.belly);

  evidence.chest_pair_fresh = false;
  health = evaluate_rgbd_coverage_health(evidence);
  EXPECT_TRUE(health.chest);
  EXPECT_TRUE(health.belly);

  evidence.belly_pointcloud_healthy = false;
  health = evaluate_rgbd_coverage_health(evidence);
  EXPECT_TRUE(health.chest);
  EXPECT_TRUE(health.belly);

  evidence.chest_camera_info_valid = false;
  evidence.belly_depth_scale_accepted = false;
  health = evaluate_rgbd_coverage_health(evidence);
  EXPECT_FALSE(health.chest);
  EXPECT_FALSE(health.belly);
}

TEST(RgbdCoverageTiming, UsesOlderPairOrPointcloudMeasurement) {
  EXPECT_DOUBLE_EQ(oldest_source_stamp(10.0, 0.10, 0.20), 9.80);
  EXPECT_DOUBLE_EQ(oldest_source_stamp(10.0, 0.25, 0.05), 9.75);
}

TEST(RgbdCoverageTiming, ClampsMeasuredClockSkewAndRejectsInvalidMeasurements) {
  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_DOUBLE_EQ(oldest_source_stamp(10.0, -0.01, 0.10), 9.90);
  EXPECT_DOUBLE_EQ(oldest_source_stamp(10.0, 0.10, -0.01), 9.90);
  EXPECT_DOUBLE_EQ(oldest_source_stamp(10.0, -0.021, 0.10), 0.0);
  EXPECT_DOUBLE_EQ(oldest_source_stamp(10.0, 0.10, -0.021), 0.0);
  EXPECT_DOUBLE_EQ(oldest_source_stamp(10.0, infinity, 0.10), 0.0);
  EXPECT_DOUBLE_EQ(oldest_source_stamp(10.0, 0.10, nan), 0.0);
  EXPECT_DOUBLE_EQ(oldest_source_stamp(0.0, 0.10, 0.10), 0.0);
}

TEST(CoverageEvaluator, VisualRequiresFreshHealthyChestAndBelly) {
  CoverageEvaluator evaluator(CoverageProfile::kVisual, {0.30, 0.20, 0.40});
  CoverageInputs inputs;
  inputs.now_sec = 20.0;
  inputs.chest = healthy(19.81);
  inputs.belly = healthy(19.61);

  auto decision = evaluator.evaluate(inputs);
  EXPECT_TRUE(decision.healthy);
  EXPECT_EQ(
      decision.required_sources,
      std::vector<std::string>({"chest", "belly"}));
  EXPECT_EQ(decision.healthy_sources, decision.required_sources);

  inputs.belly.healthy = false;
  decision = evaluator.evaluate(inputs);
  EXPECT_FALSE(decision.healthy);
  EXPECT_FALSE(decision.forward_coverage);
  EXPECT_FALSE(decision.near_field_coverage);
  EXPECT_EQ(decision.reason, "belly_unhealthy");
}

TEST(CoverageEvaluator, FusionShadowRequiresAllSourcesInStableOrder) {
  CoverageEvaluator evaluator(CoverageProfile::kFusionShadow);
  CoverageInputs inputs;
  inputs.now_sec = 30.0;
  inputs.airy = healthy(29.9);
  inputs.chest = healthy(29.9);

  auto decision = evaluator.evaluate(inputs);
  EXPECT_FALSE(decision.healthy);
  EXPECT_EQ(
      decision.required_sources,
      std::vector<std::string>({"airy", "chest", "belly"}));
  EXPECT_EQ(
      decision.healthy_sources,
      std::vector<std::string>({"airy", "chest"}));
  EXPECT_EQ(decision.reason, "belly_missing");

  inputs.belly = healthy(30.01);
  decision = evaluator.evaluate(inputs);
  EXPECT_FALSE(decision.healthy);
  EXPECT_EQ(decision.reason, "belly_future_header_stamp");
}

TEST(CoverageEvaluator, RejectsInvalidFreshness) {
  EXPECT_THROW(
      CoverageEvaluator(CoverageProfile::kLidar, {0.0, 0.30, 0.30}),
      std::invalid_argument);
}

}  // namespace
