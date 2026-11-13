#include <gtest/gtest.h>

#include "sunhorizon_livo_bringup/candidate_admission_diagnostics.hpp"

using sunhorizon_livo_bringup::CandidateAdmissionDiagnostics;
using sunhorizon_livo_bringup::MeasurementFreshnessDecision;

TEST(CandidateAdmissionDiagnosticsTest, ClassifiesFreshnessAndTracksConsecutiveRejects)
{
  CandidateAdmissionDiagnostics diagnostics;

  diagnostics.observe_freshness({false, 0.6, "measurement_stale"});
  diagnostics.observe_freshness({false, -0.1, "measurement_stamp_in_future"});
  auto snapshot = diagnostics.snapshot();
  EXPECT_EQ(snapshot.freshness_stale_rejections, 1U);
  EXPECT_EQ(snapshot.freshness_future_rejections, 1U);
  EXPECT_EQ(snapshot.freshness_invalid_rejections, 0U);
  EXPECT_EQ(snapshot.current_consecutive_freshness_rejections, 2U);
  EXPECT_EQ(snapshot.maximum_consecutive_freshness_rejections, 2U);
  EXPECT_EQ(snapshot.current_freshness_reason, "measurement_stamp_in_future");

  diagnostics.observe_freshness({true, 0.1, "ok"});
  diagnostics.observe_freshness({false, 0.0, "measurement_stamp_invalid"});
  diagnostics.observe_freshness({false, 0.0, "measurement_age_non_finite"});
  snapshot = diagnostics.snapshot();
  EXPECT_EQ(snapshot.freshness_stale_rejections, 1U);
  EXPECT_EQ(snapshot.freshness_future_rejections, 1U);
  EXPECT_EQ(snapshot.freshness_invalid_rejections, 2U);
  EXPECT_EQ(snapshot.current_consecutive_freshness_rejections, 2U);
  EXPECT_EQ(snapshot.maximum_consecutive_freshness_rejections, 2U);
  EXPECT_EQ(snapshot.current_freshness_reason, "measurement_age_non_finite");
}

TEST(CandidateAdmissionDiagnosticsTest, RecordsAggregateSuppressionWithoutChangingTotalsOnClear)
{
  CandidateAdmissionDiagnostics diagnostics;

  diagnostics.record_aggregate_suppression("stale:image");
  diagnostics.record_aggregate_suppression("");
  auto snapshot = diagnostics.snapshot();
  EXPECT_EQ(snapshot.aggregate_suppressions, 2U);
  EXPECT_EQ(
    snapshot.current_aggregate_suppression_reason,
    "all_inputs_unhealthy_unspecified");

  diagnostics.clear_aggregate_suppression();
  snapshot = diagnostics.snapshot();
  EXPECT_EQ(snapshot.aggregate_suppressions, 2U);
  EXPECT_EQ(snapshot.current_aggregate_suppression_reason, "none");
}

TEST(CandidateAdmissionDiagnosticsTest, CountsPublishedCandidatesAndTracksMaximumGaps)
{
  CandidateAdmissionDiagnostics diagnostics;

  diagnostics.record_candidate_published(1'000'000'000LL, 2'000'000'000LL);
  diagnostics.record_candidate_published(1'100'000'000LL, 2'080'000'000LL);
  diagnostics.record_candidate_published(1'350'000'000LL, 2'300'000'000LL);
  auto snapshot = diagnostics.snapshot();
  EXPECT_EQ(snapshot.candidate_published, 3U);
  EXPECT_NEAR(snapshot.maximum_candidate_sensor_stamp_gap_sec, 0.25, 1.0e-12);
  EXPECT_NEAR(snapshot.maximum_candidate_receipt_gap_sec, 0.22, 1.0e-12);

  diagnostics.record_candidate_published(1'200'000'000LL, 2'200'000'000LL);
  diagnostics.record_candidate_published(0, 0);
  snapshot = diagnostics.snapshot();
  EXPECT_EQ(snapshot.candidate_published, 5U);
  EXPECT_NEAR(snapshot.maximum_candidate_sensor_stamp_gap_sec, 0.25, 1.0e-12);
  EXPECT_NEAR(snapshot.maximum_candidate_receipt_gap_sec, 0.22, 1.0e-12);
}
