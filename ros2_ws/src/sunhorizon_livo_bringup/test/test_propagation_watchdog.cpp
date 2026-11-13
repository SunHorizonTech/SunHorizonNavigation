#include <gtest/gtest.h>

#include "sunhorizon_livo_bringup/propagation_watchdog.hpp"

namespace
{

using sunhorizon_livo_bringup::PropagationWatchdog;

TEST(PropagationWatchdogTest, RequiresTwoCorrectedAnchorsBeforePropagation)
{
  PropagationWatchdog watchdog(0.5, 1.0, 0.25);
  EXPECT_EQ(
    watchdog.evaluate_propagated(10'000'000'000LL).reason,
    "waiting_for_corrected_odometry");
  ASSERT_TRUE(watchdog.observe_corrected(10'000'000'000LL).accepted);
  EXPECT_FALSE(watchdog.tracking_established());
  EXPECT_EQ(
    watchdog.evaluate_propagated(10'010'000'000LL).reason,
    "waiting_for_stable_corrected_odometry");
  ASSERT_TRUE(watchdog.observe_corrected(10'100'000'000LL).accepted);
  EXPECT_TRUE(watchdog.tracking_established());
  EXPECT_EQ(
    watchdog.evaluate_propagated(10'110'000'000LL).reason,
    "waiting_for_stable_propagated_odometry");
  EXPECT_FALSE(watchdog.ready());
  ASSERT_TRUE(watchdog.observe_corrected(10'500'000'000LL).accepted);
  EXPECT_EQ(
    watchdog.evaluate_propagated(10'600'000'000LL).reason,
    "waiting_for_stable_propagated_odometry");
  ASSERT_TRUE(watchdog.observe_corrected(10'610'000'000LL).accepted);
  EXPECT_TRUE(watchdog.evaluate_propagated(10'620'000'000LL).accepted);
  EXPECT_TRUE(watchdog.ready());
}

TEST(PropagationWatchdogTest, StartupCorrectedGapBeginsNewAnchorEpochWithoutLatch)
{
  PropagationWatchdog watchdog(0.5, 0.1, 0.25);
  ASSERT_TRUE(watchdog.observe_corrected(10'000'000'000LL).accepted);
  EXPECT_EQ(
    watchdog.evaluate_propagated(10'600'000'000LL).reason,
    "waiting_for_stable_corrected_odometry");
  ASSERT_TRUE(watchdog.observe_corrected(10'700'000'000LL).accepted);
  EXPECT_FALSE(watchdog.latched());
  EXPECT_FALSE(watchdog.tracking_established());
  ASSERT_TRUE(watchdog.observe_corrected(10'800'000'000LL).accepted);
  EXPECT_TRUE(watchdog.tracking_established());
  EXPECT_EQ(
    watchdog.evaluate_propagated(10'810'000'000LL).reason,
    "waiting_for_stable_propagated_odometry");
}

TEST(PropagationWatchdogTest, StartupPropagationGapRestartsWithoutLatch)
{
  PropagationWatchdog watchdog(0.5, 0.1, 0.25);
  ASSERT_TRUE(watchdog.observe_corrected(10'000'000'000LL).accepted);
  ASSERT_TRUE(watchdog.observe_corrected(10'100'000'000LL).accepted);
  EXPECT_EQ(
    watchdog.evaluate_propagated(10'110'000'000LL).reason,
    "waiting_for_stable_propagated_odometry");
  EXPECT_EQ(
    watchdog.evaluate_propagated(10'300'000'000LL).reason,
    "waiting_for_stable_propagated_odometry");
  EXPECT_FALSE(watchdog.latched());
  EXPECT_EQ(
    watchdog.evaluate_propagated(10'400'000'000LL).reason,
    "waiting_for_recent_corrected_odometry");
  EXPECT_TRUE(watchdog.tracking_established());
  ASSERT_TRUE(watchdog.observe_corrected(10'500'000'000LL).accepted);
  EXPECT_EQ(
    watchdog.evaluate_propagated(10'500'000'000LL).reason,
    "waiting_for_stable_propagated_odometry");
  EXPECT_EQ(
    watchdog.evaluate_propagated(10'600'000'000LL).reason,
    "waiting_for_stable_propagated_odometry");
  EXPECT_EQ(
    watchdog.evaluate_propagated(10'700'000'000LL).reason,
    "waiting_for_stable_propagated_odometry");
  ASSERT_TRUE(watchdog.observe_corrected(10'800'000'000LL).accepted);
  EXPECT_EQ(
    watchdog.evaluate_propagated(10'800'000'000LL).reason,
    "waiting_for_stable_propagated_odometry");
  EXPECT_EQ(
    watchdog.evaluate_propagated(10'900'000'000LL).reason,
    "waiting_for_stable_propagated_odometry");
  ASSERT_TRUE(watchdog.observe_corrected(11'000'000'000LL).accepted);
  EXPECT_TRUE(watchdog.evaluate_propagated(11'000'000'000LL).accepted);
  EXPECT_TRUE(watchdog.ready());
}

TEST(PropagationWatchdogTest, StopsAndRecoversWhenPropagationOutlivesCorrection)
{
  PropagationWatchdog watchdog(0.5, 1.0, 0.25);
  ASSERT_TRUE(watchdog.observe_corrected(10'000'000'000LL).accepted);
  ASSERT_TRUE(watchdog.observe_corrected(10'100'000'000LL).accepted);
  EXPECT_EQ(
    watchdog.evaluate_propagated(10'200'000'000LL).reason,
    "waiting_for_stable_propagated_odometry");
  ASSERT_TRUE(watchdog.observe_corrected(10'500'000'000LL).accepted);
  ASSERT_TRUE(watchdog.evaluate_propagated(10'700'000'000LL).accepted);
  const auto stale = watchdog.evaluate_propagated(10'760'000'000LL);
  EXPECT_FALSE(stale.accepted);
  EXPECT_EQ(stale.reason, "propagation_unanchored");
  EXPECT_FALSE(watchdog.latched());
  EXPECT_FALSE(watchdog.ready());
  EXPECT_TRUE(watchdog.recovering_from_unanchored());
  EXPECT_EQ(watchdog.unanchored_event_count(), 1U);
  EXPECT_DOUBLE_EQ(watchdog.maximum_observed_horizon_sec(), 0.26);

  ASSERT_TRUE(watchdog.observe_corrected(10'800'000'000LL).accepted);
  EXPECT_EQ(
    watchdog.evaluate_propagated(10'810'000'000LL).reason,
    "waiting_for_stable_propagated_odometry");
  ASSERT_TRUE(watchdog.observe_corrected(11'200'000'000LL).accepted);
  EXPECT_TRUE(watchdog.evaluate_propagated(11'310'000'000LL).accepted);
  EXPECT_TRUE(watchdog.ready());
  EXPECT_FALSE(watchdog.recovering_from_unanchored());
  EXPECT_EQ(watchdog.unanchored_event_count(), 1U);
}

TEST(PropagationWatchdogTest, EstablishedCorrectedGapFailsClosedThenRecovers)
{
  PropagationWatchdog watchdog(0.5, 1.0, 0.25);
  ASSERT_TRUE(watchdog.observe_corrected(10'000'000'000LL).accepted);
  ASSERT_TRUE(watchdog.observe_corrected(10'100'000'000LL).accepted);
  EXPECT_EQ(
    watchdog.evaluate_propagated(10'150'000'000LL).reason,
    "waiting_for_stable_propagated_odometry");
  ASSERT_TRUE(watchdog.observe_corrected(10'500'000'000LL).accepted);
  ASSERT_TRUE(watchdog.evaluate_propagated(10'660'000'000LL).accepted);
  const auto corrected_gap = watchdog.observe_corrected(11'100'000'000LL);
  EXPECT_FALSE(corrected_gap.accepted);
  EXPECT_EQ(corrected_gap.reason, "vendor_odometry_measurement_gap");
  EXPECT_FALSE(watchdog.ready());
  EXPECT_FALSE(watchdog.latched());
  EXPECT_FALSE(watchdog.tracking_established());
  EXPECT_TRUE(watchdog.recovering_from_unanchored());
  EXPECT_EQ(watchdog.unanchored_event_count(), 1U);
  EXPECT_EQ(
    watchdog.evaluate_propagated(11'110'000'000LL).reason,
    "waiting_for_stable_corrected_odometry");
  ASSERT_TRUE(watchdog.observe_corrected(11'200'000'000LL).accepted);
  EXPECT_EQ(
    watchdog.evaluate_propagated(11'210'000'000LL).reason,
    "waiting_for_stable_propagated_odometry");
  ASSERT_TRUE(watchdog.observe_corrected(11'600'000'000LL).accepted);
  EXPECT_TRUE(watchdog.evaluate_propagated(11'720'000'000LL).accepted);
  EXPECT_TRUE(watchdog.ready());
  EXPECT_FALSE(watchdog.recovering_from_unanchored());
}

TEST(PropagationWatchdogTest, EstablishedPropagatedGapFailsClosedThenRecovers)
{
  PropagationWatchdog watchdog(0.5, 0.1, 0.25);
  ASSERT_TRUE(watchdog.observe_corrected(10'000'000'000LL).accepted);
  ASSERT_TRUE(watchdog.observe_corrected(10'100'000'000LL).accepted);
  EXPECT_EQ(
    watchdog.evaluate_propagated(10'150'000'000LL).reason,
    "waiting_for_stable_propagated_odometry");
  EXPECT_FALSE(watchdog.evaluate_propagated(10'240'000'000LL).accepted);
  ASSERT_TRUE(watchdog.observe_corrected(10'300'000'000LL).accepted);
  EXPECT_FALSE(watchdog.evaluate_propagated(10'330'000'000LL).accepted);
  EXPECT_FALSE(watchdog.evaluate_propagated(10'420'000'000LL).accepted);
  ASSERT_TRUE(watchdog.observe_corrected(10'500'000'000LL).accepted);
  EXPECT_FALSE(watchdog.evaluate_propagated(10'510'000'000LL).accepted);
  EXPECT_FALSE(watchdog.evaluate_propagated(10'600'000'000LL).accepted);
  ASSERT_TRUE(watchdog.observe_corrected(10'650'000'000LL).accepted);
  ASSERT_TRUE(watchdog.evaluate_propagated(10'660'000'000LL).accepted);

  const auto propagated_gap = watchdog.evaluate_propagated(10'800'000'000LL);
  EXPECT_FALSE(propagated_gap.accepted);
  EXPECT_EQ(propagated_gap.reason, "vendor_odometry_measurement_gap");
  EXPECT_FALSE(watchdog.ready());
  EXPECT_FALSE(watchdog.latched());
  EXPECT_TRUE(watchdog.tracking_established());
  EXPECT_TRUE(watchdog.recovering_from_unanchored());
  EXPECT_EQ(watchdog.unanchored_event_count(), 1U);

  ASSERT_TRUE(watchdog.observe_corrected(10'800'000'000LL).accepted);
  EXPECT_FALSE(watchdog.evaluate_propagated(10'810'000'000LL).accepted);
  EXPECT_FALSE(watchdog.evaluate_propagated(10'900'000'000LL).accepted);
  EXPECT_FALSE(watchdog.evaluate_propagated(10'990'000'000LL).accepted);
  ASSERT_TRUE(watchdog.observe_corrected(11'000'000'000LL).accepted);
  EXPECT_FALSE(watchdog.evaluate_propagated(11'080'000'000LL).accepted);
  EXPECT_FALSE(watchdog.evaluate_propagated(11'170'000'000LL).accepted);
  ASSERT_TRUE(watchdog.observe_corrected(11'200'000'000LL).accepted);
  EXPECT_FALSE(watchdog.evaluate_propagated(11'260'000'000LL).accepted);
  EXPECT_TRUE(watchdog.evaluate_propagated(11'310'000'000LL).accepted);
  EXPECT_TRUE(watchdog.ready());
  EXPECT_FALSE(watchdog.recovering_from_unanchored());
}

TEST(PropagationWatchdogTest, CorrectedSameEpochDuplicateLatches)
{
  PropagationWatchdog watchdog(0.5, 1.0, 0.25);
  ASSERT_TRUE(watchdog.observe_corrected(10'000'000'000LL).accepted);
  const auto duplicate = watchdog.observe_corrected(10'000'000'000LL);
  EXPECT_FALSE(duplicate.accepted);
  EXPECT_EQ(duplicate.reason, "vendor_odometry_measurement_non_monotonic");
  EXPECT_TRUE(watchdog.latched());
  EXPECT_FALSE(watchdog.observe_corrected(10'100'000'000LL).accepted);
}

TEST(PropagationWatchdogTest, CorrectedOlderStampRemainsLatched)
{
  PropagationWatchdog watchdog(0.5, 1.0, 0.25);
  ASSERT_TRUE(watchdog.observe_corrected(10'000'000'000LL).accepted);
  ASSERT_TRUE(watchdog.observe_corrected(10'100'000'000LL).accepted);
  const auto non_monotonic = watchdog.observe_corrected(10'099'000'000LL);
  EXPECT_FALSE(non_monotonic.accepted);
  EXPECT_EQ(non_monotonic.reason, "vendor_odometry_measurement_non_monotonic");
  EXPECT_TRUE(watchdog.latched());
  EXPECT_FALSE(watchdog.observe_corrected(10'200'000'000LL).accepted);
}

TEST(PropagationWatchdogTest, OlderCrossTopicSampleIsRejectedWithoutLatch)
{
  PropagationWatchdog watchdog(0.5, 1.0, 0.25);
  ASSERT_TRUE(watchdog.observe_corrected(10'100'000'000LL).accepted);
  ASSERT_TRUE(watchdog.observe_corrected(10'200'000'000LL).accepted);
  const auto older = watchdog.evaluate_propagated(10'190'000'000LL);
  EXPECT_FALSE(older.accepted);
  EXPECT_EQ(older.reason, "propagation_precedes_latest_correction");
  EXPECT_FALSE(watchdog.latched());
  EXPECT_EQ(
    watchdog.evaluate_propagated(10'210'000'000LL).reason,
    "waiting_for_stable_propagated_odometry");
}

TEST(PropagationWatchdogTest, ResetStartsANewEpoch)
{
  PropagationWatchdog watchdog(0.5, 1.0, 0.25);
  ASSERT_TRUE(watchdog.observe_corrected(10'000'000'000LL).accepted);
  ASSERT_TRUE(watchdog.observe_corrected(10'100'000'000LL).accepted);
  EXPECT_EQ(
    watchdog.evaluate_propagated(10'210'000'000LL).reason,
    "waiting_for_stable_propagated_odometry");
  ASSERT_TRUE(watchdog.observe_corrected(10'500'000'000LL).accepted);
  ASSERT_TRUE(watchdog.evaluate_propagated(10'710'000'000LL).accepted);
  ASSERT_FALSE(watchdog.evaluate_propagated(10'820'000'000LL).accepted);
  watchdog.reset();
  EXPECT_FALSE(watchdog.latched());
  EXPECT_FALSE(watchdog.tracking_established());
  EXPECT_FALSE(watchdog.ready());
  EXPECT_FALSE(watchdog.recovering_from_unanchored());
  EXPECT_EQ(watchdog.unanchored_event_count(), 0U);
  EXPECT_DOUBLE_EQ(watchdog.maximum_observed_horizon_sec(), 0.0);
  ASSERT_TRUE(watchdog.observe_corrected(20'000'000'000LL).accepted);
  ASSERT_TRUE(watchdog.observe_corrected(20'100'000'000LL).accepted);
  EXPECT_EQ(
    watchdog.evaluate_propagated(20'110'000'000LL).reason,
    "waiting_for_stable_propagated_odometry");
}

TEST(PropagationWatchdogTest, RejectsInvalidConfiguration)
{
  EXPECT_THROW(PropagationWatchdog(0.5, 0.1, 0.0), std::invalid_argument);
}

}  // namespace
