#include <gtest/gtest.h>
#include <limits>
#include "sunhorizon_velocity_supervisor/coverage_decision_tracker.hpp"
using namespace sunhorizon_velocity_supervisor;
namespace {
constexpr std::int64_t now = 1000000000;
CombinedCoverageEvidence sample(std::uint8_t disposition = 1) {
  CombinedCoverageEvidence e;
  e.schema = 1; e.disposition = disposition; e.region_disposition = disposition;
  e.region_code = disposition == 1 ? 1 : 3;
  e.complete = e.non_region_healthy = true;
  e.healthy = e.forward = e.near = disposition == 1;
  e.header_ros_ns = now; e.source_ros_ns = now - 10000000;
  e.frame_sequence = 7; e.evaluation_sequence = 10;
  e.frame_id = "base_footprint"; e.profile = "fusion_shadow"; e.contract = "pinned";
  e.required_sources = e.healthy_sources = {"airy", "chest", "belly"};
  return e;
}
CoverageDecisionTracker tracker() { return {"owner", "pinned"}; }
TEST(CoverageDecisionTracker, AttendedReportExpiresAgainstOriginalReceiptAt600ms) {
  auto e = sample();
  e.profile = "visual";
  e.healthy_sources = e.required_sources = {"belly", "chest"};
  CoverageDecisionTracker t("owner", "pinned", "base_footprint", "rgbd-only-v1", 1000000000, 600000000);
  ASSERT_TRUE(t.observe(e, "owner", now, now));
  EXPECT_TRUE(t.evaluate(now + 600000000, now + 600000000).valid);
  EXPECT_FALSE(t.evaluate(now + 600000001, now + 600000001).valid);
}
TEST(CoverageDecisionTracker, ClearAndHoldRequireOneConsistentMessage) {
  for (const auto classification : {1, 2}) {
    auto t = tracker(); auto e = sample(classification);
    ASSERT_TRUE(t.observe(e, "owner", now, now));
    auto d = t.evaluate(now, now);
    EXPECT_TRUE(d.valid); EXPECT_EQ(d.disposition, classification);
    EXPECT_EQ(d.frame_sequence, 7u); EXPECT_EQ(d.source_ros_ns, e.source_ros_ns);
    EXPECT_EQ(d.owner_epoch, "owner"); EXPECT_EQ(d.contract, "pinned");
  }
}
TEST(CoverageDecisionTracker, MissingDoesNotBecomeClearAndBadConfigurationRejected) {
  auto t = tracker(); EXPECT_FALSE(t.evaluate(now, now).valid);
  EXPECT_THROW(CoverageDecisionTracker("", "pinned"), std::invalid_argument);
  EXPECT_THROW(CoverageDecisionTracker("owner", ""), std::invalid_argument);
}
TEST(CoverageDecisionTracker, EveryEvidenceFieldFailsClosedAndCannotBeRearmed) {
  for (int field = 0; field < 21; ++field) {
    SCOPED_TRACE(field);
    auto e = sample(2);
    switch (field) {
      case 0: e.schema = 0; break;
      case 1: e.disposition = 3; break;
      case 2: e.region_disposition = 1; break;
      case 3: e.region_code = 1; break;
      case 4: e.non_region_healthy = false; break;
      case 5: e.complete = false; break;
      case 6: e.healthy = true; break;
      case 7: e.forward = true; break;
      case 8: e.near = true; break;
      case 9: e.rear = true; break;
      case 10: e.header_ros_ns = now + 1; break;
      case 11: e.source_ros_ns = now + 1; break;
      case 12: e.source_ros_ns = now - 300000001; break;
      case 13: e.frame_sequence = 0; break;
      case 14: e.evaluation_sequence = 0; break;
      case 15: e.frame_id = "odom"; break;
      case 16: e.profile = "visual"; break;
      case 17: e.contract = "other"; break;
      case 18: e.required_sources.pop_back(); break;
      case 19: e.healthy_sources.push_back("belly"); break;
      case 20: e.region_code = 255; break;
    }
    auto t = tracker(); EXPECT_FALSE(t.observe(e, "owner", now, now));
    EXPECT_FALSE(t.evaluate(now, now).valid);
    EXPECT_FALSE(t.observe(sample(), "owner", now, now));
  }
}
TEST(CoverageDecisionTracker, HardFaultRevocationWinsAndKeepsOriginalFault) {
  auto t = tracker(); ASSERT_TRUE(t.observe(sample(2), "owner", now, now));
  auto e = sample(); e.disposition = 0; e.source_ros_ns = 0;
  e.complete = false; e.frame_sequence = 0; e.evaluation_sequence = 0;
  EXPECT_FALSE(t.observe(e, "owner", now, now));
  EXPECT_EQ(t.evaluate(now, now).reason, "coverage_decision_hard_fault");
  t.invalidate("another");
  EXPECT_EQ(t.evaluate(now, now).reason, "coverage_decision_hard_fault");
}
TEST(CoverageDecisionTracker, HeartbeatsDoNotRenewSourceFreshness) {
  auto t = tracker(); auto e = sample(2);
  ASSERT_TRUE(t.observe(e, "owner", now, now));
  e.header_ros_ns += 200000000;
  ASSERT_TRUE(t.observe(e, "owner", now + 200000000, now + 200000000));
  EXPECT_TRUE(t.evaluate(now + 200000000, now + 200000000).valid);
  EXPECT_FALSE(t.evaluate(now + 300000000, now + 300000000).valid);
  EXPECT_FALSE(t.observe(sample(), "owner", now + 300000000, now + 300000000));
}
TEST(CoverageDecisionTracker, OwnerAndClockChangesLatch) {
  for (int scenario = 0; scenario < 4; ++scenario) {
    auto t = tracker(); ASSERT_TRUE(t.observe(sample(), "owner", now, now));
    if (scenario == 0) EXPECT_FALSE(t.observe(sample(), "other", now, now));
    if (scenario == 1) EXPECT_FALSE(t.evaluate(now - 1, now).valid);
    if (scenario == 2) EXPECT_FALSE(t.evaluate(now, now - 1).valid);
    if (scenario == 3) EXPECT_FALSE(t.evaluate(now, now + 300000001).valid);
    EXPECT_FALSE(t.evaluate(now + 300000001, now + 300000001).valid);
  }
}
TEST(CoverageDecisionTracker, TerminalIdentityCannotMutateOrReorder) {
  for (int scenario = 0; scenario < 7; ++scenario) {
    SCOPED_TRACE(scenario);
    auto t = tracker(); auto e = sample(2);
    ASSERT_TRUE(t.observe(e, "owner", now, now));
    if (scenario == 0) {e = sample();}
    if (scenario == 1) --e.source_ros_ns;
    if (scenario == 2) ++e.source_ros_ns;
    if (scenario == 3) --e.frame_sequence;
    if (scenario == 4) ++e.evaluation_sequence;
    if (scenario == 5) {++e.frame_sequence; ++e.source_ros_ns;}
    if (scenario == 6) --e.header_ros_ns;
    EXPECT_FALSE(t.observe(e, "owner", now, now));
  }
}
TEST(CoverageDecisionTracker, NewFrameCanClearButDuplicatePreservesIdentity) {
  auto t = tracker(); auto e = sample(2);
  ASSERT_TRUE(t.observe(e, "owner", now, now));
  ASSERT_TRUE(t.observe(e, "owner", now, now));
  e = sample(); ++e.frame_sequence; ++e.evaluation_sequence; ++e.source_ros_ns;
  ASSERT_TRUE(t.observe(e, "owner", now, now));
  EXPECT_EQ(t.evaluate(now, now).disposition, 1);
}
TEST(CoverageDecisionTracker, HeartbeatsPreserveFirstReceiptUntilNewSource) {
  auto t = tracker(); auto e = sample(2);
  ASSERT_TRUE(t.observe(e, "owner", now, now));
  e.header_ros_ns += 50000000;
  ASSERT_TRUE(t.observe(e, "owner", now + 50000000, now + 50000000));
  EXPECT_EQ(t.evaluate(now + 60000000, now + 60000000).receipt_steady_ns, now);
  e.header_ros_ns += 50000000; e.source_ros_ns += 50000000;
  ++e.frame_sequence; ++e.evaluation_sequence;
  ASSERT_TRUE(t.observe(e, "owner", now + 100000000, now + 100000000));
  EXPECT_EQ(t.evaluate(now + 100000000, now + 100000000).receipt_steady_ns,
            now + 100000000);
}
TEST(CoverageDecisionTracker, FirstReceiptExpiresEvenWithPermittedClockSkewAndHeartbeat) {
  auto t = tracker(); auto e = sample(2);
  ASSERT_TRUE(t.observe(e, "owner", now, now));
  e.header_ros_ns += 250000000;
  ASSERT_TRUE(t.observe(e, "owner", now + 250000000, now + 350000000));
  EXPECT_EQ(t.evaluate(now + 250000000, now + 350000000).reason,
            "coverage_decision_stale");
}
TEST(CoverageDecisionTracker, FrozenRosCannotBeKeptAliveByRepeatedHeartbeats) {
  auto t = tracker();
  ASSERT_TRUE(t.observe(sample(), "owner", now, now));
  for (int step = 1; step <= 10; ++step) {
    EXPECT_TRUE(t.observe(sample(), "owner", now, now + step * 10000000));
  }
  EXPECT_FALSE(t.observe(sample(), "owner", now, now + 110000000));
  EXPECT_EQ(t.evaluate(now, now + 110000000).reason,
            "coverage_decision_clock_progress_invalid");
}
TEST(CoverageDecisionTracker, ExtremeUntrustedStampsNeverOverflow) {
  for (auto stamp : {std::numeric_limits<std::int64_t>::min(),
                    std::numeric_limits<std::int64_t>::max()}) {
    auto t = tracker(); auto e = sample(); e.source_ros_ns = stamp;
    EXPECT_FALSE(t.observe(e, "owner", now, now));
  }
}
}  // namespace
