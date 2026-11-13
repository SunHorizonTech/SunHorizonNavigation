#define SUNHORIZON_COVERAGE_UPSTREAM_TEST
#include "../src/obstacle_coverage_node.cpp"
#include <gtest/gtest.h>
#include <thread>

namespace {
using FaultReference = sunhorizon_obstacle_coverage::RgbdFaultReference;
struct CoverageUpstreamTestPeer {
  static void receipt(ObstacleCoverageNode& n, RgbdStatus::SharedPtr s) { n.on_rgbd(s); }
  static void publish(ObstacleCoverageNode& n) { n.publish_status(); }
  static void after_snapshot(ObstacleCoverageNode& n, std::function<void()> callback) { n.after_observation_snapshot_ = std::move(callback); }
  static void observe(ObstacleCoverageNode& n, std::function<void(const FaultReference&, const ObstacleCoverageStatus&)> callback) {
    n.fault_test_observer_ = std::move(callback);
  }
  static bool trace_unallocated(ObstacleCoverageNode& n) {
    return !n.upstream_exporter_ && !n.upstream_receipt_queue_ && !n.upstream_decision_queue_ &&
        n.upstream_rgbd_sequence_ == 0 && n.upstream_decision_sequence_ == 0;
  }
};
class FaultBinding : public testing::Test {
 protected:
  void SetUp() override {
    unsetenv("SUNHORIZON_RGBD_TRACE_SOCKET");
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    const std::string marker = "typed-region-evaluation-diagnostics-v1";
    options.parameter_overrides({rclcpp::Parameter("profile", "fusion_shadow"),
      rclcpp::Parameter("belly_obstacle_points_topic", "/fault_binding/points"),
      rclcpp::Parameter("region_support_required", true),
      rclcpp::Parameter("region_typed_evaluation_required", true),
      rclcpp::Parameter("region_support_status_name", "test/region_support"),
      rclcpp::Parameter("region_support_id", "test-region"),
      rclcpp::Parameter("region_support_contract", "10:fixture-v1" + std::to_string(marker.size()) + ":" + marker),
      rclcpp::Parameter("region_support_diagnostics_topic", "/fault_binding/region")});
    node = std::make_shared<ObstacleCoverageNode>(options);
    CoverageUpstreamTestPeer::observe(*node, [&](const auto& r, const auto& s) { records.emplace_back(r, s); });
  }
  void TearDown() override { node.reset(); rclcpp::shutdown(); }
  RgbdStatus::SharedPtr status(int64_t stamp, double age) {
    auto s = std::make_shared<RgbdStatus>(); s->header.stamp = rclcpp::Time(stamp);
    s->belly_depth_age_sec = age; s->belly_pointcloud_age_sec = age + .01;
    s->belly_depth_fresh = s->belly_camera_info_valid = s->belly_depth_scale_accepted = true;
    s->belly_pointcloud_healthy = true; return s;
  }
  std::shared_ptr<ObstacleCoverageNode> node;
  std::vector<std::pair<FaultReference, ObstacleCoverageStatus>> records;
};

// Separate executable: this is the first unhealthy log call at its throttle
// site. Capture the actual trace-OFF log, not merely a formatter or observer.
TEST_F(FaultBinding, TraceOffFaultLogUsesInitialSnapshotDespiteConcurrentNewStatus) {
  const auto h = node->now().nanoseconds() - 1000000;
  const auto a = status(h, .12), b = status(h + 1, .02);
  const auto before = node->now().nanoseconds();
  CoverageUpstreamTestPeer::receipt(*node, a);
  const auto after = node->now().nanoseconds();
  CoverageUpstreamTestPeer::after_snapshot(*node, [&] {
    std::thread newer([&] { CoverageUpstreamTestPeer::receipt(*node, b); });
    newer.join();
  });
  testing::internal::CaptureStderr();
  CoverageUpstreamTestPeer::publish(*node);
  const auto log = testing::internal::GetCapturedStderr();
  ASSERT_EQ(records.size(), 1U);
  const auto& [ref, output] = records.front();
  EXPECT_FALSE(output.healthy);  // Missing other required inputs is intentional.
  EXPECT_EQ(ref.header_ns, h); EXPECT_EQ(ref.callback_sequence, 1U);
  EXPECT_EQ(ref.ages, ud::age_bits(*a)); EXPECT_EQ(ref.health, ud::health_bits(*a));
  EXPECT_GE(ref.receipt_ns, before); EXPECT_LE(ref.receipt_ns, after);
  const int64_t d = static_cast<int64_t>(output.header.stamp.sec) * 1000000000LL + output.header.stamp.nanosec;
  EXPECT_NE(log.find(sunhorizon_obstacle_coverage::rgbd_fault_reference_diagnostics(ref, d)), std::string::npos) << log;
  EXPECT_TRUE(CoverageUpstreamTestPeer::trace_unallocated(*node));
  CoverageUpstreamTestPeer::after_snapshot(*node, {});
  CoverageUpstreamTestPeer::publish(*node);
  EXPECT_EQ(records.back().first.header_ns, h + 1);
  EXPECT_EQ(records.back().first.callback_sequence, 2U);
}

TEST_F(FaultBinding, TraceOffMissingMalformedDuplicateAndRollbackAreDiagnosticOnly) {
  CoverageUpstreamTestPeer::publish(*node);
  ASSERT_FALSE(records.back().first.input_seen);
  auto s = status(node->now().nanoseconds(), .1);
  CoverageUpstreamTestPeer::receipt(*node, s); CoverageUpstreamTestPeer::publish(*node);
  EXPECT_FALSE(records.back().first.header_ambiguous);
  s->belly_depth_age_sec = .2;
  CoverageUpstreamTestPeer::receipt(*node, s); CoverageUpstreamTestPeer::publish(*node);
  EXPECT_TRUE(records.back().first.header_ambiguous);
  EXPECT_EQ(records.back().first.callback_sequence, 2U);
  s->header.stamp.nanosec = 1000000000U;
  CoverageUpstreamTestPeer::receipt(*node, s); CoverageUpstreamTestPeer::publish(*node);
  EXPECT_FALSE(records.back().first.header_valid);
  EXPECT_TRUE(records.back().first.header_ambiguous);
  s->header.stamp = rclcpp::Time(1000000000LL);
  CoverageUpstreamTestPeer::receipt(*node, s); CoverageUpstreamTestPeer::publish(*node);
  EXPECT_TRUE(records.back().first.header_ambiguous);
  EXPECT_TRUE(CoverageUpstreamTestPeer::trace_unallocated(*node));
}
}  // namespace
