#define SUNHORIZON_COVERAGE_UPSTREAM_TEST
#include "../src/obstacle_coverage_node.cpp"
#include <gtest/gtest.h>

namespace {
struct CoverageUpstreamTestPeer {
  static void receipt(ObstacleCoverageNode& n, RgbdStatus::SharedPtr s) { n.on_rgbd(s); }
  static void publish(ObstacleCoverageNode& n) { n.publish_status(); }
  static void after_snapshot(ObstacleCoverageNode& n, std::function<void()> callback) { n.after_observation_snapshot_ = std::move(callback); }
  static void observe(ObstacleCoverageNode& n, std::function<void(const ud::Record&, const ObstacleCoverageDecision&)> callback) {
    n.upstream_test_observer_ = std::move(callback);
  }
  static bool allocated(ObstacleCoverageNode& n) { return bool(n.upstream_exporter_); }
};

class UpstreamBinding : public testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    setenv("SUNHORIZON_RGBD_TRACE_SOCKET", "/data/navigation/evidence/component-test.sock", 1);
    rclcpp::NodeOptions options;
    const std::string marker = "typed-region-evaluation-diagnostics-v1";
    options.parameter_overrides({rclcpp::Parameter("profile", "fusion_shadow"),
      rclcpp::Parameter("belly_obstacle_points_topic", "/upstream_binding/points"),
      rclcpp::Parameter("region_support_required", true),
      rclcpp::Parameter("region_typed_evaluation_required", true),
      rclcpp::Parameter("region_support_status_name", "test/region_support"),
      rclcpp::Parameter("region_support_id", "test-region"),
      rclcpp::Parameter("region_support_contract", "10:fixture-v1" + std::to_string(marker.size()) + ":" + marker),
      rclcpp::Parameter("region_support_diagnostics_topic", "/upstream_binding/region")});
    node = std::make_shared<ObstacleCoverageNode>(options);
    CoverageUpstreamTestPeer::observe(*node, [&](const auto& r, const auto& d) { records.emplace_back(r,d); });
  }
  void TearDown() override { node.reset(); unsetenv("SUNHORIZON_RGBD_TRACE_SOCKET"); rclcpp::shutdown(); }
  RgbdStatus::SharedPtr status(int64_t stamp, double age) {
    auto s = std::make_shared<RgbdStatus>(); s->header.stamp = rclcpp::Time(stamp);
    s->belly_depth_age_sec = age; s->belly_pointcloud_age_sec = age + .01;
    s->belly_depth_fresh = s->belly_camera_info_valid = s->belly_depth_scale_accepted = true;
    s->belly_pointcloud_healthy = true; return s;
  }
  std::shared_ptr<ObstacleCoverageNode> node;
  std::vector<std::pair<ud::Record, ObstacleCoverageDecision>> records;
};

TEST_F(UpstreamBinding, ConcurrentNewRgbdAfterInitialSnapshotCannotReplaceActualDecisionReference) {
  const auto h = node->now().nanoseconds() - 1000000;
  const auto old = status(h, .12), newer = status(h + 1000001, .02);
  const auto before = node->now().nanoseconds();
  CoverageUpstreamTestPeer::receipt(*node, old);
  const auto after = node->now().nanoseconds();
  CoverageUpstreamTestPeer::after_snapshot(*node, [&] { CoverageUpstreamTestPeer::receipt(*node, newer); });
  CoverageUpstreamTestPeer::publish(*node);
  ASSERT_EQ(records.size(), 1U);
  const auto& [reference, decision] = records.front();
  EXPECT_EQ(reference.status_header_ns, h);
  EXPECT_EQ(reference.callback_sequence, 1U);
  EXPECT_EQ(reference.ages, ud::age_bits(*old));
  EXPECT_EQ(reference.health, ud::health_bits(*old));
  EXPECT_GE(reference.receipt_ros_ns, before); EXPECT_LE(reference.receipt_ros_ns, after);
  EXPECT_EQ(reference.decision_header_ns, static_cast<int64_t>(decision.coverage.header.stamp.sec) * 1000000000LL + decision.coverage.header.stamp.nanosec);
  EXPECT_EQ(reference.event, ud::Event::kDecisionPublishReturn);
  EXPECT_LE(reference.snapshot_begin_ns, reference.snapshot_end_ns);
  CoverageUpstreamTestPeer::after_snapshot(*node, {});
  CoverageUpstreamTestPeer::publish(*node);
  EXPECT_EQ(records.back().first.status_header_ns, h + 1000001);
  EXPECT_EQ(records.back().first.callback_sequence, 2U);
}

TEST_F(UpstreamBinding, MissingRepeatedAndRollbackHeadersRemainDiagnosticAmbiguity) {
  CoverageUpstreamTestPeer::publish(*node);
  ASSERT_FALSE(records.back().first.input_seen);
  auto s = status(node->now().nanoseconds(), .1);
  CoverageUpstreamTestPeer::receipt(*node, s); CoverageUpstreamTestPeer::publish(*node);
  EXPECT_FALSE(records.back().first.header_ambiguous);
  CoverageUpstreamTestPeer::receipt(*node, s); CoverageUpstreamTestPeer::publish(*node);
  EXPECT_TRUE(records.back().first.header_ambiguous);
  s->header.stamp = rclcpp::Time(1000000000LL);
  CoverageUpstreamTestPeer::receipt(*node, s); CoverageUpstreamTestPeer::publish(*node);
  EXPECT_TRUE(records.back().first.header_ambiguous);
  unsetenv("SUNHORIZON_RGBD_TRACE_SOCKET");
  auto off = std::make_shared<ObstacleCoverageNode>();
  EXPECT_FALSE(CoverageUpstreamTestPeer::allocated(*off));
}
}  // namespace
