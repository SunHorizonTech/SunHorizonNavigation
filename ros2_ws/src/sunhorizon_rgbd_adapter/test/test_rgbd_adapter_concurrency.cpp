#define SUNHORIZON_RGBD_ADAPTER_TEST
#include "../src/rgbd_adapter_node.cpp"

#include <atomic>
#include <condition_variable>
#include <thread>
#include <gtest/gtest.h>

namespace {

struct RgbdAdapterTestPeer {
  static void info(RgbdAdapter& node) {
    auto info = std::make_shared<CameraInfo>();
    info->width = 4; info->height = 4; info->header.frame_id = "camera";
    info->k = {4., 0., 2., 0., 4., 2., 0., 0., 1.};
    node.color_info_ = info; node.depth_info_ = info; node.belly_info_ = info;
    node.update_chest_info_status();
  }
  static void distinct_calibration(RgbdAdapter& node) {
    node.color_info_ = std::make_shared<CameraInfo>(*node.color_info_);
    node.depth_info_ = std::make_shared<CameraInfo>(*node.depth_info_);
    node.color_info_->distortion_model = "rational_polynomial";
    node.color_info_->d = {-.027, .034, -.0004, .0007, -.013, 0, 0, 0};
    node.depth_info_->distortion_model = "plumb_bob";
    node.depth_info_->d.assign(5, 0.);
  }
  static Image::SharedPtr depth(double stamp, bool valid = true) {
    auto image = std::make_shared<Image>();
    image->header.stamp = rclcpp::Time(static_cast<int64_t>(stamp * 1e9));
    image->header.frame_id = "camera";
    image->width = 4; image->height = 4; image->step = 8; image->encoding = "16UC1";
    image->data.resize(32);
    for (std::size_t i = 0; i < 32; i += 2) { image->data[i] = valid ? 232 : 0; image->data[i+1] = valid ? 3 : 0; }
    return image;
  }
  static void chest(RgbdAdapter& node, double stamp, bool valid = true) {
    auto d = depth(stamp, valid);
    auto color = std::make_shared<Image>(*d);
    color->encoding = "rgb8"; color->step = 12; color->data.resize(48);
    node.on_color(color); node.on_depth(d);
  }
  static void belly(RgbdAdapter& node, double stamp, bool valid = true) { node.on_belly_depth(depth(stamp, valid)); }
  static auto snapshot(RgbdAdapter& node) { return node.status_.snapshot(node.now().seconds(), 10.); }
  static auto diagnostic(RgbdAdapter& node) { return node.belly_stage_diagnostics_.snapshot(); }
  static auto group(RgbdAdapter& node, bool chest) { return chest ? node.chest_group_ : node.belly_group_; }
  static void hook(RgbdAdapter& node, int stage, std::function<void()> fn) {
    if (stage == 0) node.before_chest_mask_ = std::move(fn);
    else if (stage == 1) node.before_chest_points_publish_ = std::move(fn);
    else node.before_belly_points_publish_ = std::move(fn);
  }
  static void observe(RgbdAdapter& node, std::function<void(const RgbdStatus&)> fn) { node.status_test_observer_ = std::move(fn); }
  static void observe_upstream(RgbdAdapter& node, std::function<void(const ud::Record&, const RgbdStatus&)> fn) {
    node.upstream_status_test_observer_ = std::move(fn);
  }
  static void belly_ns(RgbdAdapter& node, int64_t stamp) {
    auto image = depth(1.); image->header.stamp = rclcpp::Time(stamp); node.on_belly_depth(image);
  }
  static bool upstream_allocated(RgbdAdapter& node) { return bool(node.upstream_exporter_); }
  static auto cloud(RgbdAdapter& node, const Image& image) { return node.make_pointcloud(image, *node.belly_info_); }
};

struct Gate {
  std::mutex mutex; std::condition_variable cv; bool entered{false}; bool released{false};
  void block() { std::unique_lock<std::mutex> lock(mutex); entered = true; cv.notify_all(); cv.wait_for(lock, std::chrono::seconds(3), [&] { return released; }); }
  bool wait() { std::unique_lock<std::mutex> lock(mutex); return cv.wait_for(lock, std::chrono::seconds(2), [&] { return entered; }); }
  void release() { std::lock_guard<std::mutex> lock(mutex); released = true; cv.notify_all(); }
};

class ConcurrentAdapter : public testing::Test {
 protected:
  virtual bool diagnostic_enabled() const { return false; }
  virtual bool upstream_enabled() const { return false; }
  void SetUp() override {
    if (upstream_enabled()) setenv("SUNHORIZON_RGBD_TRACE_SOCKET", "/data/navigation/evidence/component-test.sock", 1);
    else unsetenv("SUNHORIZON_RGBD_TRACE_SOCKET");
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("chest_alignment_calibration_accepted", true),
      rclcpp::Parameter("chest_depth_scale_accepted", true),
      rclcpp::Parameter("belly_projection_calibration_accepted", true),
      rclcpp::Parameter("belly_depth_scale_accepted", true),
      rclcpp::Parameter("calibration_report_sha256", std::string(64, 'a')),
      rclcpp::Parameter("chest_component_filter_enabled", true),
      rclcpp::Parameter("chest_component_neighbor_delta_m", .05),
      rclcpp::Parameter("chest_component_min_pixels", int64_t{2}),
      rclcpp::Parameter("chest_near_component_min_pixels", int64_t{3}),
      rclcpp::Parameter("chest_near_component_max_depth_m", .6)});
    if (diagnostic_enabled()) options.append_parameter_override("belly_stage_diagnostics_enabled", true);
    node = std::make_shared<RgbdAdapter>(options);
    RgbdAdapterTestPeer::info(*node);
    RgbdAdapterTestPeer::observe(*node, [&](const auto& status) {
      std::lock_guard<std::mutex> lock(observed_mutex); observed.push_back(status); cv.notify_all();
    });
    RgbdAdapterTestPeer::observe_upstream(*node, [&](const auto& record, const auto& status) {
      std::lock_guard<std::mutex> lock(observed_mutex); upstream_observed.emplace_back(record, status); cv.notify_all();
    });
    executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 3);
    executor->add_node(node);
    worker = std::thread([&] { executor->spin(); });
    // Even an empty test must observe a real executor callback before teardown:
    // cancel() before spin() starts can be overwritten by spin's entry flag.
    ASSERT_TRUE(status_until([](const auto&) { return true; }));
  }
  void TearDown() override {
    for (const auto& gate : gates) gate->release();
    // Context shutdown also covers a failed startup barrier followed by a late
    // spin() entry; cancel alone does not cover that ordering.
    rclcpp::shutdown(); executor->cancel(); worker.join(); executor->remove_node(node);
    timers.clear(); node.reset(); executor.reset();
    unsetenv("SUNHORIZON_RGBD_TRACE_SOCKET");
  }
  void schedule(bool chest, std::function<void()> fn) {
    auto once = std::make_shared<std::atomic<bool>>(false);
    timers.push_back(node->create_wall_timer(std::chrono::milliseconds(5), [once, fn] {
      if (!once->exchange(true)) fn();
    }, RgbdAdapterTestPeer::group(*node, chest)));
  }
  bool status_until(std::function<bool(const RgbdStatus&)> condition) {
    std::unique_lock<std::mutex> lock(observed_mutex);
    return cv.wait_for(lock, std::chrono::seconds(2), [&] {
      return !observed.empty() && condition(observed.back());
    });
  }
  std::shared_ptr<Gate> block(int stage) {
    auto gate = std::make_shared<Gate>(); gates.push_back(gate);
    RgbdAdapterTestPeer::hook(*node, stage, [gate] { gate->block(); });
    return gate;
  }
  std::shared_ptr<RgbdAdapter> node;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::thread worker;
  std::vector<rclcpp::TimerBase::SharedPtr> timers;
  std::vector<std::shared_ptr<Gate>> gates;
  std::mutex observed_mutex; std::condition_variable cv; std::vector<RgbdStatus> observed;
  std::vector<std::pair<ud::Record, RgbdStatus>> upstream_observed;
};

class DiagnosticAdapter : public ConcurrentAdapter {
 protected:
  bool diagnostic_enabled() const override { return true; }
};
class ExactDiagnosticAdapter : public ConcurrentAdapter {
 protected:
  bool upstream_enabled() const override { return true; }
};

TEST_F(ConcurrentAdapter, ExactDiagnosticsDefaultOffHasNoQueueOrExporter) {
  EXPECT_FALSE(RgbdAdapterTestPeer::upstream_allocated(*node));
  const auto snapshot = RgbdAdapterTestPeer::snapshot(*node);
  EXPECT_EQ(snapshot.belly_depth_callback_sequence, 0U);
}

TEST_F(ExactDiagnosticAdapter, BlockedPointsPublishBindsDepthBPointsAToActualStatusBits) {
  const auto source_a = node->now().nanoseconds() - 1000000;
  schedule(false, [&, source_a] { RgbdAdapterTestPeer::belly_ns(*node, source_a); });
  ASSERT_TRUE(status_until([](const auto& s) { return s.belly_pointcloud_healthy; }));
  auto gate = block(2);
  const auto source_b = source_a + 1000001;
  schedule(false, [&, source_b] { RgbdAdapterTestPeer::belly_ns(*node, source_b); });
  ASSERT_TRUE(gate->wait());
  std::unique_lock<std::mutex> lock(observed_mutex);
  ASSERT_TRUE(cv.wait_for(lock, std::chrono::seconds(2), [&] {
    return !upstream_observed.empty() && upstream_observed.back().first.depth_source_ns == source_b &&
        upstream_observed.back().first.points_source_ns == source_a;
  }));
  const auto [record, output] = upstream_observed.back();
  EXPECT_EQ(record.depth_callback_sequence, 2U);
  EXPECT_EQ(record.points_callback_sequence, 1U);
  EXPECT_EQ(record.status_header_ns, static_cast<int64_t>(output.header.stamp.sec) * 1000000000LL + output.header.stamp.nanosec);
  EXPECT_EQ(record.ages, ud::age_bits(output));
  EXPECT_EQ(record.health, ud::health_bits(output));
  EXPECT_LE(record.snapshot_begin_ns, record.snapshot_end_ns);
  lock.unlock(); gate->release();
}

TEST_F(ExactDiagnosticAdapter, DiagnosticEnableDoesNotChangeProjectedCloudBytes) {
  unsetenv("SUNHORIZON_RGBD_TRACE_SOCKET");
  auto baseline = std::make_shared<RgbdAdapter>();
  RgbdAdapterTestPeer::info(*baseline);
  const auto image = RgbdAdapterTestPeer::depth(node->now().seconds());
  EXPECT_EQ(RgbdAdapterTestPeer::cloud(*node, *image), RgbdAdapterTestPeer::cloud(*baseline, *image));
  EXPECT_FALSE(RgbdAdapterTestPeer::upstream_allocated(*baseline));
}

TEST_F(ConcurrentAdapter, NormalizationPreservesSeparateColorAndDepthCalibration) {
  RgbdAdapterTestPeer::distinct_calibration(*node);
  auto observer = std::make_shared<rclcpp::Node>("paired_calibration_observer");
  std::mutex mutex;
  std::condition_variable ready;
  CameraInfo::SharedPtr color, depth;
  auto color_sub = observer->create_subscription<CameraInfo>(
    "/sunhorizon/rgbd/color/camera_info", 10, [&](CameraInfo::SharedPtr value) {
      std::lock_guard<std::mutex> lock(mutex); color = value; ready.notify_all();
    });
  auto depth_sub = observer->create_subscription<CameraInfo>(
    "/sunhorizon/rgbd/depth/camera_info", 10, [&](CameraInfo::SharedPtr value) {
      std::lock_guard<std::mutex> lock(mutex); depth = value; ready.notify_all();
    });
  executor->add_node(observer);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while ((!color_sub->get_publisher_count() || !depth_sub->get_publisher_count()) &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  schedule(true, [&] { RgbdAdapterTestPeer::chest(*node, node->now().seconds()); });
  bool received;
  {
    std::unique_lock<std::mutex> lock(mutex);
    received = ready.wait_for(lock, std::chrono::seconds(2), [&] { return color && depth; });
  }
  executor->remove_node(observer);
  ASSERT_TRUE(received);
  EXPECT_EQ(color->distortion_model, "rational_polynomial");
  EXPECT_NEAR(color->d[0], -.027, 1e-12);
  EXPECT_EQ(depth->distortion_model, "plumb_bob");
  EXPECT_EQ(depth->d, std::vector<double>(5, 0.));
  EXPECT_EQ(depth->k, color->k);
  EXPECT_EQ(depth->header, color->header);
}

TEST_F(ConcurrentAdapter, ChestComputationCannotBlockBellyOrRenewOldMeasurement) {
  auto gate = block(0);
  const double source = node->now().seconds();
  schedule(true, [&, source] { RgbdAdapterTestPeer::chest(*node, source); });
  ASSERT_TRUE(gate->wait());
  schedule(false, [&] { RgbdAdapterTestPeer::belly(*node, node->now().seconds()); });
  ASSERT_TRUE(status_until([](const auto& s) { return s.belly_pointcloud_healthy; }));
  ASSERT_TRUE(status_until([](const auto& s) { return s.chest_pair_age_sec > .3 && !s.chest_pair_fresh; }));
  const auto state = RgbdAdapterTestPeer::snapshot(*node);
  EXPECT_NEAR(state.chest_pair_stamp, source, 1e-6);
  EXPECT_EQ(state.chest_points_stamp, 0.);
  EXPECT_GT(state.belly_points_stamp, 0.);
  gate->release();
}

TEST_F(ConcurrentAdapter, ChestPublishBoundaryDoesNotAdvancePointsUntilReturn) {
  auto gate = block(1);
  const double source = node->now().seconds();
  schedule(true, [&, source] { RgbdAdapterTestPeer::chest(*node, source); });
  ASSERT_TRUE(gate->wait());
  EXPECT_EQ(RgbdAdapterTestPeer::snapshot(*node).chest_points_stamp, 0.);
  schedule(false, [&] { RgbdAdapterTestPeer::belly(*node, node->now().seconds()); });
  ASSERT_TRUE(status_until([](const auto& s) { return s.belly_pointcloud_healthy; }));
  gate->release();
  ASSERT_TRUE(status_until([](const auto& s) { return s.chest_pointcloud_age_sec < 1.; }));
  EXPECT_NEAR(RgbdAdapterTestPeer::snapshot(*node).chest_points_stamp, source, 1e-6);
}

TEST_F(ConcurrentAdapter, BellyPendingInvalidAndOutOfOrderPreservePublishedSource) {
  auto gate = block(2);
  const double source = node->now().seconds();
  schedule(false, [&, source] { RgbdAdapterTestPeer::belly(*node, source); });
  ASSERT_TRUE(gate->wait());
  ASSERT_TRUE(status_until([](const auto& s) { return s.belly_depth_fresh && !s.belly_pointcloud_healthy; }));
  EXPECT_EQ(RgbdAdapterTestPeer::snapshot(*node).belly_points_stamp, 0.);
  gate->release();
  ASSERT_TRUE(status_until([](const auto& s) { return s.belly_pointcloud_healthy; }));
  const double invalid_stamp = source + .001;
  schedule(false, [&, invalid_stamp] { RgbdAdapterTestPeer::belly(*node, invalid_stamp, false); });
  ASSERT_TRUE(status_until([](const auto& s) { return s.reason.find("belly_depth_invalid_or_empty") != std::string::npos; }));
  auto state = RgbdAdapterTestPeer::snapshot(*node);
  EXPECT_NEAR(state.belly_points_stamp, source, 1e-6);
  EXPECT_NEAR(state.belly_depth_stamp, invalid_stamp, 1e-6);
  schedule(false, [&, source] { RgbdAdapterTestPeer::belly(*node, source); });
  ASSERT_TRUE(status_until([](const auto& s) { return s.reason.find("belly_non_monotonic_timestamp") != std::string::npos; }));
  state = RgbdAdapterTestPeer::snapshot(*node);
  EXPECT_NEAR(state.belly_points_stamp, source, 1e-6);
  EXPECT_NEAR(state.belly_depth_stamp, invalid_stamp, 1e-6);
}

TEST_F(ConcurrentAdapter, BellyDiagnosticDefaultDisabled) {
  const double source = node->now().seconds();
  schedule(false, [&, source] { RgbdAdapterTestPeer::belly(*node, source); });
  ASSERT_TRUE(status_until([](const auto& s) { return s.belly_pointcloud_healthy; }));
  const auto diagnostic = RgbdAdapterTestPeer::diagnostic(*node);
  EXPECT_EQ(diagnostic.inflight.sequence, 0U);
  EXPECT_EQ(diagnostic.last_finished.sequence, 0U);
}

TEST_F(DiagnosticAdapter, BellyDiagnosticIdentifiesPendingPublishAndInvalidFrame) {
  auto gate = block(2);
  const double source = node->now().seconds();
  schedule(false, [&, source] { RgbdAdapterTestPeer::belly(*node, source); });
  ASSERT_TRUE(gate->wait());
  auto diagnostic = RgbdAdapterTestPeer::diagnostic(*node);
  ASSERT_GT(diagnostic.inflight.sequence, 0U);
  EXPECT_GT(diagnostic.inflight.projection_end_ns, 0);
  EXPECT_GT(diagnostic.inflight.publish_begin_ns, 0);
  EXPECT_EQ(diagnostic.inflight.publish_return_ns, 0);
  EXPECT_EQ(diagnostic.last_finished.sequence, 0U);
  EXPECT_EQ(RgbdAdapterTestPeer::snapshot(*node).belly_points_stamp, 0.);
  gate->release();
  ASSERT_TRUE(status_until([](const auto& s) { return s.belly_pointcloud_healthy; }));
  const auto invalid_source = source + .001;
  schedule(false, [&, invalid_source] { RgbdAdapterTestPeer::belly(*node, invalid_source, false); });
  ASSERT_TRUE(status_until([&](const auto& s) {
    return s.reason.find("belly_depth_invalid_or_empty") != std::string::npos &&
        RgbdAdapterTestPeer::diagnostic(*node).last_finished.outcome ==
            sunhorizon_rgbd_adapter::BellyExit::kInvalidDepthOrInfo;
  }));
  diagnostic = RgbdAdapterTestPeer::diagnostic(*node);
  EXPECT_EQ(diagnostic.inflight.sequence, 0U);
  EXPECT_EQ(diagnostic.last_finished.outcome,
      sunhorizon_rgbd_adapter::BellyExit::kInvalidDepthOrInfo);
  EXPECT_EQ(diagnostic.last_finished.publish_return_ns, 0);
  EXPECT_NEAR(RgbdAdapterTestPeer::snapshot(*node).belly_points_stamp, source, 1e-6);
}

TEST(RgbdStatusSnapshot, BoundedStatisticsRemainUnhealthyUntilOverflowLeavesWindow) {
  sunhorizon_rgbd_adapter::RgbdStatusSnapshot state;
  for (std::size_t i = 0; i <= state.kMaxWindowEvents; ++i) state.rejected(10., 10.);
  auto snapshot = state.snapshot(20., 10.);
  EXPECT_EQ(snapshot.window_rejected, state.kMaxWindowEvents);
  EXPECT_TRUE(snapshot.statistics_overflow);
  EXPECT_EQ(snapshot.rejected_pairs, state.kMaxWindowEvents + 1);
  state.accepted(20., 19., 10.);
  EXPECT_TRUE(state.snapshot(20., 10.).statistics_overflow);
  snapshot = state.snapshot(20.001, 10.);
  EXPECT_FALSE(snapshot.statistics_overflow);
  EXPECT_EQ(snapshot.window_rejected, 0U);
  EXPECT_EQ(snapshot.window_accepted, 1U);
}

}  // namespace
