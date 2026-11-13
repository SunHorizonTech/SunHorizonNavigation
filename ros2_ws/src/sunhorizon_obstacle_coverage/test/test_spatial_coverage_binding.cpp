#define SUNHORIZON_COVERAGE_UPSTREAM_TEST
#include "../src/obstacle_coverage_node.cpp"
#include <gtest/gtest.h>
#include <thread>
#include <future>
#include "sunhorizon_velocity_supervisor/spatial_coverage_decision_tracker.hpp"

namespace {
struct CoverageUpstreamTestPeer {
  static void cancel_timer(ObstacleCoverageNode& n, bool pause_events = true) {
    n.status_timer_->cancel();
    if (pause_events) n.spatial_publication_.reset();
  }
  static void publish(ObstacleCoverageNode& n) { n.publish_status(); }
  static void receive(ObstacleCoverageNode& n, SpatialCoverageEvidence::ConstSharedPtr msg,
                      const rclcpp::MessageInfo& info) { n.on_spatial(msg, info); }
  static void observe(ObstacleCoverageNode& n,
      std::function<void(const ObstacleCoverageSpatialDecision&)> callback) {
    n.spatial_test_observer_ = std::move(callback);
  }
  static void after_snapshot(ObstacleCoverageNode& n, std::function<void()> callback) {
    n.after_observation_snapshot_ = std::move(callback);
  }
  static std::string endpoint_error(ObstacleCoverageNode& n) { return n.spatial_endpoint_error(); }
  static bool legacy_absent(ObstacleCoverageNode& n) { return !n.decision_pub_ && !n.region_tracker_; }
  static std::string owner(ObstacleCoverageNode& n) { return n.spatial_owner_gid_; }
  static void endpoints(ObstacleCoverageNode& n, std::vector<rclcpp::TopicEndpointInfo> peers) {
    n.spatial_test_endpoints_ = [peers] { return peers; };
  }
  static bool direct_only(ObstacleCoverageNode& n,const std::shared_ptr<tf2_ros::Buffer>& buffer) {
    return !n.airy_sub_&&!n.belly_sub_&&!n.rgbd_sub_&&!n.region_sub_&&!n.spatial_sub_&&
      !n.tf_listener_ && !n.tf_callback_group_ && n.transforms_==buffer;
  }
  static void no_airy(ObstacleCoverageNode& n) {
    const std::lock_guard<std::mutex> lock(n.observations_mutex_);n.airy_cloud_={};
  }
  static bool tf_group_is_independent(ObstacleCoverageNode& n) {
    return n.tf_listener_ && n.tf_callback_group_ &&
      n.tf_callback_group_!=n.status_callback_group_ && n.tf_callback_group_!=n.airy_callback_group_ &&
      n.tf_callback_group_!=n.rgbd_callback_group_ &&
      n.tf_callback_group_->automatically_add_to_executor_with_node();
  }
  static bool has_transform(ObstacleCoverageNode& n,const std::string& child) {
    return n.tf_buffer_.canTransform("base_footprint",child,tf2::TimePointZero);
  }
  static geometry_msgs::msg::TransformStamped transform(ObstacleCoverageNode& n,const std::string& child) {
    return n.tf_buffer_.lookupTransform("base_footprint",child,tf2::TimePointZero);
  }
  static double belly_receipt(ObstacleCoverageNode& n) {return n.belly_cloud_.receipt_stamp_sec;}
  static bool seed_at_original_receipt(ObstacleCoverageNode& n,const SpatialCoverageEvidence& input,
      int64_t ros,int64_t steady) {
    const std::lock_guard<std::mutex> lock(n.observations_mutex_);
    const bool accepted=n.spatial_tracker_->observe(
      sunhorizon_obstacle_coverage::spatial_evidence_observation(input),n.spatial_owner_gid_,ros,steady);
    if(accepted){n.spatial_wire_=input;n.spatial_seen_=true;}
    return accepted;
  }
  static void actual_endpoints(ObstacleCoverageNode& n) { n.spatial_test_endpoints_ = {}; }
  static void sources(ObstacleCoverageNode& n, bool healthy = true) {
    const std::lock_guard<std::mutex> lock(n.observations_mutex_);
    const auto now = n.now().seconds() - .001;
    n.airy_cloud_ = {true, healthy, now, now, "base_footprint"};
    n.belly_cloud_ = {true, healthy, now, now, "base_footprint"};
    n.chest_status_ = {true, healthy, now, now};
    n.belly_status_ = {true, healthy, now, now};
  }
};

class SpatialBinding : public testing::Test {
 protected:
  void SetUp() override {
    unsetenv("SUNHORIZON_RGBD_TRACE_SOCKET");
    rclcpp::init(0, nullptr);
    producer = std::make_shared<rclcpp::Node>("spatial_fixture");
    producer->declare_parameter("camera_freshness_profile",
      sunhorizon_obstacle_coverage::kStrictCameraFreshness);
    publisher = producer->create_publisher<SpatialCoverageEvidence>(kSpatialInputTopic, latest_status_qos());
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    std::vector<rclcpp::TopicEndpointInfo> peers;
    do {
      peers = producer->get_publishers_info_by_topic(kSpatialInputTopic);
      if (peers.size() == 1) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while (std::chrono::steady_clock::now() < end);
    ASSERT_EQ(peers.size(), 1U);
    gid = spatial_gid_hex(peers[0].endpoint_gid().data(), peers[0].endpoint_gid().size());
    std::copy(peers[0].endpoint_gid().begin(), peers[0].endpoint_gid().end(), info.publisher_gid.data);
    node = std::make_shared<ObstacleCoverageNode>(options());
    CoverageUpstreamTestPeer::cancel_timer(*node);
    do {
      if (CoverageUpstreamTestPeer::endpoint_error(*node).empty()) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while (std::chrono::steady_clock::now() < end);
    ASSERT_TRUE(CoverageUpstreamTestPeer::endpoint_error(*node).empty());
    CoverageUpstreamTestPeer::observe(*node, [&](const auto& value) { output.push_back(value); });
  }
  void TearDown() override {
    node.reset(); publisher.reset(); producer.reset(); rclcpp::shutdown();
  }
  rclcpp::NodeOptions options() const {
    rclcpp::NodeOptions out;
    out.parameter_overrides({rclcpp::Parameter("profile", "fusion_shadow"),
      rclcpp::Parameter("belly_obstacle_points_topic", "/spatial_fixture/processed"),
      rclcpp::Parameter("belly_frame_id", "base_footprint"),
      rclcpp::Parameter("chest_frame_id", "base_footprint"),
      rclcpp::Parameter("spatial_visibility_required", true),
      rclcpp::Parameter("spatial_producer_gid", gid),
      rclcpp::Parameter("spatial_producer_node", "/spatial_fixture"),
      rclcpp::Parameter("spatial_contract", "spatial-test-v1"),
      rclcpp::Parameter("spatial_query_digest", "query-sha"),
      rclcpp::Parameter("spatial_producer_epoch", "producer-epoch"),
      rclcpp::Parameter("spatial_clock_boot_id", "test-boot"),
      rclcpp::Parameter("spatial_source_ids", std::vector<int64_t>{0}),
      rclcpp::Parameter("spatial_source_epochs", std::vector<std::string>{"camera-epoch"}),
      rclcpp::Parameter("spatial_source_frames", std::vector<std::string>{"base_footprint"})});
    return out;
  }
  SpatialCoverageEvidence::SharedPtr evidence(uint64_t sequence, bool clear = true) {
    auto msg = std::make_shared<SpatialCoverageEvidence>();
    const auto r = node->now().nanoseconds(); const auto s = steady_nanoseconds();
    msg->header.stamp = rclcpp::Time(r); msg->header.frame_id = "base_footprint";
    msg->schema_version = 1; msg->contract = "spatial-test-v1"; msg->query_digest = "query-sha";
    msg->producer_epoch = "producer-epoch"; msg->clock_boot_id = "test-boot";
    msg->anchor_source_ros_ns = r - 10000000; msg->anchor_frame_sequence = sequence;
    msg->evaluation_sequence = sequence; msg->input_generation = sequence;
    msg->evaluation_steady_ns = s; msg->evaluation_complete = true;
    msg->total_cells = 4; msg->free_cells = clear ? 4 : 3; msg->unknown_cells = clear ? 0 : 1;
    msg->reason = clear ? "clear" : "unknown";
    sunhorizon_navigation_interfaces::msg::SpatialSourceEvidence source;
    source.source_id = 0; source.contribution = 1; source.producer_epoch = "camera-epoch";
    source.frame_id = "base_footprint"; source.frame_sequence = sequence;
    source.source_ros_ns = msg->anchor_source_ros_ns;
    source.first_receipt_ros_ns = r - 5000000; source.first_receipt_steady_ns = s - 5000000;
    source.valid_until_ros_ns = source.source_ros_ns + 300000000;
    source.valid_until_steady_ns = source.first_receipt_steady_ns +
        (source.valid_until_ros_ns - source.first_receipt_ros_ns);
    source.geometry_valid = true; msg->sources.push_back(source); return msg;
  }
  void receive(const SpatialCoverageEvidence::SharedPtr& msg) {
    CoverageUpstreamTestPeer::receive(*node, msg, rclcpp::MessageInfo(info));
  }
  void publish(bool upstream_healthy = true) {
    CoverageUpstreamTestPeer::sources(*node, upstream_healthy);
    CoverageUpstreamTestPeer::publish(*node);
  }
  void startup_mode() {
    auto opts=options(); auto parameters=opts.parameter_overrides();
    for(auto& p:parameters) if(p.get_name()=="spatial_producer_gid") p=rclcpp::Parameter("spatial_producer_gid", "");
    parameters.emplace_back("spatial_producer_identity_mode", "startup-pin-once-v1");
    opts.parameter_overrides(parameters); node.reset();
    node=std::make_shared<ObstacleCoverageNode>(opts); CoverageUpstreamTestPeer::cancel_timer(*node);
    CoverageUpstreamTestPeer::observe(*node,[&](const auto& value){output.push_back(value);});
    const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(3);
    while(node->get_publishers_info_by_topic(kSpatialInputTopic).empty() && std::chrono::steady_clock::now()<end)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::shared_ptr<rclcpp::Node> producer;
  rclcpp::Publisher<SpatialCoverageEvidence>::SharedPtr publisher;
  std::shared_ptr<ObstacleCoverageNode> node;
  rmw_message_info_t info{};
  std::string gid;
  std::vector<ObstacleCoverageSpatialDecision> output;
};

TEST_F(SpatialBinding, ImmediateConstructionDestructionHasNoPrivateTfThreadToJoin) {
  // Deliberately do not spin: this is the former cancel-before-spin window.
  // No sleep, extended fixture timeout or reduced case selection is involved.
  for(int i=0;i<64;++i) {
    auto fresh=std::make_shared<ObstacleCoverageNode>(rclcpp::NodeOptions().use_global_arguments(false));
    ASSERT_TRUE(CoverageUpstreamTestPeer::tf_group_is_independent(*fresh));
    fresh.reset();
  }
}

TEST_F(SpatialBinding, ExistingExecutorConsumesRealDynamicAndTransientLocalStaticTf) {
  node.reset(); // Static publisher must precede the new listener subscription.
  auto dynamic=producer->create_publisher<tf2_msgs::msg::TFMessage>("/tf",tf2_ros::DynamicListenerQoS());
  auto fixed=producer->create_publisher<tf2_msgs::msg::TFMessage>("/tf_static",tf2_ros::StaticListenerQoS());
  tf2_msgs::msg::TFMessage a,b;geometry_msgs::msg::TransformStamped edge;
  edge.header.stamp=producer->now();edge.header.frame_id="base_footprint";
  edge.child_frame_id="coverage_static_fixture";edge.transform.rotation.w=1.;edge.transform.translation.x=.25;
  a.transforms.push_back(edge);
  edge.header.frame_id="coverage_static_fixture";edge.child_frame_id="coverage_dynamic_fixture";
  edge.transform.translation.x=0.;edge.transform.translation.y=.5;b.transforms.push_back(edge);
  // Publish static once before constructing the listener: real late-join replay.
  fixed->publish(a);
  node=std::make_shared<ObstacleCoverageNode>(options());CoverageUpstreamTestPeer::cancel_timer(*node);
  ASSERT_TRUE(CoverageUpstreamTestPeer::tf_group_is_independent(*node));
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),3U);
  executor.add_node(node);executor.add_node(producer);
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(!CoverageUpstreamTestPeer::has_transform(*node,"coverage_dynamic_fixture") &&
      std::chrono::steady_clock::now()<deadline) {
    dynamic->publish(b);executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(CoverageUpstreamTestPeer::has_transform(*node,"coverage_dynamic_fixture"));
  const auto actual=CoverageUpstreamTestPeer::transform(*node,"coverage_dynamic_fixture");
  EXPECT_DOUBLE_EQ(actual.transform.translation.x,.25);EXPECT_DOUBLE_EQ(actual.transform.translation.y,.5);
  executor.remove_node(node);executor.remove_node(producer);
}

TEST_F(SpatialBinding, StartupPinsOnlyAuthenticatedFirstCallbackAndNeverGraphAlone) {
  startup_mode(); publish();
  EXPECT_TRUE(CoverageUpstreamTestPeer::owner(*node).empty());
  ASSERT_FALSE(output.empty()); EXPECT_EQ(output.back().decision.disposition,0);
  receive(evidence(1)); publish();
  EXPECT_EQ(CoverageUpstreamTestPeer::owner(*node),gid);
  EXPECT_EQ(output.back().decision.disposition,1);
}

TEST_F(SpatialBinding, StartupWrongMessageGidIsTerminalAndCannotLaterAdoptValidOwner) {
  startup_mode(); auto wrong=info; wrong.publisher_gid.data[0]^=0xff;
  CoverageUpstreamTestPeer::receive(*node,evidence(1),rclcpp::MessageInfo(wrong));
  receive(evidence(2)); publish();
  EXPECT_TRUE(CoverageUpstreamTestPeer::owner(*node).empty());
  EXPECT_EQ(output.back().decision.disposition,0);
}

TEST_F(SpatialBinding, StartupUnknownDiscoveryMetadataIsPendingUntilQualifiedCallback) {
  startup_mode(); auto peers = producer->get_publishers_info_by_topic(kSpatialInputTopic);
  ASSERT_EQ(peers.size(), 1U);
  peers[0].node_name() = "_NODE_NAME_UNKNOWN_"; peers[0].node_namespace() = "_NODE_NAMESPACE_UNKNOWN_";
  CoverageUpstreamTestPeer::endpoints(*node, peers);
  receive(evidence(1)); publish();
  EXPECT_TRUE(CoverageUpstreamTestPeer::owner(*node).empty());
  EXPECT_EQ(output.back().decision.disposition, 0);
  CoverageUpstreamTestPeer::actual_endpoints(*node);
  publish(); EXPECT_TRUE(CoverageUpstreamTestPeer::owner(*node).empty());
  receive(evidence(2)); publish(); EXPECT_EQ(output.back().decision.disposition, 1);
  // The same incomplete discovery after qualification is a terminal loss.
  CoverageUpstreamTestPeer::endpoints(*node, peers); publish();
  EXPECT_EQ(output.back().decision.disposition, 0);
  CoverageUpstreamTestPeer::actual_endpoints(*node);
  receive(evidence(3)); publish(); EXPECT_EQ(output.back().decision.disposition, 0);
}

TEST_F(SpatialBinding, StartupUnknownMetadataCannotHideWrongMessageGid) {
  startup_mode(); auto peers = producer->get_publishers_info_by_topic(kSpatialInputTopic);
  ASSERT_EQ(peers.size(), 1U); peers[0].node_name() = "_NODE_NAME_UNKNOWN_";
  CoverageUpstreamTestPeer::endpoints(*node, peers);
  auto wrong = info; wrong.publisher_gid.data[0] ^= 1;
  CoverageUpstreamTestPeer::receive(*node, evidence(1), rclcpp::MessageInfo(wrong));
  CoverageUpstreamTestPeer::actual_endpoints(*node);
  receive(evidence(2)); publish();
  EXPECT_TRUE(CoverageUpstreamTestPeer::owner(*node).empty());
  EXPECT_EQ(output.back().decision.disposition, 0);
}

TEST_F(SpatialBinding, StartupKnownWrongFqnRemainsTerminal) {
  startup_mode(); auto peers = producer->get_publishers_info_by_topic(kSpatialInputTopic);
  ASSERT_EQ(peers.size(), 1U); peers[0].node_name() = "different_producer";
  CoverageUpstreamTestPeer::endpoints(*node, peers);
  receive(evidence(1)); publish(); EXPECT_EQ(output.back().decision.disposition, 0);
  CoverageUpstreamTestPeer::actual_endpoints(*node);
  receive(evidence(2)); publish();
  EXPECT_TRUE(CoverageUpstreamTestPeer::owner(*node).empty());
  EXPECT_EQ(output.back().decision.disposition, 0);
}

TEST_F(SpatialBinding, StartupFrozenOwnerLossCannotRepinReplacement) {
  startup_mode(); receive(evidence(1)); publish(); ASSERT_EQ(output.back().decision.disposition,1);
  publisher.reset();
  const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(3);
  while(!node->get_publishers_info_by_topic(kSpatialInputTopic).empty() && std::chrono::steady_clock::now()<end)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  publish(); EXPECT_EQ(output.back().decision.disposition,0);
  publisher=producer->create_publisher<SpatialCoverageEvidence>(kSpatialInputTopic,latest_status_qos());
  receive(evidence(2)); publish();
  EXPECT_EQ(CoverageUpstreamTestPeer::owner(*node),gid);
  EXPECT_EQ(output.back().decision.disposition,0);
}

TEST_F(SpatialBinding, StartupRejectsPreconfiguredGidAndUnknownIdentityMode) {
  auto opts=options(); auto p=opts.parameter_overrides();
  p.emplace_back("spatial_producer_identity_mode","startup-pin-once-v1");opts.parameter_overrides(p);
  EXPECT_THROW(std::make_shared<ObstacleCoverageNode>(opts),std::runtime_error);
  p.back()=rclcpp::Parameter("spatial_producer_identity_mode","auto-rebind");opts.parameter_overrides(p);
  EXPECT_THROW(std::make_shared<ObstacleCoverageNode>(opts),std::runtime_error);
}

TEST_F(SpatialBinding, WireConversionIsExactAndMalformedStampRejected) {
  auto m = evidence(1); const auto value = sunhorizon_obstacle_coverage::spatial_evidence_observation(*m);
  EXPECT_EQ(value.header_ros_ns, static_cast<int64_t>(m->header.stamp.sec) * 1000000000LL + m->header.stamp.nanosec);
  EXPECT_EQ(value.sources[0].valid_until_steady_ns, m->sources[0].valid_until_steady_ns);
  EXPECT_EQ(value.sources[0].contribution, m->sources[0].contribution);
  m->header.stamp.nanosec = 1000000000U;
  EXPECT_EQ(sunhorizon_obstacle_coverage::spatial_evidence_observation(*m).header_ros_ns, 0);
}

TEST_F(SpatialBinding, VisualNodePublishesActualClearHoldFaultConsumedByFinalTracker) {
  auto opts=options();auto parameters=opts.parameter_overrides();
  for(auto& p:parameters)if(p.get_name()=="profile")p=rclcpp::Parameter("profile","visual");
  opts.parameter_overrides(parameters);node.reset();node=std::make_shared<ObstacleCoverageNode>(opts);
  CoverageUpstreamTestPeer::cancel_timer(*node);
  CoverageUpstreamTestPeer::observe(*node,[&](const auto& value){output.push_back(value);});
  sunhorizon_obstacle_coverage::SpatialEvidenceTrackerConfig cfg;
  cfg.owner_epoch=gid;cfg.contract="spatial-test-v1";cfg.query_digest="query-sha";
  cfg.clock_boot_id="test-boot";cfg.frame_id="base_footprint";cfg.producer_epoch="producer-epoch";
  cfg.source_identities={{0,"camera-epoch","base_footprint"}};
  sunhorizon_velocity_supervisor::SpatialCoverageDecisionTracker final(cfg,"rgbd-only-v1");
  auto offer=[&](const auto& envelope){
    const auto& wire=envelope.decision;const auto& c=wire.coverage;
    sunhorizon_velocity_supervisor::CombinedCoverageEvidence e;
    e.schema=wire.schema_version;e.disposition=wire.disposition;e.non_region_healthy=wire.non_region_healthy;
    e.complete=wire.region_evaluation_complete;e.region_code=wire.region_evaluation_code;
    e.region_disposition=wire.region_disposition;e.source_ros_ns=wire.region_source_stamp_ns;
    e.frame_sequence=wire.region_frame_sequence;e.evaluation_sequence=wire.region_evaluation_sequence;
    e.healthy=c.healthy;e.forward=c.forward_coverage;e.near=c.near_field_coverage;e.rear=c.rear_coverage;
    e.frame_id=c.header.frame_id;e.profile=c.profile;e.contract=wire.region_contract;
    e.header_ros_ns=rclcpp::Time(c.header.stamp).nanoseconds();
    e.required_sources=c.required_sources;e.healthy_sources=c.healthy_sources;
    return final.observe(e,sunhorizon_obstacle_coverage::spatial_evidence_observation(envelope.spatial),
      gid,node->now().nanoseconds(),steady_nanoseconds());
  };
  auto publish_depth=[&](bool healthy=true){CoverageUpstreamTestPeer::sources(*node,healthy);
    CoverageUpstreamTestPeer::no_airy(*node);CoverageUpstreamTestPeer::publish(*node);};
  receive(evidence(1));publish_depth();ASSERT_EQ(output.back().decision.disposition,1);
  EXPECT_EQ(output.back().decision.coverage.profile,"visual");
  EXPECT_EQ(output.back().decision.coverage.required_sources,(std::vector<std::string>{"chest","belly"}));
  ASSERT_TRUE(offer(output.back()));EXPECT_EQ(final.evaluate(node->now().nanoseconds(),steady_nanoseconds()).disposition,1);
  receive(evidence(2,false));publish_depth();ASSERT_EQ(output.back().decision.disposition,2);
  ASSERT_TRUE(offer(output.back()));EXPECT_EQ(final.evaluate(node->now().nanoseconds(),steady_nanoseconds()).disposition,2);
  publish_depth(false);ASSERT_EQ(output.back().decision.disposition,0);EXPECT_FALSE(offer(output.back()));
  EXPECT_FALSE(final.evaluate(node->now().nanoseconds(),steady_nanoseconds()).valid);
}

TEST_F(SpatialBinding, MissingThenClearThenWaitAndHardUpstreamRemainDistinct) {
  publish(); ASSERT_EQ(output.size(), 1U); EXPECT_EQ(output.back().decision.disposition, 0);
  receive(evidence(1)); publish();
  ASSERT_EQ(output.back().decision.schema_version, 2); EXPECT_EQ(output.back().decision.disposition, 1);
  EXPECT_TRUE(output.back().decision.coverage.healthy);
  EXPECT_EQ(output.back().decision.region_source_stamp_ns, 0);
  EXPECT_EQ(output.back().decision.region_frame_sequence, 0U);
  EXPECT_TRUE(output.back().decision.region_contract.empty());
  EXPECT_TRUE(CoverageUpstreamTestPeer::legacy_absent(*node));
  receive(evidence(2, false)); publish();
  EXPECT_EQ(output.back().decision.disposition, 2); EXPECT_FALSE(output.back().decision.coverage.healthy);
  EXPECT_TRUE(output.back().decision.non_region_healthy);
  publish(false); EXPECT_EQ(output.back().decision.disposition, 0);
  EXPECT_FALSE(output.back().decision.non_region_healthy);
}

TEST_F(SpatialBinding, InitialWireAndValidatedDispositionStayTogetherAcrossConcurrentNewFrame) {
  const auto a = evidence(1); receive(a);
  CoverageUpstreamTestPeer::after_snapshot(*node, [&] {
    auto b = evidence(2, false); std::thread callback([&] { receive(b); }); callback.join();
  });
  publish(); ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output.back().spatial.anchor_source_ros_ns, a->anchor_source_ros_ns);
  EXPECT_EQ(output.back().spatial.anchor_frame_sequence, 1U);
  EXPECT_EQ(output.back().decision.disposition, 1);
  CoverageUpstreamTestPeer::after_snapshot(*node, {}); publish();
  EXPECT_EQ(output.back().spatial.anchor_frame_sequence, 2U);
  EXPECT_EQ(output.back().decision.disposition, 2);
}

TEST_F(SpatialBinding, ExpiredInitialSnapshotCannotBeReplacedByFreshWaitAfterTfWork) {
  const auto initial = evidence(1); receive(initial);
  CoverageUpstreamTestPeer::after_snapshot(*node, [&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(310)); receive(evidence(2, false));
  });
  publish(); ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output.back().decision.disposition, 0);
  EXPECT_FALSE(output.back().decision.coverage.healthy);
  EXPECT_NE(output.back().decision.coverage.reason.find("spatial_snapshot_expired_after_tf"), std::string::npos);
  // The publication retains the initial immutable wire. A fresh concurrent
  // Wait cannot replace its source clocks or extend its expired validity.
  EXPECT_EQ(output.back().spatial.anchor_frame_sequence, initial->anchor_frame_sequence);
  EXPECT_EQ(output.back().spatial.anchor_source_ros_ns, initial->anchor_source_ros_ns);
  ASSERT_EQ(output.back().spatial.sources.size(), initial->sources.size());
  EXPECT_EQ(output.back().spatial.sources[0].first_receipt_steady_ns, initial->sources[0].first_receipt_steady_ns);
  EXPECT_EQ(output.back().spatial.sources[0].valid_until_ros_ns, initial->sources[0].valid_until_ros_ns);
  EXPECT_EQ(output.back().spatial.sources[0].valid_until_steady_ns, initial->sources[0].valid_until_steady_ns);
  CoverageUpstreamTestPeer::after_snapshot(*node, {});
  // Coverage observation recovery requires another new anchor. Unknown stays
  // Wait and cannot qualify motion; only a complete new Clear reports Clear.
  receive(evidence(3, false)); publish();
  EXPECT_EQ(output.back().spatial.anchor_frame_sequence, 3U);
  EXPECT_EQ(output.back().decision.disposition, 2);
  EXPECT_FALSE(output.back().decision.coverage.healthy);
  EXPECT_FALSE(output.back().decision.coverage.forward_coverage);
  EXPECT_FALSE(output.back().decision.coverage.near_field_coverage);
  EXPECT_TRUE(output.back().decision.non_region_healthy);
  receive(evidence(4)); publish();
  EXPECT_EQ(output.back().spatial.anchor_frame_sequence, 4U);
  EXPECT_EQ(output.back().decision.disposition, 1);
  EXPECT_TRUE(output.back().decision.coverage.healthy);
}

TEST_F(SpatialBinding, SameAnchorMutationAndCallbackPublisherChangeLatchHard) {
  auto a = evidence(1); receive(a); publish(); ASSERT_EQ(output.back().decision.disposition, 1);
  a->free_cells = 3; a->unknown_cells = 1; receive(a); publish();
  EXPECT_EQ(output.back().decision.disposition, 0);
  EXPECT_NE(output.back().decision.coverage.reason.find("spatial_same_anchor_mutated"), std::string::npos);
}

TEST_F(SpatialBinding, WrongCallbackGidCannotUseCorrectGraphOwner) {
  auto wrong = info; wrong.publisher_gid.data[0] ^= 1;
  CoverageUpstreamTestPeer::receive(*node, evidence(1), rclcpp::MessageInfo(wrong)); publish();
  EXPECT_EQ(output.back().decision.disposition, 0);
  EXPECT_NE(output.back().decision.coverage.reason.find("spatial_producer_message_gid_mismatch"), std::string::npos);
}

TEST_F(SpatialBinding, PositiveOccupancyAndTerminalRevocationBothStopImmediately) {
  auto occupied = evidence(1, false);
  occupied->free_cells = 2; occupied->occupied_cells = 1; occupied->sources[0].contribution = 3;
  receive(occupied); publish(); EXPECT_EQ(output.back().decision.disposition, 0);
  EXPECT_NE(output.back().decision.coverage.reason.find("spatial_occupied"), std::string::npos);
}

TEST_F(SpatialBinding, RemovedOccupancyNeedsNewCompleteClearObservation) {
  auto occupied = evidence(1, false);
  occupied->free_cells = 2; occupied->occupied_cells = 1;
  occupied->sources[0].contribution = 3;
  receive(occupied); publish();
  ASSERT_EQ(output.back().decision.disposition, 0);
  receive(evidence(2, false)); publish();
  EXPECT_EQ(output.back().decision.disposition, 0);
  EXPECT_FALSE(output.back().decision.coverage.healthy);
  receive(evidence(3)); publish();
  EXPECT_EQ(output.back().decision.disposition, 1);
  EXPECT_TRUE(output.back().decision.coverage.healthy);
  EXPECT_TRUE(output.back().spatial.evaluation_complete);
  EXPECT_EQ(output.back().spatial.anchor_frame_sequence, 3U);
}

TEST_F(SpatialBinding, TerminalInvalidationCannotLeavePreviousClearActive) {
  auto m = evidence(1); receive(m); publish(); ASSERT_EQ(output.back().decision.disposition, 1);
  m->evaluation_complete = false; m->reason = "exact_info_conflict";
  receive(m); publish(); EXPECT_EQ(output.back().decision.disposition, 0);
  receive(evidence(2)); publish(); EXPECT_EQ(output.back().decision.disposition, 0);
}

TEST_F(SpatialBinding, DuplicateActualEndpointIsRejected) {
  receive(evidence(1)); publish(); ASSERT_EQ(output.back().decision.disposition, 1);
  auto second = producer->create_publisher<SpatialCoverageEvidence>(kSpatialInputTopic, latest_status_qos());
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (CoverageUpstreamTestPeer::endpoint_error(*node) != "spatial_producer_not_unique" &&
      std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(std::chrono::milliseconds(5));
  ASSERT_EQ(CoverageUpstreamTestPeer::endpoint_error(*node), "spatial_producer_not_unique");
  publish(); EXPECT_EQ(output.back().decision.disposition, 0);
}

TEST_F(SpatialBinding, ExplicitPinsAndLegacySpatialMutualExclusionAreRequired) {
  auto params = options().parameter_overrides();
  params.emplace_back("region_support_required", true);
  auto bad = rclcpp::NodeOptions().parameter_overrides(params);
  EXPECT_THROW(std::make_shared<ObstacleCoverageNode>(bad), std::runtime_error);
  params = options().parameter_overrides();
  for (auto& p : params) if (p.get_name() == "spatial_producer_gid") p = rclcpp::Parameter(p.get_name(), "");
  bad.parameter_overrides(params); EXPECT_THROW(std::make_shared<ObstacleCoverageNode>(bad), std::runtime_error);
}

TEST_F(SpatialBinding, SameRaw350PolicyRequiresExplicitProfileAndReadOnlyPins) {
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("spatial_query_digest", "other")).successful);
  auto params = options().parameter_overrides();
  params.emplace_back("belly_freshness_sec", .35);
  auto selected = rclcpp::NodeOptions().parameter_overrides(params);
  EXPECT_THROW(std::make_shared<ObstacleCoverageNode>(selected), std::runtime_error);
  params.emplace_back("region_upstream_status_deadline_profile", "raw-status-350ms-v1");
  selected.parameter_overrides(params);
  auto allowed = std::make_shared<ObstacleCoverageNode>(selected);
  EXPECT_NE(allowed, nullptr);
  params.emplace_back("belly_obstacle_freshness_sec", .35);
  selected.parameter_overrides(params);
  EXPECT_THROW(std::make_shared<ObstacleCoverageNode>(selected), std::runtime_error);
}

TEST_F(SpatialBinding, DirectOwnerUsesActualPublisherAndSamePersistentTrackerWithoutDdsInputs) {
  producer->declare_parameter("producer_epoch","producer-epoch");
  producer->declare_parameter("clock_boot_id","test-boot");
  auto buffer=std::make_shared<tf2_ros::Buffer>(producer->get_clock());
  sunhorizon_obstacle_coverage::CoverageDirectOptions direct;
  direct.node_options=options();direct.transforms=buffer;
  direct.spatial_owner=producer;direct.spatial_publisher=publisher;
  bool failed=false;direct.producer_failed=[&]{return failed;};
  node.reset();node=std::make_shared<ObstacleCoverageNode>(options(),direct);
  CoverageUpstreamTestPeer::cancel_timer(*node);
  CoverageUpstreamTestPeer::observe(*node,[&](const auto& value){output.push_back(value);});
  EXPECT_TRUE(CoverageUpstreamTestPeer::direct_only(*node,buffer));
  EXPECT_EQ(CoverageUpstreamTestPeer::owner(*node),gid);
  auto cloud=std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud->header.stamp=node->now();cloud->header.frame_id="base_footprint";
  cloud->width=cloud->height=1;cloud->point_step=cloud->row_step=12;cloud->data.resize(12);
  const auto receipt=node->now().nanoseconds()-1000000;
  node->receive_belly(cloud,receipt);
  EXPECT_DOUBLE_EQ(CoverageUpstreamTestPeer::belly_receipt(*node),double(receipt)/1e9);
  CoverageUpstreamTestPeer::sources(*node);
  const auto first=evidence(1);ASSERT_TRUE(node->receive_spatial(first).accepted());
  CoverageUpstreamTestPeer::publish(*node);ASSERT_FALSE(output.empty());
  EXPECT_TRUE(output.back().decision.non_region_healthy);
  EXPECT_EQ(output.back().spatial.anchor_frame_sequence,1u);
  EXPECT_TRUE(node->receive_spatial(first).accepted()); // Exact repeat cannot create a new generation.
  auto negative=evidence(2);negative->evaluation_complete=false;
  EXPECT_FALSE(node->receive_spatial(negative).accepted());
  EXPECT_FALSE(node->receive_spatial(evidence(3)).accepted()); // No reconstruction/requalification.
  CoverageUpstreamTestPeer::publish(*node);EXPECT_FALSE(output.back().decision.coverage.healthy);
  node->revoke("source_fault");EXPECT_FALSE(node->receive_spatial(evidence(4)).accepted());
  node->stop();EXPECT_THROW(node->receive_belly(cloud,receipt),std::logic_error);
}
TEST_F(SpatialBinding, DirectRevokeAndConcurrentPositiveCannotRequalifyOrRenewTheTracker) {
  producer->declare_parameter("producer_epoch","producer-epoch");
  producer->declare_parameter("clock_boot_id","test-boot");
  sunhorizon_obstacle_coverage::CoverageDirectOptions direct;
  direct.node_options=options();direct.transforms=std::make_shared<tf2_ros::Buffer>(producer->get_clock());
  direct.spatial_owner=producer;direct.spatial_publisher=publisher;direct.producer_failed=[] {return false;};
  node.reset();node=std::make_shared<ObstacleCoverageNode>(options(),direct);
  CoverageUpstreamTestPeer::cancel_timer(*node);
  CoverageUpstreamTestPeer::observe(*node,[&](const auto& value){output.push_back(value);});
  CoverageUpstreamTestPeer::sources(*node);ASSERT_TRUE(node->receive_spatial(evidence(1)).accepted());
  const auto positive=evidence(2);
  std::promise<void> release;auto start=release.get_future().share();
  std::thread producer_input([&]{start.wait();(void)node->receive_spatial(positive);});
  std::thread revoker([&]{start.wait();node->revoke("source_terminal_fault");});
  release.set_value();producer_input.join();revoker.join();
  const auto rejected=node->receive_spatial(evidence(3));EXPECT_FALSE(rejected.accepted());
  EXPECT_EQ(rejected.state,sunhorizon_obstacle_coverage::SpatialDeliveryState::InternalFault);
  EXPECT_FALSE(rejected.reason.empty());
  node->revoke("cannot_reset_or_replace_first_fault");
  CoverageUpstreamTestPeer::publish(*node);ASSERT_FALSE(output.empty());
  EXPECT_FALSE(output.back().decision.coverage.healthy);
  EXPECT_FALSE(output.back().decision.non_region_healthy);
  EXPECT_EQ(output.back().decision.disposition,0u);
  EXPECT_NE(output.back().decision.coverage.reason.find("source_terminal_fault"),std::string::npos);
}
TEST_F(SpatialBinding, ExpiredOldSnapshotRecoversOnlyOnFreshNewAnchor) {
  using State=sunhorizon_obstacle_coverage::SpatialDeliveryState;
  producer->declare_parameter("producer_epoch","producer-epoch");
  producer->declare_parameter("clock_boot_id","test-boot");
  sunhorizon_obstacle_coverage::CoverageDirectOptions direct;
  direct.node_options=options();direct.transforms=std::make_shared<tf2_ros::Buffer>(producer->get_clock());
  direct.spatial_owner=producer;direct.spatial_publisher=publisher;direct.producer_failed=[] {return false;};
  node.reset();node=std::make_shared<ObstacleCoverageNode>(options(),direct);
  CoverageUpstreamTestPeer::cancel_timer(*node);
  CoverageUpstreamTestPeer::observe(*node,[&](const auto& value){output.push_back(value);});
  auto old=evidence(989);
  const int64_t shift=400000000;
  const auto old_header=rclcpp::Time(old->header.stamp).nanoseconds()-shift;
  old->header.stamp=rclcpp::Time(old_header);old->evaluation_steady_ns-=shift;old->anchor_source_ros_ns-=shift;
  for(auto& source:old->sources){source.source_ros_ns-=shift;source.first_receipt_ros_ns-=shift;
    source.first_receipt_steady_ns-=shift;source.valid_until_ros_ns-=shift;source.valid_until_steady_ns-=shift;}
  ASSERT_TRUE(CoverageUpstreamTestPeer::seed_at_original_receipt(*node,*old,old_header,old->evaluation_steady_ns));
  const auto result=node->receive_spatial(old);
  EXPECT_EQ(result.state,State::MotionUnavailable);EXPECT_FALSE(result.accepted());
  EXPECT_EQ(result.reason,"spatial_frozen_evidence_expired");
  publish();ASSERT_FALSE(output.empty());
  EXPECT_EQ(output.back().decision.disposition,0u);
  EXPECT_FALSE(output.back().decision.coverage.healthy);
  EXPECT_EQ(output.back().spatial.anchor_frame_sequence,989u); // Expired frame never renewed.
  ASSERT_TRUE(node->receive_spatial(evidence(992)).accepted());
  publish();EXPECT_EQ(output.back().decision.disposition,1u);
  EXPECT_TRUE(output.back().decision.coverage.healthy);
  EXPECT_EQ(output.back().spatial.anchor_frame_sequence,992u);
  ASSERT_TRUE(node->receive_spatial(evidence(995,false)).accepted());
  publish();EXPECT_EQ(output.back().decision.disposition,2u);
  EXPECT_FALSE(output.back().decision.coverage.healthy); // Unknown is Wait, never Clear.
  EXPECT_EQ(output.back().spatial.anchor_frame_sequence,995u);
  auto occupied=evidence(998);occupied->free_cells=3;occupied->occupied_cells=1;
  occupied->sources[0].contribution=3;
  EXPECT_EQ(node->receive_spatial(occupied).state,State::MotionUnavailable);
  EXPECT_EQ(node->receive_spatial(evidence(1001,false)).state,State::MotionUnavailable);
  publish();EXPECT_EQ(output.back().decision.disposition,0u);
  EXPECT_EQ(node->receive_spatial(evidence(1002)).state,State::Accepted);
  publish();EXPECT_EQ(output.back().decision.disposition,1u);
  auto bad=evidence(1004);bad->producer_epoch="changed";
  EXPECT_EQ(node->receive_spatial(bad).state,State::InternalFault);
  EXPECT_EQ(node->receive_spatial(evidence(1005)).state,State::InternalFault);
}
TEST_F(SpatialBinding, DirectFactoryCannotAuthenticateConfiguredIdentityWithoutActualObjects) {
  sunhorizon_obstacle_coverage::CoverageDirectOptions direct;direct.node_options=options();
  direct.transforms=std::make_shared<tf2_ros::Buffer>(producer->get_clock());
  EXPECT_THROW(sunhorizon_obstacle_coverage::make_coverage_direct_runtime(direct),std::invalid_argument);
  producer->declare_parameter("producer_epoch","wrong-epoch");
  producer->declare_parameter("clock_boot_id","test-boot");
  direct.spatial_owner=producer;direct.spatial_publisher=publisher;direct.producer_failed=[] {return false;};
  EXPECT_THROW(sunhorizon_obstacle_coverage::make_coverage_direct_runtime(direct),std::invalid_argument);
}

TEST_F(SpatialBinding, DirectEvidencePublishesWithoutHeartbeatAndRetainsOriginalDeadline) {
  producer->declare_parameter("producer_epoch","producer-epoch");
  producer->declare_parameter("clock_boot_id","test-boot");
  sunhorizon_obstacle_coverage::CoverageDirectOptions direct;
  direct.node_options=options();
  direct.transforms=std::make_shared<tf2_ros::Buffer>(producer->get_clock());
  direct.spatial_owner=producer;direct.spatial_publisher=publisher;
  direct.producer_failed=[] {return false;};
  node.reset();node=std::make_shared<ObstacleCoverageNode>(options(),direct);
  CoverageUpstreamTestPeer::cancel_timer(*node, false);
  CoverageUpstreamTestPeer::sources(*node);
  auto delivered=std::make_shared<std::promise<ObstacleCoverageSpatialDecision>>();
  auto result=delivered->get_future();
  CoverageUpstreamTestPeer::observe(*node,[delivered](const auto& value){delivered->set_value(value);});
  const auto input=evidence(10);
  ASSERT_TRUE(node->receive_spatial(input).accepted());
  ASSERT_EQ(result.wait_for(std::chrono::seconds(2)),std::future_status::ready);
  const auto decision=result.get();
  EXPECT_EQ(decision.decision.disposition,1u);
  EXPECT_EQ(decision.spatial.anchor_frame_sequence,10u);
  ASSERT_EQ(decision.spatial.sources.size(),1u);
  EXPECT_EQ(decision.spatial.sources[0].source_ros_ns,input->sources[0].source_ros_ns);
  EXPECT_EQ(decision.spatial.sources[0].first_receipt_steady_ns,input->sources[0].first_receipt_steady_ns);
  EXPECT_EQ(decision.spatial.sources[0].valid_until_ros_ns,input->sources[0].valid_until_ros_ns);
  EXPECT_EQ(decision.spatial.sources[0].valid_until_steady_ns,input->sources[0].valid_until_steady_ns);
  node->stop();  // Joins publication before the observer and promise are destroyed.
}
}  // namespace
