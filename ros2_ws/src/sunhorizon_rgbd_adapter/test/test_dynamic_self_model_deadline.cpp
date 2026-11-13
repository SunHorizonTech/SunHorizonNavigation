#include <gtest/gtest.h>
#include <typeinfo>
#include "sunhorizon_rgbd_adapter/dynamic_self_model.hpp"
#include "sunhorizon_rgbd_adapter/spatial_source_transform.hpp"
#include "sunhorizon_rgbd_adapter/spatial_leg_pairing.hpp"
#include "sunhorizon_rgbd_adapter/spatial_frame_timing.hpp"

namespace dm = sunhorizon_rgbd_adapter::dynamic_self_model;

TEST(DynamicSelfModelDeadline, ExpiredAllowanceIsExactlyTfTimeoutWithoutSleep) {
  const auto deadline = dm::steady_ns() - 1;
  try {
    (void)dm::remaining(deadline);
    FAIL() << "expired TF allowance must throw";
  } catch (const tf2::TimeoutException& error) {
    EXPECT_EQ(typeid(error), typeid(tf2::TimeoutException));
    EXPECT_STREQ(error.what(), "model_tf_budget_exhausted");
  }
}

TEST(DynamicSelfModelDeadline, PositiveAllowanceRemainsBoundedByItsDeadline) {
  constexpr int64_t allowance = 1000000000;
  const auto deadline = dm::steady_ns() + allowance;
  const auto left = dm::remaining(deadline).nanoseconds();
  EXPECT_GT(left, 0);
  EXPECT_LE(left, allowance);
}

TEST(DynamicSelfModelDeadline, InvalidRigidTransformIsStillNotTfTimeout) {
  geometry_msgs::msg::Transform transform;
  transform.rotation.x = transform.rotation.y = transform.rotation.z = transform.rotation.w = 0.;
  try {
    (void)dm::rigid(transform);
    FAIL() << "invalid rigid transform must remain fatal";
  } catch (const std::runtime_error& error) {
    EXPECT_EQ(typeid(error), typeid(std::runtime_error));
    EXPECT_EQ(dynamic_cast<const tf2::TransformException*>(&error), nullptr);
    EXPECT_STREQ(error.what(), "model_invalid_rigid_tf");
  }
}

namespace {
constexpr int64_t t0 = 100000000000, t1 = 102000000000;
class SpatialSourceTransform : public ::testing::Test {
 protected:
  std::shared_ptr<rclcpp::Clock> clock{std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)};
  tf2_ros::Buffer buffer{clock};
  void edge(const std::string& parent, const std::string& child,
      int64_t stamp, double x = 0., bool is_static = false) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = parent;tf.child_frame_id = child;
    tf.header.stamp = rclcpp::Time(stamp, RCL_SYSTEM_TIME);
    tf.transform.rotation.w = 1.;tf.transform.translation.x = x;
    ASSERT_TRUE(buffer.setTransform(tf, "deterministic_test_tree", is_static));
  }
  void relative_chain(bool current = true) {
    buffer.setUsingDedicatedThread(true); // Preloaded test tree; no subscriber or executor.
    edge("base_footprint", "base_link", t0, 0.25);
    edge("base_link", "waist_yaw_link", t0, 0.5);
    if (current) {
      edge("base_footprint", "base_link", t1, 0.25);
      edge("base_link", "waist_yaw_link", t1, 0.5);
    }
    edge("waist_yaw_link", "camera_belly_link", t0, 0.75, true);
    edge("camera_belly_link", "camera_belly_depth_optical_frame", t0, 0.5, true);
  }
  geometry_msgs::msg::TransformStamped query(int64_t source, int64_t anchor) {
    return sunhorizon_rgbd_adapter::spatial_source_transform(buffer,
        "base_footprint", "camera_belly_depth_optical_frame", source, anchor,
        RCL_SYSTEM_TIME, dm::steady_ns() + 20000000);
  }
};
} // namespace

TEST_F(SpatialSourceTransform, EqualTimeUsesFreshRelativeChainWithoutFixedAncestor) {
  relative_chain();edge("odom", "base_footprint", t0);
  EXPECT_THROW(buffer.lookupTransform("base_footprint", rclcpp::Time(t1, RCL_SYSTEM_TIME),
      "camera_belly_depth_optical_frame", rclcpp::Time(t1, RCL_SYSTEM_TIME),
      "odom", rclcpp::Duration::from_nanoseconds(0)), tf2::TransformException);
  const auto result = query(t1, t1);
  EXPECT_DOUBLE_EQ(result.transform.translation.x, 2.);
  EXPECT_EQ(rclcpp::Time(result.header.stamp, RCL_SYSTEM_TIME).nanoseconds(), t1);
}

TEST_F(SpatialSourceTransform, EqualTimeStillRejectsStaleRequiredRelativeChain) {
  relative_chain(false);edge("odom", "base_footprint", t0);edge("odom", "base_footprint", t1);
  EXPECT_THROW(query(t1, t1), tf2::TransformException);
}

TEST_F(SpatialSourceTransform, DifferentTimesRetainFixedFrameMotionCompensation) {
  relative_chain();edge("odom", "base_footprint", t0);edge("odom", "base_footprint", t1, 10.);
  const auto result = query(t0, t1);
  EXPECT_DOUBLE_EQ(result.transform.translation.x, -8.);
  // Direct source-time relative lookup is 2m; it cannot substitute for -8m.
  EXPECT_DOUBLE_EQ(buffer.lookupTransform("base_footprint", "camera_belly_depth_optical_frame",
      rclcpp::Time(t0, RCL_SYSTEM_TIME), rclcpp::Duration::from_nanoseconds(0)).transform.translation.x, 2.);
}

TEST_F(SpatialSourceTransform, DifferentTimesRejectMissingFixedFrameCoverage) {
  relative_chain();edge("odom", "base_footprint", t0);
  EXPECT_THROW(query(t0, t1), tf2::TransformException);
}

namespace {
class SpatialLegPairing : public ::testing::Test {
 protected:
  std::shared_ptr<rclcpp::Clock> clock{std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)};
  tf2_ros::Buffer buffer{clock};
  std::vector<dm::AssetSpec> specs;
  SpatialLegPairing() {
    buffer.setUsingDedicatedThread(true);
    for (unsigned i=0;i<12;++i) specs.push_back({"","",0,"leg_"+std::to_string(i),"base_link"});
  }
  void at(unsigned index,int64_t stamp) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id="base_link";tf.child_frame_id=specs[index].link_name;
    tf.header.stamp=rclcpp::Time(stamp,RCL_SYSTEM_TIME);tf.transform.rotation.w=1.;
    ASSERT_TRUE(buffer.setTransform(tf,"deterministic_test_tree",false));
  }
  auto query(int64_t source=t1) {
    return sunhorizon_rgbd_adapter::spatial_leg_availability(buffer,specs,source,RCL_SYSTEM_TIME);
  }
};
}

TEST_F(SpatialLegPairing, FutureUpperBoundWaitsThenAdmitsOnlyExactSourceTransforms) {
  sunhorizon_rgbd_adapter::SpatialLegPendingTiming timing;
  const auto observed=[&](){return sunhorizon_rgbd_adapter::spatial_leg_availability(buffer,specs,t1,RCL_SYSTEM_TIME,&timing);};
  for (unsigned i=0;i<12;++i) at(i,t0);
  EXPECT_EQ(observed(),sunhorizon_rgbd_adapter::SpatialLegAvailability::FuturePending);
  EXPECT_EQ(timing.first_link_index,0);EXPECT_EQ(timing.source_query_ns,t1);EXPECT_EQ(timing.latest_upper_ns,t0);
  for (unsigned i=0;i<11;++i) at(i,t1);
  EXPECT_EQ(observed(),sunhorizon_rgbd_adapter::SpatialLegAvailability::FuturePending);
  EXPECT_EQ(timing.first_link_index,11);EXPECT_EQ(timing.source_query_ns,t1);EXPECT_EQ(timing.latest_upper_ns,t0);
  at(11,t1);
  EXPECT_EQ(observed(),sunhorizon_rgbd_adapter::SpatialLegAvailability::ExactReady);
  EXPECT_EQ(timing.first_link_index,-1);EXPECT_EQ(timing.source_query_ns,0);EXPECT_EQ(timing.latest_upper_ns,0);
}

TEST_F(SpatialLegPairing, PastExtrapolationIsNotFuturePending) {
  for (unsigned i=0;i<12;++i) at(i,t1);
  EXPECT_THROW(query(t0),tf2::ExtrapolationException);
}

TEST_F(SpatialLegPairing, EarlierFutureLinkDoesNotHideDisconnectedRequiredLink) {
  for (unsigned i=0;i<11;++i) at(i,t0);
  EXPECT_THROW(query(),tf2::TransformException);
}

TEST_F(SpatialLegPairing, SourceAndCompleteModelCountRemainRequired) {
  EXPECT_THROW(query(0),std::invalid_argument);
  specs.pop_back();
  EXPECT_THROW(query(),std::invalid_argument);
}

namespace st = sunhorizon_rgbd_adapter;
TEST(SpatialFrameTiming, RetryUpdatesSameRecordWithoutChangingOriginalIngress) {
  st::SpatialFrameTimingRing ring;
  ring.ingress(1,100,110,120,0,0);ring.paired(1,130);ring.evaluate(1,140,false);
  ring.future_wait(1,150,2,100,90);ring.stage(1,st::SpatialTimingStage::LegCheckEnd,151);
  ring.evaluate(1,160,true);ring.future_wait(1,170,4,100,95);ring.stage(1,st::SpatialTimingStage::PublishReturn,180);
  ring.evaluate(1,190,true); // A stale-before-lookup attempt must retain the last actual query.
  const auto snapshot=ring.snapshot_once();ASSERT_EQ(snapshot.count,1u);
  const auto& row=snapshot.records[0];
  EXPECT_EQ(row.source_ns,100);EXPECT_EQ(row.callback_ros_ns,110);EXPECT_EQ(row.callback_steady_ns,120);
  EXPECT_EQ(row.rmw_source_ns,0);EXPECT_EQ(row.rmw_received_ns,0);EXPECT_EQ(row.paired_ns,130);
  EXPECT_EQ(row.first_evaluate_ns,140);EXPECT_EQ(row.last_evaluate_ns,190);EXPECT_EQ(row.attempts,3u);
  EXPECT_TRUE(row.last_evaluate_timer);EXPECT_EQ(row.first_future_wait_ns,150);EXPECT_EQ(row.last_future_wait_ns,170);
  EXPECT_EQ(row.stages[static_cast<std::size_t>(st::SpatialTimingStage::LegCheckEnd)],0);
  EXPECT_EQ(row.stages[static_cast<std::size_t>(st::SpatialTimingStage::PublishReturn)],0);
  EXPECT_EQ(row.last_future_link_index,4);EXPECT_EQ(row.last_future_source_query_ns,100);EXPECT_EQ(row.last_future_latest_upper_ns,95);
  EXPECT_FALSE(ring.snapshot_once().claimed);
}

TEST(SpatialFrameTiming, FrozenHintSurvivesWrapWhileTerminalTailContinues) {
  st::SpatialFrameTimingRing ring;
  ring.ingress(1,100,110,120,101,102);ring.paired(1,130);ring.dropped(1);
  ring.freeze_hint(200,210,220);
  for (uint64_t seq=2;seq<=300;++seq) ring.ingress(seq,seq*100,seq*100+1,seq*100+2,0,0);
  ring.freeze_hint(999,999,999);
  const auto hint=ring.hint_snapshot_once();ASSERT_EQ(hint.count,1u);
  EXPECT_EQ(hint.records[0].sequence,1u);EXPECT_EQ(hint.records[0].source_ns,100);
  EXPECT_EQ(hint.records[0].rmw_source_ns,101);EXPECT_EQ(hint.records[0].rmw_received_ns,102);
  EXPECT_EQ(hint.records[0].paired_ns,130);EXPECT_TRUE(hint.records[0].dropped);
  EXPECT_EQ(hint.trigger_d_ns,200);EXPECT_EQ(hint.trigger_ros_ns,210);EXPECT_EQ(hint.trigger_steady_ns,220);
  EXPECT_EQ(hint.overwritten,0u);EXPECT_FALSE(ring.hint_snapshot_once().claimed);
  ring.stage(1,st::SpatialTimingStage::PublishReturn,999); // Evicted identity cannot alter a new slot.
  const auto tail=ring.snapshot_once();ASSERT_EQ(tail.count,256u);
  EXPECT_EQ(tail.overwritten,44u);EXPECT_EQ(tail.records.front().sequence,45u);
  EXPECT_EQ(tail.records[255].sequence,300u);EXPECT_EQ(tail.records[0].stages[13],0);
}

TEST(SpatialFrameTiming, OnlyExactObservedExpiryTokenAfterFirstValidHeaderIsAHint) {
  constexpr auto exact="spatial_visibility:spatial_frozen_evidence_expired";
  EXPECT_FALSE(st::spatial_expiry_timing_hint(exact,200,0));
  EXPECT_FALSE(st::spatial_expiry_timing_hint(exact,99,100));
  EXPECT_FALSE(st::spatial_expiry_timing_hint("other:spatial_frozen_evidence_expired",200,100));
  EXPECT_FALSE(st::spatial_expiry_timing_hint("spatial_visibility:spatial_frozen_evidence_expired_suffix",200,100));
  EXPECT_TRUE(st::spatial_expiry_timing_hint(exact,100,100));
  EXPECT_TRUE(st::spatial_expiry_timing_hint(std::string("upstream_ok;")+exact+";other",200,100));
  st::SpatialFrameTimingRing ring;EXPECT_FALSE(ring.hint_snapshot_once().claimed);
}

TEST(SpatialPrivateTfTiming, PartialHintIsImmutableAndWrapCannotConfuseCallbackIdentity) {
  st::SpatialPrivateTfTimingRing ring;st::SpatialPrivateTfTiming value;
  value.callback_ros_ns=110;value.callback_steady_ns=120;value.message_count=24;value.scanned_count=24;
  value.edge(0,90);value.edge(0,100);value.edge(11,95);
  const auto first=ring.ingress(value);
  ring.stage(first,st::SpatialPrivateTfTimingRing::Stage::OwnerStart,130);
  ring.freeze_hint(200,210,220);
  ring.stage(first,st::SpatialPrivateTfTimingRing::Stage::OwnerEnd,140);ring.inserted(first,24,150);ring.end(first,160);
  for (unsigned i=0;i<300;++i) ring.ingress(value);
  ring.freeze_hint(999,999,999);ring.end(first,999); // Evicted callback cannot update another slot.
  const auto hint=ring.hint_snapshot_once();ASSERT_EQ(hint.count,1u);
  const auto& row=hint.records[0];EXPECT_EQ(row.sequence,first);EXPECT_EQ(row.owner_start_ns,130);
  EXPECT_EQ(row.owner_end_ns,0);EXPECT_EQ(row.callback_end_ns,0);EXPECT_EQ(row.inserted_end_ns,0);
  EXPECT_EQ(row.link_count[0],2u);EXPECT_EQ(row.link_min_source_ns[0],90);EXPECT_EQ(row.link_max_source_ns[0],100);
  EXPECT_EQ(row.link_count[11],1u);EXPECT_EQ(row.link_count[10],0u);EXPECT_EQ(row.link_min_source_ns[10],0);
  EXPECT_EQ(hint.trigger_d_ns,200);EXPECT_EQ(hint.trigger_steady_ns,220);EXPECT_FALSE(ring.hint_snapshot_once().claimed);
  const auto tail=ring.snapshot_once();ASSERT_EQ(tail.count,256u);EXPECT_EQ(tail.overwritten,45u);
  EXPECT_EQ(tail.records.front().sequence,46u);EXPECT_EQ(tail.records[255].sequence,301u);
  for (std::size_t i=0;i<tail.count;++i) EXPECT_EQ(tail.records[i].callback_end_ns,0);
  EXPECT_FALSE(ring.snapshot_once().claimed);
}

TEST(SpatialPrivateTfTiming, ExceptionScopeExitRetainsEndAndDoesNotInventRmwTimes) {
  st::SpatialPrivateTfTimingRing ring;st::SpatialPrivateTfTiming value;
  value.message_count=300;value.scanned_count=256;value.edge(12,999); // Out-of-model indices do not enlarge storage.
  const auto seq=ring.ingress(value);
  try {
    st::SpatialPrivateTfTimingEnd scope(ring,seq,+[]()->int64_t{return 180;});
    ring.stage(seq,st::SpatialPrivateTfTimingRing::Stage::OwnerStart,100);
    ring.stage(seq,st::SpatialPrivateTfTimingRing::Stage::OwnerEnd,110);
    ring.inserted(seq,2,120);ring.outcome(seq,3);
    throw std::runtime_error("existing_private_rejection");
  } catch (const std::runtime_error&) {}
  const auto snapshot=ring.snapshot_once();ASSERT_EQ(snapshot.count,1u);const auto& row=snapshot.records[0];
  EXPECT_EQ(row.rmw_source_ns,0);EXPECT_EQ(row.rmw_received_ns,0);EXPECT_EQ(row.callback_end_ns,180);
  EXPECT_EQ(row.outcome,3);EXPECT_EQ(row.inserted_count,2u);EXPECT_EQ(row.inserted_end_ns,120);
  EXPECT_EQ(row.message_count,300u);EXPECT_EQ(row.scanned_count,256u);
}
