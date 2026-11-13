#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <cmath>
#include <utility>
#include <memory>
#include <future>
#include <thread>
#include <atomic>
#include "sunhorizon_perception_runtime/joined_future.hpp"
#include "sunhorizon_perception_runtime/role_completion.hpp"
#include "sunhorizon_perception_runtime/ground_input_health.hpp"
#include "sunhorizon_perception_runtime/registration_ground_geometry.hpp"
#include "sunhorizon_perception_runtime/spatial_startup_gate.hpp"
#include "sunhorizon_perception_runtime/thread_scheduling.hpp"
#include "sunhorizon_perception_runtime/compute_thread_control.hpp"
#include "sunhorizon_perception_runtime/tf_failure_detail.hpp"
#include "sunhorizon_perception_runtime/spatial_evidence_delivery.hpp"
#include "sunhorizon_perception_runtime/source_time_transform.hpp"
#include "sunhorizon_obstacle_coverage/spatial_evidence_tracker.hpp"
#include "sunhorizon_navigation_interfaces/msg/spatial_coverage_evidence.hpp"
#include <limits>
#include <typeinfo>
#include "sunhorizon_camera_runtime/latest_frame_worker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include "sunhorizon_perception_runtime/rgbd_health.hpp"
#include "sunhorizon_perception_runtime/joint_kinematics.hpp"
#include "sunhorizon_rgbd_adapter/spatial_source_transform.hpp"
#include "sunhorizon_rgbd_adapter/region_depth_pairing.hpp"

namespace {
namespace runtime=sunhorizon_perception_runtime;
namespace cam=sunhorizon_camera_runtime;
constexpr int64_t kSource=10000000000LL;
TEST(RegistrationGroundInput, PExpiredBelly545Chest539CannotWriteOrCreditAndNewReadyRecovers) {
  using Q=runtime::ground_relative::Qualification;
  for(const auto sequence:{545U,539U}) {
    (void)sequence;int map_marks=17,body_handoffs=0,credits=0,observations=0;
    const auto failed=runtime::registration_ground_input_health(true,
      {6,"pcd_registration_original_evidence_expired"},Q::InvalidTransform);
    EXPECT_FALSE(failed.available);
    runtime::detail::complete_role(failed.available,[]{return true;},[&]{++map_marks;++body_handoffs;},
      [&]{++credits;},[&]{++observations;},[](auto){});
    EXPECT_EQ(map_marks,17);EXPECT_EQ(body_handoffs,0);EXPECT_EQ(credits,0);EXPECT_EQ(observations,0);
    for(const auto qualification:{Q::Qualified,Q::InsufficientSupportGeometry,Q::ContradictorySupport}) {
      const auto ready=runtime::registration_ground_input_health(true,{0,"original_livo_history_registration_batch"},qualification);
      EXPECT_TRUE(ready.available); // Genuine Obstacle/Uncertain stays deliverable.
      runtime::detail::complete_role(ready.available,[]{return true;},[&]{++body_handoffs;},
        [&]{++credits;},[&]{++observations;},[](auto){});
    }
    EXPECT_EQ(body_handoffs,3);EXPECT_EQ(credits,3);EXPECT_EQ(observations,3);
  }
}
TEST(RegistrationGroundWait, PendingHistoryBecomesReadyWithoutRenewingOriginalFrame) {
  runtime::ground_relative::SourceIdentity identity{"belly","epoch","calibration","optical","registration_map",
    17,kSource,kSource+1000000,20000000000LL,300000000};
  const auto original=identity;auto token=std::make_shared<std::atomic<bool>>(false);
  int queries=0,waits=0,delivered=0,completed=0;
  auto batch=runtime::detail::wait_for_registration_ground_geometry(identity,[&](const auto& seen) {
    EXPECT_EQ(seen.source_ns,original.source_ns);EXPECT_EQ(seen.sequence,original.sequence);
    EXPECT_EQ(seen.first_receipt_ros_ns,original.first_receipt_ros_ns);
    EXPECT_EQ(seen.first_receipt_steady_ns,original.first_receipt_steady_ns);
    EXPECT_EQ(seen.expiry_budget_ns,original.expiry_budget_ns);
    runtime::RegistrationGroundGeometryBatch result;result.reset_revoked=token;
    result.diagnostic=++queries<3?runtime::ground_relative::SourceTransformDiagnostic{3,"odometry_history_target_after_newest"}:
      runtime::ground_relative::SourceTransformDiagnostic{0,"original_livo_history_registration_batch"};return result;
  },[]{return true;},[]{return false;},[&]{++waits;});
  ASSERT_TRUE(batch);EXPECT_EQ(batch->diagnostic.provider_state,0);EXPECT_EQ(queries,3);EXPECT_EQ(waits,2);
  runtime::detail::complete_role(bool(batch),[]{return true;},[&]{++delivered;},[&]{++completed;},[]{},[](auto){});
  EXPECT_EQ(delivered,1);EXPECT_EQ(completed,1);
}
TEST(RegistrationGroundWait, OriginalExpiryStopAndResetDropWithoutCompletionOrAnotherQuery) {
  runtime::ground_relative::SourceIdentity identity;identity.source_ns=kSource;
  identity.first_receipt_steady_ns=20000000000LL;identity.expiry_budget_ns=300000000;
  for(const auto cause:{0,1,2}) {
    auto token=std::make_shared<std::atomic<bool>>(false);bool fresh=true,stopped=false;
    int queries=0,waits=0,delivered=0,completed=0;
    const auto batch=runtime::detail::wait_for_registration_ground_geometry(identity,[&](const auto&) {
      ++queries;runtime::RegistrationGroundGeometryBatch result;result.reset_revoked=token;
      result.diagnostic={3,"odometry_history_target_after_newest"};return result;
    },[&]{return fresh;},[&]{return stopped;},[&]{++waits;if(cause==0)fresh=false;
      if(cause==1)stopped=true;if(cause==2)token->store(true);});
    EXPECT_FALSE(batch);EXPECT_EQ(queries,1);EXPECT_EQ(waits,1);
    runtime::detail::complete_role(bool(batch),[]{return true;},[&]{++delivered;},[&]{++completed;},[]{},[](auto){});
    EXPECT_EQ(delivered,0);EXPECT_EQ(completed,0);
  }
}
TEST(RegistrationGroundWait, RealProviderFailuresNeverRetry) {
  runtime::ground_relative::SourceIdentity identity;identity.first_receipt_steady_ns=20000000000LL;
  for(const auto& failure:{runtime::ground_relative::SourceTransformDiagnostic{3,"odometry_history_gap_exceeds_tolerance"},
      runtime::ground_relative::SourceTransformDiagnostic{5,"odometry_history_target_too_old"},
      runtime::ground_relative::SourceTransformDiagnostic{6,"pcd_registration_original_evidence_expired"},
      runtime::ground_relative::SourceTransformDiagnostic{7,"pcd_query_scan_identity_mismatch"},
      runtime::ground_relative::SourceTransformDiagnostic{8,"source_fault"}}) {
    int queries=0,waits=0;
    auto batch=runtime::detail::wait_for_registration_ground_geometry(identity,[&](const auto&) {
      ++queries;runtime::RegistrationGroundGeometryBatch result;result.diagnostic=failure;return result;
    },[]{return true;},[]{return false;},[&]{++waits;});
    ASSERT_TRUE(batch);EXPECT_EQ(batch->diagnostic.reason,failure.reason);EXPECT_EQ(queries,1);EXPECT_EQ(waits,0);
  }
}
TEST(RegistrationGroundWait, PendingEpochCannotChangeAndOldReceiptCannotBorrowNewEpoch) {
  runtime::ground_relative::SourceIdentity identity;identity.first_receipt_steady_ns=20000000000LL;
  auto first=std::make_shared<std::atomic<bool>>(false),next=std::make_shared<std::atomic<bool>>(false);
  int queries=0,waits=0;
  auto changed=runtime::detail::wait_for_registration_ground_geometry(identity,[&](const auto&) {
    runtime::RegistrationGroundGeometryBatch result;result.reset_revoked=++queries==1?first:next;
    result.diagnostic=queries==1?runtime::ground_relative::SourceTransformDiagnostic{3,"odometry_history_target_after_newest"}:
      runtime::ground_relative::SourceTransformDiagnostic{0,"original_livo_history_registration_batch"};return result;
  },[]{return true;},[]{return false;},[&]{++waits;});
  EXPECT_FALSE(changed);EXPECT_EQ(queries,2);EXPECT_EQ(waits,1);
  queries=waits=0;
  auto old=runtime::detail::wait_for_registration_ground_geometry(identity,[&](const auto&) {
    ++queries;runtime::RegistrationGroundGeometryBatch result;result.reset_revoked=next;
    result.reset_barrier_steady_ns=identity.first_receipt_steady_ns;
    result.diagnostic={0,"original_livo_history_registration_batch"};return result;
  },[]{return true;},[]{return false;},[&]{++waits;});
  EXPECT_FALSE(old);EXPECT_EQ(queries,1);EXPECT_EQ(waits,0);
}
TEST(RegistrationGroundWait, OriginalRosAndSteadyDeadlinesAreCheckedBeforeAndAfterQuery) {
  runtime::ground_relative::SourceIdentity identity;identity.source_ns=kSource;
  identity.first_receipt_ros_ns=kSource+1000000;identity.first_receipt_steady_ns=20000000000LL;
  identity.expiry_budget_ns=300000000;
  for(const auto cause:{0,1,2}) {
    auto token=std::make_shared<std::atomic<bool>>(false);
    auto ros=identity.first_receipt_ros_ns,steady=identity.first_receipt_steady_ns;
    int queries=0,waits=0;
    const auto batch=runtime::detail::wait_for_registration_ground_geometry(identity,[&](const auto&) {
      ++queries;runtime::RegistrationGroundGeometryBatch result;result.reset_revoked=token;
      result.diagnostic=cause==2?runtime::ground_relative::SourceTransformDiagnostic{0,"original_livo_history_registration_batch"}:
        runtime::ground_relative::SourceTransformDiagnostic{3,"odometry_history_target_after_newest"};
      if(cause==2)ros=identity.source_ns+identity.expiry_budget_ns+1;return result;
    },[&]{return sunhorizon_rgbd_adapter::region_times_fresh(identity.source_ns,
      identity.first_receipt_ros_ns,identity.first_receipt_steady_ns,ros,steady,.3);},[]{return false;},[&] {
      ++waits;if(cause==0)ros=identity.source_ns+identity.expiry_budget_ns+1;
      if(cause==1)steady=identity.first_receipt_steady_ns+identity.expiry_budget_ns+1;
    });
    EXPECT_FALSE(batch);EXPECT_EQ(queries,1);EXPECT_EQ(waits,cause==2?0:1);
  }
}
class RuntimeDirectBoundary : public ::testing::Test {
 protected:
  static void SetUpTestSuite(){int argc=0;char ** argv=nullptr;rclcpp::init(argc,argv);}
  static void TearDownTestSuite(){rclcpp::shutdown();}
};
runtime::RgbdHealthConfig healthy_config() {
  runtime::RgbdHealthConfig config;
  config.statistics_window_sec=1.;config.minimum_healthy_pair_rate_hz=1.;config.target_rate_hz=10.;
  config.chest_alignment_calibration_accepted=true;config.chest_depth_scale_accepted=true;
  config.belly_projection_calibration_accepted=true;config.belly_depth_scale_accepted=true;
  config.calibration_report_sha256=std::string(64,'a');return config;
}
cam::CameraFrame frame(const std::string & role,uint64_t sequence,int64_t source) {
  cam::CameraFrame f;f.role=role;f.sequence=sequence;f.source_validated=true;
  f.first_sdk_callback_ros_ns=source;f.first_sdk_callback_steady_ns=source;
  auto depth=std::make_shared<sensor_msgs::msg::Image>();
  depth->header.stamp=rclcpp::Time(source);depth->header.frame_id=role+"_optical";
  depth->width=4;depth->height=4;depth->step=8;depth->encoding="16UC1";depth->data.resize(32);
  for(size_t i=0;i<16;++i){const uint16_t value=1000;std::memcpy(depth->data.data()+i*2,&value,2);}
  auto info=std::make_shared<sensor_msgs::msg::CameraInfo>();info->header=depth->header;
  info->width=4;info->height=4;info->k={4.,0.,1.5,0.,4.,1.5,0.,0.,1.};
  f.source_stamp=depth->header.stamp;f.depth=depth;f.depth_info=info;
  if(role=="chest"){
    auto color=std::make_shared<sensor_msgs::msg::Image>(*depth);
    color->encoding="rgb8";color->step=12;color->data.resize(48);
    f.color=color;f.color_info=info;
  }
  return f;
}
TEST_F(RuntimeDirectBoundary, ImmutableSdkFrameAdmissionDoesNotAcknowledgeCloud) {
  runtime::RgbdHealth health(healthy_config());
  const auto belly=frame("belly",1,kSource),chest=frame("chest",1,kSource);
  ASSERT_TRUE(health.admit(belly,kSource,kSource).computation_allowed);
  ASSERT_TRUE(health.admit(chest,kSource,kSource).computation_allowed);
  auto state=health.snapshot(kSource,kSource);
  EXPECT_FALSE(state.belly_pointcloud_healthy);EXPECT_FALSE(state.chest_pointcloud_healthy);
  ASSERT_TRUE(health.complete(belly,kSource,kSource+1,kSource+1));
  ASSERT_TRUE(health.complete(chest,kSource,kSource+1,kSource+1));
  EXPECT_EQ(health.snapshot(kSource+2,kSource+2).state,state.STATE_HEALTHY);
  // The admission thread can move ahead while the egress worker holds an
  // older result. Completion preserves that result's original source identity.
  const auto newer=frame("belly",2,kSource+100000000);
  ASSERT_TRUE(health.admit(newer,kSource+100000000,kSource+100000000).accepted);
  EXPECT_FALSE(health.complete(newer,kSource,kSource+100000001,kSource+100000001));
  health.fault("sdk_timestamp_regression");
  state=health.snapshot(kSource+100000002,kSource+100000002);
  EXPECT_EQ(state.state,state.STATE_UNHEALTHY);
  EXPECT_FALSE(state.belly_pointcloud_healthy);EXPECT_FALSE(state.chest_pointcloud_healthy);
}
TEST_F(RuntimeDirectBoundary, CameraInfoAndPixelRatioAreCheckedWithoutProjection) {
  runtime::RgbdHealth health(healthy_config());auto input=frame("belly",1,kSource);
  auto bad=std::make_shared<sensor_msgs::msg::Image>(*input.depth);
  std::fill(bad->data.begin(),bad->data.end(),0);input.depth=bad;
  EXPECT_FALSE(health.admit(input,kSource,kSource).accepted);
  input=frame("belly",2,kSource+100000000);
  auto info=std::make_shared<sensor_msgs::msg::CameraInfo>(*input.depth_info);
  info->header.frame_id="other_camera";input.depth_info=info;
  EXPECT_FALSE(health.admit(input,kSource+100000000,kSource+100000000).accepted);
  EXPECT_FALSE(health.snapshot(kSource+100000000,kSource+100000000).belly_camera_info_valid);
}
TEST_F(RuntimeDirectBoundary, DroppedCameraQueryKeepsOnlyTheOriginalAcceptedCompletionLease) {
  runtime::RgbdHealth health(healthy_config());
  auto original=frame("belly",1,kSource);
  ASSERT_TRUE(health.admit(original,kSource,kSource).computation_allowed);
  ASSERT_TRUE(health.complete(original,kSource,kSource+1,kSource+1));
  auto delayed=frame("belly",2,kSource+10000000);
  delayed.first_sdk_callback_ros_ns=delayed.first_sdk_callback_steady_ns=kSource+290000000;
  ASSERT_TRUE(health.admit(delayed,kSource+290000000,kSource+290000000).computation_allowed);
  // Still within the camera's 300ms contract, outside the unchanged FK 250ms
  // query limit. The runtime drops it without body output or completion credit.
  ASSERT_EQ(runtime::joint_frame_disposition(runtime::JointKinematicsStatus::QueryExpired,true,true),
    runtime::JointFrameDisposition::Unavailable);
  auto current=health.snapshot(kSource+299000000,kSource+299000000);
  EXPECT_TRUE(current.belly_pointcloud_healthy);
  EXPECT_NEAR(current.belly_pointcloud_age_sec,.299,1e-12);
  current=health.snapshot(kSource+300000001,kSource+300000001);
  EXPECT_FALSE(current.belly_pointcloud_healthy);
  EXPECT_NEAR(current.belly_pointcloud_age_sec,.300000001,1e-12); // No restamp by drop.
  auto next=frame("belly",3,kSource+310000000);
  ASSERT_TRUE(health.admit(next,kSource+310000000,kSource+310000000).computation_allowed);
  ASSERT_TRUE(health.complete(next,kSource+310000000,kSource+310000001,kSource+310000001));
  current=health.snapshot(kSource+310000002,kSource+310000002);
  EXPECT_TRUE(current.belly_pointcloud_healthy);
  EXPECT_NEAR(current.belly_pointcloud_age_sec,.000000002,1e-12);
}
TEST_F(RuntimeDirectBoundary, InjectedGlobalTfUsesSourceAndAnchorTimes) {
  // BufferCore transform insertion needs no executor, DDS or live camera.
  auto clock=std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);tf2_ros::Buffer buffer(clock);
  buffer.setUsingDedicatedThread(true);
  geometry_msgs::msg::TransformStamped optical;
  optical.header.frame_id="base_link";optical.child_frame_id="optical";
  optical.transform.rotation.w=1.;optical.transform.translation.x=.2;
  ASSERT_TRUE(buffer.setTransform(optical,"test_fixture",true));
  for(const auto point: {std::pair<int64_t,double>{kSource,0.}, {kSource+100000000,1.}}){
    geometry_msgs::msg::TransformStamped pose;
    pose.header.stamp=rclcpp::Time(point.first,RCL_ROS_TIME);pose.header.frame_id="odom";
    pose.child_frame_id="base_link";pose.transform.rotation.w=1.;pose.transform.translation.x=point.second;
    ASSERT_TRUE(buffer.setTransform(pose,"test_fixture",false));
  }
  const auto deadline=sunhorizon_rgbd_adapter::dynamic_self_model::steady_ns()+20000000;
  const auto exact=sunhorizon_rgbd_adapter::spatial_source_transform(buffer,"base_link","optical",
    kSource,kSource,RCL_ROS_TIME,deadline);
  EXPECT_DOUBLE_EQ(exact.transform.translation.x,.2);
  const auto compensated=sunhorizon_rgbd_adapter::spatial_source_transform(buffer,"base_link","optical",
    kSource,kSource+100000000,RCL_ROS_TIME,deadline);
  EXPECT_NEAR(compensated.transform.translation.x,-.8,1.e-12);
  EXPECT_THROW(sunhorizon_rgbd_adapter::spatial_source_transform(buffer,"base_link","optical",
    kSource-100000000,kSource+100000000,RCL_ROS_TIME,deadline),tf2::TransformException);
}
TEST_F(RuntimeDirectBoundary, ZeroWaitCoreLookupPreservesDynamicWaistAndSensorInterpolationBits) {
  auto clock=std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);tf2_ros::Buffer buffer(clock);
  buffer.setUsingDedicatedThread(true);
  geometry_msgs::msg::TransformStamped sensor;
  sensor.header.frame_id="waist_yaw_link";sensor.child_frame_id="airy_lidar";
  sensor.transform.rotation.w=1.;sensor.transform.translation.x=.096;
  sensor.transform.translation.z=.06502;
  ASSERT_TRUE(buffer.setTransform(sensor,"same_original_sensor_asset",true));
  for(const auto point:{std::pair<int64_t,double>{kSource,-.04},{kSource+100000000,.07}}) {
    geometry_msgs::msg::TransformStamped waist;
    waist.header.stamp=rclcpp::Time(point.first,RCL_ROS_TIME);
    waist.header.frame_id="base_link";waist.child_frame_id="waist_yaw_link";
    waist.transform.translation.z=.15424;
    waist.transform.rotation.z=std::sin(point.second/2.);
    waist.transform.rotation.w=std::cos(point.second/2.);
    ASSERT_TRUE(buffer.setTransform(waist,"same_measured_waist_owner",false));
  }
  for(int64_t offset=0;offset<=100000000;offset+=1000000) {
    const auto ns=kSource+offset;
    const auto old=buffer.lookupTransform("airy_lidar","base_link",rclcpp::Time(ns,RCL_ROS_TIME),
      rclcpp::Duration::from_nanoseconds(0));
    const auto direct=runtime::detail::source_time_transform(buffer,"airy_lidar","base_link",ns);
    EXPECT_EQ(old.header,direct.header);EXPECT_EQ(old.child_frame_id,direct.child_frame_id);
    const auto values=[](const auto& t){return std::array<double,7>{t.translation.x,t.translation.y,
      t.translation.z,t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w};};
    const auto a=values(old.transform),b=values(direct.transform);
    EXPECT_EQ(0,std::memcmp(a.data(),b.data(),sizeof(a))); // Includes signed-zero/ULP identity.
  }
  // This call remains non-blocking and preserves the original unavailable
  // outcomes. It does not replace a missing/future pose with a cached success.
  for(const auto ns:{kSource-1,kSource+100000001}) {
    EXPECT_THROW(buffer.lookupTransform("airy_lidar","base_link",rclcpp::Time(ns,RCL_ROS_TIME),
      rclcpp::Duration::from_nanoseconds(0)),tf2::ExtrapolationException);
    EXPECT_THROW(runtime::detail::source_time_transform(buffer,"airy_lidar","base_link",ns),
      tf2::ExtrapolationException);
  }
  EXPECT_THROW(buffer.lookupTransform("missing_sensor","base_link",rclcpp::Time(kSource,RCL_ROS_TIME),
    rclcpp::Duration::from_nanoseconds(0)),tf2::LookupException);
  EXPECT_THROW(runtime::detail::source_time_transform(buffer,"missing_sensor","base_link",kSource),
    tf2::LookupException);
  EXPECT_THROW(runtime::detail::source_time_transform(buffer,"airy_lidar","base_link",0),
    std::invalid_argument);
}
TEST_F(RuntimeDirectBoundary, BodyOnlyChestDoesNotRequireCrossTimeOdom) {
  auto clock=std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);tf2_ros::Buffer buffer(clock);
  buffer.setUsingDedicatedThread(true);
  geometry_msgs::msg::TransformStamped optical;
  optical.header.frame_id="base_link";optical.child_frame_id="chest_optical";
  optical.transform.rotation.w=1.;optical.transform.translation.x=.2;
  ASSERT_TRUE(buffer.setTransform(optical,"test_fixture",true));
  const auto deadline=sunhorizon_rgbd_adapter::dynamic_self_model::steady_ns()+20000000;
  EXPECT_DOUBLE_EQ(sunhorizon_rgbd_adapter::spatial_source_transform(buffer,"base_link","chest_optical",
    kSource,kSource,RCL_ROS_TIME,deadline).transform.translation.x,.2);
  // Including chest in spatial fusion still requires the measured odom chain
  // between its own source and the belly anchor; static geometry cannot fake it.
  EXPECT_THROW(sunhorizon_rgbd_adapter::spatial_source_transform(buffer,"base_link","chest_optical",
    kSource,kSource+100000000,RCL_ROS_TIME,deadline),tf2::TransformException);
}
TEST(JoinedRoleTask, PendingCancellationAndSkippedTaskMakeFutureReady) {
  using Task=std::packaged_task<int()>;
  std::promise<void> entered,release;auto released=release.get_future().share();
  cam::LatestFrameWorker<Task> worker([](Task& task){task();},[]{return true;},[](auto){});
  auto running=std::make_unique<Task>([&]{entered.set_value();released.wait();return 1;});
  auto running_result=running->get_future();worker.replace(std::move(running));entered.get_future().wait();
  auto pending=std::make_unique<Task>([]{return 2;});
  auto cancelled=pending->get_future();worker.replace(std::move(pending));
  worker.request_stop(); // Pending packaged task is destroyed outside the lane lock.
  EXPECT_EQ(cancelled.wait_for(std::chrono::seconds(0)),std::future_status::ready);
  EXPECT_THROW(cancelled.get(),std::future_error);
  release.set_value();worker.join();EXPECT_EQ(running_result.get(),1);

  cam::LatestFrameWorker<Task> skipped([](Task& task){task();},[]{return false;},[](auto){});
  auto task=std::make_unique<Task>([]{return 3;});auto skipped_result=task->get_future();
  skipped.replace(std::move(task));
  ASSERT_EQ(skipped_result.wait_for(std::chrono::seconds(2)),std::future_status::ready);
  EXPECT_THROW(skipped_result.get(),std::future_error);skipped.request_stop();skipped.join();
}
TEST(JoinedRoleTask, BellyExceptionJoinsChestAndChestExceptionPropagates) {
  std::promise<void> entered,release,finished;auto released=release.get_future().share();
  std::packaged_task<int()> task([&]{entered.set_value();released.wait();return 4;});
  auto result=task.get_future();std::thread chest(std::move(task));entered.get_future().wait();
  std::promise<void> unwinding;auto coordinator=std::async(std::launch::async,[&]{
    try {runtime::JoinedFuture<int> joined(std::move(result));unwinding.set_value();throw 1;}
    catch(int) {finished.set_value();}
  });
  unwinding.get_future().wait();auto done=finished.get_future();
  EXPECT_EQ(done.wait_for(std::chrono::seconds(0)),std::future_status::timeout);
  release.set_value();coordinator.get();chest.join();done.get();
  std::packaged_task<int()> failing([]()->int{throw std::runtime_error("chest_failure");});
  runtime::JoinedFuture<int> failed(failing.get_future());failing();
  EXPECT_THROW(failed.get(),std::runtime_error);
}
TEST(RoleCompletion, EitherRoleHandsOffOnceWhileCoverageWaitsForOtherRole) {
  using Task=std::packaged_task<int()>;
  for(const size_t fast_role:{0u,1u}){
    std::atomic<int> delivered[2]{},credited[2]{},observed[2]{};
    std::atomic<int> coverage{0};std::atomic<bool> active{true};
    std::promise<void> entered,release,handed_off;
    auto released=release.get_future().share();
    auto allowed=[&]{return active.load();};
    cam::LatestFrameWorker<Task> belly([](Task& task){task();},allowed,[](auto){});
    cam::LatestFrameWorker<Task> chest([](Task& task){task();},allowed,[](auto){});
    cam::LatestFrameWorker<Task>* workers[]{&belly,&chest};
    auto slow=std::make_unique<Task>([&]{entered.set_value();released.wait();return 1;});
    auto slow_result=slow->get_future();workers[1-fast_role]->replace(std::move(slow));
    entered.get_future().wait();
    auto fast=std::make_unique<Task>([&]{
      runtime::detail::complete_role(true,allowed,[&]{++delivered[fast_role];},
        [&]{++credited[fast_role];},[&]{++observed[fast_role];},[](const auto&){FAIL();});
      handed_off.set_value();return 2;
    });
    auto fast_result=fast->get_future();workers[fast_role]->replace(std::move(fast));
    auto coordinator=std::async(std::launch::async,[&]{
      runtime::JoinedFuture<int> joined(std::move(slow_result));
      const auto own=fast_result.get();const auto other=joined.get();
      if(active.load())++coverage;return own+other;
    });
    EXPECT_EQ(handed_off.get_future().wait_for(std::chrono::seconds(2)),std::future_status::ready);
    EXPECT_EQ(delivered[fast_role].load(),1);EXPECT_EQ(credited[fast_role].load(),1);
    EXPECT_EQ(observed[fast_role].load(),1);EXPECT_EQ(delivered[1-fast_role].load(),0);
    EXPECT_EQ(coverage.load(),0);
    EXPECT_EQ(coordinator.wait_for(std::chrono::seconds(0)),std::future_status::timeout);
    release.set_value();EXPECT_EQ(coordinator.get(),3);EXPECT_EQ(coverage.load(),1);
    belly.request_stop();chest.request_stop();belly.join();chest.join();
    EXPECT_EQ(delivered[fast_role].load(),1);
  }
}
TEST(RoleCompletion, SinkTfExceptionLatchesBeforeJoinWithoutCreditOrCoverage) {
  using Task=std::packaged_task<int()>;
  std::atomic<bool> terminal{false};std::atomic<int> credit{0},observe{0},coverage{0};
  std::promise<void> entered,release,faulted;auto released=release.get_future().share();
  auto allowed=[&]{return !terminal.load();};
  cam::LatestFrameWorker<Task> other([](Task& task){task();},allowed,[](auto){});
  cam::LatestFrameWorker<Task> failing([](Task& task){task();},allowed,[](auto){});
  auto slow=std::make_unique<Task>([&]{entered.set_value();released.wait();return 1;});
  auto slow_result=slow->get_future();other.replace(std::move(slow));entered.get_future().wait();
  auto task=std::make_unique<Task>([&]{
    runtime::detail::complete_role(true,allowed,[]{throw tf2::TransformException("sink_tf_failure");},
      [&]{++credit;},[&]{++observe;},[&](const std::string& reason){
        EXPECT_EQ(reason,"sink_tf_failure");terminal.store(true);faulted.set_value();
      });
    return 2;
  });
  auto failed=task->get_future();failing.replace(std::move(task));
  auto coordinator=std::async(std::launch::async,[&]{
    try{
      runtime::JoinedFuture<int> joined(std::move(slow_result));
      failed.get();joined.get();++coverage;
    }catch(const tf2::TransformException&){return false;} // Must not become a soft TF miss.
    catch(const std::runtime_error&){return true;}
    return false;
  });
  EXPECT_EQ(faulted.get_future().wait_for(std::chrono::seconds(2)),std::future_status::ready);
  EXPECT_TRUE(terminal.load());EXPECT_EQ(credit.load(),0);EXPECT_EQ(observe.load(),0);EXPECT_EQ(coverage.load(),0);
  EXPECT_EQ(coordinator.wait_for(std::chrono::seconds(0)),std::future_status::timeout);
  release.set_value();EXPECT_TRUE(coordinator.get());EXPECT_EQ(coverage.load(),0);
  other.request_stop();failing.request_stop();other.join();failing.join();
}
TEST(RoleCompletion, ExpiryOrStopSuppressesDeliveryAndStopStillDrainsRunningRole) {
  int delivered=0,credit=0,observe=0;std::atomic<bool> active{true};
  auto complete=[&](bool fresh){runtime::detail::complete_role(fresh,[&]{return active.load();},
    [&]{++delivered;},[&]{++credit;},[&]{++observe;},[](const auto&){FAIL();});};
  complete(false);EXPECT_EQ(delivered,0);EXPECT_EQ(credit,0);EXPECT_EQ(observe,0);
  using Task=std::packaged_task<int()>;
  std::promise<void> entered,release;auto released=release.get_future().share();
  cam::LatestFrameWorker<Task> worker([](Task& task){task();},[&]{return active.load();},[](auto){});
  auto running=std::make_unique<Task>([&]{entered.set_value();released.wait();complete(true);return 1;});
  auto result=running->get_future();worker.replace(std::move(running));entered.get_future().wait();
  active.store(false);worker.request_stop();
  auto drained=std::async(std::launch::async,[&]{runtime::JoinedFuture<int> joined(std::move(result));});
  EXPECT_EQ(drained.wait_for(std::chrono::seconds(0)),std::future_status::timeout);
  release.set_value();drained.get();worker.join();
  EXPECT_EQ(delivered,0);EXPECT_EQ(credit,0);EXPECT_EQ(observe,0);
}
TEST(RoleCompletion, SinkThatLatchesFaultWithoutThrowingCannotGiveHealthCredit) {
  bool active=true;int delivered=0,credit=0,observe=0;
  runtime::detail::complete_role(true,[&]{return active;},
    [&]{++delivered;active=false;},[&]{++credit;},[&]{++observe;},[](const auto&){FAIL();});
  EXPECT_EQ(delivered,1);EXPECT_EQ(credit,0);EXPECT_EQ(observe,0);
}
using SpatialDeliveryEvidence=sunhorizon_navigation_interfaces::msg::SpatialCoverageEvidence;
SpatialDeliveryEvidence spatial_delivery_sample(uint64_t anchor) {
  SpatialDeliveryEvidence evidence;
  evidence.evaluation_complete=true;
  evidence.anchor_frame_sequence=anchor;
  evidence.anchor_source_ros_ns=1100+anchor;
  evidence.sources.resize(1);
  auto& source=evidence.sources.front();
  source.source_id=0;source.frame_sequence=anchor;source.source_ros_ns=evidence.anchor_source_ros_ns;
  source.first_receipt_ros_ns=1200+anchor;source.first_receipt_steady_ns=2100+anchor;
  source.valid_until_ros_ns=5000;source.valid_until_steady_ns=6000;
  return evidence;
}
TEST(SpatialDelivery, BlockedDdsCannotDelayDirectFramesOrCreditFirstPublication) {
  runtime::detail::SpatialEvidenceDelivery<SpatialDeliveryEvidence> delivery;
  runtime::SpatialStartupGate startup;startup.defer();startup.owner_ready(3000);
  ASSERT_TRUE(startup.qualify(true,1000,2000));
  std::promise<void> entered,release,latest_done;auto released=release.get_future().share();
  std::vector<SpatialDeliveryEvidence> direct,observed;
  cam::LatestFrameWorker<SpatialDeliveryEvidence> observer([&](auto& evidence){
    if(evidence.anchor_frame_sequence==1){entered.set_value();released.wait();}
    observed.push_back(evidence);
    const auto& source=evidence.sources.front();
    startup.published(source.source_ros_ns,source.first_receipt_ros_ns,source.first_receipt_steady_ns,
      source.frame_sequence,2200);
    if(evidence.anchor_frame_sequence==3)latest_done.set_value();
  },[]{return true;},[](auto){FAIL();});
  auto submit=[&](uint64_t anchor){
    return delivery.submit(spatial_delivery_sample(anchor),[](auto&){},
      [&](auto& evidence){direct.push_back(evidence);},
      [&](auto evidence){observer.replace(std::make_unique<SpatialDeliveryEvidence>(std::move(evidence)));});
  };
  EXPECT_TRUE(submit(1));entered.get_future().wait();
  EXPECT_TRUE(submit(2));EXPECT_TRUE(submit(3));
  EXPECT_EQ(direct.size(),3U);
  EXPECT_EQ(startup.snapshot().first_published_steady_ns,0);
  release.set_value();
  EXPECT_EQ(latest_done.get_future().wait_for(std::chrono::seconds(2)),std::future_status::ready);
  observer.request_stop();observer.join();
  ASSERT_EQ(observed.size(),2U);
  EXPECT_EQ(observed[0].evaluation_sequence,1U);EXPECT_EQ(observed[1].evaluation_sequence,3U);
  EXPECT_EQ(startup.snapshot().first_published_sequence,1U);
  for(size_t i=0;i<direct.size();++i){
    EXPECT_EQ(direct[i].evaluation_sequence,i+1);
    EXPECT_EQ(direct[i].sources[0].source_ros_ns,1101+static_cast<int64_t>(i));
    EXPECT_EQ(direct[i].sources[0].first_receipt_ros_ns,1201+static_cast<int64_t>(i));
    EXPECT_EQ(direct[i].sources[0].first_receipt_steady_ns,2101+static_cast<int64_t>(i));
    EXPECT_EQ(direct[i].sources[0].valid_until_ros_ns,5000);
    EXPECT_EQ(direct[i].sources[0].valid_until_steady_ns,6000);
  }
}
TEST(SpatialDelivery, ConcurrentRevocationFollowsClaimedDirectFrameAndCannotBeReplacedByPositive) {
  runtime::detail::SpatialEvidenceDelivery<SpatialDeliveryEvidence> delivery;
  std::promise<void> entered,release,revoke_started;auto released=release.get_future().share();
  std::vector<SpatialDeliveryEvidence> direct,observed;
  auto sink=[&](auto& evidence){
    if(evidence.evaluation_complete){entered.set_value();released.wait();}
    direct.push_back(evidence);
  };
  auto observe=[&](auto evidence){observed.push_back(std::move(evidence));};
  auto submitted=std::async(std::launch::async,[&]{
    return delivery.submit(spatial_delivery_sample(1),[](auto&){},sink,observe);
  });
  entered.get_future().wait();
  auto revoked=std::async(std::launch::async,[&]{
    revoke_started.set_value();
    return delivery.revoke([](auto& evidence){evidence.reason="terminal_revocation:first_fault";},sink,observe);
  });
  revoke_started.get_future().wait();
  EXPECT_EQ(revoked.wait_for(std::chrono::seconds(0)),std::future_status::timeout);
  release.set_value();EXPECT_TRUE(submitted.get());EXPECT_TRUE(revoked.get());
  EXPECT_FALSE(delivery.submit(spatial_delivery_sample(2),[](auto&){},sink,observe));
  EXPECT_FALSE(delivery.revoke([](auto&){},sink,observe));
  ASSERT_EQ(direct.size(),2U);ASSERT_EQ(observed.size(),2U);
  EXPECT_TRUE(direct[0].evaluation_complete);EXPECT_FALSE(direct[1].evaluation_complete);
  EXPECT_EQ(direct[1].evaluation_sequence,2U);
  EXPECT_EQ(direct[1].anchor_frame_sequence,1U);
  EXPECT_EQ(observed[1].reason,"terminal_revocation:first_fault");
}
TEST(SpatialDelivery, ExpiredObserverCannotRewriteLatestDirectSnapshotOrRestoreHealth) {
  runtime::detail::SpatialEvidenceDelivery<SpatialDeliveryEvidence> delivery;
  std::vector<SpatialDeliveryEvidence> direct,observed;
  auto sink=[&](auto& evidence){direct.push_back(evidence);};
  auto observe=[&](auto evidence){observed.push_back(std::move(evidence));};
  EXPECT_TRUE(delivery.submit(spatial_delivery_sample(1),[](auto&){},sink,observe));
  EXPECT_TRUE(delivery.submit(spatial_delivery_sample(2),[](auto&){},sink,observe));
  auto delayed=observed.front();
  EXPECT_FALSE(runtime::detail::expire_spatial_evidence(delayed,5000,6000));
  EXPECT_TRUE(runtime::detail::expire_spatial_evidence(delayed,5001,6000));
  EXPECT_FALSE(delayed.evaluation_complete);EXPECT_EQ(delayed.reason,"source_expired_before_egress");
  // Production still latches this observer failure. Revocation must name the
  // latest direct anchor, not the old DDS copy which just expired.
  EXPECT_TRUE(delivery.revoke([](auto& evidence){evidence.reason="terminal_revocation:source_expired_before_egress";},sink,observe));
  EXPECT_FALSE(delivery.submit(spatial_delivery_sample(3),[](auto&){},sink,observe));
  ASSERT_EQ(direct.size(),3U);
  EXPECT_EQ(direct.back().anchor_frame_sequence,2U);
  EXPECT_EQ(direct.back().evaluation_sequence,3U);EXPECT_FALSE(direct.back().evaluation_complete);
  EXPECT_EQ(direct.back().sources[0].source_ros_ns,1102);
  EXPECT_EQ(direct.back().sources[0].valid_until_ros_ns,5000);
  // An optional noncontributing chest source retains the original exception.
  auto optional=spatial_delivery_sample(4);optional.sources[0].source_id=1;
  EXPECT_FALSE(runtime::detail::expire_spatial_evidence(optional,5001,6001));
}
TEST(SpatialDelivery, Run64ExpiredPreviousEvidenceRecoversFreshObservationWithoutTerminalFault) {
  namespace coverage=sunhorizon_obstacle_coverage;
  using Observation=coverage::SpatialCoverageObservation;
  constexpr int64_t r=1792414871547435000LL,s=9000000000LL;
  coverage::SpatialEvidenceTracker tracker({"actual_gid","contract","query","boot","base","producer",
    {{0,"camera","optical"}}},coverage::SpatialRecoveryPolicy::RecoveryOnNewAnchor);
  runtime::detail::SpatialEvidenceDelivery<Observation> delivery;
  bool parent_terminal=false,producer_terminal=false;
  int computed=0;std::vector<Observation> observed;std::vector<coverage::SpatialDeliveryResult> outcomes;
  using Task=std::packaged_task<void()>;
  cam::LatestFrameWorker<Task> worker([](Task& task){task();},
    [&]{return !parent_terminal&&!producer_terminal;},[](auto){FAIL();});
  auto submit=[&](uint64_t sequence,int64_t source,int64_t sdk,int64_t header,int64_t receipt) {
    Observation value;value.schema_version=1;value.contract="contract";value.query_digest="query";
    value.clock_boot_id="boot";value.frame_id="base";value.producer_epoch="producer";value.reason="clear";
    value.header_ros_ns=r+header;value.evaluation_steady_ns=s+header;
    value.anchor_source_ros_ns=r+source;value.anchor_frame_sequence=sequence;value.input_generation=sequence;
    value.evaluation_complete=true;value.total_cells=value.free_cells=1;
    value.sources={{0,1,"camera","optical",sequence,r+source,r+sdk,s+sdk,
      r+source+300000000,s+source+300000000,true}};
    auto task=std::make_unique<Task>([&,value,receipt]() mutable {
      ++computed;
      delivery.submit(value,[](auto&){},[&](auto& evidence){
        tracker.observe(evidence,"actual_gid",r+receipt,s+receipt);
        const auto result=tracker.delivery_result();outcomes.push_back(result);
        // These are the exact two production fault-domain boundaries.
        coverage::propagate_spatial_internal_fault(result,[&](const auto&){parent_terminal=true;});
        runtime::detail::apply_spatial_delivery_result(evidence,result,[&](const auto&){producer_terminal=true;});
      },[&](auto evidence){observed.push_back(std::move(evidence));});
    });
    auto done=task->get_future();worker.replace(std::move(task));
    EXPECT_EQ(done.wait_for(std::chrono::seconds(2)),std::future_status::ready);done.get();
  };
  submit(989,0,179124830,264440573,270000000);
  ASSERT_TRUE(outcomes.back().accepted());
  submit(992,100123000,258557258,328683937,328683937);
  ASSERT_TRUE(outcomes.back().accepted());
  EXPECT_FALSE(parent_terminal);EXPECT_FALSE(producer_terminal);
  EXPECT_TRUE(observed.back().evaluation_complete);
  EXPECT_EQ(observed.back().sources[0].source_ros_ns,r+100123000);
  EXPECT_EQ(observed.back().sources[0].first_receipt_ros_ns,r+258557258);
  EXPECT_GT(observed.back().sources[0].valid_until_ros_ns,r+328683937);
  submit(995,200246000,300000000,370000000,370000000);
  worker.request_stop();worker.join();
  EXPECT_EQ(computed,3);ASSERT_EQ(observed.size(),3U);
  EXPECT_FALSE(parent_terminal);EXPECT_FALSE(producer_terminal);
  EXPECT_TRUE(observed.back().evaluation_complete);
  EXPECT_TRUE(tracker.evaluate(r+370000001,s+370000001).valid);
}
TEST(SpatialDelivery, InternalFailureAndInvalidAcceptanceStillTerminateProducer) {
  using State=sunhorizon_obstacle_coverage::SpatialDeliveryState;
  auto evidence=spatial_delivery_sample(1);bool terminal=false;
  runtime::detail::apply_spatial_delivery_result(evidence,{State::InternalFault,"identity_changed"},
    [&](const auto&){terminal=true;});
  EXPECT_TRUE(terminal);EXPECT_FALSE(evidence.evaluation_complete);
  terminal=false;
  runtime::detail::apply_spatial_delivery_result(evidence,{State::Accepted,{}},
    [&](const auto&){terminal=true;});
  EXPECT_TRUE(terminal);
}
TEST(SpatialDelivery, SinkExceptionRetainsRevocationIdentityAndStopDrainsDirectCallback) {
  runtime::detail::SpatialEvidenceDelivery<SpatialDeliveryEvidence> failed;
  std::vector<SpatialDeliveryEvidence> observed;
  EXPECT_THROW(failed.submit(spatial_delivery_sample(7),[](auto&){},
    [](auto&){throw std::runtime_error("direct_sink_failure");},
    [&](auto evidence){observed.push_back(std::move(evidence));}),std::runtime_error);
  EXPECT_TRUE(observed.empty());
  EXPECT_TRUE(failed.revoke([](auto& evidence){evidence.reason="terminal_revocation:direct_sink_failure";},
    [](auto&){},[&](auto evidence){observed.push_back(std::move(evidence));}));
  ASSERT_EQ(observed.size(),1U);EXPECT_EQ(observed[0].anchor_frame_sequence,7U);
  EXPECT_EQ(observed[0].evaluation_sequence,2U);EXPECT_FALSE(observed[0].evaluation_complete);

  runtime::detail::SpatialEvidenceDelivery<SpatialDeliveryEvidence> stopped;
  std::promise<void> entered,release;auto released=release.get_future().share();
  auto submitted=std::async(std::launch::async,[&]{
    return stopped.submit(spatial_delivery_sample(8),[](auto&){},
      [&](auto&){entered.set_value();released.wait();},[](auto){});
  });
  entered.get_future().wait();
  auto closed=std::async(std::launch::async,[&]{stopped.close();});
  EXPECT_EQ(closed.wait_for(std::chrono::seconds(0)),std::future_status::timeout);
  release.set_value();EXPECT_TRUE(submitted.get());closed.get();
  EXPECT_FALSE(stopped.submit(spatial_delivery_sample(9),[](auto&){},[](auto&){FAIL();},[](auto){FAIL();}));
  EXPECT_FALSE(stopped.revoke([](auto&){},[](auto&){FAIL();},[](auto){FAIL();}));
}
TEST(SpatialStartup, LegacyOwnerBypassesAndDeferralCannotBeRetrofitted) {
  runtime::SpatialStartupGate legacy;
  EXPECT_TRUE(legacy.allows(kSource,kSource,kSource));
  EXPECT_TRUE(legacy.publication_allowed(kSource));
  EXPECT_FALSE(legacy.snapshot().deferred);
  EXPECT_THROW(legacy.owner_ready(kSource),std::logic_error);
  legacy.note_input();EXPECT_THROW(legacy.defer(),std::logic_error);
}
TEST(SpatialStartup, RequiresOwnerThenCurrentHealthAndOnlyNewSources) {
  runtime::SpatialStartupGate gate;gate.defer();
  EXPECT_THROW(gate.defer(),std::logic_error);
  EXPECT_FALSE(gate.qualify(true,1000,2000));
  EXPECT_FALSE(gate.allows(1001,1002,2001));
  gate.owner_ready(3000);EXPECT_THROW(gate.owner_ready(4000),std::logic_error);
  EXPECT_FALSE(gate.qualify(false,1000,2000));
  EXPECT_TRUE(gate.qualify(true,1000,2000));
  // Pending SDK conversions and already-running role work must not be promoted
  // merely because the coordinator reaches egress after qualification opened.
  EXPECT_FALSE(gate.allows(999,1002,2001));
  EXPECT_FALSE(gate.allows(1000,1002,2001));
  EXPECT_FALSE(gate.allows(1001,1000,2001));
  EXPECT_FALSE(gate.allows(1001,1002,2000));
  EXPECT_TRUE(gate.allows(1001,1002,2001));
  EXPECT_FALSE(gate.qualify(false,1100,2100));
  EXPECT_FALSE(gate.qualify(true,1200,2200));
  EXPECT_EQ(gate.snapshot().opened_ros_ns,1000);
  EXPECT_EQ(gate.snapshot().opened_steady_ns,2000);
  EXPECT_EQ(gate.snapshot().deadline_steady_ns,3000);
  EXPECT_TRUE(gate.allows(1001,1002,2001)); // Loss of health cannot close/reopen it.
}
TEST(SpatialStartup, OnlyActualFirstPublicationWithinOriginalDeadlineCompletes) {
  runtime::SpatialStartupGate gate;gate.defer();gate.owner_ready(3000);
  ASSERT_TRUE(gate.qualify(true,1000,2000));
  EXPECT_FALSE(gate.snapshot().first_published_steady_ns);
  EXPECT_FALSE(gate.published(999,1002,2001,1,2100));
  EXPECT_TRUE(gate.publication_allowed(2999));
  EXPECT_FALSE(gate.publication_allowed(3000));
  EXPECT_TRUE(gate.published(1001,1002,2001,2,2999));
  EXPECT_FALSE(gate.published(1100,1101,2100,3,3100));
  EXPECT_EQ(gate.snapshot().first_published_steady_ns,2999);
  EXPECT_EQ(gate.snapshot().first_published_sequence,2u);
  EXPECT_TRUE(gate.publication_allowed(5000)); // No new grace/health gate after first proof.
  runtime::SpatialStartupGate late;late.defer();late.owner_ready(3000);
  ASSERT_TRUE(late.qualify(true,1000,2000));
  EXPECT_FALSE(late.published(1001,1002,2001,1,3000));
  EXPECT_EQ(late.snapshot().first_published_steady_ns,0);
  EXPECT_THROW(late.defer(),std::logic_error);
  runtime::SpatialStartupGate unqualified;unqualified.defer();unqualified.owner_ready(3000);
  EXPECT_FALSE(unqualified.qualify(true,1000,3000));
  EXPECT_FALSE(unqualified.qualify(true,1100,4000));
  EXPECT_FALSE(unqualified.publication_allowed(4000));
  EXPECT_EQ(unqualified.snapshot().deadline_steady_ns,3000);
  EXPECT_FALSE(unqualified.snapshot().opened);
}
TEST(SpatialStartup, ExpiredFirstProofRejectsWithoutDestroyingAcquisitionOwners) {
  runtime::SpatialStartupGate gate;gate.defer();gate.owner_ready(3000);
  ASSERT_TRUE(gate.qualify(true,1000,2000));
  // A localization hold may arrive after startup release and before its first
  // spatial publication. No new grace, exception, or late proof may follow.
  for (int64_t now : {3000,3001,5000}) {
    EXPECT_FALSE(gate.publication_allowed(now));
    EXPECT_FALSE(gate.published(1001,1002,2001,1,now));
    EXPECT_EQ(gate.snapshot().first_published_steady_ns,0);
    EXPECT_EQ(gate.snapshot().deadline_steady_ns,3000);
  }
  // Structural and invalid-clock errors remain fatal, even though expiry is
  // only unavailable startup evidence.
  runtime::SpatialStartupGate invalid;invalid.defer();invalid.owner_ready(3000);
  EXPECT_THROW(invalid.qualify(true,0,2000),std::runtime_error);
  EXPECT_THROW(invalid.owner_ready(4000),std::logic_error);
}
TEST(SpatialStartup, ConcurrentCallsCannotReplaceBarrierOrFirstPublication) {
  runtime::SpatialStartupGate gate;gate.defer();gate.owner_ready(3000);
  std::atomic<int> openings{0},publications{0};
  std::promise<void> ready;auto start=ready.get_future().share();
  std::thread a([&]{start.wait();if(gate.qualify(true,1000,2000))++openings;});
  std::thread b([&]{start.wait();if(gate.qualify(true,1001,2001))++openings;});
  ready.set_value();a.join();b.join();EXPECT_EQ(openings.load(),1);
  const auto opened=gate.snapshot();
  EXPECT_EQ(opened.opened_steady_ns-opened.opened_ros_ns,1000);
  std::thread c([&]{if(gate.published(1100,1101,2100,1,2200))++publications;});
  std::thread d([&]{if(gate.published(1100,1101,2100,2,2201))++publications;});
  c.join();d.join();EXPECT_EQ(publications.load(),1);
  EXPECT_EQ(gate.snapshot().opened_ros_ns,opened.opened_ros_ns);
  EXPECT_EQ(gate.snapshot().opened_steady_ns,opened.opened_steady_ns);
}
TEST(TfFailureDetail, KeepsOriginalReasonAndOwnerStageBoundaries) {
  runtime::detail::TfFailureDetail detail;
  detail.role="chest";detail.sequence=760;detail.source_ns=100;detail.query_ns=200;
  detail.fk_end_ns=300;detail.deadline_ns=20000300;detail.begin_ns=301;
  detail.camera_end_ns=400;detail.observations_end_ns=500;detail.spatial_begin_ns=501;
  detail.failure_ns=21000300;detail.current_ros_ns=22000300;
  detail.cpu_begin_ns=1000;detail.cpu_end_ns=1100;
  detail.scheduling.status=runtime::detail::SchedstatStatus::Available;
  detail.scheduling.runqueue_wait_ns=20000000;detail.scheduling.sampling_wall_ns=10;
  const auto message=detail.message("model_tf_budget_exhausted","timeout");
  EXPECT_EQ(message,"model_tf_budget_exhausted tf{r=chest,n=760,s=100,q=200,fk=300,"
    "d=20000300,f=21000300,ros=22000300,b=301,cpu=100,rq=20000000,rs=1,sc=10,"
    "ca=400,ob=500,sp=501,k=timeout}");
  EXPECT_EQ(detail.deadline_ns,detail.fk_end_ns+20000000); // Observation cannot renew the budget.
}
TEST(TfFailureDetail, MissingStagesAndUnavailableOrRegressedCpuRemainNegative) {
  runtime::detail::TfFailureDetail detail;
  const auto missing=detail.message("failure","lookup");
  EXPECT_NE(missing.find(",cpu=-1,rq=-1,rs=0,sc=-1,ca=-1,ob=-1,sp=-1,"),std::string::npos);
  detail.cpu_begin_ns=100;detail.cpu_end_ns=99;
  EXPECT_NE(detail.message("failure","lookup").find(",cpu=-1,"),std::string::npos);
  // An available zero CPU/runqueue delta is distinguishable from unavailable.
  detail.cpu_end_ns=100;detail.scheduling.runqueue_wait_ns=0;
  detail.scheduling.status=runtime::detail::SchedstatStatus::Available;
  EXPECT_NE(detail.message("failure","lookup").find(",cpu=0,rq=0,rs=1,"),std::string::npos);
}
TEST(TfFailureDetail, MaximumValuesAndLongReasonRetainStagesWithinOriginalLatchLimit) {
  runtime::detail::TfFailureDetail detail;
  detail.role="chest";detail.sequence=std::numeric_limits<uint64_t>::max();
  const auto maximum=std::numeric_limits<int64_t>::max();
  detail.source_ns=detail.query_ns=detail.fk_end_ns=detail.deadline_ns=maximum;
  detail.begin_ns=detail.failure_ns=detail.current_ros_ns=maximum;
  detail.camera_end_ns=detail.observations_end_ns=detail.spatial_begin_ns=maximum;
  detail.cpu_begin_ns=0;detail.cpu_end_ns=maximum;
  detail.scheduling.runqueue_wait_ns=detail.scheduling.sampling_wall_ns=maximum;
  const auto message=detail.message(("model_tf_budget_exhausted:"+std::string(1024,'x')).c_str(),"invalid_argument");
  EXPECT_LE(message.size(),512U);
  EXPECT_EQ(message.find("model_tf_budget_exhausted:"),0U);
  EXPECT_NE(message.find(",ca="+std::to_string(maximum)),std::string::npos);
  EXPECT_NE(message.find(",ob="+std::to_string(maximum)),std::string::npos);
  EXPECT_NE(message.find(",sp="+std::to_string(maximum)+",k=invalid_argument}"),std::string::npos);
}
template<class Exception> void check_tf_failure_category(const char* kind) {
  runtime::detail::TfFailureDetail detail;
  bool caught=false;
  try {
    const Exception original("model_tf_budget_exhausted");
    runtime::detail::rethrow_tf_failure(original,detail);
  } catch(const Exception& enriched) {
    caught=true;
    EXPECT_TRUE(typeid(enriched)==typeid(Exception));
    const std::string message=enriched.what();
    EXPECT_EQ(message.find("model_tf_budget_exhausted tf{"),0U);
    EXPECT_NE(message.find(std::string(",k=")+kind+"}"),std::string::npos);
  }
  EXPECT_TRUE(caught);
}
TEST(TfFailureDetail, PreservesEachNativeTfExceptionCategory) {
  check_tf_failure_category<tf2::TimeoutException>("timeout");
  check_tf_failure_category<tf2::LookupException>("lookup");
  check_tf_failure_category<tf2::ConnectivityException>("connectivity");
  check_tf_failure_category<tf2::ExtrapolationException>("extrapolation");
  check_tf_failure_category<tf2::InvalidArgumentException>("invalid_argument");
  check_tf_failure_category<tf2::TransformException>("transform");
}
TEST(TfFailureDetail, OnlyKnownLookupUnavailableCategoriesDropAFrame) {
  using runtime::detail::transient_tf_unavailable;
  EXPECT_TRUE(transient_tf_unavailable(tf2::TimeoutException("budget exhausted")));
  EXPECT_TRUE(transient_tf_unavailable(tf2::LookupException("not received")));
  EXPECT_TRUE(transient_tf_unavailable(tf2::ConnectivityException("not connected yet")));
  EXPECT_TRUE(transient_tf_unavailable(tf2::ExtrapolationException("outside cache")));
  EXPECT_FALSE(transient_tf_unavailable(tf2::InvalidArgumentException("invalid frame")));
  EXPECT_FALSE(transient_tf_unavailable(tf2::TransformException("unclassified fault")));
}
TEST(ThreadScheduling, StrictParserKeepsRealZeroWaitAndRejectsUnavailableOrMalformedCounters) {
  using namespace runtime::detail;
  ThreadSchedulingSample sample;
  ASSERT_TRUE(parse_schedstat(" 123456 0 7\n",sample));
  EXPECT_EQ(sample.status,SchedstatStatus::Available);
  EXPECT_EQ(sample.running_ns,123456u);EXPECT_EQ(sample.runqueue_wait_ns,0u);EXPECT_EQ(sample.timeslices,7u);
  ASSERT_TRUE(parse_schedstat("0 0 0\n",sample));
  EXPECT_EQ(sample.status,SchedstatStatus::CountersInactive);
  for(const auto* invalid:{"","1 2","-1 2 3","1 -2 3","1 2 3 4","1 2 3x","18446744073709551616 2 3"}){
    EXPECT_FALSE(parse_schedstat(invalid,sample));EXPECT_EQ(sample.status,SchedstatStatus::Malformed);
  }
}
TEST(ThreadScheduling, SameThreadMonotonicDeltaPreservesSchedulingAndContextSwitchEvidence) {
  using namespace runtime::detail;
  ThreadSchedulingSample before,after;before.tid=after.tid=42;
  before.read_begin_steady_ns=1000;before.read_end_steady_ns=1010;
  after.read_begin_steady_ns=2000;after.read_end_steady_ns=2020;
  ASSERT_TRUE(parse_schedstat("100 20 3",before));ASSERT_TRUE(parse_schedstat("180 70 5",after));
  before.rusage_available=after.rusage_available=true;
  before.voluntary_switches=4;before.involuntary_switches=5;
  after.voluntary_switches=7;after.involuntary_switches=9;
  auto delta=scheduling_delta(before,after);
  EXPECT_TRUE(delta.schedstat_available);EXPECT_TRUE(delta.rusage_available);
  EXPECT_EQ(delta.running_ns,80);EXPECT_EQ(delta.runqueue_wait_ns,50);EXPECT_EQ(delta.timeslices,2);
  EXPECT_EQ(delta.voluntary_switches,3);EXPECT_EQ(delta.involuntary_switches,4);
  EXPECT_EQ(delta.sampling_wall_ns,30);EXPECT_EQ(delta.tid,42);
  after.runqueue_wait_ns=before.runqueue_wait_ns;
  delta=scheduling_delta(before,after);EXPECT_TRUE(delta.schedstat_available);EXPECT_EQ(delta.runqueue_wait_ns,0);
  after.status=SchedstatStatus::ReadFailed;
  delta=scheduling_delta(before,after);EXPECT_FALSE(delta.schedstat_available);EXPECT_EQ(delta.running_ns,-1);
  EXPECT_EQ(delta.runqueue_wait_ns,-1);EXPECT_TRUE(delta.rusage_available);
  after.status=SchedstatStatus::Available;after.tid=43;
  delta=scheduling_delta(before,after);EXPECT_FALSE(delta.schedstat_available);EXPECT_FALSE(delta.rusage_available);
  EXPECT_EQ(delta.status,SchedstatStatus::ThreadChanged);
  after.tid=42;after.running_ns=99;
  delta=scheduling_delta(before,after);EXPECT_FALSE(delta.schedstat_available);
  EXPECT_EQ(delta.status,SchedstatStatus::CounterRegressed);EXPECT_EQ(delta.running_ns,-1);
  after.running_ns=std::numeric_limits<uint64_t>::max();
  delta=scheduling_delta(before,after);EXPECT_FALSE(delta.schedstat_available);
  EXPECT_EQ(delta.status,SchedstatStatus::DeltaOverflow);
}
TEST(ThreadScheduling, ActualReaderIsThreadLocalAndUnsupportedDoesNotReportZero) {
  using namespace runtime::detail;
  auto& reader=current_thread_scheduling();
  EXPECT_EQ(&reader,&current_thread_scheduling());
  const auto before=reader.sample(),after=reader.sample();const auto delta=scheduling_delta(before,after);
#if defined(__linux__)
  EXPECT_GT(before.tid,0);EXPECT_EQ(before.tid,after.tid);
  const auto other=std::async(std::launch::async,[]{return current_thread_scheduling().sample();}).get();
  EXPECT_NE(before.tid,other.tid);
  if(delta.schedstat_available){EXPECT_GE(delta.running_ns,0);EXPECT_GE(delta.runqueue_wait_ns,0);}
  else {EXPECT_EQ(delta.running_ns,-1);EXPECT_EQ(delta.runqueue_wait_ns,-1);}
#else
  EXPECT_FALSE(delta.schedstat_available);EXPECT_FALSE(delta.rusage_available);
  EXPECT_EQ(delta.status,SchedstatStatus::Unsupported);
  EXPECT_EQ(delta.running_ns,-1);EXPECT_EQ(delta.runqueue_wait_ns,-1);
  EXPECT_EQ(delta.voluntary_switches,-1);EXPECT_EQ(delta.involuntary_switches,-1);
#endif
}
struct FakeComputeThreadCalls {
  int64_t tid{42};int actual_policy{0},actual_nice{0};
  int nice_error{0},name_error{0},set_error{0},set_calls{0};
  bool wrong_policy_after{false},wrong_nice_after{false},wrong_tid_after{false};
  int affinity_error{0},set_affinity_error{0},affinity_reads{0},set_affinity_calls{0};
  bool wrong_affinity_after{false};
  std::vector<int64_t> actual_cpu_set{0,1,2,3,6,7};
  std::string name;std::vector<int64_t> targets;
  int64_t self_tid(){return wrong_tid_after && set_calls ? tid+1 : tid;}
  int policy(int64_t target){targets.push_back(target);return wrong_policy_after && set_calls ? 1 : actual_policy;}
  int nice(int64_t target,int& error){targets.push_back(target);error=nice_error;return actual_nice;}
  std::vector<int64_t> affinity(int64_t target,int& error){
    targets.push_back(target);++affinity_reads;error=affinity_error;return actual_cpu_set;
  }
  int set_affinity(int64_t target,const std::vector<int64_t>& cpus){
    targets.push_back(target);++set_affinity_calls;
    if(!set_affinity_error && !wrong_affinity_after)actual_cpu_set=cpus;
    return set_affinity_error;
  }
  int name_self(const char* value){name=value;return name_error;}
  int set_nice(int64_t target,int value){targets.push_back(target);++set_calls;
    if(!set_error && !wrong_nice_after)actual_nice=value;
    return set_error;}
};
TEST(ComputeThreadControl, OnlyCurrentTidIsChangedAndObservedValuesAreReturned) {
  using namespace runtime::detail;
  for(const auto* name:{"shn_prep_belly","shn_prep_chest","shn_body_belly","shn_body_chest","shn_geom_belly","shn_geom_chest"})
  for(const auto requested:{0,-10}){
    FakeComputeThreadCalls calls;
    const auto state=configure_self_compute_thread(calls,requested,name);
    EXPECT_TRUE(state.available);EXPECT_EQ(state.tid,42);EXPECT_EQ(state.policy,0);EXPECT_EQ(state.nice,requested);
    EXPECT_EQ(calls.set_calls,1);EXPECT_EQ(calls.name,name);
    EXPECT_FALSE(state.affinity_available);EXPECT_TRUE(state.cpu_set.empty());
    EXPECT_EQ(calls.affinity_reads,0);EXPECT_EQ(calls.set_affinity_calls,0);
    for(const auto target:calls.targets)EXPECT_EQ(target,42);
    // Later frames observe scheduling drift while retaining thread ownership.
    calls.actual_nice=5;
    const auto actual=read_self_compute_thread(calls,false,false);
    EXPECT_NO_THROW(require_compute_thread_identity(actual,state.tid));
    EXPECT_EQ(actual.nice,5);
  }
}
TEST(ComputeThreadControl, PreparationAndBodyRolesCannotReuseAnotherOwnersState) {
  using namespace runtime::detail;
  std::array<FakeComputeThreadCalls,4> calls;
  std::array<ComputeThreadState,4> states;
  const char* names[]{"shn_prep_belly","shn_prep_chest","shn_body_belly","shn_body_chest"};
  for(size_t i=0;i<calls.size();++i){
    calls[i].tid=100+static_cast<int64_t>(i);
    states[i]=configure_self_compute_thread(calls[i],-10,names[i]);
    for(const auto target:calls[i].targets)EXPECT_EQ(target,calls[i].tid);
  }
  // Scheduling drift is observable; another owner cannot reuse this state.
  calls[0].actual_nice=0;
  EXPECT_NO_THROW(require_compute_thread_identity(read_self_compute_thread(calls[0]),states[0].tid));
  for(size_t i=1;i<calls.size();++i){
    const auto actual=read_self_compute_thread(calls[i]);
    EXPECT_NO_THROW(require_compute_thread_identity(actual,states[i].tid));
    EXPECT_THROW(require_compute_thread_identity(actual,states[0].tid),std::runtime_error);
    EXPECT_EQ(calls[i].set_calls,1); // Per-frame verification never renices.
  }
  auto unavailable=states[0];unavailable.available=false;
  EXPECT_THROW(require_compute_thread_identity(unavailable,states[0].tid),std::runtime_error);
}
TEST(ComputeThreadControl, SixOwnersShareExactAllowedPoolWithoutTouchingOtherRoles) {
  using namespace runtime::detail;
  const std::vector<int64_t> cpus{8,9,10,11};
  std::array<FakeComputeThreadCalls,6> calls;
  std::array<ComputeThreadState,6> states;
  const char* names[]{"shn_prep_belly","shn_prep_chest","shn_body_belly","shn_body_chest","shn_geom_belly","shn_geom_chest"};
  for(size_t i=0;i<calls.size();++i){
    calls[i].tid=200+static_cast<int64_t>(i);
    states[i]=configure_self_compute_thread(calls[i],-10,names[i],cpus);
    EXPECT_TRUE(states[i].affinity_available);EXPECT_EQ(states[i].cpu_set,cpus);
    EXPECT_EQ(calls[i].set_affinity_calls,1);EXPECT_EQ(calls[i].affinity_reads,1);
    for(const auto target:calls[i].targets)EXPECT_EQ(target,calls[i].tid);
  }
  // Later affinity changes are observations, while TID remains authoritative.
  for(const auto& wrong:std::vector<std::vector<int64_t>>{{8},{0,10},{}}){
    calls[0].actual_cpu_set=wrong;
    EXPECT_NO_THROW(require_compute_thread_identity(read_self_compute_thread(calls[0],true,false),states[0].tid));
  }
  for(size_t i=1;i<calls.size();++i){
    const auto actual=read_self_compute_thread(calls[i],true);
    EXPECT_NO_THROW(require_compute_thread_identity(actual,states[i].tid));
    EXPECT_THROW(require_compute_thread_identity(actual,states[0].tid),std::runtime_error);
    EXPECT_EQ(calls[i].set_affinity_calls,1); // Later frames only read, never repair drift.
  }
  auto unavailable=states[1];unavailable.affinity_available=false;
  EXPECT_NO_THROW(require_compute_thread_identity(unavailable,states[1].tid));
}
TEST(ComputeThreadControl, CpuSetValidationAndSetOrReadFailuresCannotAuthorizeWork) {
  using namespace runtime::detail;
  const std::vector<int64_t> cpus{8,9,10,11};
  const std::vector<std::vector<int64_t>> invalid{{8,9,10},{8,9,10,11,11},
    {11,10,9,8},{4,5,8,9},{-1,8,9,10},{0,1,2,3,6,7},{std::numeric_limits<int64_t>::max()}};
  for(const auto& value:invalid){
    FakeComputeThreadCalls calls;
    EXPECT_THROW(configure_self_compute_thread(calls,-10,"shn_prep_belly",value),std::invalid_argument);
    EXPECT_TRUE(calls.targets.empty());EXPECT_EQ(calls.set_affinity_calls,0);EXPECT_EQ(calls.set_calls,0);
  }
  for(const auto* role:{static_cast<const char*>(nullptr),"","shn_body_test","shn_prep_head"})
  for(const auto& pool:std::vector<std::vector<int64_t>>{{},cpus}){
    FakeComputeThreadCalls calls;
    EXPECT_THROW(configure_self_compute_thread(calls,-10,role,pool),std::invalid_argument);
    EXPECT_TRUE(calls.targets.empty());EXPECT_EQ(calls.set_affinity_calls,0);EXPECT_EQ(calls.set_calls,0);
  }
  for(int failure=0;failure<4;++failure){
    FakeComputeThreadCalls calls;
    if(failure==0)calls.set_affinity_error=EPERM;
    if(failure==1)calls.affinity_error=EACCES;
    if(failure==2){calls.wrong_affinity_after=true;calls.actual_cpu_set={8};} // Incomplete pool cannot pass exact readback.
    if(failure==3){calls.wrong_affinity_after=true;calls.actual_cpu_set.clear();}
    EXPECT_THROW(configure_self_compute_thread(calls,-10,"shn_body_chest",cpus),std::runtime_error);
    EXPECT_EQ(calls.set_affinity_calls,1);
  }
  FakeComputeThreadCalls drift;
  const auto state=configure_self_compute_thread(drift,0,"shn_body_belly",cpus);
  drift.affinity_error=EIO;
  drift.actual_policy=-1;drift.nice_error=EIO;
  const auto observed=read_self_compute_thread(drift,true,false);
  EXPECT_FALSE(observed.affinity_available);EXPECT_TRUE(observed.cpu_set.empty());
  EXPECT_EQ(observed.policy,-1);EXPECT_EQ(observed.nice,-100);
  EXPECT_NO_THROW(require_compute_thread_identity(observed,state.tid));
  EXPECT_EQ(drift.set_affinity_calls,1);EXPECT_EQ(state.nice,0); // Does not imply a nice change.
}
TEST(ComputeThreadControl, NativeSelfAffinityMovesOnlyEachOwnerIntoAllowedComputePool) {
#if defined(__linux__)
  using namespace runtime::detail;
  const std::vector<int64_t> background{0,1,2,3,6,7},cpus{8,9,10,11};
  ComputeThreadSyscalls main_calls;
  ComputeThreadState before;
  try{before=read_self_compute_thread(main_calls,true);}
  catch(const std::exception& error){GTEST_SKIP()<<"native scheduling read unavailable: "<<error.what();}
  if(before.policy!=0 || before.nice!=0)
    GTEST_SKIP()<<"requires inherited SCHED_OTHER/nice0; never changes the main test thread";
  for(const auto& pool:{background,cpus})for(const auto cpu:pool)
    if(std::find(before.cpu_set.begin(),before.cpu_set.end(),cpu)==before.cpu_set.end())
      GTEST_SKIP()<<"current allowed mask must cover both background and compute pools";
  const char* names[]{"shn_body_belly","shn_body_chest","shn_prep_belly","shn_prep_chest","shn_geom_belly","shn_geom_chest"};
  for(size_t role=0;role<6;++role){
    const auto& expected=cpus;
    std::promise<ComputeThreadState> completed;auto result=completed.get_future();
    std::thread worker([&]{
      try{
        ComputeThreadSyscalls calls;
        const auto inherited=read_self_compute_thread(calls,true);
        if(calls.set_affinity(inherited.tid,background)!=0)
          throw std::runtime_error("test_background_affinity_set_failed");
        require_compute_thread_state(read_self_compute_thread(calls,true),inherited.tid,0,background);
        const auto configured=configure_self_compute_thread(calls,0,names[role],cpus);
        const auto actual=read_self_compute_thread(calls,true);
        require_compute_thread_state(actual,configured.tid,0,expected);
        completed.set_value(actual);
      }catch(...){completed.set_exception(std::current_exception());}
    });
    worker.join();
    // Check the untouched main thread even when a worker syscall failed.
    const auto after=read_self_compute_thread(main_calls,true);
    EXPECT_EQ(after.tid,before.tid);EXPECT_EQ(after.cpu_set,before.cpu_set);
    EXPECT_EQ(after.policy,before.policy);EXPECT_EQ(after.nice,before.nice);
    const auto actual=result.get();
    EXPECT_NE(actual.tid,before.tid);EXPECT_TRUE(actual.affinity_available);EXPECT_EQ(actual.cpu_set,expected);
  }
#else
  GTEST_SKIP()<<"native Linux self-affinity test is unavailable on this platform";
#endif
}
TEST(ComputeThreadControl, InvalidRequestsAndSyscallFailuresNeverAuthorizeWork) {
  using namespace runtime::detail;
  const int64_t invalid_values[]{-11,-9,1,19,std::numeric_limits<int64_t>::max()};
  for(const auto invalid:invalid_values)
    EXPECT_THROW(validate_compute_thread_nice(invalid),std::invalid_argument);
  FakeComputeThreadCalls invalid;
  EXPECT_THROW(configure_self_compute_thread(invalid,-9,"shn_body_belly"),std::invalid_argument);
  EXPECT_TRUE(invalid.targets.empty());EXPECT_EQ(invalid.set_calls,0);
  for(const auto* name:{"shn_prep_belly","shn_prep_chest","shn_body_belly","shn_body_chest","shn_geom_belly","shn_geom_chest"})
  for(int failure=0;failure<8;++failure){
    FakeComputeThreadCalls calls;
    if(failure==0)calls.tid=-1;
    if(failure==1)calls.actual_policy=-1;
    if(failure==2)calls.actual_policy=1; // SCHED_FIFO is rejected, never changed.
    if(failure==3)calls.nice_error=EACCES;
    if(failure==4)calls.name_error=EINVAL;
    if(failure==5)calls.set_error=EPERM;
    if(failure==6)calls.wrong_policy_after=true;
    if(failure==7)calls.wrong_nice_after=true;
    EXPECT_THROW(configure_self_compute_thread(calls,-10,name),std::runtime_error);
    EXPECT_LE(calls.set_calls,1);
    if(failure<5)EXPECT_EQ(calls.set_calls,0);
  }
  FakeComputeThreadCalls changed;changed.wrong_tid_after=true;
  EXPECT_THROW(configure_self_compute_thread(changed,-10,"shn_body_belly"),std::runtime_error);
}
}  // namespace

TEST(ComputeThreadControl, GeometryHelpersRetainDistinctOwnersWithinSharedPool) {
  using namespace runtime::detail;
  const std::vector<int64_t> cpus{8,9,10,11};
  for(const auto* name:{"shn_geom_belly","shn_geom_chest"}) {
    FakeComputeThreadCalls calls;const auto& expected=cpus;
    const auto startup=configure_self_compute_thread(calls,-10,name,cpus);
    EXPECT_EQ(startup.cpu_set,expected);EXPECT_EQ(calls.set_affinity_calls,1);EXPECT_EQ(calls.set_calls,1);
    calls.actual_cpu_set={0};calls.actual_nice=5;
    const auto actual=read_self_compute_thread(calls,true,false);
    EXPECT_NO_THROW(require_compute_thread_identity(actual,startup.tid));
    EXPECT_EQ(actual.cpu_set,std::vector<int64_t>({0}));EXPECT_EQ(actual.nice,5);
    EXPECT_EQ(calls.set_affinity_calls,1);EXPECT_EQ(calls.set_calls,1);
    ++calls.tid;EXPECT_THROW(require_compute_thread_identity(read_self_compute_thread(calls,true,false),startup.tid),std::runtime_error);
  }
}
