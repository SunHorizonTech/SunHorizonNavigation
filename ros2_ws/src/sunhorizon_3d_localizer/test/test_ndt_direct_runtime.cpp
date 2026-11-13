#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <memory>
#include <cmath>
#include <type_traits>
#include <mutex>
#include <string>
#include <unistd.h>

#include "sunhorizon_3d_localizer/ndt_direct_runtime.hpp"
#include "sunhorizon_3d_localizer/detail/accepted_ndt_snapshot.hpp"

namespace {
using namespace sunhorizon_3d_localizer;

class NdtDirectRuntimeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    directory = std::filesystem::temp_directory_path() /
        ("sunhorizon-ndt-direct-test-"+std::to_string(getpid()));
    std::filesystem::create_directories(directory);
    std::ofstream map(directory/"map.pcd");
    map << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
           "COUNT 1 1 1\nWIDTH 32\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
           "POINTS 32\nDATA ascii\n";
    for (int i = 0; i < 32; ++i)
      map << 1.0+i*0.1 << ' ' << (i%4)*0.2 << ' ' << (i%3)*0.1 << '\n';
  }
  void TearDown() override {
    context->shutdown("NDT direct runtime test complete");
    std::filesystem::remove_all(directory);
  }
  rclcpp::NodeOptions options() {
    rclcpp::NodeOptions result;
    result.context(context);
    result.parameter_overrides({rclcpp::Parameter("map_path", (directory/"map.pcd").string())});
    return result;
  }
  NdtDirectSinks sinks() {
    NdtDirectSinks result;
    result.publish_ros_candidates = false;
    result.status = [this](auto status) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        latest_status = std::move(status);
      }
      condition.notify_one();
    };
    return result;
  }
  bool wait_for_reason(const std::string& reason) {
    std::unique_lock<std::mutex> lock(mutex);
    return condition.wait_for(lock, std::chrono::seconds(2), [&]() {
      return latest_status && latest_status->reason == reason;
    });
  }
  std::shared_ptr<rclcpp::Context> context;
  std::filesystem::path directory;
  std::mutex mutex;
  std::condition_variable condition;
  sunhorizon_navigation_interfaces::msg::LocalizationStatus::ConstSharedPtr latest_status;
};

TEST(NdtAcceptedSnapshot, FullRegistrationKeepsMapDatumAndOriginalTimeBesidePlanarNavigationOutputs) {
  Eigen::Matrix4f registration=Eigen::Matrix4f::Identity();
  const Eigen::Matrix3f rotation=(Eigen::AngleAxisf(.4f,Eigen::Vector3f::UnitZ())*
    Eigen::AngleAxisf(-.09f,Eigen::Vector3f::UnitY())*
    Eigen::AngleAxisf(.07f,Eigen::Vector3f::UnitX())).toRotationMatrix();
  registration.block<3,3>(0,0)=rotation;registration.block<3,1>(0,3)=Eigen::Vector3f(1.2f,-.6f,.82f);
  std_msgs::msg::Header source;source.frame_id="airy_lidar";source.stamp.sec=10;source.stamp.nanosec=123;
  auto pose=std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  pose->header=source;pose->header.frame_id="map";
  pose->pose.pose.position.x=1.3;pose->pose.pose.position.y=-.7;
  pose->pose.pose.orientation.z=std::sin(.4/2.);pose->pose.pose.orientation.w=std::cos(.4/2.);
  auto odom=std::make_shared<nav_msgs::msg::Odometry>();
  odom->header=source;odom->header.frame_id="odom";odom->child_frame_id="base_footprint";
  odom->pose.pose.position.x=.2;odom->pose.pose.position.y=.3;
  odom->pose.pose.orientation.z=std::sin(.2/2.);odom->pose.pose.orientation.w=std::cos(.2/2.);
  const auto original_pose=*pose;const auto original_odom=*odom;
  NdtDirectResult accepted;
  accepted.input={10000000123LL,10000000300LL,987654321,42,"accepted-airy-owner"};
  accepted.prior_input=NdtDirectStamp{10000000023LL,10000000200LL,987654221,41,"accepted-prior-owner"};
  accepted.result_receipt_ros_ns=10000000400LL;accepted.result_receipt_steady_ns=987654421;
  accepted.map_pose=pose;accepted.odometry=odom;accepted.independent_tracking_evidence=true;
  const auto original=accepted;
  std::string map_path="/data/navigation/maps/synthetic-registration/map.pcd";
  const auto original_map_path=map_path;
  const auto snapshot=detail::make_accepted_ndt_snapshot(std::move(accepted),registration,source,"map",map_path);
  static_assert(std::is_const<typename decltype(snapshot)::element_type>::value);
  static_assert(std::is_const<typename decltype(snapshot->map_to_source)::element_type>::value);
  ASSERT_TRUE(snapshot->map_to_source);
  const auto& full=*snapshot->map_to_source;
  EXPECT_EQ(full.header.stamp,source.stamp);EXPECT_EQ(full.header.frame_id,"map");
  EXPECT_EQ(full.child_frame_id,"airy_lidar");
  EXPECT_FLOAT_EQ(full.transform.translation.x,registration(0,3));
  EXPECT_FLOAT_EQ(full.transform.translation.y,registration(1,3));
  EXPECT_FLOAT_EQ(full.transform.translation.z,registration(2,3));
  const auto q=full.transform.rotation;
  const auto recovered=Eigen::Quaterniond(q.w,q.x,q.y,q.z).toRotationMatrix();
  EXPECT_LT((recovered-rotation.cast<double>()).cwiseAbs().maxCoeff(),1.e-6);
  EXPECT_NE(q.x,0.);EXPECT_NE(q.y,0.);
  EXPECT_EQ(snapshot->map_pose,pose);EXPECT_EQ(snapshot->odometry,odom);
  EXPECT_EQ(*snapshot->map_pose,original_pose);EXPECT_EQ(*snapshot->odometry,original_odom);
  EXPECT_EQ(snapshot->map_pose->pose.pose.position.z,0.);EXPECT_EQ(snapshot->odometry->pose.pose.position.z,0.);
  EXPECT_EQ(snapshot->map_pose->pose.pose.orientation.x,0.);EXPECT_EQ(snapshot->map_pose->pose.pose.orientation.y,0.);
  EXPECT_EQ(snapshot->odometry->pose.pose.orientation.x,0.);EXPECT_EQ(snapshot->odometry->pose.pose.orientation.y,0.);
  const auto unchanged_stamp=[](const NdtDirectStamp& a,const NdtDirectStamp& b){
    EXPECT_EQ(a.source_stamp_ns,b.source_stamp_ns);EXPECT_EQ(a.receipt_ros_ns,b.receipt_ros_ns);
    EXPECT_EQ(a.receipt_steady_ns,b.receipt_steady_ns);EXPECT_EQ(a.sequence,b.sequence);EXPECT_EQ(a.source_identity,b.source_identity);
  };
  unchanged_stamp(snapshot->input,original.input);ASSERT_TRUE(snapshot->prior_input);
  unchanged_stamp(*snapshot->prior_input,*original.prior_input);
  EXPECT_EQ(snapshot->result_receipt_ros_ns,original.result_receipt_ros_ns);
  EXPECT_EQ(snapshot->result_receipt_steady_ns,original.result_receipt_steady_ns);
  EXPECT_TRUE(snapshot->independent_tracking_evidence);
  EXPECT_EQ(snapshot->registration_map_path,original_map_path);
  map_path.clear();
  registration(2,3)=0.;registration.block<3,3>(0,0)=Eigen::Matrix3f::Identity();source.stamp.nanosec=999;
  EXPECT_NE(snapshot->map_to_source->transform.translation.z,0.);
  EXPECT_EQ(snapshot->map_to_source->transform.rotation.x,q.x);
  EXPECT_EQ(snapshot->map_to_source->header.stamp.nanosec,123u);
  EXPECT_EQ(snapshot->registration_map_path,original_map_path);
}

TEST_F(NdtDirectRuntimeTest, DirectInputStillRequiresExistingInitializationGateAndSourceStamp) {
  const auto runtime = make_ndt_direct_runtime(options(), sinks());
  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud->header.stamp.sec = 1;
  cloud->header.frame_id = "airy_lidar";
  const NdtDirectStamp stamp{1000000000LL, 1000000100LL, 100, 1, "owned-airy-receipt"};
  ASSERT_TRUE(runtime->submit_points(cloud, stamp));
  ASSERT_TRUE(wait_for_reason("initial_pose_source_to_base_or_prior_basis_not_accepted"));
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_FALSE(latest_status->measurement_accepted);
    EXPECT_EQ(latest_status->header.stamp.sec, 1);
    EXPECT_EQ(latest_status->header.stamp.nanosec, 0u);
  }
  EXPECT_EQ(runtime->counters().clouds_submitted, 1u);
  EXPECT_EQ(cloud->header.stamp.sec, 1);
  // This owner has no DDS cloud/prior subscriptions to duplicate the direct
  // input. Inspect only endpoints belonging to this node, not other processes.
  for (const auto& endpoint : runtime->node()->get_subscriptions_info_by_topic("/sunhorizon/airy/points"))
    EXPECT_NE(endpoint.node_name(), runtime->node()->get_name());
}

TEST_F(NdtDirectRuntimeTest, RegistrationMapIdentityRemainsBoundToLoadedMap) {
  const auto runtime=make_ndt_direct_runtime(options(),sinks());
  const auto loaded=runtime->registration_map_sha256();
  ASSERT_EQ(loaded.size(),64u);
  // A later pathname edit must not relabel this owner's already loaded model.
  {std::ofstream changed(directory/"map.pcd",std::ios::app);changed<<"\n# later pathname version\n";}
  EXPECT_EQ(runtime->registration_map_sha256(),loaded);
  runtime->stop();
  const auto next=make_ndt_direct_runtime(options(),sinks());
  EXPECT_NE(next->registration_map_sha256(),loaded);
}

TEST_F(NdtDirectRuntimeTest, MismatchedMetadataIsRejectedAndStopClosesMailbox) {
  const auto runtime = make_ndt_direct_runtime(options(), sinks());
  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud->header.stamp.sec = 1;
  const NdtDirectStamp mismatched{1000000001LL, 1000000100LL, 100, 1, "owned-airy-receipt"};
  EXPECT_FALSE(runtime->submit_points(cloud, mismatched));
  ASSERT_TRUE(wait_for_reason("ndt_direct_cloud_stamp_or_identity_invalid"));
  EXPECT_EQ(runtime->counters().clouds_submitted, 0u);
  runtime->stop();
  EXPECT_FALSE(runtime->submit_points(cloud,
      {1000000000LL, 1000000100LL, 100, 1, "owned-airy-receipt"}));
}

TEST_F(NdtDirectRuntimeTest, DirectRuntimeCannotReplaceSourceTimeWithReplayReceiptTime) {
  auto config = options();
  auto parameters = config.parameter_overrides();
  parameters.emplace_back("use_receipt_time_for_output", true);
  config.parameter_overrides(parameters);
  EXPECT_THROW(make_ndt_direct_runtime(config, sinks()), std::runtime_error);
}
TEST_F(NdtDirectRuntimeTest, WorkerNiceIsReadOnlyAndRejectsInvalidInt64BeforeStartup) {
  for (const int64_t requested : {int64_t{-10},int64_t{1},int64_t{4294967291}}) {
    auto config=options(); auto parameters=config.parameter_overrides();
    parameters.emplace_back("direct_worker_nice",requested); config.parameter_overrides(parameters);
    EXPECT_THROW(make_ndt_direct_runtime(config,sinks()),std::invalid_argument);
  }
  const auto runtime=make_ndt_direct_runtime(options(),sinks());
  EXPECT_EQ(runtime->node()->get_parameter("direct_worker_nice").as_int(),0);
  EXPECT_TRUE(runtime->node()->describe_parameter("direct_worker_nice").read_only);
  const auto changed=runtime->node()->set_parameters({rclcpp::Parameter("direct_worker_nice",int64_t{-5})});
  ASSERT_EQ(changed.size(),1u); EXPECT_FALSE(changed.front().successful);
}
TEST_F(NdtDirectRuntimeTest, TeamPlacementIsReadOnlyAndRejectsCpuNarrowing) {
  for (const int64_t cpu : {int64_t{-1}, int64_t{4294967296}}) {
    auto config = options();
    auto parameters = config.parameter_overrides();
    parameters.emplace_back("ndt_team_cpu_affinity", std::vector<int64_t>{cpu});
    config.parameter_overrides(parameters);
    EXPECT_THROW(make_ndt_direct_runtime(config, sinks()), std::invalid_argument);
  }
  const auto runtime = make_ndt_direct_runtime(options(), sinks());
  EXPECT_TRUE(runtime->node()->get_parameter("ndt_team_cpu_affinity").as_integer_array().empty());
  EXPECT_TRUE(runtime->node()->describe_parameter("ndt_team_cpu_affinity").read_only);
  const auto changed = runtime->node()->set_parameters({
      rclcpp::Parameter("ndt_team_cpu_affinity", std::vector<int64_t>{0})});
  ASSERT_EQ(changed.size(), 1u);
  EXPECT_FALSE(changed.front().successful);
}

TEST_F(NdtDirectRuntimeTest, StatusCompatibilityUsesOriginalObserverWithoutCandidateEgress) {
  EXPECT_FALSE(NdtDirectSinks{}.publish_ros_status_compatibility);
  auto observer=std::make_shared<rclcpp::Node>("ndt_status_compatibility_test",options());
  sunhorizon_navigation_interfaces::msg::LocalizationStatus::ConstSharedPtr received;
  auto subscription=observer->create_subscription<sunhorizon_navigation_interfaces::msg::LocalizationStatus>(
    "/sunhorizon/localization/lidar/status",rclcpp::QoS(10).reliable().transient_local(),
    [&](sunhorizon_navigation_interfaces::msg::LocalizationStatus::ConstSharedPtr value){received=std::move(value);});
  auto output=sinks();output.publish_ros_status_compatibility=true;
  ASSERT_FALSE(output.publish_ros_candidates);
  const auto runtime=make_ndt_direct_runtime(options(),std::move(output));
  auto cloud=std::make_shared<sensor_msgs::msg::PointCloud2>();cloud->header.stamp.sec=1;
  cloud->header.frame_id="airy_lidar";
  ASSERT_TRUE(runtime->submit_points(cloud,{1000000000LL,1000000100LL,100,1,"owned-airy-receipt"}));
  rclcpp::ExecutorOptions executor_options;executor_options.context=context;
  rclcpp::executors::SingleThreadedExecutor executor(executor_options);executor.add_node(observer);
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(!received && std::chrono::steady_clock::now()<deadline)executor.spin_once(std::chrono::milliseconds(20));
  ASSERT_TRUE(received);EXPECT_FALSE(received->measurement_accepted);
  EXPECT_EQ(received->reason,"initial_pose_source_to_base_or_prior_basis_not_accepted");
  EXPECT_EQ(received->header.stamp.sec,1); // Observer preserves original source, never now().
  runtime->stop();executor.remove_node(observer);
}
}  // namespace

TEST_F(NdtDirectRuntimeTest, ComputeBackendDefaultsToCpuAndCannotChangeAfterStartup) {
  auto runtime = make_ndt_direct_runtime(options(), sinks());
  EXPECT_EQ(runtime->node()->get_parameter("ndt_compute_backend").as_string(), "cpu");
  EXPECT_TRUE(runtime->node()->describe_parameter("ndt_compute_backend").read_only);
  const auto changed = runtime->node()->set_parameters({
      rclcpp::Parameter("ndt_compute_backend", "cuda_direct7")});
  ASSERT_EQ(changed.size(), 1U);
  EXPECT_FALSE(changed.front().successful);
  runtime->stop();
}
