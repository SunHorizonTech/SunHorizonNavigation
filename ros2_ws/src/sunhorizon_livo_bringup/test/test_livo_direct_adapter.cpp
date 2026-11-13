#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <memory>
#include <stdexcept>
#include <rcl/time.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "sunhorizon_livo_bringup/livo_direct_adapter.hpp"

namespace {
using namespace sunhorizon_livo_bringup;

class LivoDirectAdapterTest : public ::testing::Test {
 protected:
  void SetUp() override {
    context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
  }
  void TearDown() override {context->shutdown("LIVO direct adapter test complete");}
  rclcpp::NodeOptions options() {
    rclcpp::NodeOptions result;
    result.context(context);
    result.parameter_overrides({
      rclcpp::Parameter("vendor_ros2_commit", "a09599a288c56b832114d7ad9e898248a4111d9e"),
      rclcpp::Parameter("official_airy_reference_commit", "0d2c0346107b75b59934975adec9a6eeeb913c64"),
      rclcpp::Parameter("vendor_child_to_base_transform_accepted", true)});
    return result;
  }
  LivoDirectSinks sinks() {
    LivoDirectSinks result;
    result.publish_ros_candidates = false;
    result.vendor_points = [this](auto owner, const auto& stamp) {
      points = std::move(owner); latest = stamp;
    };
    result.vendor_imu = [this](auto owner, const auto& stamp) {
      imu = std::move(owner); latest = stamp;
    };
    result.vendor_image = [this](auto owner, const auto& stamp) {
      image = std::move(owner); latest = stamp;
    };
    result.candidate = [this](auto, const auto&) {++candidates;};
    return result;
  }
  LivoDirectStamp stamp(const std::shared_ptr<LivoDirectAdapter>& adapter) {
    const int64_t now_ros = adapter->node()->now().nanoseconds();
    const int64_t now_steady = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    return {now_ros, now_ros, now_steady, 1, "owned-source-receipt"};
  }
  void pending_health_case(bool propagated, bool check_watchdog_anchor = false,
      bool visual_required = true, bool receive_images = true, bool receive_info = true,
      int invalid_required_sensor = -1) {
    auto configuration=options();
    auto parameters=configuration.parameter_overrides();
    parameters.emplace_back("use_sim_time",true);
    parameters.emplace_back("body_velocity_feedback_enabled",true);
    parameters.emplace_back("use_propagated_odometry",propagated);
    parameters.emplace_back("status_rate_hz",100.);
    parameters.emplace_back("maximum_vendor_propagation_horizon_sec",.10);
    configuration.parameter_overrides(parameters);
    auto output=sinks();
    output.vendor_images_enabled=visual_required;
    if(!visual_required)output.vendor_image={}; // No unused estimator RGB sink.
    BodyVelocityGeometryStatus corrected_geometry=BodyVelocityGeometryStatus::Ready;
    BodyVelocityGeometryStatus propagated_geometry=BodyVelocityGeometryStatus::Ready;
    BodyVelocityGeometryStatus requested_geometry=BodyVelocityGeometryStatus::Ready;
    output.body_transform_tf_mode="attended_measured_direct";
    output.source_time_vendor_child_from_base=[&](int64_t,const auto& nominal){
      return SourceTimeBodyTransform{requested_geometry,
        requested_geometry==BodyVelocityGeometryStatus::Ready ?
        std::optional<geometry_msgs::msg::Transform>{nominal}:std::nullopt};
    };
    using Status=sunhorizon_navigation_interfaces::msg::LocalizationStatus;
    Status::ConstSharedPtr latest_status;
    output.status=[&](auto value){latest_status=std::move(value);};
    const auto adapter=make_livo_direct_adapter(configuration,std::move(output));
    auto clock=adapter->node()->get_clock();
    ASSERT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()),RCL_RET_OK);
    rclcpp::ExecutorOptions executor_options;executor_options.context=context;
    rclcpp::executors::SingleThreadedExecutor executor(executor_options);
    executor.add_node(adapter->node());
    uint64_t sequence=0;
    int invalid_sensor=-1;
    const auto feed=[&](int step){
      const int64_t source=10000000000LL+int64_t(step)*10000000LL;
      ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(),source),RCL_RET_OK);
      auto metadata=stamp(adapter);metadata.sequence=++sequence;
      auto cloud=std::make_shared<sensor_msgs::msg::PointCloud2>();
      cloud->header.stamp=rclcpp::Time(source);cloud->header.frame_id="airy_lidar";
      using Field=sensor_msgs::msg::PointField;
      sensor_msgs::PointCloud2Modifier modifier(*cloud);
      modifier.setPointCloud2Fields(6,"x",1,Field::FLOAT32,"y",1,Field::FLOAT32,
        "z",1,Field::FLOAT32,"intensity",1,Field::UINT8,"timestamp",1,Field::FLOAT32,"ring",1,Field::UINT16);
      modifier.resize(1);
      *sensor_msgs::PointCloud2Iterator<float>(*cloud,"x")=1.f;
      *sensor_msgs::PointCloud2Iterator<float>(*cloud,"timestamp")=.01f;
      if(invalid_sensor==0)cloud->fields.clear();
      adapter->receive_points(cloud,metadata);
      auto imu=std::make_shared<sensor_msgs::msg::Imu>();
      imu->header=cloud->header;imu->header.frame_id="imu_link";imu->linear_acceleration.z=9.81;
      if(invalid_sensor==1)imu->linear_acceleration.z=std::numeric_limits<double>::quiet_NaN();
      adapter->receive_imu(imu,metadata);
      auto image=std::make_shared<sensor_msgs::msg::Image>();
      image->header=cloud->header;image->header.frame_id="camera_chest_color_optical_frame";
      image->width=image->height=1;image->encoding="rgb8";image->step=3;image->data={1,2,3};
      if(receive_images)adapter->receive_image(image,metadata);
      auto info=std::make_shared<sensor_msgs::msg::CameraInfo>();
      info->header=image->header;info->width=info->height=1;info->k[0]=info->k[4]=1.;
      if(receive_info)adapter->receive_camera_info(info,metadata);
      auto pose=std::make_shared<nav_msgs::msg::Odometry>();
      pose->header=cloud->header;pose->header.frame_id="camera_init";pose->child_frame_id="aft_mapped";
      pose->pose.pose.orientation.w=1.;
      if(invalid_sensor==2)pose->pose.pose.orientation.w=0.;
      requested_geometry=corrected_geometry;
      adapter->receive_corrected_odometry(pose,metadata);
      if(propagated){requested_geometry=propagated_geometry;adapter->receive_propagated_odometry(pose,metadata);}
    };
    const auto status=[&](){
      latest_status.reset();
      const auto until=std::chrono::steady_clock::now()+std::chrono::milliseconds(200);
      while(!latest_status && std::chrono::steady_clock::now()<until)
        executor.spin_once(std::chrono::milliseconds(2));
    };
    for(int i=0;i<=120;++i)feed(i);
    status();ASSERT_TRUE(latest_status);
    if(visual_required && (!receive_images || !receive_info)){
      EXPECT_FALSE(latest_status->measurement_accepted);
      EXPECT_EQ(latest_status->state,Status::STATE_INITIALIZING);
      EXPECT_EQ(candidates,0);
      EXPECT_NE(latest_status->reason.find(receive_images?"camera_info":"image"),std::string::npos);
      executor.remove_node(adapter->node());
      return;
    }
    ASSERT_TRUE(latest_status->measurement_accepted)<<latest_status->reason;
    EXPECT_EQ(latest_status->state,Status::STATE_HEALTHY);
    if(!visual_required)EXPECT_FALSE(image); // Even an offered frame is unused.
    ASSERT_GT(candidates,0);
    const int accepted_candidates=candidates;
    if(!visual_required){
      // The unused branch must not validate metadata, refresh receipt times,
      // poison health, or call an absent vendor sink. Camera/perception health
      // is owned independently by the assembled camera source, not this test.
      EXPECT_NO_THROW(adapter->receive_image(std::make_shared<sensor_msgs::msg::Image>(),{}));
      EXPECT_NO_THROW(adapter->receive_camera_info(std::make_shared<sensor_msgs::msg::CameraInfo>(),{}));
      status();ASSERT_TRUE(latest_status);
      EXPECT_TRUE(latest_status->measurement_accepted)<<latest_status->reason;
      EXPECT_EQ(latest_status->state,Status::STATE_HEALTHY);
      EXPECT_EQ(candidates,accepted_candidates);
      EXPECT_DOUBLE_EQ(latest_status->measurement_age_sec,0.);
    }
    if(invalid_required_sensor>=0){
      invalid_sensor=invalid_required_sensor;
      feed(125);status();ASSERT_TRUE(latest_status);
      EXPECT_FALSE(latest_status->measurement_accepted);
      EXPECT_EQ(candidates,accepted_candidates);
      executor.remove_node(adapter->node());
      return;
    }
    corrected_geometry=propagated_geometry=BodyVelocityGeometryStatus::FuturePending;
    feed(125);status();ASSERT_TRUE(latest_status);
    EXPECT_EQ(candidates,accepted_candidates); // A prior lease is not a new candidate.
    EXPECT_TRUE(latest_status->measurement_accepted)<<latest_status->reason;
    EXPECT_NEAR(latest_status->measurement_age_sec,.05,1.e-9); // Original source retained.
    if(check_watchdog_anchor){
      // Qualified propagation cannot use a pending correction as a new anchor.
      propagated_geometry=BodyVelocityGeometryStatus::Ready;
      feed(135);status();ASSERT_TRUE(latest_status);
      EXPECT_EQ(candidates,accepted_candidates);
      EXPECT_FALSE(latest_status->measurement_accepted);
      EXPECT_NE(latest_status->reason.find("propagation_unanchored"),std::string::npos);
    }else{
      // Sensors remain fresh; pending poses must not extend either odometry lease.
      for(int i=135;i<=185;i+=10)feed(i);
      status();ASSERT_TRUE(latest_status);
      EXPECT_EQ(candidates,accepted_candidates);
      EXPECT_FALSE(latest_status->measurement_accepted);
      EXPECT_NE(latest_status->reason.find("stale:vendor_odometry"),std::string::npos);
      EXPECT_NEAR(latest_status->measurement_age_sec,.65,1.e-9);
    }
    executor.remove_node(adapter->node());
  }
  std::shared_ptr<rclcpp::Context> context;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr points;
  sensor_msgs::msg::Imu::ConstSharedPtr imu;
  sensor_msgs::msg::Image::ConstSharedPtr image;
  LivoDirectStamp latest;
  int candidates{0};
};

TEST_F(LivoDirectAdapterTest, SharesImageAndNormalizedImuOwnerAndPreservesOriginalTiming) {
  const auto adapter = make_livo_direct_adapter(options(), sinks());
  const auto metadata = stamp(adapter);
  auto raw_imu = std::make_shared<sensor_msgs::msg::Imu>();
  raw_imu->header.stamp = rclcpp::Time(metadata.source_stamp_ns);
  raw_imu->header.frame_id = "imu_link";
  raw_imu->linear_acceleration.z = 9.81;
  adapter->receive_imu(raw_imu, metadata);
  EXPECT_EQ(imu.get(), raw_imu.get());
  EXPECT_EQ(latest.source_stamp_ns, metadata.source_stamp_ns);
  EXPECT_EQ(latest.receipt_ros_ns, metadata.receipt_ros_ns);
  EXPECT_EQ(latest.receipt_steady_ns, metadata.receipt_steady_ns);
  auto raw_image = std::make_shared<sensor_msgs::msg::Image>();
  raw_image->header.stamp = rclcpp::Time(metadata.source_stamp_ns);
  raw_image->header.frame_id = "camera_chest_color_optical_frame";
  raw_image->encoding = "rgb8";
  raw_image->width = raw_image->height = 1;
  raw_image->step = 3;
  raw_image->data = {1, 2, 3};
  adapter->receive_image(raw_image, metadata);
  EXPECT_EQ(image.get(), raw_image.get());
  EXPECT_EQ(candidates, 0);  // Sensor health alone never invents a pose.
}

TEST_F(LivoDirectAdapterTest, CorrectedPendingKeepsOnlyOriginalHealthLease) {
  pending_health_case(false);
}

TEST_F(LivoDirectAdapterTest, PropagatedPendingKeepsOnlyOriginalHealthLease) {
  pending_health_case(true);
}

TEST_F(LivoDirectAdapterTest, CorrectedPendingCannotRefreshPropagationWatchdogAnchor) {
  pending_health_case(true,true);
}

TEST_F(LivoDirectAdapterTest, LioWithoutImagesOrInfoPublishesHealthyMeasuredCandidate) {
  pending_health_case(true,false,false,false,false);
}

TEST_F(LivoDirectAdapterTest, LioDoesNotConsumeAnUnusedVisualInputOrRequireItsSink) {
  pending_health_case(true,false,false,true,true);
}

TEST_F(LivoDirectAdapterTest, VisualEstimatorCannotPublishHealthyCandidateWithoutImage) {
  pending_health_case(true,false,true,false,true);
}

TEST_F(LivoDirectAdapterTest, VisualEstimatorCannotPublishHealthyCandidateWithoutCameraInfo) {
  pending_health_case(true,false,true,true,false);
}

TEST_F(LivoDirectAdapterTest, LioStillRejectsInvalidLidarImuAndOdometry) {
  for(int channel=0;channel<3;++channel){
    SCOPED_TRACE(channel);
    pending_health_case(true,false,false,false,false,channel);
  }
}

TEST_F(LivoDirectAdapterTest, InvalidCloudStillFailsOriginalConverterAndCannotReachEstimator) {
  const auto adapter = make_livo_direct_adapter(options(), sinks());
  const auto metadata = stamp(adapter);
  auto invalid = std::make_shared<sensor_msgs::msg::PointCloud2>();
  invalid->header.stamp = rclcpp::Time(metadata.source_stamp_ns);
  invalid->header.frame_id = "airy_lidar";
  adapter->receive_points(invalid, metadata);
  EXPECT_FALSE(points);
  EXPECT_EQ(candidates, 0);
}

TEST_F(LivoDirectAdapterTest, MetadataMismatchAndMissingVendorSinkFailClosed) {
  const auto adapter = make_livo_direct_adapter(options(), sinks());
  const auto metadata = stamp(adapter);
  auto invalid = std::make_shared<sensor_msgs::msg::Imu>();
  invalid->header.stamp = rclcpp::Time(metadata.source_stamp_ns-1);
  invalid->header.frame_id = "imu_link";
  EXPECT_THROW(adapter->receive_imu(invalid, metadata), std::invalid_argument);
  EXPECT_FALSE(imu);
  auto missing = sinks();
  missing.vendor_imu = {};
  EXPECT_THROW(make_livo_direct_adapter(options(), missing), std::runtime_error);
}
TEST_F(LivoDirectAdapterTest, StatusCompatibilityRemainsAvailableWithoutCandidateEgress) {
  EXPECT_FALSE(LivoDirectSinks{}.publish_ros_status_compatibility);
  auto observer=std::make_shared<rclcpp::Node>("livo_status_compatibility_test",options());
  sunhorizon_navigation_interfaces::msg::LocalizationStatus::ConstSharedPtr received;
  auto subscription=observer->create_subscription<sunhorizon_navigation_interfaces::msg::LocalizationStatus>(
    "/sunhorizon/localization/livo/odometry_status",rclcpp::QoS(10).reliable().transient_local(),
    [&](sunhorizon_navigation_interfaces::msg::LocalizationStatus::ConstSharedPtr value){received=std::move(value);});
  auto output=sinks();output.publish_ros_status_compatibility=true;
  ASSERT_FALSE(output.publish_ros_candidates);
  const auto adapter=make_livo_direct_adapter(options(),std::move(output));
  rclcpp::ExecutorOptions executor_options;executor_options.context=context;
  rclcpp::executors::SingleThreadedExecutor executor(executor_options);
  executor.add_node(observer);executor.add_node(adapter->node());
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(!received && std::chrono::steady_clock::now()<deadline)executor.spin_once(std::chrono::milliseconds(20));
  ASSERT_TRUE(received);EXPECT_FALSE(received->measurement_accepted);
  EXPECT_EQ(received->backend,"livo_odometry");EXPECT_EQ(candidates,0);
  executor.remove_node(adapter->node());executor.remove_node(observer);
}
}  // namespace
