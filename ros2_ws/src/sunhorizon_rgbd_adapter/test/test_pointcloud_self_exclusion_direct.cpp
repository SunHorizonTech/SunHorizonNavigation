#include <gtest/gtest.h>
#include <chrono>
#include <cstring>
#include "sunhorizon_rgbd_adapter/pointcloud_self_exclusion.hpp"
#include "sunhorizon_rgbd_adapter/pointcloud_self_exclusion_direct.hpp"
namespace {
using namespace sunhorizon_rgbd_adapter;
int64_t steady_ns() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
SelfExclusionDirectOptions options(double fraction) {
  SelfExclusionDirectOptions value;
  value.node_options.use_global_arguments(false).parameter_overrides({
    {"enabled", true}, {"base_frame", "base_footprint"},
    {"exclusion_min_xyz", std::vector<double>{-.1,-.1,-.1}},
    {"exclusion_max_xyz", std::vector<double>{.1,.1,.1}},
    {"max_cloud_age_sec", .85}, {"tf_timeout_sec", .02}, {"max_removed_fraction", fraction}});
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  value.transforms = std::make_shared<tf2_ros::Buffer>(clock);
  value.transforms->setUsingDedicatedThread(true);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "base_footprint"; transform.child_frame_id = "airy_lidar";
  transform.header.stamp = clock->now(); transform.transform.rotation.w = 1.;
  EXPECT_TRUE(value.transforms->setTransform(transform, "test", true));
  value.source_identity = "/real_normalizer#points"; value.source_epoch = "test-epoch";
  value.input_frame = "airy_lidar"; value.publish_ros_compatibility = false;
  return value;
}
std::shared_ptr<SelfExclusionDirectInput> input(const rclcpp::Node::SharedPtr & node) {
  auto frame = std::make_shared<SelfExclusionDirectInput>();
  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud->header.frame_id = "airy_lidar";
  const auto now = node->now().nanoseconds(); cloud->header.stamp = rclcpp::Time(now-1000000);
  cloud->width = 20; cloud->height = 1; cloud->point_step = 12; cloud->row_step = 240;
  cloud->data.resize(240);
  for (unsigned axis=0;axis<3;++axis) {
    sensor_msgs::msg::PointField field; field.name = std::string(1,"xyz"[axis]);
    field.offset=axis*4; field.count=1; field.datatype=sensor_msgs::msg::PointField::FLOAT32; cloud->fields.push_back(field);
  }
  for(unsigned i=1;i<20;++i){const float x=1.F;std::memcpy(cloud->data.data()+i*12,&x,4);}
  frame->cloud=cloud;
  frame->stamp={123456789,now-1000000,now,steady_ns(),1,"/real_normalizer#points","test-epoch",1,2,1};
  return frame;
}
class DirectFilterTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite(){rclcpp::init(0,nullptr);}
  static void TearDownTestSuite(){rclcpp::shutdown();}
};
TEST_F(DirectFilterTest, SameOriginalGeometryBytesAndReceiptsWithoutDDSIngress) {
  std::vector<std::shared_ptr<const SelfExclusionDirectOutput>> outputs;
  auto filter=make_pointcloud_self_exclusion_direct(options(.10),[&](auto output){outputs.push_back(output);});
  auto frame=input(filter->node()); ASSERT_TRUE(filter->receive(frame)); ASSERT_EQ(outputs.size(),1U);
  const auto expected=exclude_self_points(*frame->cloud,ExclusionBox{{-.1,-.1,-.1},{.1,.1,.1}},ExclusionTransform{});
  ASSERT_TRUE(expected.cloud); EXPECT_EQ(outputs[0]->cloud->data,expected.cloud->data);
  EXPECT_GE(outputs[0]->filter_thread_cpu_sec,0.);
  EXPECT_EQ(outputs[0]->retained_count,19U); EXPECT_EQ(outputs[0]->removed_count,1U);
  EXPECT_EQ(outputs[0]->cloud->header.stamp,frame->cloud->header.stamp);
  EXPECT_EQ(outputs[0]->input->stamp.raw_source_ns,frame->stamp.raw_source_ns);
  EXPECT_EQ(outputs[0]->input->stamp.normalization_thread_cpu_ns,1);
  EXPECT_EQ(outputs[0]->input->stamp.first_receipt_steady_ns,frame->stamp.first_receipt_steady_ns);
  EXPECT_TRUE(filter->receive(frame)); EXPECT_EQ(outputs.size(),1U);
  auto changed=std::make_shared<SelfExclusionDirectInput>(*frame); changed->stamp.first_receipt_ros_ns++;
  EXPECT_FALSE(filter->receive(changed)); EXPECT_TRUE(filter->failed());
  filter->stop();
}
TEST_F(DirectFilterTest, OriginalFractionAndExactTransformFailureNeverFallBackRaw) {
  unsigned outputs=0;
  auto filter=make_pointcloud_self_exclusion_direct(options(.02),[&](auto){++outputs;});
  const auto fraction=filter->receive_result(input(filter->node()));
  EXPECT_EQ(fraction.state,SelfExclusionDirectState::MotionUnavailable);
  EXPECT_EQ(fraction.reason,"removed_fraction_exceeded_no_output");
  EXPECT_FALSE(filter->failed());EXPECT_EQ(outputs,0U);filter->stop();
  auto config=options(.10);
  config.transforms=std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  config.transforms->setUsingDedicatedThread(true);
  filter=make_pointcloud_self_exclusion_direct(std::move(config),[&](auto){++outputs;});
  const auto missing=filter->receive_result(input(filter->node()));
  EXPECT_EQ(missing.state,SelfExclusionDirectState::MotionUnavailable);
  EXPECT_EQ(missing.reason,"cloud_stamp_transform_unavailable");
  EXPECT_GT(missing.snapshot_ros_ns,0);EXPECT_GT(missing.snapshot_steady_ns,0);
  EXPECT_FALSE(filter->failed());EXPECT_EQ(outputs,0U);filter->stop();
}
TEST_F(DirectFilterTest, ExpiredFrameIsDroppedAndNextOriginalFreshFrameStillEmits) {
  std::vector<std::shared_ptr<const SelfExclusionDirectOutput>> outputs;
  auto filter=make_pointcloud_self_exclusion_direct(options(.10),[&](auto output){outputs.push_back(output);});
  auto old=input(filter->node());
  auto cloud=std::make_shared<sensor_msgs::msg::PointCloud2>(*old->cloud);
  old->stamp.source_ns-=1000000000LL;cloud->header.stamp=rclcpp::Time(old->stamp.source_ns);old->cloud=cloud;
  const auto result=filter->receive_result(old);
  EXPECT_EQ(result.state,SelfExclusionDirectState::MotionUnavailable);
  EXPECT_EQ(result.reason,"direct_filter_source_or_receipt_expired");
  EXPECT_GT(result.snapshot_ros_ns-old->stamp.source_ns,850000000LL);
  EXPECT_GE(result.snapshot_steady_ns,old->stamp.first_receipt_steady_ns);
  EXPECT_TRUE(outputs.empty());EXPECT_FALSE(filter->failed());
  const auto duplicate=filter->receive_result(old);
  EXPECT_EQ(duplicate.state,SelfExclusionDirectState::Duplicate);
  EXPECT_TRUE(outputs.empty()); // Identical replay cannot renew or produce output.
  auto next=input(filter->node());next->stamp.sequence=2;
  ASSERT_TRUE(filter->receive_result(next).accepted());ASSERT_EQ(outputs.size(),1U);
  EXPECT_EQ(outputs.front()->input->stamp.source_ns,next->stamp.source_ns);
  EXPECT_EQ(outputs.front()->input->stamp.first_receipt_steady_ns,next->stamp.first_receipt_steady_ns);
  EXPECT_FALSE(filter->failed());filter->stop();
}
TEST_F(DirectFilterTest, MissingSourceTimeTransformCanRecoverWithoutRawFallback) {
  unsigned outputs=0;
  auto config=options(.10);
  config.transforms=std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  config.transforms->setUsingDedicatedThread(true);
  auto transforms=config.transforms;
  auto filter=make_pointcloud_self_exclusion_direct(std::move(config),[&](auto){++outputs;});
  auto first=input(filter->node());
  EXPECT_EQ(filter->receive_result(first).state,SelfExclusionDirectState::MotionUnavailable);
  EXPECT_EQ(outputs,0U);EXPECT_FALSE(filter->failed());
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id="base_footprint";transform.child_frame_id="airy_lidar";
  transform.header.stamp=filter->node()->now();transform.transform.rotation.w=1.;
  ASSERT_TRUE(transforms->setTransform(transform,"test",true));
  auto next=input(filter->node());next->stamp.sequence=2;
  EXPECT_TRUE(filter->receive_result(next).accepted());EXPECT_EQ(outputs,1U);
  EXPECT_FALSE(filter->failed());filter->stop();
}
TEST_F(DirectFilterTest, StructureIdentityAndFutureReceiptRemainTerminal) {
  for(int fault=0;fault<3;++fault){
    SCOPED_TRACE(fault);unsigned outputs=0;
    auto filter=make_pointcloud_self_exclusion_direct(options(.10),[&](auto){++outputs;});
    auto frame=input(filter->node());
    if(fault==0){auto cloud=std::make_shared<sensor_msgs::msg::PointCloud2>(*frame->cloud);
      cloud->data.pop_back();frame->cloud=cloud;}
    if(fault==1)frame->stamp.epoch="other-owner";
    if(fault==2)frame->stamp.first_receipt_ros_ns+=1000000000LL;
    const auto result=filter->receive_result(frame);
    EXPECT_EQ(result.state,SelfExclusionDirectState::InternalFault);EXPECT_TRUE(filter->failed());
    EXPECT_EQ(result.reason,fault==0 ? "invalid_pointcloud_layout" :
      fault==1 ? "direct_filter_identity_or_clock_invalid" : "direct_filter_clock_regressed_or_future");
    auto next=input(filter->node());next->stamp.sequence=2;
    EXPECT_EQ(filter->receive_result(next).state,SelfExclusionDirectState::InternalFault);
    EXPECT_EQ(outputs,0U);filter->stop();
  }
}
TEST_F(DirectFilterTest, DuplicateNormalizationMutationIsNotAnAcceptedReplay) {
  unsigned outputs=0;
  auto filter=make_pointcloud_self_exclusion_direct(options(.10),[&](auto){++outputs;});
  auto frame=input(filter->node());ASSERT_TRUE(filter->receive_result(frame).accepted());
  auto changed=std::make_shared<SelfExclusionDirectInput>(*frame);++changed->stamp.normalization_end_steady_ns;
  const auto result=filter->receive_result(changed);
  EXPECT_EQ(result.state,SelfExclusionDirectState::InternalFault);
  EXPECT_EQ(result.reason,"direct_filter_duplicate_metadata_changed");
  EXPECT_TRUE(filter->failed());EXPECT_EQ(outputs,1U);filter->stop();
}
}
