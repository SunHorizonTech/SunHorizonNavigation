#include "sunhorizon_airy_adapter/direct_adapter.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <stdexcept>
using namespace sunhorizon_airy_adapter;
#define require(ok) do { if (!(ok)) throw std::runtime_error("direct adapter check failed at line " + std::to_string(__LINE__)); } while (false)
int64_t ns(const builtin_interfaces::msg::Time& t) { return int64_t(t.sec)*1000000000LL+t.nanosec; }
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr normalized;
    sensor_msgs::msg::Imu::ConstSharedPtr normalized_imu;
    AiryDirectStamp cloud_stamp, imu_stamp;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("point_calibration_valid",true),
      rclcpp::Parameter("imu_axis_calibration_valid",true),
      rclcpp::Parameter("linear_acceleration_scale",9.80665),
      rclcpp::Parameter("output_imu_frame",std::string("airy_imu")),
      rclcpp::Parameter("header_time_mode",std::string("fixed_offset")),
      rclcpp::Parameter("fixed_header_time_offset_sec",2.0),
      rclcpp::Parameter("point_rotation_xyzw",std::vector<double>{0.,0.,std::sqrt(.5),std::sqrt(.5)})});
    auto adapter=make_airy_direct_adapter(options,
      {[&](auto p,const auto& s){normalized=p;cloud_stamp=s;},
       [&](auto p,const auto& s){normalized_imu=p;imu_stamp=s;}},"sdk#points","sdk#imu");
    auto cloud=std::make_shared<sensor_msgs::msg::PointCloud2>();
    cloud->header.frame_id="rslidar";
    cloud->header.stamp=rclcpp::Time(adapter->node()->now().nanoseconds()-2020000000LL);
    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2Fields(4,"x",1,sensor_msgs::msg::PointField::FLOAT32,
      "y",1,sensor_msgs::msg::PointField::FLOAT32,"z",1,sensor_msgs::msg::PointField::FLOAT32,
      "timestamp",1,sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(1);
    sensor_msgs::PointCloud2Iterator<float> x(*cloud,"x"),y(*cloud,"y"),z(*cloud,"z"),t(*cloud,"timestamp");
    *x=1.;*y=2.;*z=3.;*t=.01;
    const auto original=cloud->data;
    const auto receipt=adapter->node()->now().nanoseconds();
    const auto steady=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    AiryDirectStamp input{ns(cloud->header.stamp),ns(cloud->header.stamp),receipt,steady,1,"sdk#points"};
    adapter->receive_points(cloud,input);
    require(normalized && normalized->header.frame_id=="airy_lidar" && cloud->data==original);
    sensor_msgs::PointCloud2ConstIterator<float> nx(*normalized,"x"),ny(*normalized,"y"),nz(*normalized,"z");
    require(std::abs(*nx+2.)<1e-5 && std::abs(*ny-1.)<1e-5 && std::abs(*nz-3.)<1e-5);
    require(std::abs(cloud_stamp.source_ns-input.raw_source_ns-2000000000LL)<512);
    require(cloud_stamp.raw_source_ns==input.raw_source_ns && cloud_stamp.first_receipt_ros_ns==receipt &&
      cloud_stamp.first_receipt_steady_ns==steady && cloud_stamp.sequence==1 && cloud_stamp.source_identity=="/sunhorizon_airy_adapter#points");
    auto imu=std::make_shared<sensor_msgs::msg::Imu>();imu->header=cloud->header;
    imu->linear_acceleration.x=1.;imu->angular_velocity.y=2.;
    input.source_identity="sdk#imu";adapter->receive_imu(imu,input);
    require(normalized_imu && normalized_imu->header.frame_id=="airy_imu");
    require(std::abs(normalized_imu->linear_acceleration.x-9.80665)<1e-9 && normalized_imu->angular_velocity.y==2.);
    require(imu_stamp.raw_source_ns==input.raw_source_ns && imu_stamp.first_receipt_steady_ns==steady);
    bool refused=false;try {adapter->receive_imu(imu,input);}catch(const std::invalid_argument&){refused=true;}
    require(refused);input.sequence=2;input.source_identity="other";refused=false;
    try {adapter->receive_imu(imu,input);}catch(const std::invalid_argument&){refused=true;}require(refused);
    adapter.reset();rclcpp::shutdown();return 0;
  } catch (...) {rclcpp::shutdown();throw;}
}
