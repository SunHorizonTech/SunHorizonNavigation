#include "sunhorizon_perception_runtime/rgbd_health.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace sunhorizon_perception_runtime {
namespace {
std::int64_t stamp_ns(const builtin_interfaces::msg::Time & time) {
  return std::int64_t(time.sec)*1000000000LL+time.nanosec;
}
double seconds(const builtin_interfaces::msg::Time & time) {return double(stamp_ns(time))/1.e9;}
sunhorizon_navigation_core::ImageMetadata metadata(const sensor_msgs::msg::Image & image) {
  return {seconds(image.header.stamp),image.width,image.height,image.encoding,image.header.frame_id};
}
sunhorizon_navigation_core::CameraIntrinsics intrinsics(const sensor_msgs::msg::CameraInfo & info) {
  return {info.width,info.height,info.k[0],info.k[4],info.k[2],info.k[5],info.header.frame_id};
}
bool shape_valid(const sensor_msgs::msg::Image & image, std::uint32_t bytes_per_pixel) {
  return image.width>0 && image.height>0 &&
    std::uint64_t(image.width)*bytes_per_pixel<=image.step &&
    std::uint64_t(image.step)*image.height<=image.data.size();
}
bool info_matches(const sensor_msgs::msg::Image & image,const sensor_msgs::msg::CameraInfo & info) {
  return image.width==info.width && image.height==info.height &&
    image.header.frame_id==info.header.frame_id && stamp_ns(image.header.stamp)==stamp_ns(info.header.stamp);
}
double valid_ratio(const sensor_msgs::msg::Image & image,const RgbdHealthConfig & config) {
  if ((image.encoding!="16UC1" && image.encoding!="mono16") || !shape_valid(image,2)) return 0.;
  std::size_t sampled=0,valid=0;
  const auto stride=static_cast<std::uint64_t>(std::max<std::int64_t>(2,config.pointcloud_pixel_stride*2));
  for (std::uint64_t row=0;row<image.height;row+=stride) {
    for (std::uint64_t column=0;column<image.width;column+=stride) {
      const auto offset=std::size_t(row)*image.step+column*2;
      std::uint16_t raw=0;std::memcpy(&raw,image.data.data()+offset,sizeof(raw));
      if (image.is_bigendian) raw=std::uint16_t((raw>>8U)|(raw<<8U));
      const double meters=double(raw)*config.depth_scale_m;
      ++sampled;if (raw!=0 && meters>=config.min_depth_m && meters<=config.max_depth_m) ++valid;
    }
  }
  return sampled?double(valid)/double(sampled):0.;
}
template<class T> T parameter(rclcpp::Node & node,const std::string & name,const T & initial) {
  if (!node.has_parameter(name)) return node.declare_parameter<T>(name,initial);
  return node.get_parameter(name).get_value<T>();
}
}  // namespace

RgbdHealthConfig read_rgbd_health_config(rclcpp::Node & node,const std::string & prefix) {
  RgbdHealthConfig c;
#define RGBD_PARAM(field) c.field=parameter(node,prefix+#field,c.field)
  RGBD_PARAM(target_rate_hz);RGBD_PARAM(status_rate_hz);RGBD_PARAM(max_pair_delta_sec);
  RGBD_PARAM(freshness_timeout_sec);RGBD_PARAM(statistics_window_sec);
  RGBD_PARAM(minimum_healthy_pair_rate_hz);RGBD_PARAM(maximum_rejected_pair_ratio);
  RGBD_PARAM(depth_scale_m);RGBD_PARAM(min_depth_m);RGBD_PARAM(max_depth_m);
  RGBD_PARAM(minimum_valid_depth_ratio);RGBD_PARAM(queue_size);RGBD_PARAM(pointcloud_pixel_stride);
  RGBD_PARAM(chest_alignment_calibration_accepted);RGBD_PARAM(chest_depth_scale_accepted);
  RGBD_PARAM(belly_projection_calibration_accepted);RGBD_PARAM(belly_depth_scale_accepted);
  RGBD_PARAM(calibration_report_sha256);
#undef RGBD_PARAM
  return c;
}
RgbdHealth::RgbdHealth(rclcpp::Node & node,const std::string & prefix)
: core_(read_rgbd_health_config(node,prefix)) {}
RgbdHealth::RgbdHealth(RgbdHealthConfig config):core_(std::move(config)) {}
const RgbdHealthConfig & RgbdHealth::config() const noexcept {return core_.config();}

RgbdAdmission RgbdHealth::admit(const sunhorizon_camera_runtime::CameraFrame & frame,
  std::int64_t now_ros_ns,std::int64_t now_steady_ns)
{
  RgbdHealthObservation input;
  input.role=frame.role;input.sequence=frame.sequence;
  input.receipt_ros_ns=frame.first_sdk_callback_ros_ns;
  input.receipt_steady_ns=frame.first_sdk_callback_steady_ns;
  input.source_validated=frame.source_validated;
  input.depth_scale_m=frame.depth_scale_m;
  if (frame.depth) {
    input.depth=metadata(*frame.depth);input.depth_source_ns=stamp_ns(frame.depth->header.stamp);
    input.depth_shape_valid=(frame.depth->encoding=="16UC1" || frame.depth->encoding=="mono16") &&
      shape_valid(*frame.depth,2);
    input.valid_depth_ratio=valid_ratio(*frame.depth,core_.config());
    input.source_validated=input.source_validated && stamp_ns(frame.source_stamp)==input.depth_source_ns;
    if (frame.depth_info) {
      input.depth_info=intrinsics(*frame.depth_info);
      input.depth_info_matches=info_matches(*frame.depth,*frame.depth_info);
    }
  }
  if (frame.color) {
    input.color=metadata(*frame.color);input.color_source_ns=stamp_ns(frame.color->header.stamp);
    input.color_shape_valid=(frame.color->encoding=="rgb8" || frame.color->encoding=="bgr8") &&
      shape_valid(*frame.color,3);
    if (frame.color_info) {
      input.color_info=intrinsics(*frame.color_info);
      input.color_info_matches=info_matches(*frame.color,*frame.color_info);
    }
  }
  return core_.admit(input,now_ros_ns,now_steady_ns);
}
bool RgbdHealth::complete(const sunhorizon_camera_runtime::CameraFrame & frame,
  std::int64_t cloud_source_stamp_ns,std::int64_t now_ros_ns,std::int64_t now_steady_ns)
{
  return core_.complete(frame.role,frame.sequence,cloud_source_stamp_ns,now_ros_ns,now_steady_ns);
}
void RgbdHealth::reject_computation(const sunhorizon_camera_runtime::CameraFrame & frame,const std::string & reason,bool upstream_until_completion) {
  core_.reject_computation(frame.role,frame.sequence,reason,upstream_until_completion);
}
void RgbdHealth::fault(const std::string & reason) {core_.fault(reason);}

sunhorizon_navigation_interfaces::msg::RgbdStatus RgbdHealth::snapshot(
  std::int64_t now_ros_ns,std::int64_t now_steady_ns)
{
  const auto s=core_.snapshot(now_ros_ns,now_steady_ns);
  sunhorizon_navigation_interfaces::msg::RgbdStatus o;
  o.header.stamp=rclcpp::Time(now_ros_ns);o.header.frame_id="base_link";
  o.state=s.state;
  o.chest_pair_fresh=s.chest_pair_fresh;o.chest_camera_info_valid=s.source.chest_info_valid;
  o.chest_alignment_accepted=config().chest_alignment_calibration_accepted;
  o.chest_depth_scale_accepted=config().chest_depth_scale_accepted;
  o.chest_pointcloud_healthy=s.chest_pointcloud_healthy;
  o.belly_depth_fresh=s.belly_depth_fresh;o.belly_camera_info_valid=s.source.belly_info_valid;
  o.belly_depth_scale_accepted=config().belly_depth_scale_accepted;
  o.belly_pointcloud_healthy=s.belly_pointcloud_healthy;
  o.last_pair_delta_ms=s.source.last_pair_delta_sec*1000.;
  o.accepted_pair_rate_hz=s.accepted_pair_rate_hz;o.rejected_pair_ratio=s.rejected_pair_ratio;
  o.chest_pair_age_sec=s.chest_pair_age_sec;o.belly_depth_age_sec=s.belly_depth_age_sec;
  o.chest_pointcloud_age_sec=s.chest_pointcloud_age_sec;o.belly_pointcloud_age_sec=s.belly_pointcloud_age_sec;
  o.accepted_pairs=s.source.accepted_pairs;o.rejected_pairs=s.source.rejected_pairs;o.reason=s.reason;
  return o;
}
}  // namespace sunhorizon_perception_runtime
