#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include "sunhorizon_livo_bringup/body_odometry_gauge.hpp"
#include "sunhorizon_livo_bringup/causal_body_velocity.hpp"
#include <geometry_msgs/msg/transform.hpp>
#include <sunhorizon_navigation_core/body_velocity_evidence.hpp>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sunhorizon_navigation_interfaces/msg/localization_status.hpp>

namespace sunhorizon_livo_bringup {
// Optional wall-clock observations of the existing vendor output FIFO.
// Never used as input receipts, measurement timestamps or freshness evidence.
struct LivoDirectDeliveryTiming {
  int64_t output_lane_enqueue_steady_ns{0};
  int64_t output_lane_dequeue_steady_ns{0};
  int64_t geometry_wait_begin_steady_ns{0};
  int64_t geometry_wait_end_steady_ns{0};
};
struct LivoDirectStamp {
  int64_t source_stamp_ns{0};
  int64_t receipt_ros_ns{0};
  int64_t receipt_steady_ns{0};
  uint64_t sequence{0};
  std::string source_identity;
  std::optional<sunhorizon_navigation_core::BodyVelocityEvidence> body_velocity{};
  std::optional<LivoDirectDeliveryTiming> delivery_timing{};
};

struct SourceTimeBodyTransform {
  BodyVelocityGeometryStatus status{BodyVelocityGeometryStatus::Unavailable};
  std::optional<geometry_msgs::msg::Transform> transform;
};

struct LivoDirectSinks {
  // Assembly snapshots the constructed vendor owner's common.img_en once.
  // This is not a ROS parameter: users cannot disable required VIO health
  // independently of the estimator. Standalone/legacy callers require RGB.
  bool vendor_images_enabled{true};
  // Bound to the SAME source-time joint owner and accepted nominal calibration.
  // No TF listener or alternate kinematic tree is created by the adapter.
  std::string body_transform_tf_mode;
  std::function<SourceTimeBodyTransform(
      int64_t, const geometry_msgs::msg::Transform&)> source_time_vendor_child_from_base;
  // Wire these to the sole vendor estimator's bounded input mailboxes. They
  // execute on the caller's input lane and must never run estimation/publish.
  std::function<void(sensor_msgs::msg::PointCloud2::ConstSharedPtr, const LivoDirectStamp&)>
      vendor_points;
  std::function<void(sensor_msgs::msg::Imu::ConstSharedPtr, const LivoDirectStamp&)> vendor_imu;
  std::function<void(sensor_msgs::msg::Image::ConstSharedPtr, const LivoDirectStamp&)> vendor_image;
  std::function<void(nav_msgs::msg::Odometry::ConstSharedPtr, const LivoDirectStamp&)> candidate;
  std::function<void(sunhorizon_navigation_interfaces::msg::LocalizationStatus::ConstSharedPtr)> status;
  bool publish_ros_candidates{true};
  // Opt-in low-rate legacy status only when candidate ROS egress is disabled.
  // False preserves existing callers' candidate/status coupling by default.
  bool publish_ros_status_compatibility{false};
};

class LivoDirectAdapter {
 public:
  virtual ~LivoDirectAdapter() = default;
  virtual std::shared_ptr<rclcpp::Node> node() = 0;
  virtual void reset_body_velocity(int64_t original_reset_steady_ns) = 0;
  virtual BodyOdometryGaugeOrigin body_odometry_gauge_origin() = 0;
  virtual void receive_points(sensor_msgs::msg::PointCloud2::ConstSharedPtr, const LivoDirectStamp&) = 0;
  virtual void receive_imu(sensor_msgs::msg::Imu::ConstSharedPtr, const LivoDirectStamp&) = 0;
  virtual void receive_image(sensor_msgs::msg::Image::ConstSharedPtr, const LivoDirectStamp&) = 0;
  virtual void receive_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr, const LivoDirectStamp&) = 0;
  // Only the vendor's already gated, immutable output snapshots enter here.
  // _state reads and cloud_registered(now) are not measurement evidence.
  virtual void receive_corrected_odometry(nav_msgs::msg::Odometry::ConstSharedPtr, const LivoDirectStamp&) = 0;
  virtual void receive_propagated_odometry(nav_msgs::msg::Odometry::ConstSharedPtr, const LivoDirectStamp&) = 0;
};

// Reuses the existing adapter's conversion, rate gates, canonical geometry,
// continuity/watchdog and aggregate input admission. Creates no sensor/vendor
// DDS subscriptions or intermediate points/RGB publishers. Root owns separate
// scheduling lanes and replaces the previous adapter owner when wiring this.
std::shared_ptr<LivoDirectAdapter> make_livo_direct_adapter(
    const rclcpp::NodeOptions& options, LivoDirectSinks sinks);
}  // namespace sunhorizon_livo_bringup
