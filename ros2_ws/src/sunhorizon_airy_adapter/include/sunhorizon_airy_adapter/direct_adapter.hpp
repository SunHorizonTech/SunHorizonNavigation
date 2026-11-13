#pragma once
#include <array>
#include <vector>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
namespace sunhorizon_airy_adapter {
struct AiryDirectStamp {
  int64_t raw_source_ns{0}, source_ns{0}, first_receipt_ros_ns{0}, first_receipt_steady_ns{0};
  uint64_t sequence{0};
  std::string source_identity;
  int64_t normalization_start_steady_ns{0}, normalization_end_steady_ns{0}, normalization_thread_cpu_ns{0};
  std::shared_ptr<const std::vector<std::array<float,3>>> ray_origins;
};
struct AiryDirectSinks {
  std::function<void(sensor_msgs::msg::PointCloud2::ConstSharedPtr,const AiryDirectStamp&)> points;
  std::function<void(sensor_msgs::msg::Imu::ConstSharedPtr,const AiryDirectStamp&)> imu;
};
// Reuses the standalone calibration, timestamp and shared clock gates. Call
// points/IMU on separate ordered bounded lanes; sinks must not block or retain
// references to stamps. No sensor DDS subscription or private worker is created.
class AiryDirectAdapter {
 public:
  virtual ~AiryDirectAdapter() = default;
  virtual rclcpp::Node::SharedPtr node() = 0;
  virtual void receive_points(sensor_msgs::msg::PointCloud2::ConstSharedPtr,const AiryDirectStamp&) = 0;
  virtual void receive_imu(sensor_msgs::msg::Imu::ConstSharedPtr,const AiryDirectStamp&) = 0;
};
std::shared_ptr<AiryDirectAdapter> make_airy_direct_adapter(
  const rclcpp::NodeOptions&, AiryDirectSinks, std::string points_owner, std::string imu_owner);
rclcpp::Node::SharedPtr make_airy_adapter_node(const rclcpp::NodeOptions& = rclcpp::NodeOptions());
}  // namespace sunhorizon_airy_adapter
