#pragma once
#include <array>
#include <atomic>
#include "sunhorizon_rgbd_adapter/lidar_self_audit.hpp"
#include <vector>
#include <cstdint>
#include <functional>
#include <memory>
#include <limits>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "sunhorizon_rgbd_adapter/pointcloud_self_exclusion.hpp"
#include "sunhorizon_obstacle_coverage/dynamic_self_geometry.hpp"
namespace sunhorizon_rgbd_adapter {
struct SelfExclusionDirectStamp {
  int64_t raw_source_ns{0}, source_ns{0}, first_receipt_ros_ns{0}, first_receipt_steady_ns{0};
  uint64_t sequence{0};
  std::string source_identity, epoch;
  int64_t normalization_start_steady_ns{0}, normalization_end_steady_ns{0}, normalization_thread_cpu_ns{0};
};
struct SelfExclusionDirectInput {
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
  SelfExclusionDirectStamp stamp;
  int64_t queue_admitted_steady_ns{0};
  std::shared_ptr<const std::vector<std::array<float,3>>> ray_origins;
};
struct SelfExclusionDirectOutput {
  std::shared_ptr<const SelfExclusionDirectInput> input;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
  std::size_t input_count{0}, removed_count{0}, retained_count{0};
  std::shared_ptr<const std::vector<uint8_t>> ground_blocked;
  // Same retained record order and source coordinates as cloud, after both
  // CAD and static-AABB compaction. Never reuse the input array by index.
  std::shared_ptr<const std::vector<std::array<float,3>>> ray_origins;
  std::shared_ptr<const std::vector<uint64_t>> sample_ids;
  std::shared_ptr<const LidarSelfAudit> lidar_audit;
  double tf_wall_sec{0.}, filter_wall_sec{0.}, filter_thread_cpu_sec{-1.};
};
using LidarModelProvider = std::function<std::shared_ptr<const
  sunhorizon_obstacle_coverage::dynamic_self_geometry::PreparedModel>(const std::string&,int64_t)>;
using LidarModelProviderFactory = std::function<LidarModelProvider(std::size_t)>;
using SelfExclusionDirectSink = std::function<void(std::shared_ptr<const SelfExclusionDirectOutput>)>;
struct SelfExclusionDirectOptions {
  rclcpp::NodeOptions node_options;
  std::shared_ptr<tf2_ros::Buffer> transforms;
  std::string source_identity, source_epoch, input_frame;
  // Retained coverage compatibility boundary, on its own observer worker.
  bool publish_ros_compatibility{true};
  // Optional shared CAD owner. Exact point-time FK, same immutable twelve mesh
  // assets as RGB-D. Zero tolerance deletes only exact first-surface matches;
  // nonzero values require an explicit measured matching error contract.
  LidarModelProvider lidar_model;
  double lidar_surface_radius_bound_m{std::numeric_limits<double>::infinity()};
  double lidar_surface_tolerance_m{0.};
  bool lidar_remove_self_surfaces{false};
  // One closure per scan; limit is its original record count. Numerical geometry
  // may share storage, but every original source-time query still runs its gates.
  LidarModelProviderFactory lidar_model_for_scan;
  // Shared one-shot request; disabled/closed means no audit allocations/copies.
  std::shared_ptr<std::atomic<bool>> lidar_audit_requested;
};
enum class SelfExclusionDirectState {Accepted, Duplicate, MotionUnavailable, InternalFault};
struct SelfExclusionDirectResult {
  SelfExclusionDirectState state{SelfExclusionDirectState::InternalFault};
  std::string reason;
  int64_t snapshot_ros_ns{0}, snapshot_steady_ns{0};
  bool accepted() const {return state==SelfExclusionDirectState::Accepted;}
};
class PointcloudSelfExclusionDirect {
 public:
  virtual ~PointcloudSelfExclusionDirect() = default;
  virtual rclcpp::Node::SharedPtr node() = 0;
  // Synchronous original filter computation. Call only on a bounded compute lane.
  virtual bool receive(std::shared_ptr<const SelfExclusionDirectInput>) = 0;
  virtual SelfExclusionDirectResult receive_result(std::shared_ptr<const SelfExclusionDirectInput>) = 0;
  virtual bool failed() const = 0;
  virtual void stop() = 0; // External owner, after admission/compute quiesces.
};
// Reuses the standalone node's original parameter parser, exact-time transforms,
// disjoint-volume/shadow geometry and removed-fraction gates. No DDS input or
// TransformListener is created. Dynamic belly mode is a separate implementation.
std::shared_ptr<PointcloudSelfExclusionDirect> make_pointcloud_self_exclusion_direct(
  SelfExclusionDirectOptions, SelfExclusionDirectSink);
}  // namespace sunhorizon_rgbd_adapter
