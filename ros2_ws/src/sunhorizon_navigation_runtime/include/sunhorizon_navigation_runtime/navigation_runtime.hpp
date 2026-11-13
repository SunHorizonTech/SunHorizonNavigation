#pragma once
#include "sunhorizon_navigation_runtime/obstacle_source_policy.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include "sunhorizon_obstacle_coverage/coverage_direct_runtime.hpp"
#include "sunhorizon_rgbd_adapter/pointcloud_self_exclusion_direct.hpp"
#include <tf2_ros/buffer.h>
#include <std_msgs/msg/u_int64.hpp>
#include "sunhorizon_airy_adapter/direct_adapter.hpp"
#include "sunhorizon_3d_localizer/ndt_direct_runtime.hpp"
#include "sunhorizon_livo_bringup/livo_direct_adapter.hpp"
#include "sunhorizon_localization_manager/localization_direct_runtime.hpp"
#include "sunhorizon_perception_runtime/runtime.hpp"

namespace sunhorizon_navigation_runtime {
struct NavigationRuntimeOptions {
  std::string obstacle_source_mode{"lidar-rgbd-v1"};
  rclcpp::NodeOptions runtime_node,airy_node,airy_adapter_node,camera_node,perception_node,joint_node;
  rclcpp::NodeOptions livo_adapter_node,livo_vendor_node,ndt_node,localization_manager_node;
  std::string airy_config_path;
  std::string kinematics_tf_mode; // Explicit production_direct or receipted attended_direct.
  std::shared_ptr<tf2_ros::Buffer> direct_tf_buffer;
  bool publish_ros_localization_candidates{false};
  bool publish_ros_canonical_compatibility{false};
  bool debug_publish_body_clouds{false};
  rclcpp::NodeOptions coverage_node;
  bool debug_publish_raw_sensors{false};
  // Explicit unified startup contract; standalone/older embedding owners keep
  // their existing spatial publication behavior unless they request deferral.
  bool defer_spatial_evidence_until_startup{false};
  sunhorizon_perception_runtime::SpatialQueryPolicy spatial_query;
};
struct NavigationRuntimeSinks {
  // These callbacks transfer immutable owners into bounded downstream lanes.
  // Register the actual Nav2/body consumer before start(); it is mandatory.
  sunhorizon_perception_runtime::PerceptionOutputSink perception_output;
  sunhorizon_perception_runtime::RegionOutputSink region_output;
  sunhorizon_perception_runtime::JointKinematics::TransformSink tf_compatibility;
  std::function<void(sensor_msgs::msg::PointCloud2::ConstSharedPtr,
    const sunhorizon_airy_adapter::AiryDirectStamp &)> airy_frame;
  std::function<void(nav_msgs::msg::Odometry::ConstSharedPtr,
    const sunhorizon_livo_bringup::LivoDirectStamp &)> livo_candidate;
  std::function<void(std::shared_ptr<const sunhorizon_3d_localizer::NdtDirectResult>)> ndt_accepted;
  std::function<void(std::shared_ptr<const sunhorizon_localization_manager::CanonicalLocalizationState>)> canonical_localization;
  std::function<void(std::shared_ptr<const sunhorizon_localization_manager::CanonicalLocalizationHealth>)> localization_health;
  std::function<void(sunhorizon_navigation_interfaces::msg::LocalizationStatus::ConstSharedPtr)> livo_status,ndt_status;
  // ACK follows the vendor's gated corrected snapshot after ZUPT application.
  std::function<void(std::uint64_t,const sunhorizon_livo_bringup::LivoDirectStamp &)> zupt_applied;
  std::function<void(const std::string &)> fault;
};
// Actual single-process ownership: one camera shared by perception and VIO,
// one Airy SDK, one original vendor estimator, one NDT owner. Sensor/corrected
// callbacks only admit immutable owners; original component gates remain active.
class NavigationRuntime {
 public:
  ~NavigationRuntime();
  // Add manager/Nav2 control nodes before start. Their heavy processing must
  // remain on their existing workers; no sensor/estimator is constructed here.
  void add_control_node(rclcpp::Node::SharedPtr);
  void add_control_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr);
  void start();
  void bind_collision_height_responsibility(const sunhorizon_perception_runtime::CollisionHeightBinding&);
  void admit_filtered_airy(std::shared_ptr<const sunhorizon_rgbd_adapter::SelfExclusionDirectOutput>);
  void release_spatial_startup(int64_t original_deadline_steady_ns); // Once, after actual STARTUP success.
  sunhorizon_perception_runtime::SpatialStartupStatus spatial_startup_status() const;
  // Identity/measurement evidence only, never an authorization to move. Source
  // and first receipts are frozen at the original accepted callbacks; the
  // DiagnosticArray header is the observation/publication time only.
  diagnostic_msgs::msg::DiagnosticArray direct_owner_snapshot() const;
  bool submit_stationary_zupt(std_msgs::msg::UInt64::ConstSharedPtr,
    const sunhorizon_livo_bringup::LivoDirectStamp &);
  void stop(); // External owner, never a callback. Stops sources and joins lanes.
  bool failed() const;
  std::string failure_reason() const;
  std::shared_ptr<tf2_ros::Buffer> direct_tf_buffer() const;
  sunhorizon_perception_runtime::ground_relative::GroundReferenceOwner::ConstSharedPtr ground_reference_owner() const;
  sunhorizon_perception_runtime::RegistrationGroundGeometryProvider registration_ground_geometry_provider() const;
  std::string airy_ground_calibration_id() const;
  std::shared_ptr<const sunhorizon_obstacle_coverage::dynamic_self_geometry::PreparedModel>
    lidar_self_model(const std::string& source_frame,int64_t source_ns) const;
  sunhorizon_rgbd_adapter::LidarModelProvider lidar_model_provider_for_scan(std::size_t maximum_entries) const;
  double lidar_surface_radius_bound(const std::string& source_frame) const;
  double self_surface_diagnostic_tolerance_m() const;
  rclcpp::Node::SharedPtr node() const;
  rclcpp::Node::SharedPtr airy_adapter_node() const;
  std::shared_ptr<sunhorizon_localization_manager::LocalizationDirectRuntime> localization_manager() const;
 private:
  struct Impl;
  explicit NavigationRuntime(std::unique_ptr<Impl>);
  std::unique_ptr<Impl> impl_;
  friend std::shared_ptr<NavigationRuntime> make_navigation_runtime(NavigationRuntimeOptions,NavigationRuntimeSinks);
};
// Injection boundary for the next manager/Nav2 owner: configure existing
// component NodeOptions and connect the real result/status/body/fault sinks.
// No fallback sensor subscriptions, transport bridge, or final velocity output.
std::shared_ptr<NavigationRuntime> make_navigation_runtime(
  NavigationRuntimeOptions,NavigationRuntimeSinks);
}  // namespace sunhorizon_navigation_runtime
