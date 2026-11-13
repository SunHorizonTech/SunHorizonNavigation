#pragma once
#include "sunhorizon_navigation_runtime/obstacle_source_policy.hpp"
#include "sunhorizon_rgbd_adapter/pointcloud_self_exclusion_direct.hpp"
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <nav2_util/direct_observation.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "sunhorizon_localization_manager/localization_direct_runtime.hpp"
#include "sunhorizon_perception_runtime/runtime.hpp"

namespace sunhorizon_navigation_runtime {
struct Nav2BodySourceBinding {
  std::string camera_role; // Existing camera role: belly or chest.
  // Global layer may be empty when the retained global map has no such source.
  // A nonempty layer can bind body, raw clearing, or both original source sets.
  std::string local_layer, local_source, global_layer, global_source, collision_source;
  // Original independent RGB-D projection only. A dual source may occur both
  // here and in local_source/global_source, with unchanged marking/clearing flags.
  // Each actual camera clearing sink must have observation_persistence=0;
  // delivery is one in-flight plus one replaceable latest pending observation.
  std::vector<std::string> local_clearing_sources, global_clearing_sources;
};
struct Nav2AirySourceBinding {
  std::string local_layer, global_layer, collision_source;
  std::vector<std::string> local_sources, global_sources;
};
struct Nav2AssemblyOptions {
  std::string obstacle_source_mode{"lidar-rgbd-v1"};
  rclcpp::NodeOptions host_node, planner_node, controller_node, collision_node;
  std::vector<Nav2BodySourceBinding> body_sources;
  bool classified_obstacles_enabled{false};
  // The attended camera profile also verifies the forward-only controller and measured stop.
  int64_t canonical_age_limit_ns() const {
    return camera_maximum_age_ns == sunhorizon_obstacle_coverage::kOneSecondCameraAgeNs
      && classified_obstacles_enabled && obstacle_source_mode == "rgbd-only-v1"
      ? 500000000LL : 300000000LL;
  }
  int64_t camera_maximum_age_ns{sunhorizon_obstacle_coverage::kDefaultCameraAgeNs};
  sunhorizon_perception_runtime::ground_relative::GroundReferenceOwner::ConstSharedPtr ground_reference_owner;
  sunhorizon_perception_runtime::RegistrationGroundGeometryProvider registration_ground_geometry;
  std::string airy_ground_calibration_id;
  rclcpp::NodeOptions airy_filter_node;
  sunhorizon_rgbd_adapter::LidarModelProvider airy_lidar_model;
  double airy_lidar_surface_radius_bound_m{std::numeric_limits<double>::infinity()};
  double airy_lidar_surface_tolerance_m{0.};
  bool airy_lidar_remove_self_surfaces{false};
  nav2_util::DirectSourceContract airy_input_contract;
  Nav2AirySourceBinding airy_source;
  bool publish_filtered_airy_compatibility{true};
  std::function<void(std::shared_ptr<const sunhorizon_rgbd_adapter::SelfExclusionDirectOutput>)> filtered_airy;
  // Same canonical owner/epoch/frame and accepted age used by the controller.
  nav2_util::DirectSourceContract canonical_odometry;
  std::shared_ptr<sunhorizon_localization_manager::LocalizationDirectRuntime> canonical_owner;
  int64_t localization_health_maximum_age_ns{0}; // Explicit retained watchdog limit.
  // Bounded revoke/latch callback only. External owner calls stop(), never this callback.
  std::function<void(const std::string &)> fault;
  sunhorizon_perception_runtime::SpatialQueryPolicy spatial_query;
  sunhorizon_rgbd_adapter::LidarModelProviderFactory airy_lidar_model_for_scan;
};
// Owns real upstream components and bounded cloud/odometry lanes. Existing
// typed command/evidence DDS and the independent final supervisor stay active.
// Costmap YAML is read through the original process-global ROS --params-file
// arguments, since the upstream controller/planner own their child costmaps.
class Nav2Assembly {
 public:
  ~Nav2Assembly();
  std::vector<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr> nodes() const;
  // Configure existing plugins and bind their direct sources before acquisition.
  // Spin nodes() before starting upstream sources; activate only after healthy
  // canonical TF is admitted. Activation stays on this object's worker.
  void start();
  std::shared_ptr<const sunhorizon_perception_runtime::CollisionHeightBinding> collision_height_binding() const;
  // These methods perform bounded ownership admission only.
  bool consume_body(std::shared_ptr<const sunhorizon_perception_runtime::PerceptionOutput>);
  bool consume_airy(sensor_msgs::msg::PointCloud2::ConstSharedPtr,
    const nav2_util::DirectObservationMetadata &,
    std::shared_ptr<const std::vector<std::array<float,3>>> ray_origins = {});
  bool consume_localization(std::shared_ptr<const sunhorizon_localization_manager::CanonicalLocalizationState>);
  bool consume_health(std::shared_ptr<const sunhorizon_localization_manager::CanonicalLocalizationHealth>);
  // Bounded terminal revoke, safe from source/fault callbacks; no joins here.
  void revoke(const std::string & reason) noexcept;
  void stop(); // External owner, after runtime acquisition/control callbacks stop.
  bool initialized() const; // First activation completed; acquisition may continue during hold.
  bool motion_held() const; // Latched; only a new runtime session can restore control.
  bool active() const; // Initialized and control permission has not been revoked.
  bool failed() const;
  std::string failure_reason() const;
 private:
  struct Impl;
  explicit Nav2Assembly(std::unique_ptr<Impl>);
  std::unique_ptr<Impl> impl_;
  friend std::shared_ptr<Nav2Assembly> make_nav2_assembly(Nav2AssemblyOptions,std::shared_ptr<tf2_ros::Buffer>);
};
std::shared_ptr<Nav2Assembly> make_nav2_assembly(Nav2AssemblyOptions,std::shared_ptr<tf2_ros::Buffer>);
}  // namespace sunhorizon_navigation_runtime
