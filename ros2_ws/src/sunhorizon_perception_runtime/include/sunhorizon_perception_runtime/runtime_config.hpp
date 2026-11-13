#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "sunhorizon_perception_runtime/perception_engine.hpp"
#include "sunhorizon_perception_runtime/collision_height_query.hpp"
#include "sunhorizon_perception_runtime/joint_kinematics.hpp"
#include "sunhorizon_rgbd_adapter/dynamic_self_model.hpp"
#include "sunhorizon_rgbd_adapter/region_depth_producer.hpp"

namespace sunhorizon_perception_runtime {
struct RuntimeSourceConfig {
  std::string name, frame, epoch, calibration;
  double max_removed_fraction{.02};
  bool body_enabled{false}, projection_accepted{false};
  // The migrated belly pipeline must already have explicitly selected dynamic
  // mesh filtering. A spatial mesh manifest cannot upgrade an AABB body profile.
  bool self_filter_enabled{false};
  std::string self_exclusion_mode;
  EngineConfig engine;
  std::optional<sunhorizon_rgbd_adapter::RegionProducerOptions> region;
  std::string region_producer_node_name;
  sunhorizon_obstacle_coverage::RegionCalibrationContract attestations;
};
struct RuntimeConfig {
  bool enabled{false}, include_chest{false};
  int64_t camera_maximum_age_ns{sv::kMaximumEvidenceAgeNs};
  std::string contract, query_digest, producer_epoch, clock_boot_id, common_frame;
  SpatialQueryPolicy query_policy;
  sv::Query query;  // Owner sets each belly anchor stamp before processing.
  // Both sources are always configured and attested. include_chest controls
  // optional spatial fusion only; SDK admission/body computation still uses both.
  std::array<RuntimeSourceConfig,2> sources;
  std::vector<sunhorizon_rgbd_adapter::dynamic_self_model::AssetSpec> model_specs;
  std::string urdf_path, urdf_sha256;
  JointKinematicsOptions joint_kinematics;
};

// full-region-v1 retains the existing canonical query digest. The explicit
// collision-height mode binds both original and effective boxes in its digest.
// New per-source body controls use belly_/chest_ plus the old component and
// pointcloud parameter names. All declared parameters are read-only.
RuntimeConfig load_runtime_config(rclcpp::Node&);
SpatialQueryPolicy load_spatial_query_policy(rclcpp::Node&);

// Apply on each immutable SDK CameraInfo before admitting its paired frame.
// Retains the old exact canonical calibration and rectified-pinhole checks.
bool runtime_camera_info_matches(const RuntimeSourceConfig&,const sensor_msgs::msg::CameraInfo&);
}  // namespace sunhorizon_perception_runtime
