#pragma once

#include <cstddef>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace sunhorizon_perception_runtime {
struct JointStartupOptions {
  std::string robot_serial, release_sha;
  std::string source_receipt_path, source_receipt_sha256, source_evidence_sha256;
  std::string mapping_receipt_path, mapping_receipt_sha256;
  std::string source_review_authority_sha256, mapping_review_authority_sha256;
  std::string production_tf_receipt_path, production_tf_receipt_sha256;
  // Explicitly selected; absence of production acceptance never downgrades it.
  std::string tf_mode;
  std::string attended_usage, attended_tf_asset_path, attended_tf_asset_sha256;
  std::string attended_tf_receipt_path, attended_tf_receipt_sha256;
};
struct ValidatedJointStartup {
  std::vector<std::string> expected_source_names, selected_source_names, output_joint_names;
  std::vector<std::size_t> selected_source_indices;
  std::string merged_urdf_xml, merged_urdf_sha256;
  std::vector<std::string> leg_target_frames;
  std::vector<geometry_msgs::msg::TransformStamped> attended_static_edges;
  // Explicitly bound name alias only; no changed joint geometry/extrinsics.
  std::string waist_source_joint_name, waist_output_joint_name;
};

JointStartupOptions load_joint_startup_options(rclcpp::Node& node);

// Always validates actual source/mapping bytes, all bound evidence, independent
// review signatures, release/robot/authority pins and mapping semantics. With
// legacy model supplied, attended_global_tf validates only the original signed
// geometry and builds base_link-relative leg FK; production_direct additionally
// requires complete production sensor TF acceptance. There is no fallback.
// Original legacy URDF bytes/SHA remain the mesh contract and are never written.
ValidatedJointStartup validate_joint_startup(
    const JointStartupOptions& options,
    const std::string& legacy_urdf_path = {},
    const std::string& legacy_urdf_sha256 = {});
}  // namespace sunhorizon_perception_runtime
