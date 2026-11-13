#pragma once

#include <utility>
#include <Eigen/Geometry>
#include "sunhorizon_3d_localizer/ndt_direct_runtime.hpp"

namespace sunhorizon_3d_localizer::detail {
// Called only at the existing fully accepted NDT result outlet. This seals the
// original registration independently of the already constructed planar pose
// and odometry; it neither evaluates acceptance nor changes their ownership.
inline std::shared_ptr<const NdtDirectResult> make_accepted_ndt_snapshot(
    NdtDirectResult accepted,const Eigen::Matrix4f& map_to_source,
    const std_msgs::msg::Header& source_header,const std::string& map_frame,
    const std::string& registration_map_path) {
  auto full=std::make_shared<geometry_msgs::msg::TransformStamped>();
  full->header=source_header;full->header.frame_id=map_frame;
  full->child_frame_id=source_header.frame_id;
  full->transform.translation.x=map_to_source(0,3);
  full->transform.translation.y=map_to_source(1,3);
  full->transform.translation.z=map_to_source(2,3);
  const Eigen::Quaternionf orientation(map_to_source.block<3,3>(0,0));
  const auto normalized=orientation.normalized();
  full->transform.rotation.x=normalized.x();full->transform.rotation.y=normalized.y();
  full->transform.rotation.z=normalized.z();full->transform.rotation.w=normalized.w();
  accepted.map_to_source=std::move(full);
  accepted.registration_map_path=registration_map_path;
  return std::make_shared<const NdtDirectResult>(std::move(accepted));
}
} // namespace sunhorizon_3d_localizer::detail
