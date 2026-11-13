#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <tf2/buffer_core.h>

#include "sunhorizon_perception_runtime/joint_kinematics.hpp"
#include "sunhorizon_rgbd_adapter/dynamic_self_model.hpp"

namespace sunhorizon_perception_runtime::detail {

struct AttendedRoleGeometry {
  sunhorizon_rgbd_adapter::dynamic_self_model::geometry::RigidTransform optical_from_base;
  sunhorizon_rgbd_adapter::dynamic_self_model::geometry::RigidTransform common_from_optical;
};

// Compile only the already validated, immutable attended asset. This is not a
// cache of a successful lookup in the shared, dynamically updated TF buffer.
// The caller still owns the original source-time FK and deadline checks.
class AttendedGeometry {
 public:
  AttendedGeometry(const JointKinematicsOptions& options, const std::string& common_frame,
      const std::array<std::string, 2>& optical_frames)
      : geometry_(build(options, common_frame, optical_frames)),source_from_base_(build_sources(options)) {}

  const AttendedRoleGeometry* select(const std::string& tf_mode, std::size_t role,
      int64_t source_ns, int64_t anchor_ns) const noexcept {
    if (tf_mode != "attended_direct" || role >= geometry_.size() ||
        source_ns <= 0 || source_ns != anchor_ns) return nullptr;
    return &geometry_[role];
  }

  // Only frames in this exact validated immutable installation asset are
  // available. Production/dynamic waist modes must retain source-time TF.
  const sunhorizon_rgbd_adapter::dynamic_self_model::geometry::RigidTransform*
  select_source(const std::string& tf_mode,const std::string& frame,int64_t source_ns) const noexcept {
    if(tf_mode!="attended_direct" || source_ns<=0)return nullptr;
    const auto found=source_from_base_.find(frame);
    return found==source_from_base_.end()?nullptr:&found->second;
  }

 private:
  static std::map<std::string,sunhorizon_rgbd_adapter::dynamic_self_model::geometry::RigidTransform>
  build_sources(const JointKinematicsOptions& options) {
    namespace dm=sunhorizon_rgbd_adapter::dynamic_self_model;
    // build() has already rejected invalid identities, cycles, disconnected
    // trees and duplicate writers; compile the same eight static edges only.
    tf2::BufferCore fixed;std::set<std::string> frames;
    for(const auto& edge:options.attended_static_edges){
      if(!fixed.setTransform(edge,"validated_attended_fixed_asset",true))
        throw std::invalid_argument("attended_geometry_fixed_edge_rejected");
      frames.insert(edge.header.frame_id);frames.insert(edge.child_frame_id);
    }
    std::map<std::string,dm::geometry::RigidTransform> result;
    for(const auto& frame:frames)result.emplace(frame,dm::rigid(
      fixed.lookupTransform(frame,"base_link",tf2::TimePointZero).transform));
    return result;
  }
  static std::array<AttendedRoleGeometry, 2> build(const JointKinematicsOptions& options,
      const std::string& common_frame, const std::array<std::string, 2>& optical_frames) {
    namespace dm = sunhorizon_rgbd_adapter::dynamic_self_model;
    if (options.tf_mode != "attended_direct" ||
        options.attended_usage != "attended_heading_correction" ||
        !dm::sha_valid(options.attended_tf_asset_sha256) ||
        !dm::sha_valid(options.attended_tf_receipt_sha256) ||
        options.attended_static_edges.size() != 8 || common_frame.empty() ||
        optical_frames[0].empty() || optical_frames[1].empty() ||
        optical_frames[0] == optical_frames[1])
      throw std::invalid_argument("attended_geometry_requires_validated_fixed_asset");

    std::map<std::string, std::string> parents;
    std::set<std::string> frames;
    for (const auto& edge : options.attended_static_edges) {
      const auto& parent = edge.header.frame_id;
      const auto& child = edge.child_frame_id;
      if (parent.empty() || child.empty() || parent == child ||
          !parents.emplace(child, parent).second)
        throw std::invalid_argument("attended_geometry_fixed_tree_conflict");
      frames.insert(parent); frames.insert(child);
      try {
        if (!dm::rigid_valid(dm::rigid(edge.transform)))
          throw std::invalid_argument("invalid rigid transform");
      } catch (const std::exception&) {
        throw std::invalid_argument("attended_geometry_invalid_rigid");
      }
    }
    // BufferCore permits replacing a child and accepts disconnected forests;
    // reject both, and cycles, before asking it to compose the accepted tree.
    std::set<std::string> roots;
    for (const auto& frame : frames) {
      std::set<std::string> path;
      auto cursor = frame;
      for (;;) {
        if (!path.insert(cursor).second)
          throw std::invalid_argument("attended_geometry_fixed_tree_cycle");
        const auto parent = parents.find(cursor);
        if (parent == parents.end()) { roots.insert(cursor); break; }
        cursor = parent->second;
      }
    }
    if (roots.size() != 1)
      throw std::invalid_argument("attended_geometry_fixed_tree_disconnected");
    tf2::BufferCore fixed;
    for (const auto& edge : options.attended_static_edges)
      if (!fixed.setTransform(edge, "validated_attended_fixed_asset", true))
        throw std::invalid_argument("attended_geometry_fixed_edge_rejected");
    std::array<AttendedRoleGeometry, 2> result;
    try {
      // Every edge is explicitly static. TimePointZero here compiles constants;
      // it is never used to substitute latest-time evidence during evaluation.
      for (std::size_t role = 0; role < result.size(); ++role) {
        result[role].optical_from_base = dm::rigid(fixed.lookupTransform(
            optical_frames[role], "base_link", tf2::TimePointZero).transform);
        result[role].common_from_optical = dm::rigid(fixed.lookupTransform(
            common_frame, optical_frames[role], tf2::TimePointZero).transform);
      }
    } catch (const tf2::TransformException&) {
      throw std::invalid_argument("attended_geometry_required_fixed_path_missing");
    }
    return result;
  }

  const std::array<AttendedRoleGeometry, 2> geometry_;
  const std::map<std::string,sunhorizon_rgbd_adapter::dynamic_self_model::geometry::RigidTransform> source_from_base_;
};

}  // namespace sunhorizon_perception_runtime::detail
