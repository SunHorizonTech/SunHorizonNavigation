#pragma once

#include <string>

#include "sunhorizon_navigation_core/types.hpp"

namespace sunhorizon_navigation_core {

struct VisualPoseLimits {
  double max_translation_jump_m{0.30};
  double max_rotation_jump_rad{0.2617993877991494};
  double max_input_age_sec{0.25};
};

struct VisualPoseDecision {
  bool accepted{false};
  double translation_jump_m{0.0};
  double rotation_jump_rad{0.0};
  std::string reason;
};

struct PlanarPoseDecomposition {
  Pose3D odom_to_base_footprint;
  Pose3D base_footprint_to_base_link;
};

class VisualPoseGate {
 public:
  explicit VisualPoseGate(VisualPoseLimits limits = {});
  VisualPoseDecision evaluate(double now_sec, double stamp_sec, const Pose3D& pose);

 private:
  VisualPoseLimits limits_;
  bool have_previous_{false};
  double previous_stamp_sec_{0.0};
  Pose3D previous_pose_;
};

PlanarPoseDecomposition decompose_visual_pose(const Pose3D& pose);

}  // namespace sunhorizon_navigation_core
