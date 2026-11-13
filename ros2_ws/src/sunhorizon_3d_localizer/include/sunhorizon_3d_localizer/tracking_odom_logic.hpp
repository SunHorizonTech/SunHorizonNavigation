#pragma once

#include <cstdint>
#include <string>

#include <Eigen/Core>

namespace sunhorizon_3d_localizer {

struct TrackingOdomLimits {
  double maximum_gap_sec{0.25};
  double maximum_translation_rate_mps{2.0};
  double maximum_rotation_rate_radps{2.0};
  double maximum_translation_step_m{0.30};
  double maximum_rotation_step_rad{0.2617993877991494};
};

struct TrackingOdomSample {
  std::int64_t stamp_ns{0};
  std::string frame_id;
  std::string child_frame_id;
  Eigen::Matrix4d raw_odom_to_source{Eigen::Matrix4d::Identity()};
};

struct TrackingOdomDecision {
  bool accepted{false};
  bool latched{false};
  bool has_previous{false};
  double dt_sec{0.0};
  double translation_step_m{0.0};
  double rotation_step_rad{0.0};
  std::string reason;
  Eigen::Matrix4d odom_to_base{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d planar_odom_to_base{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d planar_relative_motion{Eigen::Matrix4d::Identity()};
};

struct TransformStepDecision {
  bool accepted{false};
  double translation_step_m{0.0};
  double rotation_step_rad{0.0};
  std::string reason;
};

bool finite_transform(const Eigen::Matrix4d& transform);
double rotation_distance(
    const Eigen::Matrix4d& first, const Eigen::Matrix4d& second);
Eigen::Matrix4d planarize(const Eigen::Matrix4d& transform);
Eigen::Matrix4d planarize_with_z_reference(
    const Eigen::Matrix4d& transform, double z_reference_m);

TransformStepDecision evaluate_transform_step(
    const Eigen::Matrix4d& previous,
    const Eigen::Matrix4d& current,
    double maximum_translation_step_m,
    double maximum_rotation_step_rad);

class TrackingOdomGate {
 public:
  TrackingOdomGate(
      std::string expected_frame,
      std::string expected_child_frame,
      Eigen::Matrix4d prior_basis,
      Eigen::Matrix4d source_to_base,
      TrackingOdomLimits limits);

  TrackingOdomDecision evaluate(const TrackingOdomSample& sample);

  bool latched() const { return latched_; }
  const std::string& latched_reason() const { return latched_reason_; }

 private:
  TrackingOdomDecision reject(const std::string& reason);

  std::string expected_frame_;
  std::string expected_child_frame_;
  Eigen::Matrix4d prior_basis_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d source_to_base_{Eigen::Matrix4d::Identity()};
  TrackingOdomLimits limits_;
  bool latched_{false};
  bool previous_valid_{false};
  std::string latched_reason_;
  std::int64_t previous_stamp_ns_{0};
  Eigen::Matrix4d previous_odom_to_base_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d previous_planar_odom_to_base_{Eigen::Matrix4d::Identity()};
};

}  // namespace sunhorizon_3d_localizer
