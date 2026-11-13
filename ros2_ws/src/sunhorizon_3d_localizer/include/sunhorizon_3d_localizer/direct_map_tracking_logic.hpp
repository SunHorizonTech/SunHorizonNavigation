#pragma once

#include <cstdint>
#include <string>

#include <Eigen/Core>

namespace sunhorizon_3d_localizer {

struct DirectMapTrackingOutput {
  Eigen::Matrix4d map_to_source{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d planar_map_to_base{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d map_equivalent_odom_to_base{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d map_to_odom{Eigen::Matrix4d::Identity()};
};

struct DirectMapTrackingLimits {
  double maximum_translation_step_m{0.30};
  double maximum_rotation_step_rad{0.2617993877991494};
  double maximum_translation_rate_mps{2.0};
  double maximum_rotation_rate_radps{2.0};
};

struct DirectMapTrackingDecision {
  bool accepted{false};
  bool has_previous_measurement{false};
  double dt_sec{0.0};
  double translation_step_m{0.0};
  double rotation_step_rad{0.0};
  double translation_rate_mps{0.0};
  double rotation_rate_radps{0.0};
  std::string reason;
  DirectMapTrackingOutput output;
};

// Source time must increase; elapsed time determines measured rates, without
// imposing an inter-match deadline. Source/result freshness remains the
// localizer owner's responsibility, independently of these spatial gates.
class DirectMapTrackingState {
 public:
  void reset(
      const Eigen::Matrix4d& accepted_initial_map_to_source,
      const Eigen::Matrix4d& source_to_base);

  const Eigen::Matrix4d& next_initial_guess() const;
  DirectMapTrackingDecision evaluate(
      std::int64_t stamp_ns,
      const Eigen::Matrix4d& candidate_map_to_source,
      const DirectMapTrackingLimits& limits);

  bool initialized() const { return initialized_; }

 private:
  bool initialized_{false};
  bool accepted_measurement_valid_{false};
  std::int64_t last_accepted_stamp_ns_{0};
  Eigen::Matrix4d source_to_base_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d last_accepted_map_to_source_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d last_accepted_planar_map_to_base_{Eigen::Matrix4d::Identity()};
};

}  // namespace sunhorizon_3d_localizer
