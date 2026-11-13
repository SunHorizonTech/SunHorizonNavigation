#pragma once

#include <optional>
#include <string>

#include "sunhorizon_navigation_core/types.hpp"

namespace sunhorizon_navigation_core {

struct LocalizationLimits {
  double measurement_timeout_sec{0.50};
  double minimum_score{2.0};
  unsigned int maximum_iterations{30};
  double maximum_initial_to_result_distance_m{1.0};
  double maximum_position_covariance{0.04};
  double maximum_yaw_covariance{0.01};
  double maximum_position_jump_m{0.50};
  double maximum_yaw_jump_rad{0.35};
};

struct LocalizationMeasurement {
  double now_sec{0.0};
  double stamp_sec{0.0};
  Pose3D pose;
  bool converged{false};
  double score{0.0};
  unsigned int iterations{0};
  double initial_to_result_distance_m{0.0};
  double position_covariance{0.0};
  double yaw_covariance{0.0};
};

struct LocalizationDecision {
  bool accepted{false};
  std::string reason;
};

class LocalizationGate {
 public:
  explicit LocalizationGate(LocalizationLimits limits = {});
  LocalizationDecision evaluate(const LocalizationMeasurement& measurement);
  void reset();

 private:
  LocalizationLimits limits_;
  std::optional<Pose3D> last_pose_;
};

}  // namespace sunhorizon_navigation_core
