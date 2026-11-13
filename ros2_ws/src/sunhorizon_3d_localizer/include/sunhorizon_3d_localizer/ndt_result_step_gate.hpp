#pragma once

#include <cstdint>
#include <limits>
#include <string>

#include <Eigen/Core>

namespace sunhorizon_3d_localizer {

struct NdtResultStepLimits {
  double maximum_gap_sec{0.50};
  double maximum_translation_step_m{0.05};
  double maximum_yaw_step_rad{0.03490658503988659};
};

// Only explicit attended startup may select the 10 cm candidate envelope.
// This does not change quality, yaw, timing, or output-authority gates.
bool valid_result_translation_step_limit(double limit_m, bool attended_profile);

struct NdtResultStepDecision {
  bool accepted{false};
  bool has_baseline{false};
  bool establishes_baseline{false};
  bool gap_recovered{false};
  double dt_sec{std::numeric_limits<double>::quiet_NaN()};
  double translation_step_m{std::numeric_limits<double>::quiet_NaN()};
  double yaw_step_rad{std::numeric_limits<double>::quiet_NaN()};
  std::string reason;
};

// Mirrors the localization manager's canonical TF decision from the exact
// planar base poses published as paired NDT evidence.  This is deliberately
// distinct from the raw 3-D result * inverse(prior) correction retained by the
// NDT node for its next registration initial guess.
Eigen::Matrix4d canonical_map_to_odom_from_planar_base_poses(
    const Eigen::Matrix4d& planar_map_to_base,
    const Eigen::Matrix4d& planar_odom_to_base);

// Admits a bounded planar step between fully accepted NDT corrections.  The
// caller supplies map->odom, not the robot's map pose: real robot motion is
// already represented by the continuous odometry prior and must not be
// mistaken for a localization jump.  A rejected step never advances the
// baseline.  A long result gap is reported but can recover automatically only
// when the new correction remains inside the same strict translation/yaw
// envelope; liveness is enforced independently by the result timeout.  The
// caller must skip this gate in direct-map mode, where canonical map->odom is
// identically identity and DirectMapTrackingState owns the explicit-reset gap
// contract.
class NdtResultStepGate {
 public:
  void reset();

  NdtResultStepDecision evaluate(
      std::int64_t stamp_ns,
      const Eigen::Matrix4d& candidate_map_to_odom,
      const NdtResultStepLimits& limits);

  bool has_baseline() const { return baseline_valid_; }

 private:
  bool baseline_valid_{false};
  std::int64_t baseline_stamp_ns_{0};
  Eigen::Matrix4d baseline_map_to_odom_{Eigen::Matrix4d::Identity()};
};

}  // namespace sunhorizon_3d_localizer
