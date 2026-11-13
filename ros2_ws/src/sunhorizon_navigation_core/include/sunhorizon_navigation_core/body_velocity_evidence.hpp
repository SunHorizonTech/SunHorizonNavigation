#pragma once
#include <cstdint>

namespace sunhorizon_navigation_core {
// The pose is at window_end_ns. The velocity is a causal window estimate whose
// effective observation time is retained separately; it is never restamped as
// an instantaneous measurement. Reset epochs cannot share estimator history.
struct BodyVelocityEvidence {
  int64_t window_begin_ns{0}, window_end_ns{0}, effective_source_ns{0};
  uint64_t reset_epoch{0};
  uint32_t sample_count{0};
  double variance_vx{0.0}, variance_vy{0.0}, variance_wz{0.0};
  bool valid{false};
  bool measured_waist{false};
  // Fixed vertical datum selected by the canonical owner in this explicit
  // odometry epoch. It is not a measured floor/foot height.
  int64_t odom_z_origin_source_ns{0};
  uint64_t odom_z_origin_epoch{0};
  double odom_z_origin_m{0.0};
};
}  // namespace sunhorizon_navigation_core
