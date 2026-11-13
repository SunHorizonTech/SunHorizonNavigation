#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace sunhorizon_obstacle_coverage {
inline constexpr const char* kStrictCameraFreshness = "strict-300ms-v1";
inline constexpr const char* kLowSpeedCameraFreshness = "attended-low-speed-500ms-v1";
inline constexpr const char* kOneSecondCameraFreshness = "attended-low-speed-1000ms-v1";
inline constexpr std::int64_t kOneSecondCameraAgeNs = 1000000000;
inline constexpr std::int64_t kDefaultCameraAgeNs = 300000000;
inline constexpr std::int64_t kSupportedCameraAgeNs = 500000000;
// Explicit startup policy, never a grace period added at receipt or processing.
// The extended policy is an attended low-speed engineering setting, not a
// calibrated stopping-distance guarantee. Startup still requires the measured
// forward-only stopping controller and the original safety heartbeat leases.
inline std::int64_t camera_freshness_age_ns(const std::string& profile) {
  if (profile == kStrictCameraFreshness) return kDefaultCameraAgeNs;
  if (profile == kLowSpeedCameraFreshness) return kSupportedCameraAgeNs;
  if (profile == kOneSecondCameraFreshness) return kOneSecondCameraAgeNs;
  throw std::invalid_argument("unknown_camera_freshness_profile");
}
inline void validate_camera_age(std::int64_t age) {
  if (age != kDefaultCameraAgeNs && age != kSupportedCameraAgeNs && age != kOneSecondCameraAgeNs)
    throw std::invalid_argument("unsupported_camera_freshness_age");
}
inline void validate_low_speed_camera_limits(double vx, double vy, double wz) {
  if (!std::isfinite(vx) || vx <= 0. || vx > .05 ||
      !std::isfinite(vy) || vy != 0. || !std::isfinite(wz) || wz < 0. || wz > .03)
    throw std::invalid_argument("extended_camera_age_requires_attended_low_speed_limits");
}
}  // namespace sunhorizon_obstacle_coverage
