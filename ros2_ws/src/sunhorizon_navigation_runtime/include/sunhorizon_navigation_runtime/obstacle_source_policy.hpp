#pragma once
#include <stdexcept>
#include <string>

namespace sunhorizon_navigation_runtime {
// Routing responsibility only. Sensor/LIO ownership and depth spatial Unknown
// qualification remain independent of this selection.
inline bool obstacle_uses_airy(const std::string& mode) {
  if(mode=="lidar-rgbd-v1")return true;
  if(mode=="rgbd-only-v1")return false;
  throw std::invalid_argument("unsupported_obstacle_source_mode");
}
inline void require_obstacle_coverage_profile(const std::string& mode,
    const std::string& profile,bool spatial_required) {
  if(!spatial_required || profile!=(obstacle_uses_airy(mode)?"fusion_shadow":"visual"))
    throw std::invalid_argument("obstacle_source_coverage_responsibility_mismatch");
}
} // namespace sunhorizon_navigation_runtime
