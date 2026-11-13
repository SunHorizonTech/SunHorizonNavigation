#include "sunhorizon_3d_localizer/local_map_target_logic.hpp"

#include <cmath>

namespace sunhorizon_3d_localizer {

bool local_map_target_contains(
    double point_x, double point_y, double point_z,
    const LocalMapTargetCenter& center,
    const LocalMapTargetCropLimits& limits) noexcept {
  if (!std::isfinite(point_x) || !std::isfinite(point_y) ||
      !std::isfinite(point_z) || !std::isfinite(center.x) ||
      !std::isfinite(center.y) || !std::isfinite(center.z) ||
      !std::isfinite(limits.radius_m) || limits.radius_m <= 0.0 ||
      !std::isfinite(limits.z_half_extent_m) ||
      limits.z_half_extent_m <= 0.0) {
    return false;
  }
  const double dx = point_x - center.x;
  const double dy = point_y - center.y;
  const double dz = std::abs(point_z - center.z);
  return dx * dx + dy * dy <= limits.radius_m * limits.radius_m &&
      dz <= limits.z_half_extent_m;
}

bool local_map_target_refresh_required(
    const LocalMapTargetCenter& current_center,
    const LocalMapTargetCenter& accepted_source_position,
    double refresh_distance_m) noexcept {
  if (!std::isfinite(current_center.x) || !std::isfinite(current_center.y) ||
      !std::isfinite(accepted_source_position.x) ||
      !std::isfinite(accepted_source_position.y) ||
      !std::isfinite(refresh_distance_m) || refresh_distance_m <= 0.0) {
    return false;
  }
  const double dx = accepted_source_position.x - current_center.x;
  const double dy = accepted_source_position.y - current_center.y;
  return dx * dx + dy * dy >= refresh_distance_m * refresh_distance_m;
}

bool local_map_target_has_minimum_points(
    std::uint64_t point_count, std::uint64_t minimum_point_count) noexcept {
  return minimum_point_count > 0U && point_count >= minimum_point_count;
}

}  // namespace sunhorizon_3d_localizer
