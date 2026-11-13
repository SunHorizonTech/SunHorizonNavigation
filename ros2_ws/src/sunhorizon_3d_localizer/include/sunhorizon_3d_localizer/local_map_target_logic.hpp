#pragma once

#include <cstdint>

namespace sunhorizon_3d_localizer {

struct LocalMapTargetCenter {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct LocalMapTargetCropLimits {
  double radius_m{0.0};
  double z_half_extent_m{0.0};
};

bool local_map_target_contains(
    double point_x, double point_y, double point_z,
    const LocalMapTargetCenter& center,
    const LocalMapTargetCropLimits& limits) noexcept;

bool local_map_target_refresh_required(
    const LocalMapTargetCenter& current_center,
    const LocalMapTargetCenter& accepted_source_position,
    double refresh_distance_m) noexcept;

bool local_map_target_has_minimum_points(
    std::uint64_t point_count, std::uint64_t minimum_point_count) noexcept;

}  // namespace sunhorizon_3d_localizer
