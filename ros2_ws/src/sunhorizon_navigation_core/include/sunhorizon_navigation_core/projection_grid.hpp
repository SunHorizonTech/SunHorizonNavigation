#pragma once

#include <cstdint>
#include <vector>

#include "sunhorizon_navigation_core/types.hpp"

namespace sunhorizon_navigation_core {

struct ProjectionConfig {
  double resolution{0.05};
  double min_ground_z{-0.25};
  double max_ground_z{0.10};
  double min_obstacle_z{0.10};
  double max_obstacle_z{1.80};
  double padding_m{1.0};
  unsigned int minimum_ground_points_per_cell{1};
  unsigned int minimum_points_per_cell{2};
  unsigned int free_space_dilation_radius_cells{1};
  unsigned int wall_closing_radius_cells{1};
  unsigned int unknown_border_cells{1};
};

struct ProjectionGrid {
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  unsigned int width{0};
  unsigned int height{0};
  std::vector<std::int8_t> cells;  // -1 unknown, 0 free, 100 occupied
};

ProjectionGrid project_points(
    const std::vector<Point3D>& points, const ProjectionConfig& config);

}  // namespace sunhorizon_navigation_core
