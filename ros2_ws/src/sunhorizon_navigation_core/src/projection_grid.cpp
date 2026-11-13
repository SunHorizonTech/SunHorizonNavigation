#include "sunhorizon_navigation_core/projection_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace sunhorizon_navigation_core {

ProjectionGrid project_points(
    const std::vector<Point3D>& points, const ProjectionConfig& config) {
  if (config.resolution <= 0.0) {
    throw std::invalid_argument("resolution must be positive");
  }

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  std::vector<Point3D> filtered;
  filtered.reserve(points.size());
  for (const auto& point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    const bool ground = point.z >= config.min_ground_z && point.z <= config.max_ground_z;
    const bool obstacle = point.z >= config.min_obstacle_z && point.z <= config.max_obstacle_z;
    if (!ground && !obstacle) continue;
    filtered.push_back(point);
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
  }
  if (filtered.empty()) {
    throw std::invalid_argument("no points remain in ground or obstacle height bands");
  }

  ProjectionGrid grid;
  grid.resolution = config.resolution;
  grid.origin_x = std::floor((min_x - config.padding_m) / config.resolution) * config.resolution;
  grid.origin_y = std::floor((min_y - config.padding_m) / config.resolution) * config.resolution;
  grid.width = static_cast<unsigned int>(
      std::ceil((max_x + config.padding_m - grid.origin_x) / config.resolution)) + 1U;
  grid.height = static_cast<unsigned int>(
      std::ceil((max_y + config.padding_m - grid.origin_y) / config.resolution)) + 1U;

  std::vector<unsigned int> obstacle_counts(
      static_cast<std::size_t>(grid.width) * grid.height, 0U);
  std::vector<unsigned int> ground_counts(obstacle_counts.size(), 0U);
  for (const auto& point : filtered) {
    const auto x = static_cast<unsigned int>((point.x - grid.origin_x) / config.resolution);
    const auto y = static_cast<unsigned int>((point.y - grid.origin_y) / config.resolution);
    if (x < grid.width && y < grid.height) {
      const std::size_t index = static_cast<std::size_t>(y) * grid.width + x;
      if (point.z >= config.min_obstacle_z && point.z <= config.max_obstacle_z) {
        ++obstacle_counts[index];
      }
      if (point.z >= config.min_ground_z && point.z <= config.max_ground_z) {
        ++ground_counts[index];
      }
    }
  }

  // Unknown is the safe default. Only measured ground establishes free space.
  grid.cells.assign(obstacle_counts.size(), -1);
  std::vector<std::size_t> free_cells;
  std::vector<std::size_t> occupied;
  for (std::size_t index = 0; index < obstacle_counts.size(); ++index) {
    if (ground_counts[index] >= config.minimum_ground_points_per_cell) {
      free_cells.push_back(index);
    }
    if (obstacle_counts[index] >= config.minimum_points_per_cell) {
      occupied.push_back(index);
    }
  }

  const int free_radius = static_cast<int>(config.free_space_dilation_radius_cells);
  for (const auto index : free_cells) {
    const int x = static_cast<int>(index % grid.width);
    const int y = static_cast<int>(index / grid.width);
    for (int dy = -free_radius; dy <= free_radius; ++dy) {
      for (int dx = -free_radius; dx <= free_radius; ++dx) {
        const int nx = x + dx;
        const int ny = y + dy;
        if (nx >= 0 && ny >= 0 && nx < static_cast<int>(grid.width) &&
            ny < static_cast<int>(grid.height)) {
          grid.cells[static_cast<std::size_t>(ny) * grid.width + nx] = 0;
        }
      }
    }
  }

  // Obstacles always override ground/free evidence.
  const int radius = static_cast<int>(config.wall_closing_radius_cells);
  for (const auto index : occupied) {
    const int x = static_cast<int>(index % grid.width);
    const int y = static_cast<int>(index / grid.width);
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        const int nx = x + dx;
        const int ny = y + dy;
        if (nx >= 0 && ny >= 0 && nx < static_cast<int>(grid.width) &&
            ny < static_cast<int>(grid.height)) {
          grid.cells[static_cast<std::size_t>(ny) * grid.width + nx] = 100;
        }
      }
    }
  }

  const unsigned int border = std::min(
      config.unknown_border_cells, std::min(grid.width / 2U, grid.height / 2U));
  for (unsigned int y = 0; y < grid.height; ++y) {
    for (unsigned int x = 0; x < grid.width; ++x) {
      if (x < border || y < border || x >= grid.width - border || y >= grid.height - border) {
        const std::size_t index = static_cast<std::size_t>(y) * grid.width + x;
        if (grid.cells[index] != 100) grid.cells[index] = -1;
      }
    }
  }
  return grid;
}

}  // namespace sunhorizon_navigation_core
