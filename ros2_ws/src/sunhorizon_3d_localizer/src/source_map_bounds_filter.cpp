#include "sunhorizon_3d_localizer/source_map_bounds_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace sunhorizon_3d_localizer {
namespace {

bool finite_point(const pcl::PointXYZ& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
      std::isfinite(point.z);
}

bool valid_bounds(const MapAxisAlignedBounds& bounds) {
  return std::isfinite(bounds.minimum_x) &&
      std::isfinite(bounds.minimum_y) &&
      std::isfinite(bounds.minimum_z) &&
      std::isfinite(bounds.maximum_x) &&
      std::isfinite(bounds.maximum_y) &&
      std::isfinite(bounds.maximum_z) &&
      bounds.minimum_x <= bounds.maximum_x &&
      bounds.minimum_y <= bounds.maximum_y &&
      bounds.minimum_z <= bounds.maximum_z;
}

bool valid_limits(const SourceMapBoundsFilterLimits& limits) {
  return std::isfinite(limits.margin_m) && limits.margin_m >= 0.0 &&
      std::isfinite(limits.minimum_retained_fraction) &&
      limits.minimum_retained_fraction > 0.0 &&
      limits.minimum_retained_fraction <= 1.0 &&
      limits.minimum_retained_points > 0U;
}

}  // namespace

MapAxisAlignedBounds compute_map_axis_aligned_bounds(
    const pcl::PointCloud<pcl::PointXYZ>& target) {
  if (target.empty()) {
    throw std::invalid_argument("source_map_bounds_target_empty");
  }
  MapAxisAlignedBounds output{
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity()};
  for (const auto& point : target.points) {
    if (!finite_point(point)) {
      throw std::invalid_argument("source_map_bounds_target_non_finite");
    }
    output.minimum_x = std::min(output.minimum_x, static_cast<double>(point.x));
    output.minimum_y = std::min(output.minimum_y, static_cast<double>(point.y));
    output.minimum_z = std::min(output.minimum_z, static_cast<double>(point.z));
    output.maximum_x = std::max(output.maximum_x, static_cast<double>(point.x));
    output.maximum_y = std::max(output.maximum_y, static_cast<double>(point.y));
    output.maximum_z = std::max(output.maximum_z, static_cast<double>(point.z));
  }
  return output;
}

SourceMapBoundsFilterResult filter_source_by_map_bounds(
    const pcl::PointCloud<pcl::PointXYZ>& source,
    const Eigen::Matrix4f& initial_map_to_source,
    const MapAxisAlignedBounds& target_bounds,
    const SourceMapBoundsFilterLimits& limits) {
  if (!valid_bounds(target_bounds)) {
    throw std::invalid_argument("source_map_bounds_target_invalid");
  }
  if (!valid_limits(limits)) {
    throw std::invalid_argument("source_map_bounds_limits_invalid");
  }
  if (!initial_map_to_source.array().isFinite().all()) {
    throw std::invalid_argument("source_map_bounds_initial_transform_non_finite");
  }

  SourceMapBoundsFilterResult output;
  output.input_points = source.size();
  output.retained->reserve(source.size());
  const double minimum_x = target_bounds.minimum_x - limits.margin_m;
  const double minimum_y = target_bounds.minimum_y - limits.margin_m;
  const double minimum_z = target_bounds.minimum_z - limits.margin_m;
  const double maximum_x = target_bounds.maximum_x + limits.margin_m;
  const double maximum_y = target_bounds.maximum_y + limits.margin_m;
  const double maximum_z = target_bounds.maximum_z + limits.margin_m;
  for (const auto& point : source.points) {
    if (!finite_point(point)) continue;
    const Eigen::Vector4f map_point = initial_map_to_source *
        Eigen::Vector4f(point.x, point.y, point.z, 1.0F);
    if (!map_point.array().isFinite().all()) continue;
    if (map_point.x() >= minimum_x && map_point.x() <= maximum_x &&
        map_point.y() >= minimum_y && map_point.y() <= maximum_y &&
        map_point.z() >= minimum_z && map_point.z() <= maximum_z) {
      output.retained->push_back(point);
    }
  }
  output.retained_points = output.retained->size();
  output.rejected_points = output.input_points - output.retained_points;
  if (output.input_points > 0U) {
    output.retained_fraction = static_cast<double>(output.retained_points) /
        static_cast<double>(output.input_points);
  }
  if (output.retained_points < limits.minimum_retained_points) {
    output.reason = "source_map_bounds_retained_points_limit";
    return output;
  }
  if (output.retained_fraction < limits.minimum_retained_fraction) {
    output.reason = "source_map_bounds_retained_fraction_limit";
    return output;
  }
  output.accepted = true;
  output.reason = "ok";
  return output;
}

}  // namespace sunhorizon_3d_localizer
