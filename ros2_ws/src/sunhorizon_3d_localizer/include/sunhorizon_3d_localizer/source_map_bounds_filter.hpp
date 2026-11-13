#pragma once

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <string>

namespace sunhorizon_3d_localizer {

struct MapAxisAlignedBounds {
  double minimum_x{0.0};
  double minimum_y{0.0};
  double minimum_z{0.0};
  double maximum_x{0.0};
  double maximum_y{0.0};
  double maximum_z{0.0};
};

struct SourceMapBoundsFilterLimits {
  double margin_m{1.0};
  double minimum_retained_fraction{0.6};
  std::size_t minimum_retained_points{500};
};

struct SourceMapBoundsFilterResult {
  pcl::PointCloud<pcl::PointXYZ>::Ptr retained{
      new pcl::PointCloud<pcl::PointXYZ>()};
  std::size_t input_points{0};
  std::size_t retained_points{0};
  std::size_t rejected_points{0};
  double retained_fraction{0.0};
  bool accepted{false};
  std::string reason{"source_map_bounds_overlap_unavailable"};
};

// Returns a finite AABB for a finite, nonempty target. Invalid targets throw;
// silently accepting an unusable map would weaken the pre-alignment gate.
MapAxisAlignedBounds compute_map_axis_aligned_bounds(
    const pcl::PointCloud<pcl::PointXYZ>& target);

// Projects every source point through the initial map<-source estimate, but
// returns retained points in their original source frame for NDT alignment.
// The decision requires both cardinality and retained-fraction thresholds so a
// tiny in-map subset cannot manufacture a healthy registration.
SourceMapBoundsFilterResult filter_source_by_map_bounds(
    const pcl::PointCloud<pcl::PointXYZ>& source,
    const Eigen::Matrix4f& initial_map_to_source,
    const MapAxisAlignedBounds& target_bounds,
    const SourceMapBoundsFilterLimits& limits);

}  // namespace sunhorizon_3d_localizer
