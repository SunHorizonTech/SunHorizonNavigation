#pragma once

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <limits>
#include <memory>

namespace sunhorizon_3d_localizer {

// One-way source-to-target nearest-neighbor diagnostics for an already
// transformed NDT source cloud. The fraction denominator is every source point;
// invalid or unmatched source points are therefore non-inliers. RMSE is metres
// and includes inliers only. These values are observational only: PCL's
// mean-squared nearest-neighbor fitness over the actual alignment source
// remains the acceptance gate.
struct NdtRegistrationQuality {
  std::size_t source_points{0};
  std::size_t target_points{0};
  std::size_t inlier_points{0};
  // Mean squared nearest-neighbor distance over every source point. This is
  // deliberately unbounded by the diagnostic inlier radius.
  double all_point_mean_squared_nearest_neighbor_m2{
      std::numeric_limits<double>::quiet_NaN()};
  double source_inlier_fraction{std::numeric_limits<double>::quiet_NaN()};
  double source_to_target_inlier_rmse_m{
      std::numeric_limits<double>::quiet_NaN()};
};

// The target map changes much less often than the source scan. Build the
// nearest-neighbor index only when NDT changes its actual input target, not on
// every match.
class NdtRegistrationQualityIndex {
 public:
  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

  void set_target(const PointCloud::ConstPtr& target);
  void clear();
  NdtRegistrationQuality compute(
      const PointCloud& aligned_source, double inlier_max_distance_m) const;

 private:
  PointCloud::ConstPtr target_;
  std::unique_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> search_;
};

NdtRegistrationQuality compute_ndt_registration_quality(
    const pcl::PointCloud<pcl::PointXYZ>& aligned_source,
    const pcl::PointCloud<pcl::PointXYZ>& target,
    double inlier_max_distance_m);

}  // namespace sunhorizon_3d_localizer
