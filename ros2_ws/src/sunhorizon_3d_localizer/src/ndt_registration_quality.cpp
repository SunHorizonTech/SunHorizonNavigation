#include "sunhorizon_3d_localizer/ndt_registration_quality.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

namespace sunhorizon_3d_localizer {
namespace {

bool finite_point(const pcl::PointXYZ& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
      std::isfinite(point.z);
}

}  // namespace

void NdtRegistrationQualityIndex::set_target(
    const PointCloud::ConstPtr& target) {
  clear();
  if (!target || target->empty()) return;

  bool all_finite = true;
  for (const auto& point : target->points) {
    if (!finite_point(point)) {
      all_finite = false;
      break;
    }
  }
  if (all_finite) {
    target_ = target;
  } else {
    auto finite_target = PointCloud::Ptr(new PointCloud());
    finite_target->reserve(target->size());
    for (const auto& point : target->points) {
      if (finite_point(point)) finite_target->push_back(point);
    }
    target_ = finite_target;
  }
  if (target_->empty()) {
    clear();
    return;
  }
  search_ = std::make_unique<pcl::KdTreeFLANN<pcl::PointXYZ>>();
  search_->setInputCloud(target_);
}

void NdtRegistrationQualityIndex::clear() {
  search_.reset();
  target_.reset();
}

NdtRegistrationQuality NdtRegistrationQualityIndex::compute(
    const PointCloud& aligned_source, double inlier_max_distance_m) const {
  if (!std::isfinite(inlier_max_distance_m) ||
      inlier_max_distance_m <= 0.0) {
    throw std::invalid_argument(
        "NDT diagnostic inlier distance must be finite and positive");
  }

  NdtRegistrationQuality output;
  output.source_points = aligned_source.size();
  output.target_points = target_ ? target_->size() : 0U;
  if (aligned_source.empty() || !search_ || output.target_points == 0U) {
    return output;
  }
  const double maximum_squared_distance =
      inlier_max_distance_m * inlier_max_distance_m;
  double all_point_squared_distance_sum = 0.0;
  double squared_distance_sum = 0.0;
  std::size_t nearest_neighbor_points = 0U;
  std::vector<int> neighbor_index(1);
  std::vector<float> neighbor_squared_distance(1);
  for (const auto& point : aligned_source.points) {
    if (!finite_point(point) ||
        search_->nearestKSearch(
            point, 1, neighbor_index, neighbor_squared_distance) != 1) {
      continue;
    }
    const double squared_distance = neighbor_squared_distance.front();
    if (!std::isfinite(squared_distance)) continue;
    ++nearest_neighbor_points;
    all_point_squared_distance_sum += squared_distance;
    if (squared_distance > maximum_squared_distance) continue;
    ++output.inlier_points;
    squared_distance_sum += squared_distance;
  }

  if (nearest_neighbor_points == output.source_points) {
    output.all_point_mean_squared_nearest_neighbor_m2 =
        all_point_squared_distance_sum /
        static_cast<double>(output.source_points);
  }
  output.source_inlier_fraction = static_cast<double>(output.inlier_points) /
      static_cast<double>(output.source_points);
  if (output.inlier_points > 0U) {
    output.source_to_target_inlier_rmse_m = std::sqrt(
        squared_distance_sum / static_cast<double>(output.inlier_points));
  }
  return output;
}

NdtRegistrationQuality compute_ndt_registration_quality(
    const pcl::PointCloud<pcl::PointXYZ>& aligned_source,
    const pcl::PointCloud<pcl::PointXYZ>& target,
    double inlier_max_distance_m) {
  auto target_pointer = pcl::PointCloud<pcl::PointXYZ>::ConstPtr(
      new pcl::PointCloud<pcl::PointXYZ>(target));
  NdtRegistrationQualityIndex index;
  index.set_target(target_pointer);
  return index.compute(aligned_source, inlier_max_distance_m);
}

}  // namespace sunhorizon_3d_localizer
