#pragma once
#include "sunhorizon_rgbd_adapter/depth_projection.hpp"
#include "sunhorizon_rgbd_adapter/body_geometry_cuda.hpp"
#include "sunhorizon_rgbd_adapter/dynamic_self_point_filter.hpp"
namespace sunhorizon_rgbd_adapter {
// Shared exact native-raster admission for BODY and spatial coverage policy.
bool qualified_native_depth_raster(const sensor_msgs::msg::Image&,
    const sensor_msgs::msg::CameraInfo&,double scale);
// Exact original rectified native Image+CameraInfo additionally qualify a native
// +/-0.5 pixel measured-surface interval. Missing calibration/source proof keeps
// legacy center-ray behavior; pixel stride never expands the native cell.
struct FusedDepthSelfFilterResult {
  sensor_msgs::msg::PointCloud2 cloud;
  // On success cloud.data owns
  // all retained bytes, while filter.data is empty (no second output copy).
  DynamicFilterResult filter;
  std::vector<uint64_t> retained_sample_ids;
  struct SelfSupportSample {std::array<float,3> xyz;uint64_t sample_id;};
  // Same original valid samples removed only by complete BODY SelfSurface.
  // Denominator evidence only; never an obstacle, ground endpoint or ray.
  std::vector<SelfSupportSample> self_support;
  double geometry_helper_cpu_ms{0.};
  // Opt-in offline CUDA profiling: original admission/count/projection/packing.
  double cuda_projection_pack_ms{0.};
};
// Preserves the two-stage center-ray filter when native proof is absent.
// Qualified native pixels add the bounded measured-surface interval above.
// Invalid projection inputs throw the same exceptions. Incomplete filtering returns no
// cloud bytes and keeps partial legacy counters. Zero projected points still
// fail, while successfully removing every projected point is complete/empty.
// Model storage and image/calibration/mask must remain immutable for this call.
// The shared model must have been built with exactly budget.query.
FusedDepthSelfFilterResult project_depth_filter_dynamic_self(
    const sensor_msgs::msg::Image& depth, const sensor_msgs::msg::CameraInfo& info,
    double scale, double minimum, double maximum, uint32_t stride,
    const sunhorizon_obstacle_coverage::dynamic_self_geometry::PreparedModel& model,
    double tolerance_m, const DynamicFilterBudget& budget = {},
    const std::vector<uint8_t>* keep = nullptr,
    double mask_max_depth = std::numeric_limits<double>::infinity(),
    BodyGeometryWorker* geometry_worker = nullptr,
    BodyGeometryCuda* geometry_cuda = nullptr,
    double stereo_baseline_m = 0.);
}
