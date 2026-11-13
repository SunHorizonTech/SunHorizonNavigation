#include "sunhorizon_obstacle_coverage/region_depth_support.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sunhorizon_obstacle_coverage {
namespace {
constexpr std::size_t kMaximumRasterElements = 4 * 1024 * 1024;
constexpr double kRigidTolerance = 1.e-6;

template <std::size_t N>
bool finite(const std::array<double, N>& values) {
  return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); });
}

bool rigid(const std::array<double, 9>& r) {
  if (!finite(r)) return false;
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      double dot = 0.;
      for (std::size_t k = 0; k < 3; ++k) dot += r[k * 3 + i] * r[k * 3 + j];
      if (!std::isfinite(dot) || std::abs(dot - (i == j ? 1. : 0.)) > kRigidTolerance) return false;
    }
  }
  const double det = r[0] * (r[4] * r[8] - r[5] * r[7]) -
    r[1] * (r[3] * r[8] - r[5] * r[6]) + r[2] * (r[3] * r[7] - r[4] * r[6]);
  return std::isfinite(det) && std::abs(det - 1.) <= kRigidTolerance;
}

enum class Intersection { kMiss, kHit, kInvalid };
Intersection intersect(const RegionDepthInput& in, const std::array<double, 3>& ray,
                       double* entry, double* exit) {
  if (!finite(ray)) return Intersection::kInvalid;
  *entry = 0.;
  *exit = std::numeric_limits<double>::infinity();
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double origin = in.optical_to_base_translation[axis];
    if (ray[axis] == 0.) {
      if (origin < in.box_min[axis] || origin > in.box_max[axis]) return Intersection::kMiss;
      continue;
    }
    double a = (in.box_min[axis] - origin) / ray[axis];
    double b = (in.box_max[axis] - origin) / ray[axis];
    if (!std::isfinite(a) || !std::isfinite(b)) return Intersection::kInvalid;
    if (a > b) std::swap(a, b);
    *entry = std::max(*entry, a);
    *exit = std::min(*exit, b);
    if (*entry > *exit) return Intersection::kMiss;
  }
  if (!std::isfinite(*entry) || !std::isfinite(*exit)) return Intersection::kInvalid;
  return *exit > 0. ? Intersection::kHit : Intersection::kMiss;
}
}  // namespace

RegionDepthSupport evaluate_region_depth_support(const RegionDepthInput& in) {
  RegionDepthSupport out;
  auto unknown = [&out](const char* reason) { out.reason = reason; return out; };
  const auto& c = in.contract;
  if (!(c.rectified_pinhole_verified && c.metric_axial_depth_verified &&
        c.depth_intrinsics_registration_verified && c.optical_to_base_extrinsics_verified &&
        c.source_time_and_transform_verified)) return unknown("unverified_input_contract");
  if (in.width == 0 || in.height == 0 || in.width > 4096 || in.height > 4096 ||
      in.depth_m == nullptr || in.row_stride_elements < in.width ||
      in.row_stride_elements > kMaximumRasterElements ||
      in.height > kMaximumRasterElements / in.row_stride_elements ||
      in.depth_elements < (in.height - 1) * in.row_stride_elements + in.width ||
      in.depth_elements > kMaximumRasterElements) return unknown("invalid_or_unbounded_raster");
  if (!(std::isfinite(in.fx) && in.fx > 0. && std::isfinite(in.fy) && in.fy > 0. &&
        std::isfinite(in.cx) && std::isfinite(in.cy) &&
        std::isfinite(in.minimum_depth_m) && in.minimum_depth_m > 0. &&
        std::isfinite(in.maximum_depth_m) && in.maximum_depth_m > in.minimum_depth_m &&
        std::isfinite(in.clear_margin_m) && in.clear_margin_m >= 0.)) {
    return unknown("invalid_intrinsics_or_depth_limits");
  }
  if (!finite(in.box_min) || !finite(in.box_max)) return unknown("invalid_box");
  for (std::size_t i = 0; i < 3; ++i) {
    if (in.box_min[i] >= in.box_max[i]) return unknown("invalid_box");
  }
  if (!rigid(in.optical_to_base_rotation) || !finite(in.optical_to_base_translation)) {
    return unknown("invalid_rigid_transform");
  }

  // Positive optical Z makes the convex box's projected extrema occur at its
  // vertices. Vertices bound the exhaustive pixel search; they are NOT depth
  // samples or a substitute for the per-pixel checks below.
  double umin = std::numeric_limits<double>::infinity(), vmin = umin;
  double umax = -umin, vmax = -umin;
  for (unsigned corner = 0; corner < 8; ++corner) {
    std::array<double, 3> optical{};
    for (std::size_t i = 0; i < 3; ++i) {
      for (std::size_t j = 0; j < 3; ++j) {
        const double base = (corner & (1U << j)) ? in.box_max[j] : in.box_min[j];
        optical[i] += in.optical_to_base_rotation[j * 3 + i] *
          (base - in.optical_to_base_translation[j]);
      }
    }
    if (!finite(optical)) return unknown("nonfinite_projection");
    if (optical[2] <= 0.) return unknown("box_behind_or_crossing_projection_plane");
    const double u = in.fx * (optical[0] / optical[2]) + in.cx;
    const double v = in.fy * (optical[1] / optical[2]) + in.cy;
    if (!std::isfinite(u) || !std::isfinite(v)) return unknown("nonfinite_projection");
    if (u < -.5 || v < -.5 || u > in.width - .5 || v > in.height - .5) {
      return unknown("box_outside_image_fov");
    }
    umin = std::min(umin, u); umax = std::max(umax, u);
    vmin = std::min(vmin, v); vmax = std::max(vmax, v);
  }
  const int u0 = std::max(0, static_cast<int>(std::floor(umin)));
  const int u1 = std::min(static_cast<int>(in.width) - 1, static_cast<int>(std::ceil(umax)));
  const int v0 = std::max(0, static_cast<int>(std::floor(vmin)));
  const int v1 = std::min(static_cast<int>(in.height) - 1, static_cast<int>(std::ceil(vmax)));
  for (int v = v0; v <= v1; ++v) {
    for (int u = u0; u <= u1; ++u) {
      ++out.tested_pixels;
      const std::array<double, 3> optical_ray{(u - in.cx) / in.fx, (v - in.cy) / in.fy, 1.};
      std::array<double, 3> ray{};
      for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) ray[i] += in.optical_to_base_rotation[i * 3 + j] * optical_ray[j];
      }
      double entry = 0., exit = 0.;
      const auto hit = intersect(in, ray, &entry, &exit);
      if (hit == Intersection::kInvalid) return unknown("nonfinite_ray_intersection");
      if (hit == Intersection::kMiss) continue;
      ++out.intersecting_rays;
      const double depth = in.depth_m[static_cast<std::size_t>(v) * in.row_stride_elements + u];
      if (!std::isfinite(depth) || depth < in.minimum_depth_m || depth > in.maximum_depth_m) {
        ++out.invalid_depth_rays;
      } else if (depth < entry) {
        ++out.front_occluded_rays;
      } else if (depth <= exit) {
        ++out.obstacle_rays;
      } else if (!std::isfinite(exit + in.clear_margin_m)) {
        return unknown("nonfinite_clear_margin_boundary");
      } else if (depth >= exit + in.clear_margin_m) {
        ++out.clear_rays;
      } else {
        ++out.margin_band_rays;
      }
    }
  }
  if (out.intersecting_rays == 0) return unknown("no_intersecting_pixel_centres");
  // Only the completed, bounded raster traversal can attest a closed result.
  // Each hit above contributes to exactly one count; early numerical rejection
  // retains the default incomplete/Rejected/HardFault even with partial counts.
  const auto classified = out.clear_rays + out.obstacle_rays + out.invalid_depth_rays +
      out.front_occluded_rays + out.margin_band_rays;
  if (classified != out.intersecting_rays || out.intersecting_rays > out.tested_pixels)
    return unknown("inconsistent_ray_counts");
  out.evaluation_complete = true;
  if (out.invalid_depth_rays || out.front_occluded_rays || out.margin_band_rays) {
    out.evaluation_code = RegionEvaluationCode::kIncompleteRays;
    if (out.margin_band_rays && !out.invalid_depth_rays &&
        !out.front_occluded_rays && !out.obstacle_rays)
      out.disposition = RegionDisposition::kGeometryHold;
    // Counts retain every cause; a nearer obstacle or invalid pixel must not be
    // hidden by the majority of clear samples or by another valid obstacle.
    return unknown("incomplete_ray_support");
  }
  out.state = out.obstacle_rays ? RegionSupportState::kObstacleSupported : RegionSupportState::kClearSupported;
  out.reason = out.obstacle_rays ? "all_rays_supported_with_obstacle" : "all_rays_supported_clear";
  out.evaluation_code = out.obstacle_rays ? RegionEvaluationCode::kObstacle : RegionEvaluationCode::kClear;
  out.disposition = out.obstacle_rays ? RegionDisposition::kHardFault : RegionDisposition::kClear;
  return out;
}
}  // namespace sunhorizon_obstacle_coverage
