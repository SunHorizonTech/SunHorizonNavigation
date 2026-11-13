#include "sunhorizon_obstacle_coverage/region_depth_support.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

using namespace sunhorizon_obstacle_coverage;
#define CHECK(condition) do { if (!(condition)) throw std::runtime_error( \
  std::string(__func__) + ":" + std::to_string(__LINE__) + ": " #condition); } while (false)

namespace {
struct Fixture {
  std::vector<float> pixels = std::vector<float>(25, 3.F);
  RegionDepthInput in;
  Fixture() {
    in.box_min = {-.25, -.25, 1.}; in.box_max = {.25, .25, 2.};
    in.width = in.height = 5; in.fx = in.fy = 4.; in.cx = in.cy = 2.;
    in.depth_m = pixels.data(); in.depth_elements = pixels.size(); in.row_stride_elements = 5;
    in.optical_to_base_rotation = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
    in.minimum_depth_m = .25; in.maximum_depth_m = 5.; in.clear_margin_m = .25;
    in.contract = {true, true, true, true, true};
  }
  RegionDepthSupport run() const {
    const auto r = evaluate_region_depth_support(in);
    if (r.evaluation_complete) {
      CHECK(r.intersecting_rays > 0 && r.intersecting_rays <= r.tested_pixels);
      CHECK(r.clear_rays + r.obstacle_rays + r.invalid_depth_rays +
            r.front_occluded_rays + r.margin_band_rays == r.intersecting_rays);
      CHECK(r.evaluation_code != RegionEvaluationCode::kRejected);
    } else {
      CHECK(r.evaluation_code == RegionEvaluationCode::kRejected);
      CHECK(r.disposition == RegionDisposition::kHardFault);
    }
    return r;
  }
};

void exhaustive_rays_preserve_single_pixel_obstacle_and_hole() {
  Fixture f;
  auto r = f.run();
  // Analytic geometry: only u,v=1,2,3 hit this box, including grazing edges.
  CHECK(r.state == RegionSupportState::kClearSupported);
  CHECK(r.intersecting_rays == 9 && r.clear_rays == 9);
  f.pixels[12] = 1.5F;
  r = f.run();
  CHECK(r.state == RegionSupportState::kObstacleSupported);
  CHECK(r.obstacle_rays == 1 && r.clear_rays == 8);
  f.pixels[11] = 0.F;
  r = f.run();
  CHECK(r.state == RegionSupportState::kUnknown);
  CHECK(r.obstacle_rays == 1 && r.invalid_depth_rays == 1);
  CHECK(r.reason == "incomplete_ray_support");
}

void endpoints_margin_and_front_occlusion_are_distinct() {
  Fixture f;
  for (float endpoint : {1.F, 2.F}) {
    f.pixels[12] = endpoint;
    CHECK(f.run().obstacle_rays == 1);
    CHECK(f.run().state == RegionSupportState::kObstacleSupported);
  }
  f.pixels[12] = .5F;
  CHECK(f.run().front_occluded_rays == 1 && f.run().state == RegionSupportState::kUnknown);
  f.pixels[12] = 2.125F;
  CHECK(f.run().margin_band_rays == 1 && f.run().state == RegionSupportState::kUnknown);
  f.pixels[12] = 2.25F;
  CHECK(f.run().state == RegionSupportState::kClearSupported);
  f.in.clear_margin_m = 0.; f.pixels[12] = 2.F;
  CHECK(f.run().state == RegionSupportState::kObstacleSupported);  // equality is never free
  f.pixels[12] = std::nextafter(2.F, 3.F);
  CHECK(f.run().state == RegionSupportState::kClearSupported);
}

void every_invalid_depth_is_unknown_and_outside_pixels_do_not_vote() {
  Fixture f;
  for (float invalid : {0.F, -.1F, .125F, 6.F,
       std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()}) {
    f.pixels[12] = invalid;
    CHECK(f.run().invalid_depth_rays == 1 && f.run().state == RegionSupportState::kUnknown);
  }
  f.pixels[12] = 3.F; f.pixels[0] = 0.F;
  CHECK(f.run().state == RegionSupportState::kClearSupported);
  f.in.maximum_depth_m = 3.;
  CHECK(f.run().state == RegionSupportState::kClearSupported);  // inclusive sensor validity
}

void axial_depth_is_not_euclidean_range() {
  Fixture f;
  f.pixels.assign(81, 3.F);
  f.in.depth_m = f.pixels.data(); f.in.depth_elements = f.pixels.size();
  f.in.width = f.in.height = 9; f.in.row_stride_elements = 9; f.in.cx = f.in.cy = 4.;
  f.in.box_min = {.95, -.1, 1.}; f.in.box_max = {1.1, .1, 1.2};
  // Ray (1,0,1) has axial interval [1,1.1], different from range [sqrt(2),1.1sqrt(2)].
  f.pixels[4 * 9 + 8] = 1.05F;
  const auto r = f.run();
  CHECK(r.intersecting_rays == 1 && r.obstacle_rays == 1);
  CHECK(r.state == RegionSupportState::kObstacleSupported);
}

void rotated_translated_optical_frame_keeps_same_physical_rays() {
  Fixture f;
  f.in.optical_to_base_rotation = {0., 0., 1., 1., 0., 0., 0., 1., 0.};
  f.in.optical_to_base_translation = {3., -2., .5};
  f.in.box_min = {4., -2.25, .25}; f.in.box_max = {5., -1.75, .75};
  f.pixels[12] = 1.5F;
  const auto r = f.run();
  CHECK(r.intersecting_rays == 9 && r.obstacle_rays == 1 && r.clear_rays == 8);
  CHECK(r.state == RegionSupportState::kObstacleSupported);
}

void fov_half_pixel_boundary_is_explicit_and_not_expandable() {
  Fixture f;
  f.in.box_min[0] = -.625;  // fx*(-.625/1)+cx == -.5, exactly the physical pixel edge
  CHECK(f.run().state == RegionSupportState::kClearSupported);
  f.in.box_min[0] = std::nextafter(-.625, -1.);
  CHECK(f.run().reason == "box_outside_image_fov");
  f.in.box_min[0] = -.25; f.in.box_max[1] = .626;
  CHECK(f.run().reason == "box_outside_image_fov");
}

void projection_plane_and_subpixel_unsampled_box_are_unknown() {
  Fixture f;
  f.in.box_min[2] = -2.; f.in.box_max[2] = -1.;
  CHECK(f.run().reason == "box_behind_or_crossing_projection_plane");
  f.in.box_max[2] = 2.;
  CHECK(f.run().reason == "box_behind_or_crossing_projection_plane");
  f.in.box_min[2] = 0.;
  CHECK(f.run().reason == "box_behind_or_crossing_projection_plane");
  f.in.box_min = {.01, .01, 1.}; f.in.box_max = {.02, .02, 1.1};
  CHECK(f.run().reason == "no_intersecting_pixel_centres");
  CHECK(f.run().intersecting_rays == 0);
}

void calibration_contract_defaults_fail_closed_individually() {
  Fixture f;
  RegionCalibrationContract defaults;
  f.in.contract = defaults;
  CHECK(f.run().reason == "unverified_input_contract");
  for (auto field : {&RegionCalibrationContract::rectified_pinhole_verified,
       &RegionCalibrationContract::metric_axial_depth_verified,
       &RegionCalibrationContract::depth_intrinsics_registration_verified,
       &RegionCalibrationContract::optical_to_base_extrinsics_verified,
       &RegionCalibrationContract::source_time_and_transform_verified}) {
    f.in.contract = {true, true, true, true, true}; f.in.contract.*field = false;
    CHECK(f.run().reason == "unverified_input_contract");
  }
}

void malformed_layout_rejected_and_row_padding_never_sampled() {
  Fixture f;
  f.in.row_stride_elements = 4;
  CHECK(f.run().reason == "invalid_or_unbounded_raster");
  f.in.row_stride_elements = 5; f.in.depth_elements = 24;
  CHECK(f.run().reason == "invalid_or_unbounded_raster");
  f.in.depth_elements = 25; f.in.depth_m = nullptr;
  CHECK(f.run().reason == "invalid_or_unbounded_raster");
  f.in.depth_m = f.pixels.data(); f.in.row_stride_elements = std::numeric_limits<std::size_t>::max();
  CHECK(f.run().reason == "invalid_or_unbounded_raster");
  f.in.row_stride_elements = 5; f.in.height = std::numeric_limits<std::uint32_t>::max();
  CHECK(f.run().reason == "invalid_or_unbounded_raster");
  f.in.height = 5; f.in.depth_elements = 4 * 1024 * 1024 + 1;
  CHECK(f.run().reason == "invalid_or_unbounded_raster");
  f.pixels.assign(35, 0.F);
  for (int v = 0; v < 5; ++v) for (int u = 0; u < 5; ++u) f.pixels[v * 7 + u] = 3.F;
  f.in.depth_m = f.pixels.data(); f.in.depth_elements = 35; f.in.row_stride_elements = 7;
  CHECK(f.run().state == RegionSupportState::kClearSupported);
}

void malformed_geometry_and_numerical_overflow_fail_closed() {
  Fixture f;
  for (auto r : {std::array<double, 9>{-1.,0.,0.,0.,1.,0.,0.,0.,1.},
                 std::array<double, 9>{1.,.1,0.,0.,1.,0.,0.,0.,1.},
                 std::array<double, 9>{1.01,0.,0.,0.,1.,0.,0.,0.,1.}}) {
    f.in.optical_to_base_rotation = r;
    CHECK(f.run().reason == "invalid_rigid_transform");
  }
  f.in.optical_to_base_rotation = {1.,0.,0.,0.,1.,0.,0.,0.,1.};
  f.in.optical_to_base_translation[0] = std::numeric_limits<double>::infinity();
  CHECK(f.run().reason == "invalid_rigid_transform");
  f.in.optical_to_base_translation[0] = 0.;
  f.in.box_min[0] = std::numeric_limits<double>::quiet_NaN();
  CHECK(f.run().reason == "invalid_box");
  f.in.box_min[0] = .25;
  CHECK(f.run().reason == "invalid_box");
  f.in.box_min[0] = -.25; f.in.fx = 0.;
  CHECK(f.run().reason == "invalid_intrinsics_or_depth_limits");
  f.in.fx = 4.; f.in.clear_margin_m = std::numeric_limits<double>::infinity();
  CHECK(f.run().reason == "invalid_intrinsics_or_depth_limits");
  f.in.clear_margin_m = .25; f.in.box_min[0] = -std::numeric_limits<double>::max();
  f.in.optical_to_base_translation[0] = std::numeric_limits<double>::max();
  CHECK(f.run().state == RegionSupportState::kUnknown);
  CHECK(f.run().reason == "nonfinite_projection");
}
void classification_defaults_and_legacy_aggregate_order() {
  RegionDepthSupport r;
  CHECK(!r.evaluation_complete && r.evaluation_code == RegionEvaluationCode::kRejected);
  CHECK(r.disposition == RegionDisposition::kHardFault);
  // Existing aggregate field order remains valid; trailing classification defaults reject.
  RegionDepthSupport old{RegionSupportState::kUnknown, "old", 1, 2, 3, 4, 5, 6, 7};
  CHECK(old.tested_pixels == 1 && old.margin_band_rays == 7);
  CHECK(!old.evaluation_complete && old.disposition == RegionDisposition::kHardFault);
}

void complete_classification_preserves_clear_and_obstacle() {
  Fixture f;
  auto r = f.run();
  CHECK(r.evaluation_complete && r.evaluation_code == RegionEvaluationCode::kClear);
  CHECK(r.disposition == RegionDisposition::kClear);
  f.pixels[12] = 1.5F; r = f.run();
  CHECK(r.evaluation_complete && r.evaluation_code == RegionEvaluationCode::kObstacle);
  CHECK(r.disposition == RegionDisposition::kHardFault);
}

void only_pure_margin_incomplete_is_geometry_hold() {
  Fixture f;
  f.pixels[12] = 2.125F;
  auto r = f.run();
  CHECK(r.evaluation_complete && r.evaluation_code == RegionEvaluationCode::kIncompleteRays);
  CHECK(r.disposition == RegionDisposition::kGeometryHold);
  CHECK(r.state == RegionSupportState::kUnknown && r.reason == "incomplete_ray_support");
  for (float mixed : {0.F, .5F, 1.F, std::numeric_limits<float>::quiet_NaN()}) {
    f.pixels[11] = mixed; r = f.run();
    CHECK(r.evaluation_complete && r.evaluation_code == RegionEvaluationCode::kIncompleteRays);
    CHECK(r.disposition == RegionDisposition::kHardFault);
    CHECK(r.margin_band_rays == 1 && r.state == RegionSupportState::kUnknown);
  }
  f.pixels[12] = 3.F;
  for (float invalid : {0.F, .5F, std::numeric_limits<float>::infinity()}) {
    f.pixels[11] = invalid; r = f.run();
    CHECK(r.evaluation_complete && r.evaluation_code == RegionEvaluationCode::kIncompleteRays);
    CHECK(r.disposition == RegionDisposition::kHardFault);
  }
}

void float_margin_boundary_does_not_change_classification_threshold() {
  Fixture f;
  f.pixels[12] = std::nextafter(2.25F, 0.F);
  CHECK(f.run().disposition == RegionDisposition::kGeometryHold);
  f.pixels[12] = 2.25F;
  CHECK(f.run().disposition == RegionDisposition::kClear);
  f.pixels[12] = std::nextafter(2.25F, 3.F);
  CHECK(f.run().disposition == RegionDisposition::kClear);
  f.pixels[12] = 2.F;
  CHECK(f.run().evaluation_code == RegionEvaluationCode::kObstacle);
  f.pixels[12] = std::nextafter(2.F, 3.F);
  CHECK(f.run().disposition == RegionDisposition::kGeometryHold);
}

void rejected_inputs_never_claim_complete_evaluation() {
  Fixture f;
  f.in.contract.source_time_and_transform_verified = false;
  CHECK(!f.run().evaluation_complete);
  f.in.contract.source_time_and_transform_verified = true;
  f.in.box_min = {.01, .01, 1.}; f.in.box_max = {.02, .02, 1.1};
  CHECK(!f.run().evaluation_complete);  // Full traversal with no rays is not support.
  f.in.box_min = {-.25, -.25, 1.}; f.in.box_max = {.25, .25, 2.};
  f.in.fx = std::numeric_limits<double>::quiet_NaN();
  CHECK(!f.run().evaluation_complete);
  f.in.fx = 4.; f.in.box_min[0] = -std::numeric_limits<double>::max();
  f.in.optical_to_base_translation[0] = std::numeric_limits<double>::max();
  CHECK(f.run().reason == "nonfinite_projection" && !f.run().evaluation_complete);
}

}  // namespace

int main() {
  try {
    exhaustive_rays_preserve_single_pixel_obstacle_and_hole();
    endpoints_margin_and_front_occlusion_are_distinct();
    every_invalid_depth_is_unknown_and_outside_pixels_do_not_vote();
    axial_depth_is_not_euclidean_range();
    rotated_translated_optical_frame_keeps_same_physical_rays();
    fov_half_pixel_boundary_is_explicit_and_not_expandable();
    projection_plane_and_subpixel_unsampled_box_are_unknown();
    calibration_contract_defaults_fail_closed_individually();
    malformed_layout_rejected_and_row_padding_never_sampled();
    malformed_geometry_and_numerical_overflow_fail_closed();
    classification_defaults_and_legacy_aggregate_order();
    complete_classification_preserves_clear_and_obstacle();
    only_pure_margin_incomplete_is_geometry_hold();
    float_margin_boundary_does_not_change_classification_threshold();
    rejected_inputs_never_claim_complete_evaluation();
    std::cout << "15 region depth support scenarios passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
