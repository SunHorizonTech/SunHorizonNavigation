#include "sunhorizon_perception_runtime/near_ground_clearance.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace n = sunhorizon_perception_runtime::near_ground_clearance;
namespace g = sunhorizon_perception_runtime::ground_relative;
namespace sv = sunhorizon_obstacle_coverage::spatial_visibility;
namespace {
std::size_t checks = 0;
void check(bool ok) {if (++checks && !ok) throw std::runtime_error("near ground clearance check " + std::to_string(checks));}
template<class F> void rejects(F&& fn) {bool rejected = false; try {fn();} catch (const std::exception&) {rejected = true;} check(rejected);}
g::ClassifierConfig classifier() {
  g::ClassifierConfig c;
  c.ground_band_min_m = -.01; c.ground_band_max_m = .01; c.maximum_support_residual_m = .02;
  c.maximum_point_error_m = .01; c.maximum_transform_translation_error_m = .01;
  c.maximum_transform_angle_error_rad = .02; c.maximum_plane_offset_error_m = .01;
  c.maximum_plane_angle_error_rad = .02; c.maximum_normal_tilt_rad = .1;
  c.rotation_tolerance = c.normal_norm_tolerance = 1.e-12;
  c.minimum_support_span_x_m = c.minimum_support_span_y_m = .1; c.minimum_support_double_area_m2 = .005;
  c.minimum_samples_per_cell = 1; c.minimum_training_cells = c.minimum_holdout_cells = 3;
  c.continuous_support_radius_cells = 1; c.maximum_points = 1000;
  c.maximum_support_cells = 100; c.maximum_support_checks = 10000;
  return c;
}
g::SourceFrame ground_source() {
  g::SourceFrame s;
  s.identity = {"belly", "epoch", "synthetic-calibration", "camera", "odom", 12,
    1000000000, 1001000000, 5000000000, 300000000};
  s.to_common.rotation = {1., 0., 0., 0., -1., 0., 0., 0., -1.};
  s.to_common.translation = {0., 0., .1};
  s.to_common.source_frame = "camera"; s.to_common.common_frame = "odom";
  s.to_common.source_ns = s.to_common.common_ns = s.identity.source_ns;
  s.to_common.qualified = s.measurement_qualified = true;
  for (int y = 0; y < 5; ++y) for (int x = 0; x < 5; ++x)
    s.points.push_back({{-.2 + x * .1, -(-.2 + y * .1), 1.}, g::ReturnKind::Measured});
  return s;
}
g::GroundReference reference(const g::SourceFrame& s) {
  g::GroundReference r;
  r.provenance_id = "synthetic-independent-survey"; r.calibration_id = "synthetic-survey-calibration";
  r.common_frame = "odom"; r.expected_source_id = s.identity.source_id; r.expected_source_epoch = s.identity.epoch;
  r.expected_source_calibration_id = s.identity.calibration_id;
  r.valid_for_source_ns = s.identity.source_ns; r.valid_for_common_ns = s.to_common.common_ns;
  r.source_ns = s.identity.source_ns; r.first_receipt_ros_ns = s.identity.first_receipt_ros_ns;
  r.first_receipt_steady_ns = s.identity.first_receipt_steady_ns; r.expiry_budget_ns = 600000000;
  r.independently_qualified = r.source_time_transform_qualified = true;
  r.unit_normal = {0., 0., 1.}; r.anchor = {0., 0., -.9};
  r.support.origin_x_m = r.support.origin_y_m = -.25; r.support.resolution_m = .1;
  r.support.width = r.support.height = 5;
  for (uint32_t y = 0; y < 5; ++y) for (uint32_t x = 0; x < 5; ++x) r.support.required_cells.push_back({x, y});
  return r;
}
n::TileGrid grid() {return {"synthetic-sampling-contract", -.1, -.1, .1, 2, 2, -.95, .15, 0, 16};}
struct Depth {
  std::vector<float> values = std::vector<float>(21 * 21, 1.f);
  std::vector<sv::ReturnKind> kinds = std::vector<sv::ReturnKind>(21 * 21, sv::ReturnKind::kExternal);
  sv::Source source() const {
    sv::Source s;
    s.source_id = "belly"; s.epoch = "epoch"; s.frame_id = "camera"; s.frame_sequence = 12;
    s.source_stamp_ros_ns = 1000000000; s.local_receipt_ros_ns = 1001000000;
    s.local_receipt_steady_ns = 5000000000; s.expiry_budget_ns = 300000000;
    // Deliberately a different common frame: the new producer must build its
    // own odom view without modifying this original R/base view.
    sv::DepthRaster d;
    d.width = d.height = d.row_stride_elements = 21; d.fx = d.fy = 20.; d.cx = d.cy = 10.;
    d.depth_m = values.data(); d.depth_elements = values.size(); d.return_kinds = kinds.data(); d.kind_elements = kinds.size();
    d.minimum_depth_m = .1; d.maximum_depth_m = 3.; d.clear_margin_m = .005;
    d.calibration = {true, true, true, true, true};
    d.optical_to_common.source_frame = "camera"; d.optical_to_common.common_frame = "base_footprint";
    d.optical_to_common.source_stamp_ns = d.optical_to_common.query_stamp_ns = s.source_stamp_ros_ns;
    s.data = d; return s;
  }
};
void all_state(const n::Snapshot::ConstSharedPtr& s, sv::CellState state) {
  check(!s->tiles.empty());
  for (const auto& tile : s->tiles) check(tile.state == state);
}
void run() {
  static_assert(std::is_const<n::Snapshot::ConstSharedPtr::element_type>::value, "immutable snapshot owner");
  auto src = ground_source(); const auto c = classifier(); auto ref = reference(src); const auto cfg = grid();
  const auto classified = g::classify_obstacle_frame(src, c, ref);
  check(classified->ground().safety_qualified); check(classified->clearing_indices().size() == 25);
  Depth depth; const auto original = depth.source();
  const auto evaluate = [&](const sv::Source& input, const g::ClassifiedObstacleFrame::ConstSharedPtr& frame) {
    return n::evaluate(input, frame, c, cfg, 1005000000, 5004000000);
  };
  const auto q = n::make_query(*classified, c, cfg);
  check(q.cells.size() == 4); check(q.common_frame == "odom"); check(q.query_stamp_ns == src.identity.source_ns);
  check(q.source_contracts.size() == 1 && q.source_contracts[0].required_for_evaluation);
  for (const auto& cell : q.cells) {check(cell.min[2] >= -.89 && cell.min[2] < -.889999); check(std::abs(cell.max[2] + .8) < 1.e-12);}
  const auto free = evaluate(original, classified);
  check(free->ground_frame == classified); check(free->ground_qualified && free->evaluation_complete);
  check(free->identity.source_ns == src.identity.source_ns && free->identity.first_receipt_steady_ns == 5000000000);
  check(free->common_frame == "odom" && free->remaining_valid_ns == 295000000);
  all_state(free, sv::CellState::kFree);
  for (const auto& tile : free->tiles) {
    check(tile.support_qualified && tile.counts.intersecting_rays > 0);
    check(tile.counts.intersecting_rays == tile.counts.clear_rays);
    check(tile.coarse_z_lower_m < tile.floor_band_upper_z_m && tile.query_z_lower_m == tile.floor_band_upper_z_m);
  }
  check(std::get<sv::DepthRaster>(original.data).optical_to_common.common_frame == "base_footprint");
  check(std::get<sv::DepthRaster>(original.data).return_kinds == depth.kinds.data());
  check(!original.required_for_evaluation);

  // Endpoint support does not grant a bounded free-volume contract, even
  // when the retained ground endpoints pass the segmentation detector.
  auto support_source=src;support_source.points.clear();support_source.measurement_qualified=false;
  support_source.point_error_bound_m=support_source.transform_translation_error_bound_m=
    support_source.transform_angle_error_bound_rad=g::kUnspecified;
  for(const auto& point:src.points)for(unsigned sample=0;sample<16;++sample){
    auto retained=point;retained.sample_id=support_source.points.size();support_source.points.push_back(retained);}
  auto support_ref=ref;support_ref.independently_qualified=support_ref.source_time_transform_qualified=false;
  support_ref.plane_offset_error_bound_m=support_ref.plane_angle_error_bound_rad=g::kUnspecified;
  g::SegmentationConfig support_config;support_config.maximum_point_residual_m=.04;
  support_config.maximum_cell_median_residual_m=.03;support_config.maximum_cell_mad_m=.015;
  support_config.maximum_partition_median_difference_m=.02;support_config.maximum_normal_tilt_rad=.2;
  support_config.minimum_support_span_x_m=support_config.minimum_support_span_y_m=.1;
  support_config.minimum_support_double_area_m2=.005;support_config.minimum_samples_per_partition=1;
  support_config.minimum_supported_cells=6;support_config.continuous_support_radius_cells=1;
  support_config.maximum_points=1000;support_config.maximum_support_cells=100;support_config.maximum_support_checks=10000;
  const auto segmented=g::classify_supported_ground_frame(support_source,support_config,support_ref);
  check(segmented->ground().safety_qualified&&segmented->ground().basis==g::GroundQualification::Basis::MeasuredSupport);
  check(!segmented->clearing_indices().empty()&&std::isnan(segmented->source().point_error_bound_m));
  const auto unbounded=evaluate(original,segmented);
  check(!unbounded->ground_qualified&&!unbounded->evaluation_complete&&
    unbounded->reason=="ground_classification_not_bounded");
  all_state(unbounded,sv::CellState::kUnknown);check(n::make_query(*segmented,c,cfg).cells.empty());

  // One hole on the shared tile boundary blocks all four tiles despite every
  // other ray observing floor beyond the query. Self behind the query is also
  // deliberately Unknown here (stricter than the original visibility core).
  for (const auto kind : {sv::ReturnKind::kInvalid, sv::ReturnKind::kSelfMatched, sv::ReturnKind::kModelConflict}) {
    depth.kinds[10 * 21 + 10] = kind; all_state(evaluate(depth.source(), classified), sv::CellState::kUnknown);
  }
  depth.kinds[10 * 21 + 10] = sv::ReturnKind::kExternal;
  depth.values[10 * 21 + 10] = std::numeric_limits<float>::quiet_NaN();
  all_state(evaluate(depth.source(), classified), sv::CellState::kUnknown);
  depth.values[10 * 21 + 10] = .8f; all_state(evaluate(depth.source(), classified), sv::CellState::kUnknown);
  depth.values[10 * 21 + 10] = .993f; all_state(evaluate(depth.source(), classified), sv::CellState::kUnknown);
  depth.values[10 * 21 + 10] = .95f; all_state(evaluate(depth.source(), classified), sv::CellState::kOccupied);
  depth.values[10 * 21 + 10] = 1.f;

  auto missing_kinds = depth.source(); std::get<sv::DepthRaster>(missing_kinds.data).return_kinds = nullptr;
  all_state(evaluate(missing_kinds, classified), sv::CellState::kUnknown);
  auto wrong = original; ++wrong.frame_sequence; all_state(evaluate(wrong, classified), sv::CellState::kUnknown);
  wrong = original; ++wrong.local_receipt_steady_ns; all_state(evaluate(wrong, classified), sv::CellState::kUnknown);
  auto stale = n::evaluate(original, classified, c, cfg, 1400000000, 5400000000);
  all_state(stale, sv::CellState::kUnknown); check(!stale->evaluation_complete);
  auto old_reference = reference(src);
  old_reference.source_ns = 990000000; old_reference.first_receipt_ros_ns = 991000000;
  old_reference.first_receipt_steady_ns = 4990000000; old_reference.expiry_budget_ns = 12000000;
  auto short_reference = g::classify_obstacle_frame(src, c, old_reference);
  check(!short_reference->ground().safety_qualified);
  const auto expired_reference = evaluate(original, short_reference);
  all_state(expired_reference, sv::CellState::kUnknown); check(expired_reference->reason == "ground_unqualified");
  auto no_ref = g::conservative_unqualified_frame(src);
  all_state(evaluate(original, no_ref), sv::CellState::kUnknown); check(n::make_query(*no_ref, c, cfg).cells.empty());
  auto observed = g::classify_obstacle_frame(src, c, ref, true);
  all_state(evaluate(original, observed), sv::CellState::kUnknown);

  // A fixed registration-map asset has no historical boot/steady lease.
  // Visibility still uses the real sensor lease, and alignment reset revokes
  // clearance even when a previously classified frame remains queued.
  auto map_source=src;
  map_source.identity.common_frame=map_source.to_common.common_frame="registration_map";
  auto map_reference=reference(map_source);map_reference.common_frame="registration_map";
  map_reference.lifetime=g::GroundReference::Lifetime::RegistrationMapAsset;
  map_reference.map_asset={std::string(64,'b'),"synthetic-registration-datum"};
  map_reference.source_ns=map_reference.first_receipt_ros_ns=map_reference.first_receipt_steady_ns=map_reference.expiry_budget_ns=0;
  auto reset=std::make_shared<std::atomic<bool>>(false);
  auto alignment=std::make_shared<g::RegistrationMapAlignment>(map_reference.map_asset,5,reset,
    1301000000,5300000000LL);
  map_reference.map_binding={alignment,map_source.identity.source_id,map_source.identity.epoch,map_source.identity.source_ns};
  auto map_frame=g::classify_obstacle_frame(map_source,c,map_reference);
  check(map_frame->ground().safety_qualified);
  const auto map_free=evaluate(original,map_frame);all_state(map_free,sv::CellState::kFree);
  check(map_free->remaining_valid_ns<=296000000);
  const auto old_alignment=n::evaluate(original,map_frame,c,cfg,1301000001,5300000001LL);
  all_state(old_alignment,sv::CellState::kUnknown);check(old_alignment->reason=="ground_reference_expired");
  reset->store(true);
  auto revoked=evaluate(original,map_frame);all_state(revoked,sv::CellState::kUnknown);
  check(!revoked->ground_qualified&&revoked->reason=="ground_reference_expired");

  // Current contradictory support must block clearance even when all raster
  // rays otherwise pass. The fixed reference cannot move to this box return.
  src.points[12].position[2] = .95;
  auto conflict = g::classify_obstacle_frame(src, c, ref);
  check(conflict->ground().safety_qualified);
  all_state(evaluate(original, conflict), sv::CellState::kUnknown);
  src = ground_source();
  auto outside = cfg; outside.origin_x_m = .2;
  all_state(n::evaluate(original, classified, c, outside, 1005000000, 5004000000), sv::CellState::kUnknown);
  auto no_volume = cfg; no_volume.coarse_origin_z_m = -1.2;
  check(n::make_query(*classified, c, no_volume).cells.empty());
  all_state(n::evaluate(original, classified, c, no_volume, 1005000000, 5004000000), sv::CellState::kUnknown);
  auto out_fov = cfg; out_fov.origin_x_m = -.6; out_fov.width = 1; out_fov.resolution_m = .5;
  all_state(n::evaluate(original, classified, c, out_fov, 1005000000, 5004000000), sv::CellState::kUnknown);

  // Tilt and reference uncertainty change only the new query envelope.
  ref.unit_normal = {.02, 0., std::sqrt(1. - .02 * .02)};
  const auto tilted = g::classify_obstacle_frame(src, c, ref);
  check(tilted->ground().safety_qualified);
  const auto tq = n::make_query(*tilted, c, cfg);
  check(tq.cells.size() == 4 && tq.cells[0].min[2] > tq.cells[1].min[2]);
  ref = reference(src); ref.plane_offset_error_bound_m = .002;
  const auto bounded = g::classify_obstacle_frame(src, c, ref);
  check(bounded->ground().safety_qualified);
  check(n::make_query(*bounded, c, cfg).cells[0].min[2] > q.cells[0].min[2]);
  // Source error enlarges the sampling domain, never the resulting proof.
  // Ground is nominally behind the uninflated cell, but this amount of source
  // uncertainty reaches it, so the original Free result must disappear.
  src.point_error_bound_m = .003; src.transform_translation_error_bound_m = .003;
  auto uncertain_source = g::classify_obstacle_frame(src, c, reference(src));
  check(uncertain_source->ground().safety_qualified);
  const auto eq = n::make_query(*uncertain_source, c, cfg);
  check(eq.cells[0].min[0] < q.cells[0].min[0] && eq.cells[0].max[0] > q.cells[0].max[0]);
  const auto error_result = evaluate(original, uncertain_source);
  all_state(error_result, sv::CellState::kUnknown);
  check(error_result->tiles[0].query_z_lower_m == free->tiles[0].query_z_lower_m);
  check(error_result->tiles[0].visibility_error_bound_m >= .006);
  // The far corner of a query is an explicit angular-error lever arm even
  // when every current source return is much nearer.
  src = ground_source(); src.transform_angle_error_bound_rad = .001;
  auto angular_source = g::classify_obstacle_frame(src, c, reference(src));
  check(angular_source->ground().safety_qualified);
  auto far = cfg; far.origin_x_m = 5.;
  const auto fq = n::make_query(*angular_source, c, far);
  check(fq.cells.size() == 4);
  check(far.origin_x_m - fq.cells[0].min[0] > .005);
  auto bad = cfg; bad.maximum_tiles = 3; rejects([&] {n::make_query(*classified, c, bad);});
  bad = cfg; bad.resolution_m = 0.; rejects([&] {n::make_query(*classified, c, bad);});
  rejects([&] {n::evaluate(original, {}, c, cfg, 1005000000, 5004000000);});
}
}  // namespace
int main() {
  try {run(); std::cout << "near ground clearance checks: " << checks << '\n'; return 0;}
  catch (const std::exception& error) {std::cerr << error.what() << '\n'; return 1;}
}
