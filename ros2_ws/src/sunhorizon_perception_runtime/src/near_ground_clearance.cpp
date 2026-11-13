#include "sunhorizon_perception_runtime/near_ground_clearance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>

namespace sunhorizon_perception_runtime::near_ground_clearance {
namespace {
bool finite(double v) {return std::isfinite(v);}
bool text(const std::string& s) {
  return !s.empty() && s.size() <= 128 && std::all_of(s.begin(), s.end(),
      [](unsigned char c) {return c >= 33 && c <= 126;});
}
void require_grid(const TileGrid& c) {
  if (!text(c.contract_id) || !finite(c.origin_x_m) || !finite(c.origin_y_m) ||
      !finite(c.resolution_m) || c.resolution_m <= 0. || !c.width || !c.height ||
      !c.maximum_tiles || c.maximum_tiles > sv::kMaximumCells ||
      uint64_t(c.width) * c.height > c.maximum_tiles ||
      !finite(c.coarse_origin_z_m) || !finite(c.coarse_resolution_z_m) || c.coarse_resolution_z_m <= 0. ||
      !finite(c.origin_x_m + c.width * c.resolution_m) ||
      !finite(c.origin_y_m + c.height * c.resolution_m) ||
      !finite(c.coarse_origin_z_m + (double(c.coarse_z_index) + 1.) * c.coarse_resolution_z_m))
    throw std::invalid_argument("near_ground_clearance_invalid_tile_grid");
}
bool geometry_valid(const g::ClassifiedObstacleFrame& frame, const g::ClassifierConfig& c) {
  if (frame.ground().basis!=g::GroundQualification::Basis::BoundedReference ||
      !frame.reference() || !frame.ground().reference_geometry_valid ||
      !finite(c.ground_band_max_m) || c.ground_band_max_m < 0.) return false;
  const auto& r = *frame.reference();
  for (auto v : r.unit_normal) if (!finite(v)) return false;
  for (auto v : r.anchor) if (!finite(v)) return false;
  return r.unit_normal[2] > 0. && finite(r.plane_offset_error_bound_m) &&
      r.plane_offset_error_bound_m >= 0. && finite(r.plane_angle_error_bound_rad) &&
      r.plane_angle_error_bound_rad >= 0. && r.plane_angle_error_bound_rad <= 3.14159265358979323846;
}
std::vector<TileEvidence> tiles(const g::ClassifiedObstacleFrame& frame,
    const g::ClassifierConfig& c, const TileGrid& grid) {
  require_grid(grid);
  std::vector<TileEvidence> out;
  out.reserve(std::size_t(grid.width) * grid.height);
  const double lower = grid.coarse_origin_z_m + double(grid.coarse_z_index) * grid.coarse_resolution_z_m;
  const double upper = grid.coarse_origin_z_m + (double(grid.coarse_z_index) + 1.) * grid.coarse_resolution_z_m;
  if (!(lower < upper)) throw std::invalid_argument("near_ground_clearance_collapsed_z_slice");
  const bool geometry = geometry_valid(frame, c);
  const auto& source = frame.source();
  const bool source_bounds = finite(source.point_error_bound_m) && source.point_error_bound_m >= 0. &&
      finite(source.transform_translation_error_bound_m) && source.transform_translation_error_bound_m >= 0. &&
      finite(source.transform_angle_error_bound_rad) && source.transform_angle_error_bound_rad >= 0. &&
      source.transform_angle_error_bound_rad <= 3.14159265358979323846;
  // Query corners can be farther away than any current classified return. Both
  // lever arms participate; no observed point maximum substitutes for the query.
  double source_radius = 0.;
  for (const auto& point : source.points) if (point.kind == g::ReturnKind::Measured) {
    const auto& p = point.position;
    if (finite(p[0]) && finite(p[1]) && finite(p[2])) source_radius = std::max(source_radius, std::hypot(p[0], p[1], p[2]));
  }
  for (uint32_t y = 0; y < grid.height; ++y) for (uint32_t x = 0; x < grid.width; ++x) {
    TileEvidence tile;
    tile.id = y * grid.width + x; tile.x = x; tile.y = y;
    tile.min_xy = {grid.origin_x_m + x * grid.resolution_m, grid.origin_y_m + y * grid.resolution_m};
    tile.max_xy = {grid.origin_x_m + (double(x) + 1.) * grid.resolution_m,
      grid.origin_y_m + (double(y) + 1.) * grid.resolution_m};
    tile.coarse_z_lower_m = lower; tile.query_z_upper_m = upper;
    if (!(tile.min_xy[0] < tile.max_xy[0] && tile.min_xy[1] < tile.max_xy[1]))
      throw std::invalid_argument("near_ground_clearance_collapsed_xy_tile");
    if (geometry) {
      const auto& r = *frame.reference();
      double radius = 0.;
      for (unsigned corner = 0; corner < 8; ++corner) {
        const double px = (corner & 1 ? tile.max_xy[0] : tile.min_xy[0]) - r.anchor[0];
        const double py = (corner & 2 ? tile.max_xy[1] : tile.min_xy[1]) - r.anchor[1];
        const double pz = (corner & 4 ? upper : lower) - r.anchor[2];
        radius = std::max(radius, std::hypot(px, py, pz));
      }
      const double bound = r.plane_offset_error_bound_m +
          2. * radius * std::sin(r.plane_angle_error_bound_rad / 2.);
      double floor_upper = -std::numeric_limits<double>::infinity();
      for (unsigned corner = 0; corner < 4; ++corner) {
        const double px = (corner & 1 ? tile.max_xy[0] : tile.min_xy[0]) - r.anchor[0];
        const double py = (corner & 2 ? tile.max_xy[1] : tile.min_xy[1]) - r.anchor[1];
        floor_upper = std::max(floor_upper, r.anchor[2] +
            (c.ground_band_max_m + bound - r.unit_normal[0] * px - r.unit_normal[1] * py) / r.unit_normal[2]);
      }
      if (finite(floor_upper) && finite(radius) && finite(bound)) {
        tile.floor_band_upper_z_m = std::nextafter(floor_upper, std::numeric_limits<double>::infinity());
        tile.query_z_lower_m = std::max(lower, tile.floor_band_upper_z_m);
        if (source_bounds && tile.query_z_lower_m < upper) {
          double lever = source_radius;
          for (unsigned corner = 0; corner < 8; ++corner) {
            const double px = (corner & 1 ? tile.max_xy[0] : tile.min_xy[0]) - source.to_common.translation[0];
            const double py = (corner & 2 ? tile.max_xy[1] : tile.min_xy[1]) - source.to_common.translation[1];
            const double pz = (corner & 4 ? upper : tile.query_z_lower_m) - source.to_common.translation[2];
            lever = std::max(lever, std::hypot(px, py, pz));
          }
          const double error = source.point_error_bound_m + source.transform_translation_error_bound_m +
              2. * lever * std::sin(source.transform_angle_error_bound_rad / 2.);
          if (finite(error)) tile.visibility_error_bound_m = error == 0. ? 0. :
              std::nextafter(error, std::numeric_limits<double>::infinity());
        }
      }
    }
    out.push_back(tile);
  }
  return out;
}
sv::Query query(const g::ClassifiedObstacleFrame& frame, const TileGrid& grid,
    const std::vector<TileEvidence>& items) {
  sv::Query out;
  out.query_id = "near-ground-clearance-v1"; out.contract_id = grid.contract_id;
  out.common_frame = frame.source().identity.common_frame;
  out.query_stamp_ns = frame.source().to_common.common_ns;
  out.source_contracts.push_back({frame.source().identity.source_id, frame.source().identity.epoch, true});
  for (const auto& t : items) if (finite(t.query_z_lower_m) && t.query_z_lower_m < t.query_z_upper_m &&
      finite(t.visibility_error_bound_m)) {
    sv::AxisAlignedCell cell{t.id, {t.min_xy[0], t.min_xy[1], t.query_z_lower_m},
      {t.max_xy[0], t.max_xy[1], t.query_z_upper_m}};
    bool valid = true;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      if (t.visibility_error_bound_m > 0.) {
        cell.min[axis] = std::nextafter(cell.min[axis] - t.visibility_error_bound_m,
          -std::numeric_limits<double>::infinity());
        cell.max[axis] = std::nextafter(cell.max[axis] + t.visibility_error_bound_m,
          std::numeric_limits<double>::infinity());
      }
      valid = valid && finite(cell.min[axis]) && finite(cell.max[axis]) && cell.min[axis] < cell.max[axis];
    }
    if (valid) out.cells.push_back(cell);
  }
  return out;
}
bool same_source(const sv::Source& source, const g::SourceFrame& ground) {
  const auto& id = ground.identity;
  return source.source_id == id.source_id && source.epoch == id.epoch &&
      source.frame_id == id.source_frame && source.frame_sequence == id.sequence &&
      source.source_stamp_ros_ns == id.source_ns && source.local_receipt_ros_ns == id.first_receipt_ros_ns &&
      source.local_receipt_steady_ns == id.first_receipt_steady_ns && source.expiry_budget_ns == id.expiry_budget_ns;
}
bool fresh_reference(const g::GroundReference& r,const g::SourceIdentity& source,int64_t ros,int64_t steady) {
  if(r.lifetime==g::GroundReference::Lifetime::RegistrationMapAsset) {
    if(!g::registration_map_binding_current(r,source))return false;
    const auto& alignment=*r.map_binding.alignment;
    return ros>0&&steady>0&&ros<=alignment.geometry_valid_until_ros_ns()&&
      steady<=alignment.geometry_valid_until_steady_ns();
  }
  return r.source_ns > 0 && r.first_receipt_ros_ns >= r.source_ns && r.first_receipt_steady_ns > 0 &&
      r.expiry_budget_ns > 0 && ros >= r.first_receipt_ros_ns && steady >= r.first_receipt_steady_ns &&
      ros - r.source_ns <= r.expiry_budget_ns && ros - r.first_receipt_ros_ns <= r.expiry_budget_ns &&
      steady - r.first_receipt_steady_ns <= r.expiry_budget_ns;
}
bool supported(const TileEvidence& t, const g::GroundReference& reference,
    const g::ClassifierConfig& c, const std::map<uint64_t, bool>& passed, std::size_t& checks) {
  const auto& grid = reference.support;
  if (!grid.width || !grid.height || !finite(grid.resolution_m) || grid.resolution_m <= 0. ||
      !c.continuous_support_radius_cells || c.continuous_support_radius_cells > 32 ||
      t.min_xy[0] < grid.origin_x_m || t.min_xy[1] < grid.origin_y_m ||
      t.max_xy[0] > grid.origin_x_m + grid.width * grid.resolution_m ||
      t.max_xy[1] > grid.origin_y_m + grid.height * grid.resolution_m) return false;
  // Include the tile's entire XY extent, not merely its centre support cell.
  const auto cell = [&](double p, double origin) {return std::floor((p - origin) / grid.resolution_m);};
  const double xa = cell(t.min_xy[0], grid.origin_x_m), ya = cell(t.min_xy[1], grid.origin_y_m);
  const double xb = cell(std::nextafter(t.max_xy[0], -std::numeric_limits<double>::infinity()), grid.origin_x_m);
  const double yb = cell(std::nextafter(t.max_xy[1], -std::numeric_limits<double>::infinity()), grid.origin_y_m);
  if (!finite(xa) || !finite(ya) || !finite(xb) || !finite(yb) || xa < 0. || ya < 0. ||
      xb >= grid.width || yb >= grid.height) return false;
  const auto radius = c.continuous_support_radius_cells;
  const auto x0 = uint32_t(xa) > radius ? uint32_t(xa) - radius : 0U;
  const auto y0 = uint32_t(ya) > radius ? uint32_t(ya) - radius : 0U;
  const auto x1 = uint32_t(std::min<uint64_t>(uint64_t(xb) + radius, grid.width - 1));
  const auto y1 = uint32_t(std::min<uint64_t>(uint64_t(yb) + radius, grid.height - 1));
  for (uint32_t y = y0; y <= y1; ++y) for (uint32_t x = x0; x <= x1; ++x) {
    if (++checks > sv::kMaximumChecks) return false;
    const auto found = passed.find(uint64_t(y) * grid.width + x);
    if (found == passed.end() || !found->second) return false;
  }
  return true;
}
}  // namespace

sv::Query make_query(const g::ClassifiedObstacleFrame& frame, const g::ClassifierConfig& c, const TileGrid& grid) {
  return query(frame, grid, tiles(frame, c, grid));
}

Snapshot::ConstSharedPtr evaluate(const sv::Source& decoded_view,
    g::ClassifiedObstacleFrame::ConstSharedPtr frame, const g::ClassifierConfig& c,
    const TileGrid& grid, int64_t now_ros, int64_t now_steady) {
  if (!frame) throw std::invalid_argument("near_ground_clearance_missing_classified_frame");
  auto out = std::make_shared<Snapshot>();
  out->identity = frame->source().identity; out->common_frame = out->identity.common_frame;
  out->contract_id = grid.contract_id; out->query_stamp_ns = frame->source().to_common.common_ns;
  out->ground_frame = std::move(frame); const auto& ground = *out->ground_frame;
  out->tiles = tiles(ground, c, grid);
  const auto fail = [&](const char* reason) -> Snapshot::ConstSharedPtr {out->reason = reason; return out;};
  if (!ground.classification_complete() || !ground.ground().safety_qualified || !ground.reference())
    return fail("ground_unqualified");
  if (ground.ground().basis!=g::GroundQualification::Basis::BoundedReference)
    return fail("ground_classification_not_bounded");
  if (!fresh_reference(*ground.reference(),ground.source().identity,now_ros,now_steady)) return fail("ground_reference_expired");
  if (!same_source(decoded_view, ground.source())) return fail("source_identity_mismatch");
  auto source = decoded_view;
  auto* raster = std::get_if<sv::DepthRaster>(&source.data);
  if (!raster) return fail("depth_raster_required");
  // A missing kinds array must not inherit the core's legacy all-External default.
  if (!raster->return_kinds || !raster->kind_elements || raster->kind_elements > sv::kMaximumRasterElements)
    return fail("explicit_return_kinds_required");
  std::vector<sv::ReturnKind> kinds(raster->kind_elements, sv::ReturnKind::kInvalid);
  for (std::size_t i = 0; i < kinds.size(); ++i)
    if (raster->return_kinds[i] == sv::ReturnKind::kExternal) kinds[i] = sv::ReturnKind::kExternal;
  raster->return_kinds = kinds.data();
  const auto& tf = ground.source().to_common;
  raster->optical_to_common = {tf.rotation, tf.translation, tf.source_frame, tf.common_frame, tf.source_ns, tf.common_ns};
  source.required_for_evaluation = true;  // This separate query requires this source.
  const auto q = query(ground, grid, out->tiles);
  if (q.cells.empty()) return fail("no_above_ground_query_volume");
  const auto visibility = sv::evaluate(q, {source}, now_ros, now_steady);
  out->visibility_checks = visibility.checks;
  if (!visibility.evaluation_complete) {out->reason = visibility.reason; return out;}
  out->ground_qualified = true; out->evaluation_complete = true;
  const auto& r = *ground.reference();
  const int64_t ref_remaining = r.lifetime==g::GroundReference::Lifetime::RegistrationMapAsset?
    std::min(r.map_binding.alignment->geometry_valid_until_ros_ns()-now_ros,
      r.map_binding.alignment->geometry_valid_until_steady_ns()-now_steady):r.expiry_budget_ns-std::max({now_ros-r.source_ns,
      now_ros-r.first_receipt_ros_ns,now_steady-r.first_receipt_steady_ns});
  out->remaining_valid_ns = std::min(ref_remaining, visibility.min_remaining_valid_ns);
  std::map<uint64_t, bool> passed;
  for (const auto& cell : ground.ground().cells)
    passed.emplace(uint64_t(cell.cell.y) * r.support.width + cell.cell.x, cell.passed);
  std::size_t support_checks = 0;
  for (const auto& evidence : visibility.cells) {
    auto& tile = out->tiles.at(evidence.id);
    tile.counts = evidence.counts[0];
    tile.support_qualified = supported(tile, r, c, passed, support_checks);
    if (support_checks > sv::kMaximumChecks) {
      for (auto& previous : out->tiles) {previous.state = sv::CellState::kUnknown; previous.support_qualified = false;}
      out->evaluation_complete = false; out->reason = "support_budget_exceeded"; return out;
    }
    if (tile.support_qualified) tile.state = evidence.state;
  }
  if(!fresh_reference(r,ground.source().identity,now_ros,now_steady)) {
    for(auto& tile:out->tiles){tile.state=sv::CellState::kUnknown;tile.support_qualified=false;}
    out->ground_qualified=false;out->evaluation_complete=false;
    return fail("ground_reference_expired");
  }
  out->reason = "sampled_above_ground_visibility_only";
  return out;
}
}  // namespace sunhorizon_perception_runtime::near_ground_clearance
