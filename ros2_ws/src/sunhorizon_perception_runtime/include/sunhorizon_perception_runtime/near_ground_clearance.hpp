#pragma once

#include "sunhorizon_perception_runtime/classified_obstacle_frame.hpp"
#include "sunhorizon_obstacle_coverage/spatial_visibility_fusion.hpp"

namespace sunhorizon_perception_runtime::near_ground_clearance {
namespace g = ground_relative;
namespace sv = sunhorizon_obstacle_coverage::spatial_visibility;

// A fixed, explicitly configured XY domain and ONE existing coarse Z slice.
// This is independent of the original spatial query R; it never clips R.
struct TileGrid {
  std::string contract_id;
  double origin_x_m{g::kUnspecified}, origin_y_m{g::kUnspecified};
  double resolution_m{g::kUnspecified};
  uint32_t width{0}, height{0};
  double coarse_origin_z_m{g::kUnspecified}, coarse_resolution_z_m{g::kUnspecified};
  uint32_t coarse_z_index{0};
  std::size_t maximum_tiles{0};
};

struct TileEvidence {
  uint32_t id{0}, x{0}, y{0};
  std::array<double, 2> min_xy{}, max_xy{};
  double coarse_z_lower_m{g::kUnspecified};
  // Conservative maximum of the qualified ground band's upper boundary over
  // this tile. It does NOT say that all space below this height is Ground/Free.
  double floor_band_upper_z_m{g::kUnspecified};
  double query_z_lower_m{g::kUnspecified}, query_z_upper_m{g::kUnspecified};
  // The sampling query is inflated in XYZ by this source point/TF bound; the
  // proof still refers ONLY to the original tile and original query Z bounds.
  double visibility_error_bound_m{g::kUnspecified};
  bool support_qualified{false};
  sv::CellState state{sv::CellState::kUnknown};
  sv::Counts counts;
};

struct Snapshot {
  using ConstSharedPtr = std::shared_ptr<const Snapshot>;
  g::SourceIdentity identity;  // Original acquisition identity and clocks.
  std::string common_frame, contract_id, reason;
  int64_t query_stamp_ns{0}, remaining_valid_ns{0};
  // Owns the exact reference, its original clocks, support and classifications.
  g::ClassifiedObstacleFrame::ConstSharedPtr ground_frame;
  bool ground_qualified{false}, evaluation_complete{false};
  std::vector<TileEvidence> tiles;
  std::size_t visibility_checks{0};
};

// Pure bounded query preparation, also usable before coverage self-classifying
// the raster. Empty means no usable reference geometry / no above-floor volume.
// It grants no ground or clearing qualification by itself. Returned cells are
// inflated by the source point/TF error, including the furthest query corner's
// rotation lever arm; the raster's independent axial clear margin is retained.
sv::Query make_query(const g::ClassifiedObstacleFrame&, const g::ClassifierConfig&, const TileGrid&);

// Call with DecodedSource::source(); keep its backing owner alive and immutable.
// Uses a LOCAL source view transformed with ground_frame.source().to_common.
// The original decoded view and the old R transform/query are never changed.
// Every intersecting required pixel must have an explicit External kind and a
// valid measured depth beyond the query volume + existing axial clear margin.
// Invalid/masked/unclassified/Self/ModelConflict pixels cannot contribute Free.
// Missing ground qualification or local support returns Unknown, never clearing.
//
// Engine integration: create two required-pixel masks, R with the original
// raster, and make_query(...) with a raster VIEW using ground to_common; OR the
// masks, then classify every selected pixel with the SAME PreparedModel. Do not
// read unqueried raster pixels (whose kinds intentionally remain Invalid) as Free.
// A frame/time/budget failure must retain Unknown. No second model/FK is needed.
//
// Free here is sampled visibility ONLY for [query_z_lower, query_z_upper]. It
// never clears a whole coarse voxel, declares below-floor space Free, establishes
// a minimum detectable object, or admits motion. Consumer state changes remain
// separate; loss of old positive occupancy must preserve unobserved Unknown.
Snapshot::ConstSharedPtr evaluate(const sv::Source& decoded_view,
    g::ClassifiedObstacleFrame::ConstSharedPtr ground_frame,
    const g::ClassifierConfig&, const TileGrid&, int64_t now_ros_ns, int64_t now_steady_ns);
}  // namespace sunhorizon_perception_runtime::near_ground_clearance
