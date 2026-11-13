#include "sunhorizon_obstacle_coverage/spatial_visibility_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace sunhorizon_obstacle_coverage::spatial_visibility {
namespace {
template<std::size_t N> bool finite(const std::array<double, N>& v) {
  return std::all_of(v.begin(), v.end(), [](double x) { return std::isfinite(x); });
}
bool identity(const std::string& s) {
  return !s.empty() && s.size() <= 128 && std::all_of(s.begin(), s.end(),
      [](unsigned char c) { return c >= 33 && c <= 126; });
}
bool rigid(const RigidTransform& tf) {
  if (!finite(tf.rotation) || !finite(tf.translation)) return false;
  const auto& r = tf.rotation;
  for (std::size_t i = 0; i < 3; ++i) for (std::size_t j = 0; j < 3; ++j) {
    double dot = 0.;
    for (std::size_t k = 0; k < 3; ++k) dot += r[k*3+i] * r[k*3+j];
    if (!std::isfinite(dot) || std::abs(dot - (i == j ? 1. : 0.)) > 1.e-6) return false;
  }
  const double det = r[0]*(r[4]*r[8]-r[5]*r[7]) - r[1]*(r[3]*r[8]-r[5]*r[6]) + r[2]*(r[3]*r[7]-r[4]*r[6]);
  return std::isfinite(det) && std::abs(det - 1.) <= 1.e-6;
}
bool transform_valid(const RigidTransform& tf, const Source& s, const Query& q) {
  return tf.source_frame == s.frame_id && tf.common_frame == q.common_frame &&
      tf.source_stamp_ns == s.source_stamp_ros_ns && tf.query_stamp_ns == q.query_stamp_ns && rigid(tf);
}
bool kind_valid(ReturnKind kind) { return static_cast<unsigned>(kind) <= static_cast<unsigned>(ReturnKind::kModelConflict); }
const char* payload_error(const Source& s, const Query& q) {
  if (s.data.valueless_by_exception()) return "invalid_variant";
  if (const auto* d = std::get_if<DepthRaster>(&s.data)) {
    const auto& c = d->calibration;
    if (!(c.rectified_pinhole_verified && c.metric_axial_depth_verified && c.depth_intrinsics_registration_verified &&
        c.optical_to_base_extrinsics_verified && c.source_time_and_transform_verified)) return "unverified_depth_contract";
    if (!transform_valid(d->optical_to_common, s, q)) return "invalid_depth_transform";
    if (!d->width || !d->height || d->width > 4096 || d->height > 4096 || !d->depth_m ||
        d->row_stride_elements < d->width || d->row_stride_elements > kMaximumRasterElements ||
        d->height > kMaximumRasterElements / d->row_stride_elements ||
        d->depth_elements < (d->height-1)*d->row_stride_elements+d->width || d->depth_elements > kMaximumRasterElements)
      return "invalid_or_unbounded_raster";
    if ((d->return_kinds && (d->kind_elements < (d->height-1)*d->row_stride_elements+d->width ||
            d->kind_elements > kMaximumRasterElements)) || (!d->return_kinds && d->kind_elements)) return "invalid_kind_raster";
    if (d->return_kinds) for (std::size_t v=0;v<d->height;++v) for (std::size_t u=0;u<d->width;++u)
      if (!kind_valid(d->return_kinds[v*d->row_stride_elements+u])) return "invalid_return_kind_schema";
    if (!(std::isfinite(d->fx) && d->fx > 0. && std::isfinite(d->fy) && d->fy > 0. &&
        std::isfinite(d->cx) && std::isfinite(d->cy) && std::isfinite(d->minimum_depth_m) && d->minimum_depth_m > 0. &&
        std::isfinite(d->maximum_depth_m) && d->maximum_depth_m > d->minimum_depth_m &&
        std::isfinite(d->clear_margin_m) && d->clear_margin_m >= 0.)) return "invalid_depth_calibration";
  } else {
    const auto& p = std::get<PointReturns>(s.data);
    if (!p.metric_returns_verified || !p.source_time_and_transform_verified) return "unverified_point_contract";
    if (!transform_valid(p.sensor_to_common, s, q)) return "invalid_point_transform";
    if (p.points.size() > kMaximumPointReturns) return "unbounded_point_returns";
    if (!(std::isfinite(p.minimum_range_m) && p.minimum_range_m > 0. &&
        std::isfinite(p.maximum_range_m) && p.maximum_range_m > p.minimum_range_m)) return "invalid_point_range";
  }
  return nullptr;
}
struct SampleResult { CellState state{CellState::kUnknown}; Counts counts; };
enum class GeometryResult { kComplete, kInvalid, kBudget };
enum class Hit { kMiss, kHit, kInvalid };
Hit ray_box(const AxisAlignedCell& box, const std::array<double,3>& origin,
    const std::array<double,3>& ray, double& enter, double& exit) {
  if (!finite(ray)) return Hit::kInvalid;
  enter = 0.; exit = std::numeric_limits<double>::infinity();
  for (std::size_t i=0; i<3; ++i) {
    if (ray[i] == 0.) { if (origin[i] < box.min[i] || origin[i] > box.max[i]) return Hit::kMiss; continue; }
    double a=(box.min[i]-origin[i])/ray[i], b=(box.max[i]-origin[i])/ray[i];
    if (!std::isfinite(a) || !std::isfinite(b)) return Hit::kInvalid;
    if (a>b) std::swap(a,b);
    enter=std::max(enter,a); exit=std::min(exit,b);
    if (enter>exit) return Hit::kMiss;
  }
  if (!std::isfinite(enter) || !std::isfinite(exit)) return Hit::kInvalid;
  return exit>0. ? Hit::kHit : Hit::kMiss;
}
bool check_budget(std::size_t& checks) {
  if (checks == kMaximumChecks) return false;
  ++checks; return true;
}
GeometryResult depth_cell(const DepthRaster& d, const AxisAlignedCell& box, SampleResult& out, std::size_t& checks) {
  const auto& tf=d.optical_to_common;
  double umin=std::numeric_limits<double>::infinity(), vmin=umin, umax=-umin, vmax=-umin;
  bool full_fov=true, crosses_plane=false; unsigned positive_corners=0;
  for (unsigned corner=0; corner<8; ++corner) {
    std::array<double,3> p{};
    for (std::size_t i=0;i<3;++i) for (std::size_t j=0;j<3;++j)
      p[i]+=tf.rotation[j*3+i]*(((corner&(1U<<j))?box.max[j]:box.min[j])-tf.translation[j]);
    if (!finite(p)) return GeometryResult::kInvalid;
    if (p[2]<=0.) { crosses_plane=true; full_fov=false; continue; }
    ++positive_corners;
    const double u=d.fx*(p[0]/p[2])+d.cx, v=d.fy*(p[1]/p[2])+d.cy;
    if (!std::isfinite(u) || !std::isfinite(v)) return GeometryResult::kInvalid;
    full_fov=full_fov && u>=-.5 && v>=-.5 && u<=d.width-.5 && v<=d.height-.5;
    umin=std::min(umin,u);umax=std::max(umax,u);vmin=std::min(vmin,v);vmax=std::max(vmax,v);
  }
  if (!positive_corners) return GeometryResult::kComplete;  // Behind sensor: Unknown, not bad calibration.
  int u0=0,v0=0,u1=static_cast<int>(d.width)-1,v1=static_cast<int>(d.height)-1;
  if (!crosses_plane) {
    if (umax<-.5 || vmax<-.5 || umin>d.width-.5 || vmin>d.height-.5) return GeometryResult::kComplete;
    u0=static_cast<int>(std::max(0.,std::floor(umin)));u1=static_cast<int>(std::min(double(d.width-1),std::ceil(umax)));
    v0=static_cast<int>(std::max(0.,std::floor(vmin)));v1=static_cast<int>(std::min(double(d.height-1),std::ceil(vmax)));
  }  // Crossing the optical plane: full visible raster may contain Occupied, never Free.
  for (int v=v0;v<=v1;++v) for (int u=u0;u<=u1;++u) {
    if (!check_budget(checks)) return GeometryResult::kBudget;
    ++out.counts.tested_samples;
    const std::array<double,3> optical{(u-d.cx)/d.fx,(v-d.cy)/d.fy,1.};
    std::array<double,3> ray{};
    for (std::size_t i=0;i<3;++i) for (std::size_t j=0;j<3;++j) ray[i]+=tf.rotation[i*3+j]*optical[j];
    double enter,exit; const auto hit=ray_box(box,tf.translation,ray,enter,exit);
    if (hit==Hit::kInvalid) return GeometryResult::kInvalid;
    if (hit==Hit::kMiss) continue;
    ++out.counts.intersecting_rays;
    const auto at=static_cast<std::size_t>(v)*d.row_stride_elements+u;
    const auto kind=d.return_kinds ? d.return_kinds[at] : ReturnKind::kExternal;
    if (!kind_valid(kind)) return GeometryResult::kInvalid;
    const double depth=d.depth_m[at];
    if (kind==ReturnKind::kInvalid || !std::isfinite(depth) || depth<d.minimum_depth_m || depth>d.maximum_depth_m) {
      ++out.counts.invalid_returns; continue;
    }
    if (kind==ReturnKind::kSelfMatched) ++out.counts.self_returns;
    if (kind==ReturnKind::kModelConflict) ++out.counts.model_conflict_returns;
    if (depth<enter) { ++out.counts.front_occluded_rays; if (kind==ReturnKind::kSelfMatched) ++out.counts.self_blocked_rays; }
    else if (depth<=exit) {
      if (kind==ReturnKind::kSelfMatched) ++out.counts.self_blocked_rays;
      else ++out.counts.obstacle_returns;
    } else {
      const double boundary=exit+d.clear_margin_m;
      if (!std::isfinite(boundary)) return GeometryResult::kInvalid;
      if (depth>=boundary) ++out.counts.clear_rays;
      else ++out.counts.margin_band_rays;
    }
  }
  // Positive occupancy survives holes/partial FOV; Free requires the entire
  // projected sampling domain and every intersecting centre ray supported.
  if (out.counts.obstacle_returns) out.state=CellState::kOccupied;
  else if (full_fov && out.counts.intersecting_rays && out.counts.clear_rays==out.counts.intersecting_rays) out.state=CellState::kFree;
  return GeometryResult::kComplete;
}
GeometryResult point_cell(const PointReturns& p, const AxisAlignedCell& box, SampleResult& out, std::size_t& checks) {
  for (const auto& sample:p.points) {
    if (!check_budget(checks)) return GeometryResult::kBudget;
    ++out.counts.tested_samples;
    if (!kind_valid(sample.kind)) return GeometryResult::kInvalid;
    const double range=std::hypot(sample.position[0],sample.position[1],sample.position[2]);
    if (sample.kind==ReturnKind::kInvalid || !finite(sample.position) || !std::isfinite(range) ||
        range<p.minimum_range_m || range>p.maximum_range_m) { ++out.counts.invalid_returns;continue; }
    auto endpoint=p.sensor_to_common.translation;
    for (std::size_t i=0;i<3;++i) for (std::size_t j=0;j<3;++j) endpoint[i]+=p.sensor_to_common.rotation[i*3+j]*sample.position[j];
    if (!finite(endpoint)) return GeometryResult::kInvalid;
    bool inside=true;
    for (std::size_t i=0;i<3;++i) inside=inside && endpoint[i]>=box.min[i] && endpoint[i]<=box.max[i];
    if (!inside) continue;
    if (sample.kind==ReturnKind::kSelfMatched) ++out.counts.self_returns;
    else { ++out.counts.obstacle_returns; if (sample.kind==ReturnKind::kModelConflict) ++out.counts.model_conflict_returns; }
  }
  if (out.counts.obstacle_returns) out.state=CellState::kOccupied;
  return GeometryResult::kComplete;  // A point ray never supports whole-cell Free.
}
}  // namespace

Result evaluate(const Query& q, const std::vector<Source>& inputs, int64_t now_ros, int64_t now_steady) {
  Result out;
  auto reject=[&](const char* reason) { out.reason=reason;out.aggregate=Aggregate::kHardInvalid;out.evaluation_complete=false;return out; };
  if (!identity(q.common_frame) || !identity(q.query_id) || !identity(q.contract_id) || now_ros<=0 || now_steady<=0 ||
      q.query_stamp_ns<=0 || q.query_stamp_ns>now_ros || q.cells.empty() || q.cells.size()>kMaximumCells ||
      q.source_contracts.empty() || q.source_contracts.size()>kMaximumSources || inputs.size()>kMaximumSources) return reject("invalid_query_contract");
  out.query_id=q.query_id;out.contract_id=q.contract_id;out.query_stamp_ns=q.query_stamp_ns;
  std::set<uint32_t> cell_ids;std::set<std::string> contracts,seen;
  bool has_required=false;
  for (const auto& cell:q.cells) {
    if (!cell_ids.insert(cell.id).second || !finite(cell.min) || !finite(cell.max)) return reject("invalid_cell");
    for (std::size_t i=0;i<3;++i) if (cell.min[i]>=cell.max[i]) return reject("invalid_cell");
    CellEvidence e;e.id=cell.id;out.cells.push_back(e);
  }
  for (const auto& c:q.source_contracts) {
    if (!identity(c.source_id) || !identity(c.expected_epoch) || !contracts.insert(c.source_id).second) return reject("invalid_source_contract");
    has_required=has_required || c.required_for_evaluation;
  }
  if (!has_required) return reject("missing_required_source_contract");
  for (const auto& s:inputs) if (!contracts.count(s.source_id) || !seen.insert(s.source_id).second) return reject("duplicate_or_uncontracted_source");
  bool required_invalid=false;int64_t remaining=std::numeric_limits<int64_t>::max();
  for (std::size_t index=0;index<q.source_contracts.size();++index) {
    const auto& contract=q.source_contracts[index];SourceEvidence e;
    e.source_id=contract.source_id;e.expected_epoch=contract.expected_epoch;e.required_for_evaluation=contract.required_for_evaluation;
    const auto found=std::find_if(inputs.begin(),inputs.end(),[&](const Source& s) {return s.source_id==contract.source_id;});
    const Source* source=found==inputs.end() ? nullptr : &*found;
    if (!source) e.reason="source_missing";
    else {
      const auto& s=*source;if (identity(s.epoch)) e.epoch=s.epoch;e.frame_sequence=s.frame_sequence;e.source_stamp_ros_ns=s.source_stamp_ros_ns;
      e.local_receipt_ros_ns=s.local_receipt_ros_ns;e.local_receipt_steady_ns=s.local_receipt_steady_ns;
      e.assessment=SourceAssessment::kInvalid;
      if (!identity(s.epoch) || s.epoch!=contract.expected_epoch || !identity(s.frame_id) || !s.frame_sequence ||
          s.required_for_evaluation!=contract.required_for_evaluation) e.reason="source_identity_mismatch";
      else if (!supported_evidence_age(s.expiry_budget_ns) || s.source_stamp_ros_ns<=0 ||
          s.local_receipt_ros_ns<=0 || s.local_receipt_steady_ns<=0) e.reason="invalid_evidence_clock_or_budget";
      else if (s.source_stamp_ros_ns>now_ros || s.local_receipt_ros_ns>now_ros || s.local_receipt_steady_ns>now_steady) e.reason="future_evidence_clock";
      else {
        const auto age=std::max({now_ros-s.source_stamp_ros_ns,now_ros-s.local_receipt_ros_ns,now_steady-s.local_receipt_steady_ns});
        if (age>s.expiry_budget_ns) {e.assessment=SourceAssessment::kUnavailableExpired;e.reason="evidence_expired";}
        else if (const auto error=payload_error(s,q)) e.reason=error;
        else {e.assessment=SourceAssessment::kFresh;e.reason="fresh";e.remaining_valid_ns=s.expiry_budget_ns-age;}
      }
    }
    std::vector<SampleResult> samples(q.cells.size());
    if (e.assessment==SourceAssessment::kFresh) {
      for (std::size_t cell=0;cell<q.cells.size();++cell) {
        const auto result=std::holds_alternative<DepthRaster>(source->data) ?
            depth_cell(std::get<DepthRaster>(source->data),q.cells[cell],samples[cell],out.checks) :
            point_cell(std::get<PointReturns>(source->data),q.cells[cell],samples[cell],out.checks);
        if (result==GeometryResult::kBudget) {out.sources.push_back(e);return reject("computation_budget_exceeded");}
        if (result==GeometryResult::kInvalid) {e.assessment=SourceAssessment::kInvalid;e.reason="invalid_geometry_or_return_kind";e.remaining_valid_ns=0;break;}
      }
    }
    const uint8_t mask=uint8_t{1}<<index;
    bool contributes=false;
    for (std::size_t cell=0;cell<q.cells.size();++cell) {
      auto& target=out.cells[cell];
      if (e.assessment!=SourceAssessment::kFresh) target.unavailable_sources|=mask;
      else {
        target.counts[index]=samples[cell].counts;
        if (samples[cell].state==CellState::kOccupied) {target.occupied_sources|=mask;contributes=true;}
        else if (samples[cell].state==CellState::kFree) {target.free_sources|=mask;contributes=true;}
        else target.unknown_sources|=mask;
      }
      target.conflicting_sources=target.occupied_sources && target.free_sources;
      target.state=target.occupied_sources ? CellState::kOccupied : target.free_sources ? CellState::kFree : CellState::kUnknown;
    }
    if (e.required_for_evaluation && e.assessment!=SourceAssessment::kFresh) required_invalid=true;
    if (e.assessment==SourceAssessment::kFresh && (e.required_for_evaluation || contributes)) remaining=std::min(remaining,e.remaining_valid_ns);
    out.sources.push_back(e);
  }
  if (required_invalid) return reject("required_source_unavailable_or_invalid");
  out.evaluation_complete=true;out.min_remaining_valid_ns=remaining;
  if (std::any_of(out.cells.begin(),out.cells.end(),[](const CellEvidence& e) {return e.state==CellState::kOccupied;})) {
    out.aggregate=Aggregate::kOccupied;out.reason="positive_occupancy";
  } else if (std::all_of(out.cells.begin(),out.cells.end(),[](const CellEvidence& e) {return e.state==CellState::kFree;})) {
    out.aggregate=Aggregate::kClear;out.reason="all_cells_sampled_free";
  } else {out.aggregate=Aggregate::kVisibilityWait;out.reason="incomplete_visibility";}
  return out;
}
}  // namespace sunhorizon_obstacle_coverage::spatial_visibility
