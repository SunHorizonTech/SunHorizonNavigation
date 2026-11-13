#include "sunhorizon_obstacle_coverage/spatial_visibility_fusion.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace sv = sunhorizon_obstacle_coverage::spatial_visibility;
#define CHECK(c) do { if (!(c)) throw std::runtime_error(std::string(__func__)+":"+std::to_string(__LINE__)+": " #c); } while (false)

namespace {
constexpr int64_t kNow=1100000000, kSteady=2100000000;
struct Fixture {
  std::vector<float> pixels=std::vector<float>(25,3.F);
  std::vector<sv::ReturnKind> kinds=std::vector<sv::ReturnKind>(25,sv::ReturnKind::kExternal);
  sv::Query query;
  Fixture() {
    query.cells={{7,{-.25,-.25,1.},{.25,.25,2.}}};query.common_frame="base_footprint";
    query.query_id="query-1";query.contract_id="sampled-visibility-v1";query.query_stamp_ns=kNow;
    query.source_contracts={{"belly","belly-epoch-1",true}};
  }
  sv::Source depth(const std::string& id="belly", bool required=true) const {
    sv::Source s;s.source_id=id;s.epoch=id+"-epoch-1";s.frame_id=id+"_optical";
    s.frame_sequence=1;s.source_stamp_ros_ns=1000000000;s.local_receipt_ros_ns=1010000000;
    s.local_receipt_steady_ns=2010000000;s.expiry_budget_ns=300000000;s.required_for_evaluation=required;
    sv::DepthRaster d;d.width=d.height=5;d.fx=d.fy=4.;d.cx=d.cy=2.;
    d.depth_m=pixels.data();d.depth_elements=pixels.size();d.row_stride_elements=5;
    d.return_kinds=kinds.data();d.kind_elements=kinds.size();
    d.minimum_depth_m=.25;d.maximum_depth_m=5.;d.clear_margin_m=.25;d.calibration={true,true,true,true,true};
    d.optical_to_common.source_frame=s.frame_id;d.optical_to_common.common_frame=query.common_frame;
    d.optical_to_common.source_stamp_ns=s.source_stamp_ros_ns;d.optical_to_common.query_stamp_ns=query.query_stamp_ns;
    s.data=d;return s;
  }
  sv::Source points() const {
    auto s=depth("airy",false);s.frame_id="airy_lidar";
    sv::PointReturns p;p.minimum_range_m=.05;p.maximum_range_m=100.;
    p.metric_returns_verified=p.source_time_and_transform_verified=true;
    p.sensor_to_common.source_frame=s.frame_id;p.sensor_to_common.common_frame=query.common_frame;
    p.sensor_to_common.source_stamp_ns=s.source_stamp_ros_ns;p.sensor_to_common.query_stamp_ns=query.query_stamp_ns;
    s.data=p;return s;
  }
  sv::Result run(std::vector<sv::Source> sources={}, int64_t ros=kNow, int64_t steady=kSteady) const {
    if (sources.empty()) sources.push_back(depth());
    return sv::evaluate(query,sources,ros,steady);
  }
};
void complete_raster_support_and_exact_original_geometry() {
  Fixture f;const auto fused=f.run();CHECK(fused.aggregate==sv::Aggregate::kClear && fused.evaluation_complete);
  CHECK(fused.cells[0].free_sources==1 && fused.cells[0].counts[0].intersecting_rays==9);
  CHECK(fused.min_remaining_valid_ns==200000000);
  // Reuse the original independently tested pure geometry as the no-self oracle.
  const auto d=std::get<sv::DepthRaster>(f.depth().data);
  sunhorizon_obstacle_coverage::RegionDepthInput in;
  in.box_min=f.query.cells[0].min;in.box_max=f.query.cells[0].max;
  in.width=d.width;in.height=d.height;in.fx=d.fx;in.fy=d.fy;in.cx=d.cx;in.cy=d.cy;
  in.depth_m=d.depth_m;in.depth_elements=d.depth_elements;in.row_stride_elements=d.row_stride_elements;
  in.optical_to_base_rotation=d.optical_to_common.rotation;in.optical_to_base_translation=d.optical_to_common.translation;
  in.minimum_depth_m=d.minimum_depth_m;in.maximum_depth_m=d.maximum_depth_m;in.clear_margin_m=d.clear_margin_m;in.contract=d.calibration;
  const auto legacy=sunhorizon_obstacle_coverage::evaluate_region_depth_support(in);
  CHECK(legacy.state==sunhorizon_obstacle_coverage::RegionSupportState::kClearSupported);
  CHECK(legacy.intersecting_rays==fused.cells[0].counts[0].intersecting_rays && legacy.clear_rays==fused.cells[0].counts[0].clear_rays);
}
void positive_occupancy_survives_hole_and_unknown_front() {
  Fixture f;f.pixels[12]=1.5F;f.pixels[11]=0.F;f.pixels[13]=.5F;
  const auto r=f.run();CHECK(r.aggregate==sv::Aggregate::kOccupied && r.evaluation_complete);
  CHECK(r.cells[0].counts[0].obstacle_returns==1 && r.cells[0].counts[0].invalid_returns==1);
  CHECK(r.cells[0].counts[0].front_occluded_rays==1 && r.cells[0].free_sources==0);
}
void self_in_front_or_inside_never_clears_behind_it() {
  Fixture f;f.kinds[12]=sv::ReturnKind::kSelfMatched;
  for (float z:{.5F,1.F,1.5F,2.F}) {
    f.pixels[12]=z;const auto r=f.run();CHECK(r.aggregate==sv::Aggregate::kVisibilityWait && r.evaluation_complete);
    CHECK(r.cells[0].counts[0].self_returns==1 && r.cells[0].counts[0].self_blocked_rays==1);
    CHECK(r.cells[0].counts[0].obstacle_returns==0 && r.cells[0].free_sources==0);
  }
  // The cell strictly before a verified self surface may be supported, but
  // another cell behind that very same sample is still Unknown.
  f.pixels[12]=3.F;CHECK(f.run().aggregate==sv::Aggregate::kClear);
  f.query.cells[0].min[2]=3.5;f.query.cells[0].max[2]=4.;
  CHECK(f.run().aggregate==sv::Aggregate::kVisibilityWait);
}
void zero_invalid_and_model_conflict_are_not_deleted() {
  Fixture f;f.kinds[12]=sv::ReturnKind::kSelfMatched;f.pixels[12]=0.F;
  CHECK(f.run().aggregate==sv::Aggregate::kVisibilityWait);
  f.kinds[12]=sv::ReturnKind::kModelConflict;f.pixels[12]=1.5F;
  auto r=f.run();CHECK(r.aggregate==sv::Aggregate::kOccupied && r.cells[0].counts[0].model_conflict_returns==1);
  f.pixels[12]=.5F;CHECK(f.run().aggregate==sv::Aggregate::kVisibilityWait);
  for (float z:{0.F,-1.F,std::numeric_limits<float>::infinity(),std::numeric_limits<float>::quiet_NaN()}) {
    f.pixels[12]=z;CHECK(f.run().aggregate==sv::Aggregate::kVisibilityWait);
  }
  f.pixels[12]=1.5F;f.kinds[12]=sv::ReturnKind::kInvalid;
  CHECK(f.run().aggregate==sv::Aggregate::kVisibilityWait);  // Explicit invalid input is a hole.
}
void partial_fov_is_unknown_but_visible_occupied_is_retained() {
  Fixture f;f.query.cells[0].min[0]=-2.;
  auto r=f.run();CHECK(r.aggregate==sv::Aggregate::kVisibilityWait && r.evaluation_complete);
  CHECK(r.sources[0].assessment==sv::SourceAssessment::kFresh);
  f.pixels[12]=1.5F;CHECK(f.run().aggregate==sv::Aggregate::kOccupied);
  f.query.cells[0].min[0]=10.;f.query.cells[0].max[0]=11.;
  CHECK(f.run().aggregate==sv::Aggregate::kVisibilityWait);
  f.query.cells[0] = {7,{-.01,-.01,-2.},{.01,.01,-1.}};
  CHECK(f.run().aggregate==sv::Aggregate::kVisibilityWait); // Behind FOV, not bad calibration.
  f.query.cells[0] = {7,{.02,.02,1.},{.03,.03,2.}};
  r=f.run();CHECK(r.aggregate==sv::Aggregate::kVisibilityWait && r.cells[0].counts[0].intersecting_rays==0);
}
void cell_crossing_camera_plane_can_only_be_unknown_or_occupied() {
  Fixture f;f.query.cells[0]={7,{-.25,-.25,-1.},{.25,.25,2.}};
  CHECK(f.run().aggregate==sv::Aggregate::kVisibilityWait);
  f.pixels[12]=1.5F;CHECK(f.run().aggregate==sv::Aggregate::kOccupied);
}
void margins_endpoints_and_stride_are_conservative() {
  Fixture f;
  for (float z:{1.F,2.F}) { f.pixels[12]=z;CHECK(f.run().aggregate==sv::Aggregate::kOccupied); }
  f.pixels[12]=2.125F;CHECK(f.run().aggregate==sv::Aggregate::kVisibilityWait);
  f.pixels[12]=2.25F;CHECK(f.run().aggregate==sv::Aggregate::kClear);
  auto s=f.depth();std::get<sv::DepthRaster>(s.data).clear_margin_m=0.;f.pixels[12]=2.F;
  CHECK(f.run({s}).aggregate==sv::Aggregate::kOccupied); // Exit equality is never free.
  f.pixels[12]=std::nextafter(2.F,3.F);CHECK(f.run({s}).aggregate==sv::Aggregate::kClear);
  std::vector<float> padded(35,0.F);for (int v=0;v<5;++v) for (int u=0;u<5;++u) padded[v*7+u]=3.F;
  auto& d=std::get<sv::DepthRaster>(s.data);d.depth_m=padded.data();d.depth_elements=padded.size();d.row_stride_elements=7;d.return_kinds=nullptr;d.kind_elements=0;
  CHECK(f.run({s}).aggregate==sv::Aggregate::kClear); // Padding is not a depth sample.
}
void points_are_positive_only_and_cannot_clear_a_traversed_cell() {
  Fixture f;f.pixels[12]=0.F;f.query.source_contracts.push_back({"airy","airy-epoch-1",false});auto airy=f.points();
  std::get<sv::PointReturns>(airy.data).points={{{0.,0.,3.},sv::ReturnKind::kExternal}};
  CHECK(f.run({f.depth(),airy}).aggregate==sv::Aggregate::kVisibilityWait);
  std::get<sv::PointReturns>(airy.data).points={{{0.,0.,1.5},sv::ReturnKind::kExternal}};
  CHECK(f.run({f.depth(),airy}).aggregate==sv::Aggregate::kOccupied);
  std::get<sv::PointReturns>(airy.data).points[0].kind=sv::ReturnKind::kSelfMatched;
  CHECK(f.run({f.depth(),airy}).aggregate==sv::Aggregate::kVisibilityWait);
  std::get<sv::PointReturns>(airy.data).points[0].kind=sv::ReturnKind::kModelConflict;
  CHECK(f.run({f.depth(),airy}).aggregate==sv::Aggregate::kOccupied);
  std::get<sv::PointReturns>(airy.data).points[0].position={0.,0.,0.};
  CHECK(f.run({f.depth(),airy}).aggregate==sv::Aggregate::kVisibilityWait);
}
void different_sources_fill_unknown_and_occupied_wins_conflict() {
  Fixture f;Fixture chest;f.query.source_contracts.push_back({"chest","chest-epoch-1",false});f.pixels[12]=0.F;
  auto c=chest.depth("chest",false);CHECK(f.run({f.depth(),c}).aggregate==sv::Aggregate::kClear);
  f.pixels[12]=1.5F;const auto r=f.run({f.depth(),c});CHECK(r.aggregate==sv::Aggregate::kOccupied);
  CHECK(r.cells[0].occupied_sources==1 && r.cells[0].free_sources==2 && r.cells[0].conflicting_sources);
  // A point-cloud positive endpoint also overrides a complete depth source.
  f.pixels[12]=3.F;f.query.source_contracts.push_back({"airy","airy-epoch-1",false});auto airy=f.points();
  std::get<sv::PointReturns>(airy.data).points={{{0.,0.,1.5},sv::ReturnKind::kExternal}};
  CHECK(f.run({f.depth(),c,airy}).cells[0].occupied_sources==4);
}
void all_three_age_clocks_and_exact_300ms_boundary_are_checked() {
  Fixture f;auto s=f.depth();auto r=f.run({s},1300000000,2300000000);
  CHECK(r.aggregate==sv::Aggregate::kClear && r.min_remaining_valid_ns==0);
  CHECK(f.run({s},1300000001,2300000000).aggregate==sv::Aggregate::kHardInvalid);
  for (int clock=0;clock<3;++clock) {
    auto old=s;
    if (clock==0) old.source_stamp_ros_ns=799999999;
    if (clock==1) old.local_receipt_ros_ns=799999999;
    if (clock==2) old.local_receipt_steady_ns=1799999999;
    if (clock==0) std::get<sv::DepthRaster>(old.data).optical_to_common.source_stamp_ns=old.source_stamp_ros_ns;
    r=f.run({old});CHECK(r.aggregate==sv::Aggregate::kHardInvalid && !r.evaluation_complete);
    CHECK(r.sources[0].assessment==sv::SourceAssessment::kUnavailableExpired);
  }
  for (int clock=0;clock<3;++clock) {
    auto future=s;
    if (clock==0) future.source_stamp_ros_ns=kNow+1;
    if (clock==1) future.local_receipt_ros_ns=kNow+1;
    if (clock==2) future.local_receipt_steady_ns=kSteady+1;
    CHECK(f.run({future}).aggregate==sv::Aggregate::kHardInvalid);
  }
  for (int64_t budget:{int64_t{0},int64_t{-1},int64_t{500000001}}) {auto bad=s;bad.expiry_budget_ns=budget;CHECK(f.run({bad}).aggregate==sv::Aggregate::kHardInvalid);}
}
void optional_expired_or_invalid_source_cannot_fill_or_occupy() {
  Fixture f;f.pixels[12]=0.F;Fixture chest;f.query.source_contracts.push_back({"chest","chest-epoch-1",false});
  auto c=chest.depth("chest",false);c.source_stamp_ros_ns=700000000;
  auto r=f.run({f.depth(),c});CHECK(r.aggregate==sv::Aggregate::kVisibilityWait && r.evaluation_complete);
  CHECK(r.sources[1].assessment==sv::SourceAssessment::kUnavailableExpired && r.cells[0].unavailable_sources==2);
  c=chest.depth("chest",false);c.epoch="foreign-epoch";CHECK(f.run({f.depth(),c}).aggregate==sv::Aggregate::kVisibilityWait);
  c=chest.depth("chest",false);std::get<sv::DepthRaster>(c.data).calibration.metric_axial_depth_verified=false;
  CHECK(f.run({f.depth(),c}).aggregate==sv::Aggregate::kVisibilityWait);
  f.query.source_contracts.push_back({"airy","airy-epoch-1",false});auto airy=f.points();airy.source_stamp_ros_ns=700000000;
  std::get<sv::PointReturns>(airy.data).points={{{0.,0.,1.5},sv::ReturnKind::kExternal}};
  CHECK(f.run({f.depth(),airy}).aggregate==sv::Aggregate::kVisibilityWait);
  CHECK(f.run({f.depth()}).aggregate==sv::Aggregate::kVisibilityWait); // Missing optional sources.
}
void epochs_ids_required_flags_and_repeated_timer_never_rebind() {
  Fixture f;auto s=f.depth();s.epoch="belly-epoch-2";
  CHECK(f.run({s}).aggregate==sv::Aggregate::kHardInvalid);
  s=f.depth();CHECK(f.run({s,s}).aggregate==sv::Aggregate::kHardInvalid);
  auto other=s;other.epoch="another";CHECK(f.run({s,other}).aggregate==sv::Aggregate::kHardInvalid);
  s.required_for_evaluation=false;CHECK(f.run({s}).aggregate==sv::Aggregate::kHardInvalid);
  s=f.depth();s.frame_sequence=0;CHECK(f.run({s}).aggregate==sv::Aggregate::kHardInvalid);
  s=f.depth();auto a=f.run({s}), b=f.run({s},1200000000,2200000000);
  CHECK(a.sources[0].epoch==b.sources[0].epoch && a.sources[0].frame_sequence==b.sources[0].frame_sequence);
  CHECK(a.sources[0].local_receipt_ros_ns==b.sources[0].local_receipt_ros_ns && b.min_remaining_valid_ns<a.min_remaining_valid_ns);
  // Even erroneously refreshed receipt cannot rescue an expired source stamp.
  s.local_receipt_ros_ns=1400000000;s.local_receipt_steady_ns=2400000000;
  CHECK(f.run({s},1400000000,2400000000).aggregate==sv::Aggregate::kHardInvalid);
  CHECK(s.source_stamp_ros_ns==1000000000 && s.frame_sequence==1);
  CHECK(sv::evaluate(f.query,{},kNow,kSteady).aggregate==sv::Aggregate::kHardInvalid);
}
void query_and_payload_interfaces_fail_closed() {
  Fixture f;auto s=f.depth();
  for (int fault=0;fault<15;++fault) {
    auto q=f.query;auto bad=s;
    auto& d=std::get<sv::DepthRaster>(bad.data);
    if (fault==0) q.cells.clear();if (fault==1) q.cells[0].max[0]=q.cells[0].min[0];
    if (fault==2) q.cells[0].min[0]=std::numeric_limits<double>::quiet_NaN();
    if (fault==3) q.cells.push_back(q.cells[0]);if (fault==4) q.common_frame.clear();
    if (fault==5) q.query_stamp_ns=kNow+1;if (fault==6) q.source_contracts[0].expected_epoch.clear();
    if (fault==7) d.fx=0.;if (fault==8) d.row_stride_elements=4;if (fault==9) d.depth_m=nullptr;
    if (fault==10) d.calibration.depth_intrinsics_registration_verified=false;
    if (fault==11) d.optical_to_common.rotation[0]=-1.;if (fault==12) d.optical_to_common.source_stamp_ns++;
    if (fault==13) d.optical_to_common.query_stamp_ns++;if (fault==14) d.optical_to_common.common_frame="other";
    const auto r=sv::evaluate(q,{bad},kNow,kSteady);CHECK(r.aggregate==sv::Aggregate::kHardInvalid && !r.evaluation_complete);
  }
  f.kinds[0]=static_cast<sv::ReturnKind>(99);CHECK(f.run().aggregate==sv::Aggregate::kHardInvalid); // Schema error, even outside ROI.
}
void rotated_rigid_source_and_explicit_dual_time_tf() {
  Fixture f;f.query.cells[0]={7,{1.,-.25,-.25},{2.,.25,.25}};
  auto s=f.depth();auto& d=std::get<sv::DepthRaster>(s.data);
  d.optical_to_common.rotation={0.,0.,1.,1.,0.,0.,0.,1.,0.};
  CHECK(f.run({s}).aggregate==sv::Aggregate::kClear);
  d.optical_to_common.query_stamp_ns=d.optical_to_common.source_stamp_ns;
  CHECK(f.run({s}).aggregate==sv::Aggregate::kHardInvalid); // Source-time-only TF is not target-time compensation.
}
void finite_work_and_cell_source_bounds_are_hard_limits() {
  Fixture f;
  for (uint32_t i=1;i<=sv::kMaximumCells;++i) f.query.cells.push_back({i+7,{-.25,-.25,1.},{.25,.25,2.}});
  CHECK(f.run().aggregate==sv::Aggregate::kHardInvalid);
  f.query.cells.resize(sv::kMaximumCells);f.query.source_contracts.push_back({"airy","airy-epoch-1",false});
  auto airy=f.points();std::get<sv::PointReturns>(airy.data).points.resize(4097); // >16M checks across 4096 cells.
  const auto r=f.run({f.depth(),airy});CHECK(r.aggregate==sv::Aggregate::kHardInvalid && !r.evaluation_complete);
  CHECK(r.reason=="computation_budget_exceeded" && r.checks==sv::kMaximumChecks);
  auto q=f.query;q.source_contracts.push_back({"chest","chest-epoch-1",false});q.source_contracts.push_back({"extra","extra",false});
  CHECK(sv::evaluate(q,{f.depth()},kNow,kSteady).aggregate==sv::Aggregate::kHardInvalid);
}
}  // namespace

void explicit_half_second_lease_expires_at_source_time() {
  Fixture f;auto s=f.depth();s.expiry_budget_ns=500000000;
  CHECK(f.run({s},1500000000,2500000000).aggregate==sv::Aggregate::kClear);
  CHECK(f.run({s},1500000001,2500000001).aggregate==sv::Aggregate::kHardInvalid);
  s.expiry_budget_ns=1000000000;
  CHECK(f.run({s},2000000000,3000000000).aggregate==sv::Aggregate::kClear);
  CHECK(f.run({s},2000000001,3000000001).aggregate==sv::Aggregate::kHardInvalid);
  s.expiry_budget_ns=500000001;
  CHECK(f.run({s},1100000000,2100000000).aggregate==sv::Aggregate::kHardInvalid);
}
int main() {
  explicit_half_second_lease_expires_at_source_time();
  const std::pair<const char*,void(*)()> tests[]={
    {"complete sampling/original geometry",complete_raster_support_and_exact_original_geometry},
    {"occupied survives unknown",positive_occupancy_survives_hole_and_unknown_front},
    {"self preserves unseen space",self_in_front_or_inside_never_clears_behind_it},
    {"zero/model conflict",zero_invalid_and_model_conflict_are_not_deleted},
    {"partial/outside FOV",partial_fov_is_unknown_but_visible_occupied_is_retained},
    {"crossing optical plane",cell_crossing_camera_plane_can_only_be_unknown_or_occupied},
    {"margin/endpoint/stride",margins_endpoints_and_stride_are_conservative},
    {"positive-only points",points_are_positive_only_and_cannot_clear_a_traversed_cell},
    {"source fusion/conflicts",different_sources_fill_unknown_and_occupied_wins_conflict},
    {"independent clocks/300ms",all_three_age_clocks_and_exact_300ms_boundary_are_checked},
    {"optional unavailable",optional_expired_or_invalid_source_cannot_fill_or_occupy},
    {"epoch/generation/expiry",epochs_ids_required_flags_and_repeated_timer_never_rebind},
    {"invalid interfaces",query_and_payload_interfaces_fail_closed},
    {"rigid dual-time TF",rotated_rigid_source_and_explicit_dual_time_tf},
    {"bounded work",finite_work_and_cell_source_bounds_are_hard_limits}};
  for (const auto& test:tests) {test.second();std::cout<<"PASS "<<test.first<<'\n';}
  std::cout<<"15 spatial visibility test groups passed\n";
}
