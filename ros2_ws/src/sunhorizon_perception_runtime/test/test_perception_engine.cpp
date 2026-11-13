#include "sunhorizon_perception_runtime/perception_engine.hpp"
#include "sunhorizon_perception_runtime/collision_height_query.hpp"
#include "sunhorizon_perception_runtime/ground_classification_snapshot.hpp"

#include <cstring>
#include <cstdlib>
#include <atomic>
#include <chrono>
#include <future>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include "sunhorizon_camera_runtime/latest_frame_worker.hpp"
#include "sunhorizon_rgbd_adapter/region_depth_pairing.hpp"
#include <iostream>
#include <stdexcept>
#include <utility>

using namespace sunhorizon_perception_runtime;
namespace rgbd = sunhorizon_rgbd_adapter;
namespace {
int checks=0;
#define CHECK(x) do {++checks;if (!(x)) throw std::runtime_error(#x);} while (false)
constexpr int64_t stamp=1000000000LL, receipt=1010000000LL, steady=2010000000LL;
template<class Function> void rejects(Function&& function) {
  bool rejected=false;try {function();} catch (const std::exception&) {rejected=true;}CHECK(rejected);
}
std::vector<dg::Triangle> plane(double z,double left=-20.,double right=20.) {
  return {{{{{left,-20,z},{right,-20,z},{right,20,z}}}},
          {{{{left,-20,z},{right,20,z},{left,20,z}}}}};
}
struct Model {
  dg::Mesh mesh;
  std::vector<dg::LinkObservation> links;
  explicit Model(double z=1.) {
    CHECK(mesh.build(plane(z),{}));
    for (unsigned i=0;i<12;++i) links.push_back({&mesh,"link_"+std::to_string(i),{},true,true,true});
  }
};
struct Fixture {
  std::shared_ptr<sensor_msgs::msg::Image> image=std::make_shared<sensor_msgs::msg::Image>();
  std::shared_ptr<sensor_msgs::msg::CameraInfo> info=std::make_shared<sensor_msgs::msg::CameraInfo>();
  FrameInput input;
  sv::Query query;
  explicit Fixture(uint32_t width=3,uint32_t height=1,bool big=false,
      const std::string& source="belly",int64_t source_stamp=stamp) {
    image->width=width;image->height=height;image->step=width*2+4;
    image->encoding="16UC1";image->is_bigendian=big;image->data.resize(image->step*height,0xa5);
    image->header.frame_id=source+"_optical";
    image->header.stamp.sec=int32_t(source_stamp/1000000000LL);
    image->header.stamp.nanosec=uint32_t(source_stamp%1000000000LL);
    info->header=image->header;info->width=width;info->height=height;
    info->k={10,0,double(width-1)/2,0,10,double(height-1)/2,0,0,1};
    for (uint32_t i=0;i<width*height;++i) set(i,1000);
    query.common_frame="base_footprint";query.query_id="query";query.contract_id="visibility-v1";
    query.query_stamp_ns=stamp;query.cells={{1,{-.01,-.01,1.2},{.01,.01,1.3}}};
    query.source_contracts={{"belly","belly-epoch",true},{"chest","chest-epoch",false}};
    input.image=image;input.info=info;
    input.identity={source,source+"-epoch","calibration-1",7,source_stamp,
        source_stamp+10000000LL,steady+(source_stamp-stamp),300000000LL,source=="belly"};
    input.calibration={true,true,true,true,true};
    input.optical_to_common.source_frame=image->header.frame_id;
    input.optical_to_common.common_frame=query.common_frame;
    input.optical_to_common.source_stamp_ns=source_stamp;input.optical_to_common.query_stamp_ns=query.query_stamp_ns;
  }
  void set(std::size_t pixel,uint16_t raw) {
    const auto byte=pixel/image->width*image->step+(pixel%image->width)*2;
    image->data[byte+(image->is_bigendian?1:0)]=uint8_t(raw);
    image->data[byte+(image->is_bigendian?0:1)]=uint8_t(raw>>8);
  }
  sv::Result evaluate(const DecodedSource& source,int64_t now=1100000000LL,int64_t now_steady=2100000000LL) {
    return PerceptionEngine::evaluate(query,{&source},now,now_steady).visibility;
  }
};
rgbd::DynamicCloudView view(const sensor_msgs::msg::PointCloud2& cloud) {
  return {cloud.data.data(),cloud.data.size(),cloud.width,cloud.height,cloud.point_step,
      cloud.row_step,bool(cloud.is_bigendian),{rgbd::DynamicPointField{0,4},{4,4},{8,4}}};
}
void body_parity_and_original_identity() {
  Model model;
  for (bool big:{false,true}) for (uint32_t stride:{1U,2U,5U}) {
    Fixture f(7,3,big);EngineConfig config;config.body_stride=stride;
    for (unsigned i=0;i<21;++i) {const uint16_t raw[]{0,900,1000,1100,8000,100,9000};f.set(i,raw[i%7]);}
    PerceptionEngine engine(config);auto result=engine.process(f.input,model.links,f.query);
    const auto projected=rgbd::project_depth(*f.image,*f.info,.001,.1,8.,stride);
    const auto legacy=rgbd::filter_dynamic_self_points(view(projected),model.links,.02,config.body_budget);
    CHECK(result.body_filter.complete==legacy.complete);
    CHECK(result.body_cloud.data==legacy.data);
    CHECK(result.body_filter.input_count==legacy.input_count);
    CHECK(result.body_filter.classifications==legacy.classifications);
    CHECK(result.body_filter.node_visits==legacy.node_visits);
    CHECK(result.body_filter.triangle_tests==legacy.triangle_tests);
    CHECK(result.stats.model_preparations==1);
    CHECK(result.timings.total.cpu_ms>=0. && result.timings.total.wall_ms>=0.);
    const auto source=result.decoded.source();
    CHECK(source.source_stamp_ros_ns==stamp && source.local_receipt_ros_ns==receipt);
    CHECK(source.local_receipt_steady_ns==steady && source.frame_sequence==7);
    CHECK(result.decoded.identity().calibration_id=="calibration-1");
    const dg::PreparedModel prepared(model.links,config.body_budget.query);
    auto shared=engine.process(f.input,prepared,f.query);
    CHECK(shared.stats.model_preparations==0);
    CHECK(shared.body_cloud.data==result.body_cloud.data);
    CHECK(shared.decoded.kinds()==result.decoded.kinds());
    CHECK(!result.body_model_diagnostic && !shared.body_model_diagnostic);
    f.input.capture_body_model_diagnostic=true;
    auto diagnostic=engine.process(f.input,prepared,f.query);
    CHECK(diagnostic.body_cloud.data==shared.body_cloud.data);
    CHECK(diagnostic.body_filter.ground_blocked==shared.body_filter.ground_blocked);
    CHECK(diagnostic.body_filter.classifications==shared.body_filter.classifications);
    CHECK(bool(diagnostic.body_model_diagnostic));
    const auto& snapshot=*diagnostic.body_model_diagnostic;
    CHECK(snapshot.identity.sequence==f.input.identity.sequence);
    CHECK(snapshot.identity.source_stamp_ros_ns==f.input.identity.source_stamp_ros_ns);
    CHECK(snapshot.identity.epoch==f.input.identity.epoch);
    CHECK(snapshot.self_tolerance_m==config.self_tolerance_m && snapshot.links.size()==12);
    const auto expected=prepared.diagnostic_links();
    for(std::size_t i=0;i<expected.size();++i) {
      CHECK(snapshot.links[i].link_index==expected[i].link_index);
      CHECK(snapshot.links[i].mesh_revision==expected[i].mesh_revision);
      CHECK(snapshot.links[i].triangle_count==expected[i].triangle_count);
      CHECK(snapshot.links[i].optical_from_link.rotation==expected[i].optical_from_link.rotation);
      CHECK(snapshot.links[i].optical_from_link.translation.x==expected[i].optical_from_link.translation.x);
    }
    auto next_links=model.links;next_links[0].optical_from_link.translation.x+=.123;
    const dg::PreparedModel next_model(next_links,config.body_budget.query);
    ++f.input.identity.sequence;
    auto next=engine.process(f.input,next_model,f.query);
    CHECK(next.body_model_diagnostic->identity.sequence==snapshot.identity.sequence+1);
    CHECK(next.body_model_diagnostic->links[0].optical_from_link.translation.x!=snapshot.links[0].optical_from_link.translation.x);

  }
}
void self_quality_far_thin_and_cutoff_rounding() {
  Model model;Fixture f;PerceptionEngine engine({});
  auto self=engine.process(f.input,model.links,f.query);
  CHECK(self.body_filter.complete && self.body_cloud.data.empty());
  CHECK(f.evaluate(self.decoded).aggregate==sv::Aggregate::kVisibilityWait);
  CHECK(f.evaluate(self.decoded).cells[0].counts[0].self_blocked_rays==1);
  f.query.cells[0]={1,{-.01,-.01,.7},{.01,.01,.8}};
  f.set(0,0);f.set(2,0);f.set(1,900);
  auto unmasked=engine.process(f.input,model.links,f.query);
  CHECK(f.evaluate(unmasked.decoded).aggregate==sv::Aggregate::kClear);
  EngineConfig config;config.component_minimum_pixels=2;config.component_maximum_depth_m=1.5;
  PerceptionEngine masked_engine(config);auto masked=masked_engine.process(f.input,model.links,f.query);
  CHECK(masked.stats.quality_rejected_pixels==1);
  CHECK(masked.decoded.kinds()[1]==sv::ReturnKind::kInvalid);
  CHECK(f.evaluate(masked.decoded).aggregate==sv::Aggregate::kVisibilityWait);
  CHECK(!masked.body_filter.complete && masked.stats.body_projected_pixels==0);
  // A distant one-pixel return remains in body and coverage despite its small
  // component; it is ModelConflict against this model, so it must be retained.
  f.set(1,2000);auto far=masked_engine.process(f.input,model.links,f.query);
  CHECK(far.body_cloud.width==1 && far.stats.quality_rejected_pixels==0);
  CHECK(far.decoded.kinds()[1]==sv::ReturnKind::kModelConflict);
  CHECK(f.evaluate(far.decoded).aggregate==sv::Aggregate::kClear);
  // Raw double Z exactly at the component cutoff rounds above it as FLOAT32.
  // The rejected pixel must still be Unknown, never resurrected as External.
  config.depth_scale_m=.001003;config.component_maximum_depth_m=1.003;
  PerceptionEngine rounding(config);f.set(1,1000);
  auto rejected=rounding.process(f.input,model.links,f.query);
  CHECK(double(rejected.decoded.depth()[1])>config.component_maximum_depth_m);
  CHECK(rejected.decoded.kinds()[1]==sv::ReturnKind::kInvalid);
  CHECK(f.evaluate(rejected.decoded).aggregate==sv::Aggregate::kVisibilityWait);
  // Reject the whole near component only if every member is in the stronger
  // range; use the established component library's rule without reimplementing.
  config={};config.component_minimum_pixels=1;config.near_component_minimum_pixels=3;
  config.near_component_maximum_depth_m=1.5;config.component_maximum_depth_m=8.;
  PerceptionEngine near(config);f.set(0,900);f.set(1,900);f.set(2,0);
  CHECK(near.process(f.input,model.links,f.query).stats.quality_rejected_pixels==2);
}
void collision_height_retains_unknown_inside_the_responsibility_box() {
  // R projects to u/v [.5,5.5] with this camera, fully inside the raster.
  // A one-column camera would retain partial-FOV Unknown even after clipping.
  Model model(10.);Fixture f(7,7);PerceptionEngine engine({});
  f.query.cells={{1,{.2,.15,-.8},{.3,.25,-.7}}};
  f.input.optical_to_common.rotation={0.,0.,1.,1.,0.,0.,0.,1.,0.};
  f.input.optical_to_common.translation={0.,.2,-.75};
  constexpr std::size_t lower=2*7+3,retained=3*7+3;
  f.set(lower,0); // This ray intersects only the lower half of the original R.
  auto original=engine.process(f.input,model.links,f.query);
  const auto full_result=f.evaluate(original.decoded);
  CHECK(full_result.aggregate==sv::Aggregate::kVisibilityWait);
  CHECK(full_result.cells[0].counts[0].invalid_returns==1);
  SpatialQueryPolicy policy;policy.mode="planar-collision-height-v1";
  const auto boxes=detail::effective_spatial_boxes(policy,"base_footprint",{{{.2,.15,-.8},{.3,.25,-.7}}});
  f.query.cells[0].min=boxes[0].min;f.query.cells[0].max=boxes[0].max;
  auto limited=engine.process(f.input,model.links,f.query);
  const auto limited_result=f.evaluate(limited.decoded);
  CHECK(limited_result.aggregate==sv::Aggregate::kClear);
  CHECK(limited_result.cells[0].counts[0].invalid_returns==0);
  CHECK(limited.body_cloud.data==original.body_cloud.data);
  CHECK(limited.body_filter.classifications==original.body_filter.classifications);
  f.set(retained,0); // A missing ray in the retained collision domain remains Unknown.
  auto missing=engine.process(f.input,model.links,f.query);
  CHECK(f.evaluate(missing.decoded).aggregate==sv::Aggregate::kVisibilityWait);
  f.set(retained,1000);
  EngineConfig masked;masked.component_minimum_pixels=50;masked.component_maximum_depth_m=1.5;
  PerceptionEngine quality(masked);auto rejected=quality.process(f.input,model.links,f.query);
  CHECK(rejected.stats.quality_rejected_pixels>0);
  CHECK(f.evaluate(rejected.decoded).aggregate==sv::Aggregate::kVisibilityWait);
}
void independent_chest_belly_time_and_owned_views() {
  Model model;PerceptionEngine engine({});Fixture belly,chest(3,1,false,"chest",1100000000LL);
  belly.query.query_stamp_ns=1100000000LL;
  belly.input.optical_to_common.query_stamp_ns=belly.query.query_stamp_ns;
  chest.query=belly.query;chest.input.optical_to_common.query_stamp_ns=belly.query.query_stamp_ns;
  belly.query.cells[0]={1,{.98,-.01,.7},{1.02,.01,.8}};chest.query=belly.query;
  chest.input.optical_to_common.translation[0]=1.;
  for (unsigned i=0;i<3;++i) chest.set(i,2000);
  auto b=engine.process(belly.input,model.links,belly.query);
  auto c=engine.process(chest.input,model.links,chest.query);
  CHECK(belly.evaluate(b.decoded,1200000000LL,2200000000LL).aggregate==sv::Aggregate::kVisibilityWait);
  const auto fused=PerceptionEngine::evaluate(belly.query,{&b.decoded,&c.decoded},1200000000LL,2200000000LL);
  CHECK(fused.visibility.aggregate==sv::Aggregate::kClear);
  CHECK(c.decoded.source().source_stamp_ros_ns==1100000000LL);
  const auto transform=std::get<sv::DepthRaster>(c.decoded.source().data).optical_to_common;
  CHECK(transform.source_stamp_ns==1100000000LL && transform.query_stamp_ns==belly.query.query_stamp_ns);
  // At this instant chest remains fresh while the required belly is expired.
  const auto stale=PerceptionEngine::evaluate(belly.query,{&b.decoded,&c.decoded},1310000000LL,2310000000LL);
  CHECK(stale.visibility.aggregate==sv::Aggregate::kHardInvalid);
  CHECK(stale.visibility.sources[0].assessment==sv::SourceAssessment::kUnavailableExpired);
  CHECK(stale.visibility.sources[1].assessment==sv::SourceAssessment::kFresh);
  // Copy/move/vector relocation must bind pointers to the surviving owner's
  // data. No raw pointer is cached in DecodedSource.
  auto copy=c.decoded;std::vector<DecodedSource> relocated;
  relocated.push_back(std::move(copy));relocated.push_back(b.decoded);
  const auto copied_source=relocated.front().source();
  CHECK(std::get<sv::DepthRaster>(copied_source.data).depth_m==relocated.front().depth().data());
  CHECK(std::get<sv::DepthRaster>(copied_source.data).return_kinds==relocated.front().kinds().data());
  CHECK(std::get<sv::DepthRaster>(copied_source.data).depth_m!=c.decoded.depth().data());
  CHECK(PerceptionEngine::evaluate(belly.query,{&relocated[1],&relocated[0]},1200000000LL,2200000000LL).visibility.aggregate==sv::Aggregate::kClear);
  // Excluding optional chest from spatial fusion still computes chest body.
  chest.query.source_contracts.pop_back();
  auto body_only=engine.process(chest.input,model.links,chest.query);
  CHECK(body_only.body_filter.complete && body_only.body_cloud.width==3);
  CHECK(PerceptionEngine::evaluate(chest.query,{&b.decoded},1200000000LL,2200000000LL).visibility.aggregate==sv::Aggregate::kVisibilityWait);
  CHECK(PerceptionEngine::evaluate(chest.query,{&b.decoded,&body_only.decoded},1200000000LL,2200000000LL).visibility.aggregate==sv::Aggregate::kHardInvalid);
}
void occupancy_conflict_and_distinct_geometric_rays() {
  Model model;Fixture f;PerceptionEngine engine({});
  f.query.cells[0]={1,{-.01,-.01,1.05},{.01,.01,1.15}};
  f.set(1,1100);auto conflict=engine.process(f.input,model.links,f.query);
  CHECK(conflict.decoded.kinds()[1]==sv::ReturnKind::kModelConflict);
  CHECK(f.evaluate(conflict.decoded).aggregate==sv::Aggregate::kOccupied);
  CHECK(conflict.body_cloud.width==1);
  Fixture edge(1,1);edge.info->k={.3,0,-.1,0,1,0,0,0,1};
  edge.query.cells[0]={1,{.32,-.01,1.},{.35,.01,1.01}};
  EngineConfig config;config.depth_scale_m=.001003;config.self_tolerance_m=1.e-10;
  const auto projected=rgbd::project_depth(*edge.image,*edge.info,config.depth_scale_m,.1,8.,1);
  float x,z;std::memcpy(&x,projected.data.data(),4);std::memcpy(&z,projected.data.data()+8,4);
  const double ray=-edge.info->k[2]/edge.info->k[0];
  const double boundary=(double(x)+ray*double(z))*.5;
  CHECK(model.mesh.build(double(x)<ray*double(z)?plane(z,-20,boundary):plane(z,boundary,20),{}));
  const dg::PreparedModel prepared(model.links,config.body_budget.query);
  const auto rounded=dg::classify({double(x)/z,0,1},z,config.self_tolerance_m,prepared);
  const auto pixel=dg::classify({ray,0,1},z,config.self_tolerance_m,prepared);
  CHECK(rounded.classification!=pixel.classification);
  PerceptionEngine edge_engine(config);auto result=edge_engine.process(edge.input,prepared,edge.query);
  CHECK(result.body_filter.removed_count==(rounded.classification==dg::Classification::SelfSurface?1U:0U));
  CHECK(result.decoded.kinds()[0]==(pixel.classification==dg::Classification::SelfSurface?
      sv::ReturnKind::kSelfMatched:sv::ReturnKind::kExternal));
}
void unqualified_optional_coverage_never_blocks_accepted_body_or_contributes() {
  Model model;Fixture belly,chest(3,1,false,"chest");
  for (unsigned i=0;i<3;++i) chest.set(i,2000);
  PerceptionEngine qualified({});auto b=qualified.process(belly.input,model.links,belly.query);
  const auto reference=qualified.process(chest.input,model.links,chest.query);
  EngineConfig config;config.coverage_enabled=false;PerceptionEngine body_only(config);
  chest.input.calibration={false,false,false,false,false};
  const auto unqualified=body_only.process(chest.input,model.links,chest.query);
  CHECK(unqualified.body_filter.complete && unqualified.body_cloud.data==reference.body_cloud.data);
  CHECK(unqualified.stats.coverage_required_pixels==0 && unqualified.stats.coverage_queries==0);
  for (const auto kind:unqualified.decoded.kinds()) CHECK(kind==sv::ReturnKind::kInvalid);
  const auto fused=PerceptionEngine::evaluate(belly.query,{&b.decoded,&unqualified.decoded},1100000000LL,2100000000LL);
  CHECK(fused.visibility.aggregate==sv::Aggregate::kVisibilityWait);
  CHECK(fused.visibility.cells[0].free_sources==0 && fused.visibility.cells[0].occupied_sources==0);
  CHECK(fused.visibility.sources[1].assessment==sv::SourceAssessment::kInvalid);
  auto bad=chest.input;bad.optical_to_common.source_stamp_ns++;
  rejects([&]{body_only.process(bad,model.links,chest.query);});
  auto stale=model.links;stale[5].covers_source_time=false;
  rejects([&]{body_only.process(chest.input,stale,chest.query);});
}
void original_raw_projection_is_independent_and_preserves_headers() {
  Model model;Fixture belly(7,3),chest(7,3,false,"chest");
  EngineConfig config;config.raw_projection_enabled=true;config.body_stride=5;
  config.raw_projection_stride=2;config.component_minimum_pixels=200;
  // Belly's original raw lane ignores its body's 200-pixel component mask.
  PerceptionEngine raw(config);
  auto b=raw.project_raw(belly.input);
  const auto original=rgbd::project_depth(*belly.image,*belly.info,.001,.1,8.,2);
  CHECK(b.raw_projection_complete && b.raw_projection_cloud->data==original.data);
  CHECK(b.raw_projection_source_ns==stamp && b.raw_projection_raw_source_ns==stamp);
  CHECK(b.raw_projection_expiry_budget_ns==300000000LL);
  CHECK(b.decoded.identity().receipt_ros_ns==receipt && !b.body_filter.complete);
  CHECK(!b.prepared_quality && b.stats.model_preparations==0);
  // No FK or spatial calibration is needed to retain the original raw source.
  auto unqualified=chest.input;unqualified.calibration={false,false,false,false,false};
  rejects([&]{raw.project_raw(unqualified);}); // Original paired color header is mandatory.
  unqualified.raw_projection_header=chest.image->header;
  unqualified.raw_projection_header->stamp.nanosec=4000000;
  auto c=raw.project_raw(unqualified);
  CHECK(c.raw_projection_source_ns==stamp+4000000 && c.raw_projection_raw_source_ns==stamp);
  CHECK(c.raw_projection_cloud->header.stamp.nanosec==4000000);
  CHECK(c.decoded.identity().source_stamp_ros_ns==stamp);
  unqualified.raw_projection_header->stamp.nanosec=5000001;
  rejects([&]{raw.project_raw(unqualified);});
  unqualified.raw_projection_header->stamp.nanosec=4000000;
  unqualified.identity.receipt_ros_ns=stamp+3000000;
  rejects([&]{raw.project_raw(unqualified);}); // Color cannot postdate SDK receipt.
}
void original_chest_quality_mask_is_owned_and_reused() {
  Model model(4.);Fixture chest(7,3,false,"chest");
  EngineConfig config;config.raw_projection_enabled=true;
  config.raw_projection_use_component_mask=true;config.raw_projection_stride=2;
  config.component_minimum_pixels=3;config.near_component_minimum_pixels=4;
  config.near_component_maximum_depth_m=.6;
  chest.input.raw_projection_header=chest.image->header;
  for(unsigned i=0;i<21;++i)chest.set(i,0);
  chest.set(0,500);chest.set(1,500);chest.set(2,500); // Original near threshold removes this island.
  chest.set(18,2000);chest.set(19,2000);chest.set(20,2000);
  PerceptionEngine raw(config),body(config);
  auto r=raw.project_raw(chest.input);
  CHECK(r.raw_projection_complete && r.prepared_quality && r.prepared_quality->mask.removed_pixels==3);
  const auto reference=rgbd::project_depth(*chest.image,*chest.info,.001,.1,8.,2,&r.prepared_quality->mask.keep);
  CHECK(r.raw_projection_cloud->data==reference.data);
  auto shared=chest.input;shared.prepared_quality=r.prepared_quality;
  const auto fused=body.process(shared,model.links,chest.query);
  const auto separate=body.process(chest.input,model.links,chest.query);
  CHECK(fused.stats.quality_mask_reuses==1 && separate.stats.quality_mask_reuses==0);
  CHECK(fused.body_cloud.data==separate.body_cloud.data && fused.decoded.kinds()==separate.decoded.kinds());
  auto other=shared;other.image=std::make_shared<sensor_msgs::msg::Image>(*chest.image);
  rejects([&]{body.process(other,model.links,chest.query);});
  auto advanced=shared;++advanced.identity.sequence;
  rejects([&]{body.process(advanced,model.links,chest.query);});
  // A later raw invocation owns another workspace; its mask cannot mutate the
  // retained first frame's mask while body consumes that first frame.
  Fixture next(7,3,false,"chest");next.input.raw_projection_header=next.image->header;
  const auto next_raw=raw.project_raw(next.input);CHECK(next_raw.raw_projection_complete);
  CHECK(r.prepared_quality->mask.removed_pixels==3);
  CHECK(body.process(shared,model.links,chest.query).body_cloud.data==fused.body_cloud.data);
  CHECK(r.timings.raw_projection.cpu_ms>=0. && r.timings.raw_projection.wall_ms>=0.);
}
void prepared_sources_are_equivalent_bound_and_keep_cutoffs_distinct() {
  Model model(4.);Fixture chest(7,3,false,"chest");
  EngineConfig config;config.raw_projection_enabled=true;config.raw_projection_use_component_mask=true;
  config.raw_projection_stride=1;config.component_minimum_pixels=3;config.component_maximum_depth_m=1.5;
  chest.input.raw_projection_header=chest.image->header;
  for(unsigned i=0;i<21;++i)chest.set(i,0);
  chest.set(0,2000);chest.set(5,1000); // Both raw islands removed; body keeps the far one.
  PerceptionEngine preparation(config),raw(config),body(config);
  auto shared=chest.input;shared.prepared_quality=preparation.prepare(shared);
  const auto token=shared.prepared_quality;
  CHECK(token && token->mask_prepared && token->mask.valid && token->mask.removed_pixels==2);
  const auto original_raw=raw.project_raw(chest.input),prepared_raw=raw.project_raw(shared);
  const auto original_body=body.process(chest.input,model.links,chest.query);
  const auto prepared_body=body.process(shared,model.links,chest.query);
  CHECK(prepared_raw.raw_projection_cloud->data==original_raw.raw_projection_cloud->data);
  CHECK(prepared_raw.raw_projection_cloud->header==original_raw.raw_projection_cloud->header);
  CHECK(prepared_raw.raw_projection_cloud->width==0 && prepared_body.body_cloud.width==1);
  CHECK(prepared_body.body_cloud.data==original_body.body_cloud.data);
  CHECK(prepared_body.body_filter.classifications==original_body.body_filter.classifications);
  CHECK(prepared_body.decoded.kinds()==original_body.decoded.kinds());
  CHECK(prepared_body.stats.quality_mask_reuses==1 && prepared_raw.stats.quality_mask_reuses==1);
  CHECK(!prepared_body.raw_projection_complete && !prepared_body.raw_projection_cloud);
  CHECK(prepared_raw.prepared_quality==token && prepared_body.prepared_quality==token);
  CHECK(preparation.prepare(shared)==token);
  // The same original source/receipt clocks expire independently of preparation
  // or branch completion. No output uses consumer completion as a new receipt.
  const auto& id=prepared_raw.decoded.identity();
  CHECK(rgbd::region_times_fresh(prepared_raw.raw_projection_source_ns,id.receipt_ros_ns,id.receipt_steady_ns,
    stamp+300000000LL,steady+300000000LL,.3));
  CHECK(!rgbd::region_times_fresh(prepared_raw.raw_projection_raw_source_ns,id.receipt_ros_ns,id.receipt_steady_ns,
    stamp+300000001LL,steady+300000000LL,.3));
  CHECK(!rgbd::region_times_fresh(prepared_body.decoded.identity().source_stamp_ros_ns,id.receipt_ros_ns,id.receipt_steady_ns,
    receipt,steady+300000001LL,.3));
  for(int change=0;change<9;++change){
    auto altered=shared;
    if(change==0)++altered.identity.sequence;
    if(change==1)++altered.identity.receipt_ros_ns;
    if(change==2)++altered.identity.receipt_steady_ns;
    if(change==3)altered.identity.epoch+="changed";
    if(change==4)altered.identity.calibration_id+="changed";
    if(change==5)--altered.identity.expiry_budget_ns;
    if(change==6)altered.identity.required_for_evaluation=!altered.identity.required_for_evaluation;
    if(change==7)altered.info=std::make_shared<sensor_msgs::msg::CameraInfo>(*chest.info);
    if(change==8)++altered.raw_projection_header->stamp.nanosec;
    rejects([&]{raw.project_raw(altered);});rejects([&]{body.process(altered,model.links,chest.query);});
  }
  for(int change=0;change<3;++change){
    auto corrupted=std::make_shared<PreparedQualityMask>(*token);
    if(change==0)corrupted->mask.valid=false;
    if(change==1)corrupted->mask.keep.pop_back();
    if(change==2)corrupted->mask_prepared=false;
    auto altered=shared;altered.prepared_quality=corrupted;
    rejects([&]{preparation.prepare(altered);});rejects([&]{raw.project_raw(altered);});
    rejects([&]{body.process(altered,model.links,chest.query);});
  }
  auto changed_config=config;changed_config.component_maximum_depth_m=1.4;
  PerceptionEngine changed(changed_config);rejects([&]{changed.project_raw(shared);});
  rejects([&]{changed.process(shared,model.links,chest.query);});
  auto invalid=chest.input;invalid.raw_projection_header.reset();rejects([&]{preparation.prepare(invalid);});
  invalid=chest.input;invalid.raw_projection_header->stamp.nanosec=5000001;rejects([&]{preparation.prepare(invalid);});
  invalid=chest.input;auto layout=std::make_shared<sensor_msgs::msg::Image>(*chest.image);layout->step=1;invalid.image=layout;
  rejects([&]{preparation.prepare(invalid);});
  Fixture next(7,3,false,"chest",stamp+20000000);next.input.raw_projection_header=next.image->header;
  (void)preparation.prepare(next.input);CHECK(token->mask.removed_pixels==2);
  CHECK(raw.project_raw(shared).raw_projection_cloud->data==prepared_raw.raw_projection_cloud->data);
  // Belly raw never acquires the body's component-mask dependency.
  Fixture belly;auto belly_config=config;belly_config.raw_projection_use_component_mask=false;
  PerceptionEngine belly_prepare(belly_config),belly_raw(belly_config),belly_body(belly_config);
  auto b=belly.input;b.prepared_quality=belly_prepare.prepare(b);CHECK(!b.prepared_quality->mask_prepared);
  CHECK(belly_raw.project_raw(b).raw_projection_cloud->data==belly_raw.project_raw(belly.input).raw_projection_cloud->data);
  CHECK(belly_body.process(b,model.links,belly.query).body_cloud.data==belly_body.process(belly.input,model.links,belly.query).body_cloud.data);
  auto legacy=chest.input;legacy.identity.expiry_budget_ns=sv::kSupportedEvidenceAgeNs+1;
  CHECK(raw.project_raw(legacy).raw_projection_complete); // Original raw-only identity contract.
  rejects([&]{preparation.prepare(legacy);});
  PerceptionEngine disabled({});CHECK(!disabled.project_raw(FrameInput{}).raw_projection_complete);
}

void prepared_raw_and_body_workers_progress_independently() {
  using Work=std::shared_ptr<const FrameInput>;
  using Worker=sunhorizon_camera_runtime::LatestFrameWorker<Work>;
  Model model(4.);
  for(int block_stage:{0,1,2}){
    const bool block_raw=block_stage!=2; // Raw projection, raw sink, then body/FK lane.
    EngineConfig config;config.raw_projection_enabled=true;config.raw_projection_use_component_mask=true;
    config.component_minimum_pixels=3;
    PerceptionEngine preparation(config),raw(config),body(config);
    Fixture first(7,3,false,"chest");first.input.raw_projection_header=first.image->header;
    auto make_work=[&](FrameInput input){input.prepared_quality=preparation.prepare(input);return std::make_shared<const FrameInput>(std::move(input));};
    const auto original=make_work(first.input);
    // Prepare all later frames before blocking workers: a preparation regression
    // must fail the test rather than leave a worker waiting during unwinding.
    Fixture middle(7,3,false,"chest",stamp+20000000),newest(7,3,false,"chest",stamp+40000000);
    middle.input.identity.sequence=8;newest.input.identity.sequence=9;
    middle.input.raw_projection_header=middle.image->header;newest.input.raw_projection_header=newest.image->header;
    const auto middle_work=make_work(middle.input),newest_work=make_work(newest.input);
    std::promise<void> entered,release;auto released=release.get_future().share();
    std::promise<FrameResult> raw_first,body_first;auto rf=raw_first.get_future(),bf=body_first.get_future();
    std::promise<void> latest;auto last=latest.get_future();
    std::atomic<bool> allowed{true};std::atomic<int> errors{0};
    unsigned raw_calls=0,body_calls=0;
    auto failure=[&](auto){++errors;};
    Worker raw_worker([&](Work& work){
      if(block_stage==0 && raw_calls==0){entered.set_value();released.wait();}
      auto output=raw.project_raw(*work);
      if(block_stage==1 && raw_calls==0){entered.set_value();released.wait();}
      if(raw_calls++==0)raw_first.set_value(std::move(output));
      else if(block_raw && work->identity.sequence==9)latest.set_value();
    },[&]{return allowed.load();},failure);
    Worker body_worker([&](Work& work){
      if(!block_raw && body_calls==0){entered.set_value();released.wait();}
      auto output=body.process(*work,model.links,first.query);
      if(body_calls++==0)body_first.set_value(std::move(output));
      else if(!block_raw && work->identity.sequence==9)latest.set_value();
    },[&]{return allowed.load();},failure);
    raw_worker.replace(std::make_unique<Work>(original));body_worker.replace(std::make_unique<Work>(original));
    const auto blocked=entered.get_future().wait_for(std::chrono::seconds(2));
    const auto other=block_raw ? bf.wait_for(std::chrono::seconds(2)) : rf.wait_for(std::chrono::seconds(2));
    auto& held=block_raw ? raw_worker : body_worker;
    held.replace(std::make_unique<Work>(middle_work));
    held.replace(std::make_unique<Work>(newest_work));
    const auto queued=held.stats();release.set_value();
    const auto delivered=last.wait_for(std::chrono::seconds(2));
    allowed.store(false);raw_worker.request_stop();body_worker.request_stop();raw_worker.join();body_worker.join();
    CHECK(blocked==std::future_status::ready && other==std::future_status::ready);
    CHECK(queued.in_flight && queued.pending && queued.replaced==1);
    CHECK(delivered==std::future_status::ready && errors.load()==0);
    const auto r=rf.get(),b=bf.get();
    CHECK(r.raw_projection_complete && !b.raw_projection_complete && !b.raw_projection_cloud);
    CHECK(r.decoded.identity().sequence==7 && b.decoded.identity().sequence==7);
    CHECK(r.raw_projection_raw_source_ns==stamp && b.decoded.identity().receipt_ros_ns==receipt);
    CHECK((block_raw ? raw_calls : body_calls)==2); // Original plus latest, never renewed middle.
    const auto rejected=held.replace(std::make_unique<Work>(original));
    CHECK(rejected && (*rejected)->identity.source_stamp_ros_ns==stamp && held.stats().closed);
  }
}

void reject_incomplete_inputs_models_and_budgets() {
  Model model;Fixture f;PerceptionEngine engine({});
  auto missing=model.links;missing.pop_back();rejects([&]{engine.process(f.input,missing,f.query);});
  auto stale=model.links;stale[5].covers_source_time=false;
  rejects([&]{engine.process(f.input,stale,f.query);});
  const dg::PreparedModel old(model.links,{});CHECK(model.mesh.build(plane(1.),{}));
  for (unsigned i=0;i<3;++i) f.set(i,0);
  rejects([&]{engine.process(f.input,old,f.query);}); // Even all-invalid cannot bypass model validation.
  auto bad=f.input;bad.identity.source_stamp_ros_ns++;
  rejects([&]{engine.process(bad,model.links,f.query);});
  bad=f.input;bad.optical_to_common.query_stamp_ns++;
  rejects([&]{engine.process(bad,model.links,f.query);});
  bad=f.input;bad.optical_to_common.rotation[0]=2.;
  rejects([&]{engine.process(bad,model.links,f.query);});
  bad=f.input;bad.calibration.source_time_and_transform_verified=false;
  rejects([&]{engine.process(bad,model.links,f.query);});
  f.image->step=1;rejects([&]{engine.process(f.input,model.links,f.query);});
  Fixture valid;EngineConfig small;small.body_budget.max_points=1;
  PerceptionEngine bounded(small);rejects([&]{bounded.process(valid.input,model.links,valid.query);});
  small={};small.coverage_max_triangle_tests=1;PerceptionEngine coverage_bounded(small);
  rejects([&]{coverage_bounded.process(valid.input,model.links,valid.query);});
  small={};small.body_budget.query.max_node_visits=1;PerceptionEngine mismatch(small);
  const dg::PreparedModel original_budget(model.links,{});
  rejects([&]{mismatch.process(valid.input,original_budget,valid.query);});
  rejects([&]{PerceptionEngine::evaluate(valid.query,{nullptr},1100000000LL,2100000000LL);});
}
void classified_body_without_reference_preserves_hits_and_original_identity() {
  Model model;Fixture f;
  for (unsigned i=0;i<3;++i) f.set(i,2000);
  EngineConfig config;config.classified_obstacles_enabled=true;
  PerceptionEngine engine(config);
  const auto out=engine.process(f.input,model.links,f.query);
  CHECK(out.body_filter.complete);CHECK(out.classified_obstacles);
  const auto& classified=*out.classified_obstacles;
  CHECK(classified.classification_complete());CHECK(classified.has_unknown());
  CHECK(classified.ground().result==ground_relative::Qualification::AbsentReference);
  CHECK(!classified.ground().safety_qualified);CHECK(classified.clearing_indices().empty());
  CHECK(classified.marking_indices().empty());
  CHECK(classified.protected_hit_indices().size()==out.body_filter.retained_count);
  CHECK(classified.source().identity.source_ns==f.input.identity.source_stamp_ros_ns);
  CHECK(classified.source().identity.first_receipt_ros_ns==f.input.identity.receipt_ros_ns);
  CHECK(classified.source().identity.first_receipt_steady_ns==f.input.identity.receipt_steady_ns);
  CHECK(classified.source().identity.calibration_id==f.input.identity.calibration_id);
  for(const auto& point:classified.source().points)CHECK(point.ground_blocked&&point.model_conflict);
  for(unsigned i=0;i<3;++i)f.set(i,900);
  const auto external=engine.process(f.input,model.links,f.query);
  for(const auto& point:external.classified_obstacles->source().points)CHECK(!point.ground_blocked&&!point.model_conflict);
  for(unsigned i=0;i<3;++i)f.set(i,2000);
  CHECK(out.timings.obstacle_classify.wall_ms>=0.);
  const auto legacy=PerceptionEngine({}).process(f.input,model.links,f.query);
  CHECK(!legacy.classified_obstacles);CHECK(legacy.body_cloud.data==out.body_cloud.data);
  CHECK(legacy.decoded.kinds()==out.decoded.kinds());
}
ground_relative::GroundReferenceOwner::ConstSharedPtr engine_ground_owner(
    bool supported=false,const std::string& source="belly") {
  // Synthetic end-to-end producer fixture. No values here qualify robot data.
  using Json=nlohmann::json;
  Json classifier={{"ground_band_min_m",-.02},{"ground_band_max_m",.02},{"maximum_support_residual_m",.03},
    {"maximum_point_error_m",.01},{"maximum_transform_translation_error_m",.01},{"maximum_transform_angle_error_rad",.01},
    {"maximum_plane_offset_error_m",.01},{"maximum_plane_angle_error_rad",.01},{"maximum_normal_tilt_rad",.1},
    {"rotation_tolerance",1.e-12},{"normal_norm_tolerance",1.e-12},{"minimum_support_span_x_m",.1},
    {"minimum_support_span_y_m",.1},{"minimum_support_double_area_m2",.005},{"minimum_samples_per_cell",1},
    {"minimum_training_cells",3},{"minimum_holdout_cells",3},{"continuous_support_radius_cells",1},
    {"maximum_points",10000},{"maximum_support_cells",100},{"maximum_support_checks",100000}};
  Json cells=Json::array();for(unsigned y=0;y<5;++y)for(unsigned x=0;x<5;++x)cells.push_back({{"x",x},{"y",y}});
  Json doc={{"schema","ground-reference-owner-v1"},{"mode","qualified"},{"clock_boot_id","synthetic-engine-boot"},
    {"classifier",classifier},
    {"reference",{{"provenance_id","synthetic-engine-survey"},{"calibration_id","synthetic-floor"},{"common_frame","odom"},
      {"source_ns",stamp},{"first_receipt_ros_ns",receipt},{"first_receipt_steady_ns",steady},
      {"expiry_budget_ns",600000000},{"independently_qualified",true},{"unit_normal",{0.,0.,1.}},
      {"anchor",{0.,0.,-.9}},{"plane_offset_error_bound_m",0.},{"plane_angle_error_bound_rad",0.},
      {"support",{{"origin_x_m",-.5},{"origin_y_m",-.5},{"resolution_m",.2},{"width",5},{"height",5},{"required_cells",cells}}}}},
    {"sources",Json::array({{{"source_id","belly"},{"epoch","belly-epoch"},{"calibration_id","calibration-1"},
      {"source_frame","belly_optical"},{"measurement_provenance_id","synthetic-engine-measurement"},
      {"measurement_qualified",true},{"source_time_transform_qualified",true},{"point_error_bound_m",0.},
      {"transform_translation_error_bound_m",0.},{"transform_angle_error_bound_rad",0.}}})},
    {"qualification",{{"approved_usage","production-ground-classification"}}},
    {"near_ground_clearance",{{"contract_id","synthetic-engine-sampling"},{"origin_x_m",.15},{"origin_y_m",-.05},
      {"resolution_m",.1},{"width",1},{"height",1},{"coarse_origin_z_m",-.95},
      {"coarse_resolution_z_m",.15},{"coarse_z_index",0},{"maximum_tiles",16}}}};
  for(const auto* key:{"independent_ground_evidence_sha256","measurement_error_evidence_sha256",
      "transform_error_evidence_sha256","clock_alignment_evidence_sha256","support_selection_evidence_sha256"})
    doc["qualification"][key]=std::string(64,'a');
  auto& binding=doc["sources"][0];
  binding["source_id"]=source;binding["source_frame"]=source+"_optical";binding["epoch"]=source+"-epoch";
  std::optional<ground_relative::RegistrationMapIdentity> actual;
  if(supported) {
    actual=ground_relative::RegistrationMapIdentity{std::string(64,'b'),"synthetic-engine-pcd"};
    doc["schema"]="ground-reference-map-asset-v1";doc["mode"]="supported-segmentation";
    doc.erase("clock_boot_id");doc.erase("classifier");doc.erase("qualification");doc.erase("near_ground_clearance");
    doc["asset"]={{"pcd_sha256",actual->pcd_sha256},{"datum_id",actual->datum_id}};
    auto& ref=doc["reference"];ref["common_frame"]="registration_map";
    for(const auto* key:{"source_ns","first_receipt_ros_ns","first_receipt_steady_ns","expiry_budget_ns"})ref.erase(key);
    ref["independently_qualified"]=false;
    ref["plane_offset_error_bound_m"]=nullptr;ref["plane_angle_error_bound_rad"]=nullptr;
    binding.erase("epoch");binding.erase("measurement_provenance_id");
    binding["measurement_qualified"]=false;binding["source_time_transform_qualified"]=false;
    for(const auto* key:{"point_error_bound_m","transform_translation_error_bound_m","transform_angle_error_bound_rad"})
      binding[key]=nullptr;
    doc["segmentation"]={{"maximum_point_residual_m",.04},{"maximum_cell_median_residual_m",.03},
      {"maximum_cell_mad_m",.015},{"maximum_partition_median_difference_m",.02},{"maximum_normal_tilt_rad",.1},
      {"minimum_support_span_x_m",.1},{"minimum_support_span_y_m",.1},{"minimum_support_double_area_m2",.005},
      {"minimum_samples_per_partition",1},{"minimum_supported_cells",6},{"continuous_support_radius_cells",1},
      {"maximum_points",10000},{"maximum_support_cells",100},{"maximum_support_checks",100000}};
  }
  const auto bytes=doc.dump();unsigned char hash[32];unsigned int hash_size=0;
  CHECK(EVP_Digest(bytes.data(),bytes.size(),hash,&hash_size,EVP_sha256(),nullptr)==1 && hash_size==32);
  std::ostringstream digest;digest<<std::hex<<std::setfill('0');for(const auto b:hash)digest<<std::setw(2)<<unsigned(b);
  const auto path=std::filesystem::canonical(std::filesystem::temp_directory_path()) /
    ("engine-ground-owner-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".json");
  struct Remove {std::filesystem::path file;~Remove(){std::filesystem::remove(file);}} remove{path};
  {std::ofstream file(path);file<<bytes;}
  return ground_relative::GroundReferenceOwner::load(path.string(),digest.str(),"synthetic-engine-boot",actual);
}
void supported_chest_endpoints_do_not_promote_regional_qualification() {
  namespace g=ground_relative;
  Fixture f(21,21,false,"chest");f.info->k={20,0,10,0,20,10,0,0,1};
  constexpr std::size_t hole_pixel=10*21+10,raised_pixel=10*21+11;
  f.set(hole_pixel,0);f.set(raised_pixel,840); // A missing ray and a 16 cm raised endpoint.
  f.query.source_contracts={{"chest","chest-epoch",false}};
  EngineConfig config;config.classified_obstacles_enabled=true;config.coverage_enabled=false;
  PerceptionEngine engine(config);Model model(10.);
  const dg::PreparedModel prepared(model.links,config.body_budget.query);
  f.input.calibration={false,false,false,false,false};
  f.input.ground_owner=engine_ground_owner(true,"chest");
  g::SourceTimeTransform tf;
  tf.rotation={1.,0.,0.,0.,-1.,0.,0.,0.,-1.};tf.translation={0.,0.,.1};
  tf.source_frame="chest_optical";tf.common_frame="registration_map";tf.source_ns=tf.common_ns=stamp;tf.qualified=true;
  f.input.ground_source_transform=tf;
  auto revoked=std::make_shared<std::atomic<bool>>(false);
  f.input.ground_map_binding={std::make_shared<g::RegistrationMapAlignment>(
    f.input.ground_owner->map_identity(),7,revoked,1600000000LL,2600000000LL),"chest","chest-epoch",stamp};
  const auto out=engine.process(f.input,prepared,f.query);
  CHECK(out.classified_obstacles && out.classified_obstacles->ground().safety_qualified);
  const auto& frame=*out.classified_obstacles;
  CHECK(frame.ground().basis==g::GroundQualification::Basis::MeasuredSupport);
  CHECK(frame.clearing_indices().size()>0);
  bool raised_marked=false;
  for(const auto i:frame.clearing_indices()) {
    CHECK(frame.source().points.at(i).sample_id!=hole_pixel);
    CHECK(frame.source().points.at(i).sample_id!=raised_pixel);
  }
  for(const auto i:frame.marking_indices())
    if(frame.source().points.at(i).sample_id==raised_pixel)raised_marked=true;
  CHECK(raised_marked);
  CHECK(frame.ground().transform_diagnostic.reason.find("rgbd_regional_calibration_unverified")==std::string::npos);
  CHECK(!frame.source().measurement_qualified && std::isnan(frame.source().point_error_bound_m));
  CHECK(!frame.reference()->independently_qualified && !frame.reference()->source_time_transform_qualified);
  CHECK(!f.input.calibration.optical_to_base_extrinsics_verified && !f.input.calibration.source_time_and_transform_verified);
  CHECK(!f.input.ground_owner->clearance_grid() && out.stats.coverage_queries==0);
  const auto snapshot=nlohmann::json::parse(detail::serialize_ground_classification_snapshot(frame,f.input.ground_owner.get()));
  CHECK(snapshot["source"]["identity"]["source_ns"]==stamp);
  CHECK(snapshot["source"]["identity"]["first_receipt_steady_ns"]==steady);
  CHECK(snapshot["source"]["points"].size()==frame.source().points.size());
  CHECK(snapshot["decision_schema"]=="measured-support-branch-v3");
  // Invalid retained proof diagnostics must explicitly report invalid/null
  // counts, never size_t underflow or a clamped zero that looks admitted.
  for(int variant=0;variant<3;++variant){
    auto invalid=frame.source();for(auto& point:invalid.points)point.measurement_unreliable=false;
    auto& q=invalid.measurement_quality;q.present=true;q.component_sample_ids={1};q.stereo_mismatch_sample_ids={1};
    q.unreliable_sample_ids=variant==0?std::vector<uint64_t>{}:variant==1?std::vector<uint64_t>{1}:std::vector<uint64_t>{1,2};
    const auto diagnostic=ground_relative::conservative_unqualified_frame(std::move(invalid));
    const auto bad=nlohmann::json::parse(detail::serialize_ground_classification_snapshot(*diagnostic,nullptr));
    CHECK(bad["source"]["measurement_quality"]["direct_saturated_pixels"].is_null());
    CHECK(bad["source"]["measurement_quality"]["rejection_counts_valid"]==false);
    CHECK(bad["source"]["measurement_quality"]["rejection_counts_error"]=="invalid_rejection_partition");
  }

  CHECK(snapshot["decisions"].size()==frame.source().points.size());
  CHECK(snapshot["decisions"]==nlohmann::json(frame.decisions()));
  CHECK(snapshot["decision_names"][static_cast<int>(ground_relative::PointDecision::ObstacleAnchored)]=="ObstacleAnchored");
  CHECK(snapshot["source"]["points"][0][6]==frame.source().points[0].sample_id);
  CHECK(snapshot["source"]["points"][0][4]==frame.source().points[0].ground_blocked);
  CHECK(snapshot["source"]["points"][0][7]==frame.source().points[0].model_conflict);
  CHECK(snapshot["point_columns"][7]=="model_conflict");
  CHECK(frame.source().raster_geometry.width==21 && frame.source().raster_geometry.height==21);
  CHECK(frame.source().raster_geometry.stride==config.body_stride);
  CHECK(snapshot["source"]["raster_geometry"]["width"]==21);
  CHECK(snapshot["source"]["raster_geometry"]["height"]==21);
  CHECK(snapshot["source"]["raster_geometry"]["stride"]==config.body_stride);
  CHECK(snapshot["source"]["to_common"]["rotation"]==nlohmann::json(tf.rotation));
  CHECK(snapshot["source"]["point_error_bound_m"].is_null());
  CHECK(snapshot["reference"]["map_binding"]["alignment"]["geometry_valid_until_ros_ns"]==1600000000LL);
  CHECK(snapshot["reference"]["map_binding"]["alignment"]["geometry_valid_until_steady_ns"]==2600000000LL);
  CHECK(snapshot["reference"]["map_binding"]["alignment"]["current_at_export"]==true);
  CHECK(snapshot["ground"]["cells"].size()==frame.ground().cells.size());
  CHECK(snapshot["seed_relative_heights_m"]==nlohmann::json::parse(nlohmann::json(frame.seed_relative_heights_m()).dump()));
  CHECK(snapshot["ground"]["measured_plane_fit"]["result"]==int(frame.ground().measured_plane_fit.result));
  CHECK(snapshot["ground"]["measured_plane_fit"]["training_cells"]==frame.ground().measured_plane_fit.training_cells);
  CHECK(snapshot["ground"]["measured_plane_fit"]["anchor"]==nlohmann::json::parse(nlohmann::json(frame.ground().measured_plane_fit.anchor).dump()));
  CHECK(snapshot["owner"]["segmentation"]["maximum_point_residual_m"]==.04);
  rejects([&]{detail::write_ground_classification_snapshot("/tmp","chest",frame);});
  rejects([&]{detail::write_ground_classification_snapshot("","chest",frame);});
  rejects([&]{detail::write_ground_classification_snapshot("/tmp","../chest",frame);});
  // Belly is an admitted role, then the same path boundary rejects /tmp before
  // any serialization or I/O. This test never writes the root filesystem.
  std::string belly_path_failure;
  try {detail::write_ground_classification_snapshot("/tmp","belly",frame);}
  catch(const std::invalid_argument& error) {belly_path_failure=error.what();}
  CHECK(belly_path_failure=="ground_snapshot_outside_evidence_directory");
  rejects([&]{detail::write_rgbd_projection_audit_snapshot("/tmp","belly",7,"{}");});
  rejects([&]{detail::write_rgbd_projection_audit_snapshot("/tmp","../chest",7,"{}");});
  rejects([&]{detail::write_rgbd_projection_audit_snapshot("/tmp","chest",0,"{}");});
  rejects([&]{detail::write_rgbd_projection_audit_snapshot("/tmp","belly",7,"{}","invalid");});
  if(const auto* folder=std::getenv("SUNHORIZON_TEST_GROUND_SNAPSHOT_DIR")) {
    const auto directory=std::filesystem::path(folder)/("projection-writer-"+
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directory(directory);
    struct RemoveDirectory {std::filesystem::path p;~RemoveDirectory(){std::filesystem::remove_all(p);}} cleanup{directory};
    const auto belly=detail::write_rgbd_projection_audit_snapshot(directory.string(),"belly",7,"{\"first\":true}");
    CHECK(std::filesystem::path(belly).filename()=="belly-global-projection-audit-7.json");
    bool no_replace=false;
    try {(void)detail::write_rgbd_projection_audit_snapshot(directory.string(),"belly",7,"different");}
    catch(const std::system_error&){no_replace=true;}
    CHECK(no_replace);std::ifstream file(belly);std::string bytes((std::istreambuf_iterator<char>(file)),{});
    CHECK(bytes=="{\"first\":true}");
    const auto chest=detail::write_rgbd_projection_audit_snapshot(directory.string(),"chest",7,"{}");
    CHECK(std::filesystem::path(chest).filename()=="chest-global-projection-audit-7.json");
    const auto local=detail::write_rgbd_projection_audit_snapshot(directory.string(),"belly",7,"{\"local\":true}","local");
    CHECK(std::filesystem::path(local).filename()=="belly-local-projection-audit-7.json");
    CHECK(local!=belly);
    bool local_no_replace=false;
    try {(void)detail::write_rgbd_projection_audit_snapshot(directory.string(),"belly",7,"different","local");}
    catch(const std::system_error&){local_no_replace=true;}
    CHECK(local_no_replace);
  }

  auto sparse=frame.source();sparse.points.resize(1);
  sparse.original_projected_samples=sparse.points.size()+sparse.self_support.size()+sparse.self_support_quality_rejected_samples;
  const auto failed=ground_relative::classify_supported_ground_frame(
    std::move(sparse),f.input.ground_owner->segmentation_config(),*frame.reference());
  CHECK(failed->ground().result==ground_relative::Qualification::InsufficientSupportGeometry);
  const auto failed_snapshot=nlohmann::json::parse(detail::serialize_ground_classification_snapshot(*failed,f.input.ground_owner.get()));
  CHECK(failed_snapshot["ground"]["result"]==11 && failed_snapshot["source"]["points"].size()==1);
  CHECK(failed->ground().cells.size()==1 && failed->reference()->support.required_cells.size()==1);
  CHECK(failed_snapshot["ground"]["cells"].size()==failed->ground().cells.size());
  CHECK(failed_snapshot["reference"]["support"]["required_cells"].size()==failed->reference()->support.required_cells.size());
  const auto& sparse_cell=failed_snapshot["ground"]["cells"][0];
  CHECK(sparse_cell["samples"]==1);
  CHECK(sparse_cell["training_samples"].get<std::size_t>()+
    sparse_cell["holdout_samples"].get<std::size_t>()==1);
  CHECK(sparse_cell["x"]==failed->reference()->support.required_cells[0].x &&
    sparse_cell["y"]==failed->reference()->support.required_cells[0].y);
  for(const auto kind:out.decoded.kinds())CHECK(kind==sv::ReturnKind::kInvalid);
  for(const auto& cell:f.evaluate(out.decoded).cells)CHECK(cell.free_sources==0);
  const auto no_ground=[&](const FrameInput& input) {
    const auto result=engine.process(input,prepared,f.query);
    CHECK(result.classified_obstacles && !result.classified_obstacles->ground().safety_qualified);
    CHECK(result.classified_obstacles->clearing_indices().empty());
    CHECK(result.classified_obstacles->marking_indices().empty());
    CHECK(result.classified_obstacles->protected_hit_indices().size()==result.body_filter.retained_count);
  };
  CHECK(out.obstacle_input_health.available);
  // Deferred geometry is queried once only after BODY is complete. A cancelled
  // original-frame/reset wait discards classification, without replacing clocks.
  auto deferred=f.input;unsigned ground_queries=0;
  deferred.acquire_ground_geometry=[&]() -> std::optional<RegistrationGroundGeometryBatch> {
    ++ground_queries;return std::nullopt;
  };
  const auto cancelled=engine.process(deferred,prepared,f.query);
  CHECK(ground_queries==1 && cancelled.ground_query_cancelled);
  CHECK(cancelled.body_filter.complete && !cancelled.classified_obstacles);
  CHECK(cancelled.body_filter.retained_count==out.body_filter.retained_count);
  auto reset_token=std::make_shared<std::atomic<bool>>(true);
  deferred.acquire_ground_geometry=[&]() {
    return detail::wait_for_registration_ground_geometry(out.classified_obstacles->source().identity,
      [&](const auto&) {RegistrationGroundGeometryBatch batch;batch.reset_revoked=reset_token;return batch;},
      []{return true;},[]{return false;},[]{throw std::runtime_error("unexpected_ground_wait");});
  };
  const auto reset_cancelled=engine.process(deferred,prepared,f.query);
  CHECK(reset_cancelled.body_filter.complete && reset_cancelled.ground_query_cancelled);
  CHECK(!reset_cancelled.classified_obstacles);
  // The current batch replaces even deliberately invalid eager geometry. Its
  // original binding/deadlines and source identity survive unchanged.
  deferred.ground_source_transform.reset();deferred.ground_map_binding={};
  deferred.acquire_ground_geometry=[&]() -> std::optional<RegistrationGroundGeometryBatch> {
    ++ground_queries;RegistrationGroundGeometryBatch batch;
    batch.transforms={*f.input.ground_source_transform};
    batch.map_binding=f.input.ground_map_binding;
    batch.diagnostic=f.input.ground_transform_diagnostic;return batch;
  };
  const auto acquired=engine.process(deferred,prepared,f.query);
  CHECK(ground_queries==2 && !acquired.ground_query_cancelled);
  CHECK(acquired.classified_obstacles->decisions()==out.classified_obstacles->decisions());
  CHECK(acquired.classified_obstacles->source().identity.source_ns==f.input.identity.source_stamp_ros_ns);
  CHECK(acquired.classified_obstacles->source().identity.sequence==f.input.identity.sequence);
  CHECK(acquired.classified_obstacles->reference()->map_binding.alignment==f.input.ground_map_binding.alignment);
  // Provider failure is never replaced with the eager successful snapshot.
  deferred=f.input;
  deferred.acquire_ground_geometry=[]() -> std::optional<RegistrationGroundGeometryBatch> {
    RegistrationGroundGeometryBatch batch;
    batch.diagnostic={6,"pcd_registration_original_evidence_expired"};return batch;
  };
  const auto unavailable=engine.process(deferred,prepared,f.query);
  CHECK(!unavailable.ground_query_cancelled && !unavailable.obstacle_input_health.available);
  CHECK(unavailable.classified_obstacles->clearing_indices().empty());
  // A genuine sparse/no-fit observation remains deliverable Unknown.
  auto sparse_input=f.input;
  auto sparse_image=std::make_shared<sensor_msgs::msg::Image>(*f.image);
  sparse_image->data.assign(sparse_image->data.size(),0);
  const uint16_t one_depth=1000;std::memcpy(sparse_image->data.data(),&one_depth,2);
  sparse_input.image=sparse_image;
  const auto sparse_output=engine.process(sparse_input,prepared,f.query);
  CHECK(sparse_output.obstacle_input_health.available);
  CHECK(!sparse_output.classified_obstacles->ground().safety_qualified);
  CHECK(sparse_output.classified_obstacles->marking_indices().empty());
  CHECK(sparse_output.classified_obstacles->protected_hit_indices().size()==1);

  // P's exact failure identities remain diagnostic-only all-Uncertain frames.
  // They may not be credited as a new healthy body/obstacle observation.
  for(const auto sample:std::array<std::pair<uint64_t,int64_t>,2>{{
      {545,1792665060565484000LL},{539,1792665060519190000LL}}}) {
    const std::string role=sample.first==545?"belly":"chest";
    auto expired=f.input;expired.identity.sequence=sample.first;
    expired.identity.source_id=role;expired.identity.epoch=role+"-epoch";
    expired.identity.required_for_evaluation=role=="belly";
    expired.ground_owner=engine_ground_owner(true,role);
    expired.identity.source_stamp_ros_ns=sample.second;
    expired.identity.receipt_ros_ns=sample.second+30000000;
    expired.identity.receipt_steady_ns=steady;
    auto image=std::make_shared<sensor_msgs::msg::Image>(*f.image);
    image->header.frame_id=role+"_optical";
    image->header.stamp.sec=sample.second/1000000000LL;
    image->header.stamp.nanosec=sample.second%1000000000LL;
    auto info=std::make_shared<sensor_msgs::msg::CameraInfo>(*f.info);info->header=image->header;
    expired.image=image;expired.info=info;
    auto failed_query=f.query;failed_query.query_stamp_ns=sample.second;
    failed_query.source_contracts={{role,role+"-epoch",role=="belly"}};
    expired.optical_to_common.source_frame=role+"_optical";
    expired.optical_to_common.source_stamp_ns=sample.second;
    expired.optical_to_common.query_stamp_ns=sample.second;
    expired.ground_source_transform.reset();expired.ground_map_binding={};
    expired.ground_transform_diagnostic={6,"pcd_registration_original_evidence_expired"};
    const auto failed_output=engine.process(expired,prepared,failed_query);
    CHECK(!failed_output.obstacle_input_health.available);
    CHECK(failed_output.obstacle_input_health.reason=="registration_ground_unavailable:pcd_registration_original_evidence_expired");
    CHECK(failed_output.classified_obstacles->clearing_indices().empty());
    CHECK(failed_output.classified_obstacles->marking_indices().empty());
    CHECK(failed_output.classified_obstacles->protected_hit_indices().size()==failed_output.body_filter.retained_count);
    CHECK(failed_output.classified_obstacles->source().identity.sequence==sample.first);
    CHECK(failed_output.classified_obstacles->source().identity.source_ns==sample.second);
  }
  CHECK(registration_ground_input_health(true,{0,"ready"},g::Qualification::InsufficientSupportGeometry).available);
  CHECK(registration_ground_input_health(true,{0,"ready"},g::Qualification::ContradictorySupport).available);
  CHECK(registration_ground_input_health(false,{6,"expired"},g::Qualification::InvalidTransform).available);
  auto bad=f.input;bad.ground_source_transform.reset();no_ground(bad);
  bad=f.input;bad.ground_source_transform->source_ns++;no_ground(bad);
  bad=f.input;bad.ground_source_transform->source_frame="wrong_optical";no_ground(bad);
  bad=f.input;bad.ground_map_binding.alignment=std::make_shared<g::RegistrationMapAlignment>(
    f.input.ground_owner->map_identity(),7,revoked,receipt-1,steady-1);no_ground(bad);
  revoked->store(true);no_ground(f.input);
  // Independently bounded classification keeps its original regional gate.
  bad=f.input;bad.ground_owner=engine_ground_owner(false,"chest");
  bad.ground_map_binding={};bad.ground_source_transform->common_frame="odom";
  const auto bounded=engine.process(bad,prepared,f.query);
  CHECK(!bounded.classified_obstacles->ground().safety_qualified);
  CHECK(bounded.classified_obstacles->ground().transform_diagnostic.reason.find(
    "rgbd_regional_calibration_unverified")!=std::string::npos);
}
void near_ground_pixels_share_model_without_changing_original_R() {
  Fixture f(21,21);f.info->k={20,0,10,0,20,10,0,0,1};
  f.query.cells={{1,{-.01,-.01,.2},{.01,.01,.3}}};
  constexpr std::size_t extra_pixel=10*21+14; // x=.2 at the measured floor, outside original R.
  EngineConfig config;config.classified_obstacles_enabled=true;config.clear_margin_m=.005;
  Model model(10.);const dg::PreparedModel prepared(model.links,config.body_budget.query);
  PerceptionEngine engine(config);const auto original=engine.process(f.input,prepared,f.query);
  CHECK(original.stats.model_preparations==0);
  CHECK(original.decoded.kinds()[extra_pixel]==sv::ReturnKind::kInvalid);
  const auto original_R=f.evaluate(original.decoded);CHECK(original_R.aggregate==sv::Aggregate::kClear);
  f.input.ground_owner=engine_ground_owner();
  ground_relative::SourceTimeTransform tf;
  tf.rotation={1.,0.,0.,0.,-1.,0.,0.,0.,-1.};tf.translation={0.,0.,.1};
  tf.source_frame="belly_optical";tf.common_frame="odom";tf.source_ns=tf.common_ns=stamp;tf.qualified=true;
  f.input.ground_source_transform=tf;
  const auto evaluate_near=[&](const FrameResult& result) {
    CHECK(result.classified_obstacles && result.classified_obstacles->ground().safety_qualified);
    return near_ground_clearance::evaluate(result.decoded.source(),result.classified_obstacles,
      f.input.ground_owner->config(),*f.input.ground_owner->clearance_grid(),1100000000,2100000000);
  };
  const auto unchanged_R=[&](const FrameResult& result) {
    const auto actual=f.evaluate(result.decoded);
    CHECK(actual.aggregate==original_R.aggregate);
    const auto& before=original_R.cells[0].counts[0];const auto& after=actual.cells[0].counts[0];
    CHECK(before.intersecting_rays==after.intersecting_rays && before.clear_rays==after.clear_rays);
    CHECK(before.invalid_returns==after.invalid_returns && before.self_returns==after.self_returns);
    CHECK(std::get<sv::DepthRaster>(result.decoded.source().data).optical_to_common.common_frame=="base_footprint");
    CHECK(result.stats.model_preparations==0);
  };
  const auto expanded=engine.process(f.input,prepared,f.query);
  unchanged_R(expanded);CHECK(expanded.body_cloud.data==original.body_cloud.data);
  CHECK(expanded.decoded.kinds()[extra_pixel]==sv::ReturnKind::kExternal);
  CHECK(expanded.stats.coverage_queries>original.stats.coverage_queries);
  const auto clear=evaluate_near(expanded);
  CHECK(clear->tiles.size()==1 && clear->tiles[0].state==sv::CellState::kFree);

  // Exactly the old R allowance: new queries get no credit from unclassified
  // pixels and cannot cause the original R to throw or become unavailable.
  CHECK(original.stats.coverage_node_visits>0 && original.stats.coverage_triangle_tests>0);
  auto bounded_config=config;bounded_config.coverage_max_node_visits=original.stats.coverage_node_visits;
  bounded_config.coverage_max_triangle_tests=original.stats.coverage_triangle_tests;
  const auto bounded=PerceptionEngine(bounded_config).process(f.input,prepared,f.query);
  unchanged_R(bounded);CHECK(bounded.decoded.kinds()[extra_pixel]==sv::ReturnKind::kInvalid);
  CHECK(evaluate_near(bounded)->tiles[0].state==sv::CellState::kUnknown);

  // The extra pixel is physically classified by the shared self model even
  // when its measured self surface is BEHIND the new query volume.
  Model self_model;CHECK(self_model.mesh.build(plane(1.,.19,.21),{}));
  const dg::PreparedModel self_prepared(self_model.links,config.body_budget.query);
  const auto self=engine.process(f.input,self_prepared,f.query);
  unchanged_R(self);CHECK(self.decoded.kinds()[extra_pixel]==sv::ReturnKind::kSelfMatched);
  CHECK(evaluate_near(self)->tiles[0].state==sv::CellState::kUnknown);

  // A small quality-rejected island outside original R must not be resurrected
  // when the new query expands its requested pixel set.
  for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx)
    if(dx||dy)f.set(std::size_t(10+dy)*21+std::size_t(14+dx),0);
  auto masked_config=config;masked_config.component_minimum_pixels=2;
  const auto masked=PerceptionEngine(masked_config).process(f.input,prepared,f.query);
  unchanged_R(masked);CHECK(masked.stats.quality_rejected_pixels>0);
  CHECK(masked.decoded.kinds()[extra_pixel]==sv::ReturnKind::kInvalid);
  CHECK(evaluate_near(masked)->tiles[0].state==sv::CellState::kUnknown);
}

void same_epoch_measurement_quality_main_path() {
  namespace cam=sunhorizon_camera_runtime;
  Fixture f(7,7);for(std::size_t i=0;i<49;++i)f.set(i,2000);
  Model model(3.);EngineConfig config;config.classified_obstacles_enabled=true;
  config.raw_projection_enabled=true;config.raw_projection_stride=1;
  PerceptionEngine engine(config);const auto baseline=engine.process(f.input,model.links,f.query);
  const auto raw_baseline=engine.project_raw(f.input);
  auto q=std::make_shared<cam::DepthMeasurementQuality>();q->role="belly";
  q->source_frame=f.image->header.frame_id;q->source_ns=stamp;q->sequence=f.input.identity.sequence;
  q->opened_serial="same-device";q->capture_epoch="owner-epoch";
  q->image_identity=f.image.get();q->info_identity=f.info.get();q->width=7;q->height=7;
  q->device_us={100,100,100};q->global_us={uint64_t(stamp/1000),uint64_t(stamp/1000),uint64_t(stamp/1000)};
  q->depth_pixel_type=0;q->sdk_depth_scale_m=.001;
  q->exact_rectified_pair=true;q->stereo_match_complete=true;q->stereo_match_reason="disabled_foreground_ambiguity";q->pixels.assign(49,0);f.input.measurement_quality=q;
  const auto good=engine.process(f.input,model.links,f.query);const auto raw_good=engine.project_raw(f.input);
  CHECK(good.body_cloud.data==baseline.body_cloud.data);CHECK(good.decoded.kinds()==baseline.decoded.kinds());
  CHECK(raw_good.raw_projection_cloud->data==raw_baseline.raw_projection_cloud->data);
  const std::size_t target=24;q->pixels[target]=1;
  const auto bad=engine.process(f.input,model.links,f.query);const auto raw_bad=engine.project_raw(f.input);
  CHECK(bad.body_cloud.data==baseline.body_cloud.data); // no endpoint deletion/fakeSelf
  CHECK(bad.decoded.depth()==baseline.decoded.depth());CHECK(bad.decoded.kinds()[target]==sv::ReturnKind::kInvalid);
  CHECK(bad.stats.measurement_unreliable_pixels==1);CHECK(raw_bad.raw_projection_cloud->width+1==raw_baseline.raw_projection_cloud->width);
  const auto& point=bad.classified_obstacles->source().points[target];CHECK(point.sample_id==target);
  CHECK(bad.classified_obstacles->classes()[target]==ground_relative::PointClass::Invalid);
  CHECK(bad.classified_obstacles->decisions()[target]==ground_relative::PointDecision::MeasurementUnreliable);
  for(const auto index:bad.classified_obstacles->marking_indices())CHECK(index!=target);
  for(const auto index:bad.classified_obstacles->clearing_indices())CHECK(index!=target);
  for(const auto index:bad.classified_obstacles->protected_hit_indices())CHECK(index!=target);
  CHECK(bad.classified_obstacles->has_unknown());
  CHECK(point.measurement_unreliable);CHECK(point.ground_blocked==baseline.classified_obstacles->source().points[target].ground_blocked);
  CHECK(point.model_conflict==baseline.classified_obstacles->source().points[target].model_conflict);
  CHECK(bad.classified_obstacles->source().measurement_quality.unreliable_sample_ids==std::vector<uint64_t>{target});
  const auto audit=nlohmann::json::parse(detail::serialize_ground_classification_snapshot(*bad.classified_obstacles));
  CHECK(audit["source"]["points"][target][8].get<bool>());CHECK(!audit["source"]["measurement_quality"].is_null());CHECK(bad.classified_obstacles->classes()[target]!=ground_relative::PointClass::Ground);
  CHECK(f.evaluate(bad.decoded).cells[0].state!=sv::CellState::kFree);
  q->pixels[target]=2;const auto unassessed=engine.process(f.input,model.links,f.query);
  CHECK(unassessed.body_cloud.data==baseline.body_cloud.data);CHECK(unassessed.decoded.kinds()==baseline.decoded.kinds());
  CHECK(engine.project_raw(f.input).raw_projection_cloud->data==raw_baseline.raw_projection_cloud->data);
  CHECK(unassessed.stats.measurement_unassessed_pixels==1);
  q->pixels[target]=1;q->exact_rectified_pair=false;
  CHECK(engine.process(f.input,model.links,f.query).decoded.kinds()==baseline.decoded.kinds()); // no exactIR: no borrowed rejection
  q->exact_rectified_pair=true;q->global_us[1]+=1;rejects([&]{engine.prepare(f.input);});
  q->global_us[1]-=1;q->source_ns+=1;rejects([&]{engine.process(f.input,model.links,f.query);});q->source_ns-=1;
  q->sequence+=1;rejects([&]{engine.prepare(f.input);});q->sequence-=1;
  q->image_identity=nullptr;rejects([&]{engine.prepare(f.input);});q->image_identity=f.image.get();
  q->pixels[target]=0;const auto prepared=engine.prepare(f.input);auto changed=f.input;changed.prepared_quality=prepared;
  changed.measurement_quality=std::make_shared<cam::DepthMeasurementQuality>(*q);
  rejects([&]{engine.process(changed,model.links,f.query);}); // immutable token identity part of prepared owner
}
}  // namespace
void self_support_denominator_only() {
  namespace cam=sunhorizon_camera_runtime;
  Fixture f(3,1);Model model(1.);EngineConfig config;
  config.classified_obstacles_enabled=true;config.coverage_enabled=false;
  PerceptionEngine engine(config);
  auto q=std::make_shared<cam::DepthMeasurementQuality>();q->role="belly";
  q->source_frame=f.image->header.frame_id;q->source_ns=stamp;q->sequence=f.input.identity.sequence;
  q->opened_serial="same-device";q->capture_epoch="owner-epoch";
  q->image_identity=f.image.get();q->info_identity=f.info.get();q->width=3;q->height=1;
  q->device_us={100,100,100};q->global_us={uint64_t(stamp/1000),uint64_t(stamp/1000),uint64_t(stamp/1000)};
  q->depth_pixel_type=0;q->sdk_depth_scale_m=.001;q->exact_rectified_pair=true;
  q->stereo_match_complete=true;q->stereo_match_reason="disabled_foreground_ambiguity";
  q->pixels={0,1,0};f.input.measurement_quality=q;
  const auto all_self=engine.process(f.input,model.links,f.query);
  CHECK(all_self.body_filter.complete&&all_self.body_filter.retained_count==0);
  CHECK(all_self.body_cloud.data.empty());CHECK(all_self.classified_obstacles);
  const auto& source=all_self.classified_obstacles->source();
  CHECK(source.points.empty()&&source.self_support_complete);
  CHECK(source.original_projected_samples==3&&source.self_support.size()==2);
  CHECK(source.self_support_quality_rejected_samples==1);
  CHECK(source.self_support[0].sample_id==0&&source.self_support[1].sample_id==2);
  CHECK(all_self.classified_obstacles->marking_indices().empty());
  CHECK(all_self.classified_obstacles->clearing_indices().empty());
  f.set(2,900);
  const auto mixed=engine.process(f.input,model.links,f.query);
  const auto& m=mixed.classified_obstacles->source();
  CHECK(m.points.size()==1&&m.points[0].sample_id==2);
  CHECK(m.self_support.size()==1&&m.self_support[0].sample_id==0);
  CHECK(m.original_projected_samples==m.points.size()+m.self_support.size()+m.self_support_quality_rejected_samples);
  auto broken=m;broken.self_support.clear();rejects([&]{ground_relative::conservative_unqualified_frame(broken);});
}
void native_body_edge_is_self_without_free_space_credit() {
  Fixture f(201,1);Model model(.5);
  CHECK(model.mesh.build(plane(.5,.0075,.1),{}));
  for(unsigned i=0;i<201;++i)f.set(i,0);
  f.set(100,900);
  f.info->k={100,0,100,0,100,0,0,0,1};
  f.info->r={1,0,0,0,1,0,0,0,1};
  f.info->p={100,0,100,0,0,100,0,0,0,0,1,0};
  f.info->distortion_model="plumb_bob";f.info->d={0,0,0,0,0};
  EngineConfig config;config.classified_obstacles_enabled=true;config.body_stride=1;
  PerceptionEngine engine(config);
  const auto output=engine.process(f.input,model.links,f.query);
  CHECK(output.body_filter.complete && output.body_filter.input_count==1);
  CHECK(output.body_filter.ground_blocked==std::vector<uint8_t>{2});
  CHECK(output.body_cloud.width==1 && output.classified_obstacles);
  const auto& frame=*output.classified_obstacles;
  CHECK(frame.source().points.size()==1 && frame.source().points[0].sample_id==100);
  CHECK(frame.source().points[0].kind==ground_relative::ReturnKind::Self);
  CHECK(frame.classes()[0]==ground_relative::PointClass::Self);
  CHECK(frame.decisions()[0]==ground_relative::PointDecision::ModelProjectionEdge);
  CHECK(frame.source().self_support.empty());
  CHECK(frame.marking_indices().empty() && frame.clearing_indices().empty() && frame.protected_hit_indices().empty());
  CHECK(output.decoded.kinds()[100]==sv::ReturnKind::kInvalid);
  const auto audit=nlohmann::json::parse(detail::serialize_ground_classification_snapshot(frame));
  CHECK(audit["point_classes"][audit["classes"][0].get<std::size_t>()]=="Self");
  // A genuine foreground return near the silhouette remains external.
  f.set(100,400);
  const auto foreground=engine.process(f.input,model.links,f.query);
  CHECK(foreground.classified_obstacles->source().points[0].kind==ground_relative::ReturnKind::Measured);
  CHECK(foreground.classified_obstacles->classes()[0]!=ground_relative::PointClass::Self);
}
void stereo_body_occlusion_is_invalid_not_free() {
  namespace cam=sunhorizon_camera_runtime;
  Fixture f(201,1);Model model(.5);CHECK(model.mesh.build(plane(.5,.055,.065),{}));
  for(unsigned i=0;i<201;++i)f.set(i,0);f.set(100,1000);
  f.info->k={100,0,100,0,100,0,0,0,1};f.info->r={1,0,0,0,1,0,0,0,1};
  f.info->p={100,0,100,0,0,100,0,0,0,0,1,0};f.info->distortion_model="plumb_bob";f.info->d={0,0,0,0,0};
  EngineConfig config;config.self_tolerance_m=.03;config.classified_obstacles_enabled=true;
  config.body_stereo_occlusion_enabled=true;
  PerceptionEngine engine(config);
  auto q=std::make_shared<cam::DepthMeasurementQuality>();q->role="belly";q->source_frame=f.image->header.frame_id;
  q->source_ns=stamp;q->sequence=7;q->opened_serial="actual-sdk-owner";q->capture_epoch="same-owner-epoch";
  q->width=201;q->height=1;q->image_identity=f.image.get();q->info_identity=f.info.get();
  q->device_us={100,100,100};q->global_us={uint64_t(stamp/1000),uint64_t(stamp/1000),uint64_t(stamp/1000)};
  q->depth_pixel_type=0;q->sdk_depth_scale_m=.001;q->pixels.assign(201,0);q->exact_rectified_pair=true;
  q->stereo_match_complete=true;q->stereo_match_reason="disabled_foreground_ambiguity";
  q->stereo_body_geometry_verified=true;q->stereo_baseline_m=.12;q->right_from_depth_translation_m={-.12,0,0};
  q->rectified_depth_k=f.info->k;f.input.measurement_quality=q;
  q->geometry_depth_device_us=100;q->geometry_depth_global_us=stamp/1000;q->geometry_depth_frame_index=42;q->geometry_depth_scale_m=.001;
  f.input.sdk_depth_device_us=100;f.input.sdk_depth_global_us=stamp/1000;f.input.sdk_depth_frame_index=42;f.input.sdk_depth_scale_m=.001;
  const auto output=engine.process(f.input,model.links,f.query);
  CHECK(output.body_filter.complete&&output.body_filter.removed_count==0&&output.body_filter.retained_count==1);
  CHECK(output.body_filter.ground_blocked==std::vector<uint8_t>{3});CHECK(output.stats.stereo_body_baseline_m==.12);
  const auto& frame=*output.classified_obstacles;
  CHECK(frame.classes()[0]==ground_relative::PointClass::Invalid);
  CHECK(frame.decisions()[0]==ground_relative::PointDecision::MeasurementBodyOccluded);
  CHECK(frame.source().points[0].sample_id==100&&frame.source().points[0].measurement_body_occluded);
  CHECK(frame.source().self_support.empty());
  CHECK(frame.marking_indices().empty()&&frame.clearing_indices().empty()&&frame.protected_hit_indices().empty());
  CHECK(output.decoded.kinds()[100]==sv::ReturnKind::kInvalid);
  CHECK(!engine.project_raw(f.input).raw_projection_complete);
  q->exact_rectified_pair=false;q->pixels.assign(201,2);
  const auto ir_late=engine.process(f.input,model.links,f.query);
  CHECK(ir_late.body_filter.ground_blocked==std::vector<uint8_t>{3});
  CHECK(ir_late.stats.measurement_unassessed_pixels==201&&ir_late.stats.stereo_body_baseline_m==.12);
  CHECK(ir_late.decoded.kinds()[100]==sv::ReturnKind::kInvalid);
  f.input.sdk_depth_frame_index+=1;
  CHECK(engine.process(f.input,model.links,f.query).body_filter.ground_blocked==std::vector<uint8_t>{0});
  f.input.sdk_depth_frame_index-=1;f.input.sdk_depth_scale_m=.002;
  CHECK(engine.process(f.input,model.links,f.query).body_filter.ground_blocked==std::vector<uint8_t>{0});
  f.input.sdk_depth_scale_m=.001;q->exact_rectified_pair=true;q->pixels.assign(201,0);
  q->pixels[100]=1;const auto double_hit=engine.process(f.input,model.links,f.query);
  CHECK(double_hit.body_filter.ground_blocked==std::vector<uint8_t>{3}); // native geometry audit is preserved
  CHECK(double_hit.classified_obstacles->classes()[0]==ground_relative::PointClass::Invalid);
  CHECK(double_hit.classified_obstacles->decisions()[0]==ground_relative::PointDecision::MeasurementUnreliable);
  CHECK(double_hit.classified_obstacles->source().points[0].kind==ground_relative::ReturnKind::Measured);
  CHECK(!double_hit.classified_obstacles->source().points[0].measurement_body_occluded);
  CHECK(double_hit.classified_obstacles->source().body_occlusion.sample_ids.empty());
  q->pixels[100]=0;
  q->stereo_baseline_m=.13;const auto mismatched=engine.process(f.input,model.links,f.query);
  CHECK(mismatched.body_filter.ground_blocked==std::vector<uint8_t>{0}); // no borrowed baseline
  q->stereo_baseline_m=.12;f.set(100,300);
  const auto foreground=engine.process(f.input,model.links,f.query);
  CHECK(foreground.body_filter.ground_blocked==std::vector<uint8_t>{0});
  CHECK(!foreground.classified_obstacles->source().points[0].measurement_body_occluded);
  q->image_identity=nullptr;rejects([&]{engine.process(f.input,model.links,f.query);});
  config.raw_projection_enabled=true;rejects([&]{PerceptionEngine invalid(config);});
}
int main(int argc,char** argv) {try {
  stereo_body_occlusion_is_invalid_not_free();
  if(argc==2&&std::string(argv[1])=="--self-support-only") {
    self_support_denominator_only();std::cout<<"PASS "<<checks<<" self support engine checks\n";return 0;
  }
  self_support_denominator_only();
  native_body_edge_is_self_without_free_space_credit();
  same_epoch_measurement_quality_main_path();
  body_parity_and_original_identity();self_quality_far_thin_and_cutoff_rounding();
  independent_chest_belly_time_and_owned_views();occupancy_conflict_and_distinct_geometric_rays();
  collision_height_retains_unknown_inside_the_responsibility_box();
  unqualified_optional_coverage_never_blocks_accepted_body_or_contributes();
  original_raw_projection_is_independent_and_preserves_headers();
  original_chest_quality_mask_is_owned_and_reused();
  prepared_sources_are_equivalent_bound_and_keep_cutoffs_distinct();
  prepared_raw_and_body_workers_progress_independently();
  reject_incomplete_inputs_models_and_budgets();
  classified_body_without_reference_preserves_hits_and_original_identity();
  supported_chest_endpoints_do_not_promote_regional_qualification();
  near_ground_pixels_share_model_without_changing_original_R();
  std::cout<<"PASS "<<checks<<" perception engine checks\n";return 0;
} catch (const std::exception& error) {std::cerr<<"FAIL "<<checks<<" "<<error.what()<<'\n';return 1;}}
