#include "sunhorizon_rgbd_adapter/fused_depth_self_filter.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <typeinfo>
#include <atomic>
#include <chrono>

using namespace sunhorizon_rgbd_adapter;
namespace dsg = sunhorizon_obstacle_coverage::dynamic_self_geometry;
namespace {
int checks = 0;
#define CHECK(x) do { ++checks; if (!(x)) throw std::runtime_error(#x); } while (false)
struct Input {
  sensor_msgs::msg::Image depth;
  sensor_msgs::msg::CameraInfo info;
  std::vector<uint8_t> keep;
  double scale{.001}, minimum{.1}, maximum{8.}, mask_max{1.5};
  uint32_t stride{1};
  Input(uint32_t width=7, uint32_t height=3, bool big=false) {
    depth.header.frame_id="camera_belly_depth_optical_frame";
    depth.header.stamp.sec=123;depth.header.stamp.nanosec=456;
    depth.width=width;depth.height=height;depth.step=width*2+6;
    depth.encoding="16UC1";depth.is_bigendian=big;
    depth.data.resize(depth.step*height,0xa5);
    info.header=depth.header;info.width=width;info.height=height;
    info.k={3.125,0,2.1,0,2.75,.7,0,0,1};
    keep.resize(width*height,1);
    for(std::size_t i=0;i<keep.size();++i) {
      const uint16_t raw[]={0,1000,900,1100,3000,100,9000};
      set(i,raw[i%7]);keep[i]=(i%5!=0);
    }
  }
  void set(std::size_t i,uint16_t value) {
    const auto offset=i/depth.width*depth.step+i%depth.width*2;
    depth.data[offset+(depth.is_bigendian?1:0)]=value&255;
    depth.data[offset+(depth.is_bigendian?0:1)]=value>>8;
  }
};
std::vector<dsg::Triangle> plane(double z,double left=-20,double right=20) {
  return {{{{{left,-20,z},{right,-20,z},{right,20,z}}}},
          {{{{left,-20,z},{right,20,z},{left,20,z}}}}};
}
struct Model {
  dsg::Mesh mesh;
  std::vector<dsg::LinkObservation> links;
  Model() {CHECK(mesh.build(plane(1),{}));links={{&mesh,"leg",{},true,true,true}};}
};
DynamicCloudView view(const sensor_msgs::msg::PointCloud2& cloud) {
  return {cloud.data.data(),cloud.data.size(),cloud.width,cloud.height,
      cloud.point_step,cloud.row_step,cloud.is_bigendian,
      {DynamicPointField{0,4},{4,4},{8,4}}};
}
void same_cloud(const sensor_msgs::msg::PointCloud2& a,const sensor_msgs::msg::PointCloud2& b) {
  CHECK(a.header.frame_id==b.header.frame_id);CHECK(a.header.stamp.sec==b.header.stamp.sec);
  CHECK(a.header.stamp.nanosec==b.header.stamp.nanosec);
  CHECK(a.width==b.width);CHECK(a.height==b.height);CHECK(a.point_step==b.point_step);
  CHECK(a.row_step==b.row_step);CHECK(a.is_bigendian==b.is_bigendian);CHECK(a.is_dense==b.is_dense);
  CHECK(a.fields.size()==b.fields.size());
  for(std::size_t i=0;i<a.fields.size();++i) {
    CHECK(a.fields[i].name==b.fields[i].name);CHECK(a.fields[i].offset==b.fields[i].offset);
    CHECK(a.fields[i].count==b.fields[i].count);CHECK(a.fields[i].datatype==b.fields[i].datatype);
  }
  CHECK(a.data==b.data);
}
void same_metrics(const DynamicFilterResult& a,const DynamicFilterResult& b) {
  CHECK(a.complete==b.complete);CHECK(a.reason==b.reason);CHECK(a.input_count==b.input_count);
  CHECK(a.removed_count==b.removed_count);CHECK(a.retained_count==b.retained_count);
  CHECK(a.classifications==b.classifications);CHECK(a.node_visits==b.node_visits);
  CHECK(a.triangle_tests==b.triangle_tests);CHECK(a.ground_blocked==b.ground_blocked);
  CHECK(a.nominal_model_conflict==b.nominal_model_conflict);
  CHECK(a.measured_reference_ambiguous_count==b.measured_reference_ambiguous_count);
}
void compare_cuda(const Input& in,const dsg::PreparedModel& model,double tolerance,
    const DynamicFilterBudget& budget,const std::vector<uint8_t>* mask,
    const FusedDepthSelfFilterResult& serial) {
  if(!BodyGeometryCuda::compiled())return;
  BodyGeometryCuda cuda;
  const auto gpu=project_depth_filter_dynamic_self(in.depth,in.info,in.scale,in.minimum,in.maximum,
      in.stride,model,tolerance,budget,mask,in.mask_max,nullptr,&cuda);
  same_metrics(serial.filter,gpu.filter);same_cloud(serial.cloud,gpu.cloud);
  CHECK(serial.retained_sample_ids==gpu.retained_sample_ids);
  CHECK(serial.self_support.size()==gpu.self_support.size());
  for(std::size_t i=0;i<serial.self_support.size();++i){
    CHECK(serial.self_support[i].sample_id==gpu.self_support[i].sample_id);
    CHECK(serial.self_support[i].xyz==gpu.self_support[i].xyz);
  }
}
FusedDepthSelfFilterResult compare(const Input& in,const std::vector<dsg::LinkObservation>& links,
    double tolerance=.02,const DynamicFilterBudget& budget={},bool masked=true) {
  const auto* mask=masked?&in.keep:nullptr;
  const auto projected=project_depth(in.depth,in.info,in.scale,in.minimum,in.maximum,in.stride,mask,in.mask_max);
  const auto legacy=filter_dynamic_self_points(view(projected),links,tolerance,budget);
  const dsg::PreparedModel model(links,budget.query);
  auto fused=project_depth_filter_dynamic_self(in.depth,in.info,in.scale,in.minimum,in.maximum,
      in.stride,model,tolerance,budget,mask,in.mask_max);
  compare_cuda(in,model,tolerance,budget,mask,fused);
  BodyGeometryWorker worker([]{});
  const auto parallel=project_depth_filter_dynamic_self(in.depth,in.info,in.scale,in.minimum,in.maximum,
      in.stride,model,tolerance,budget,mask,in.mask_max,&worker);
  same_metrics(fused.filter,parallel.filter);same_cloud(fused.cloud,parallel.cloud);
  CHECK(fused.retained_sample_ids==parallel.retained_sample_ids);
  CHECK(fused.self_support.size()==parallel.self_support.size());
  if(fused.filter.complete)CHECK(fused.self_support.size()==fused.filter.removed_count);
  for(std::size_t i=0;i<fused.self_support.size();++i) {
    CHECK(fused.self_support[i].sample_id==parallel.self_support[i].sample_id);
    CHECK(fused.self_support[i].xyz==parallel.self_support[i].xyz);
    CHECK(std::find(fused.retained_sample_ids.begin(),fused.retained_sample_ids.end(),fused.self_support[i].sample_id)==fused.retained_sample_ids.end());
  }
  same_metrics(legacy,fused.filter);CHECK(fused.filter.data.empty());
  if(legacy.complete) {
    auto expected=projected;expected.data=legacy.data;expected.width=legacy.retained_count;
    expected.height=1;expected.row_step=expected.width*expected.point_step;
    same_cloud(expected,fused.cloud);
  } else {CHECK(fused.cloud.data.empty());CHECK(fused.cloud.width==0);CHECK(fused.self_support.empty());}
  return fused;
}
void invalid_projection(const Input& in,const std::vector<dsg::LinkObservation>& links) {
  std::string old_type,old_reason,new_type,new_reason;
  try {(void)project_depth(in.depth,in.info,in.scale,in.minimum,in.maximum,in.stride,&in.keep,in.mask_max);}
  catch(const std::exception& e){old_type=typeid(e).name();old_reason=e.what();}
  const dsg::PreparedModel model(links,{});
  try {(void)project_depth_filter_dynamic_self(in.depth,in.info,in.scale,in.minimum,in.maximum,
      in.stride,model,.02,{},&in.keep,in.mask_max);}
  catch(const std::exception& e){new_type=typeid(e).name();new_reason=e.what();}
  CHECK(old_type==typeid(std::invalid_argument).name());CHECK(old_type==new_type);
  CHECK(old_reason=="invalid_depth_projection_input");CHECK(old_reason==new_reason);
}
}
void native_pixel_contract() {
  Input in(1,1);in.keep[0]=1;in.set(0,1000);
  in.info.k={400,0,0,0,400,0,0,0,1};
  in.info.distortion_model="plumb_bob";in.info.d.assign(5,0.);
  in.info.r={1,0,0,0,1,0,0,0,1};in.info.p={400,0,0,0,0,400,0,0,0,0,1,0};
  dsg::Mesh mesh;auto surfaces=plane(.9,-.5,.5);
  for(auto& t:surfaces)for(auto& p:t.vertices)p.y*=.025;
  surfaces.push_back({{{{.0003,-.001,1},{.0011,-.001,1},{.0003,.001,1}}}});
  CHECK(mesh.build(surfaces,{}));CHECK(mesh.configure_measured_surface(2,1));
  const std::vector<dsg::LinkObservation> links{{&mesh,"foot",{},true,true,true}};
  const dsg::PreparedModel model(links,{});
  const auto run=[&](const Input& x,const DynamicFilterBudget& b=DynamicFilterBudget{}) {
    auto serial=project_depth_filter_dynamic_self(x.depth,x.info,x.scale,x.minimum,x.maximum,x.stride,model,.02,b,&x.keep,x.mask_max);
    compare_cuda(x,model,.02,b,&x.keep,serial);
    BodyGeometryWorker worker([]{});
    const auto parallel=project_depth_filter_dynamic_self(x.depth,x.info,x.scale,x.minimum,x.maximum,x.stride,model,.02,b,&x.keep,x.mask_max,&worker);
    same_metrics(serial.filter,parallel.filter);same_cloud(serial.cloud,parallel.cloud);
    CHECK(serial.retained_sample_ids==parallel.retained_sample_ids);return serial;
  };
  auto result=run(in);CHECK(result.filter.complete&&result.filter.removed_count==1&&result.retained_sample_ids.empty());
  for(int delta:{-30,-50,30,50}) {
    auto shifted=in;shifted.set(0,1000+delta);result=run(shifted);
    CHECK(result.filter.complete&&result.filter.removed_count==0&&result.filter.retained_count==1);
    CHECK(result.filter.ground_blocked==std::vector<uint8_t>{1});
    CHECK(result.filter.nominal_model_conflict==std::vector<uint8_t>{0});
    CHECK(result.filter.classifications[delta<0?1:3]==1);
  }
  auto uncertain=in;uncertain.set(0,990);result=run(uncertain);
  CHECK(result.filter.complete&&result.filter.removed_count==1&&result.filter.retained_count==0);
  CHECK(result.filter.measured_reference_ambiguous_count==0);
  // An ordinary valid projection is not sufficient proof of a native cell.
  // All incomplete/changed calibration cases fall back byte/metric identically.
  for(int field=0;field<13;++field) {
    auto bad=in;
    switch(field) {
      case 0:bad.info.header.stamp.nanosec++;break;
      case 1:bad.info.distortion_model="equidistant";break;
      case 2:bad.info.d.clear();break;
      case 3:bad.info.d[0]=.001;break;
      case 4:bad.info.r[0]=0;break;
      case 5:bad.info.p[3]=.01;break;
      case 6:bad.info.binning_x=2;break;
      case 7:bad.info.roi.x_offset=1;break;
      case 8:bad.info.roi.width=1;break;
      case 9:bad.info.roi.do_rectify=true;break;
      case 10:bad.info.k[1]=.001;break;
      case 11:bad.info.p[0]=NAN;break;
      case 12:bad.info.binning_y=1;break;
    }
    const auto fallback=run(bad);const auto cloud=project_depth(bad.depth,bad.info,bad.scale,bad.minimum,bad.maximum,1,&bad.keep,bad.mask_max);
    const auto legacy=filter_dynamic_self_points(view(cloud),model,.02);
    same_metrics(legacy,fallback.filter);CHECK(fallback.filter.removed_count==1);
    CHECK(fallback.cloud.data==legacy.data);
  }
  auto endian=in;endian.depth.is_bigendian=true;endian.set(0,1000);CHECK(run(endian).filter.removed_count==1);
  auto stride=in;stride.stride=2;CHECK(run(stride).filter.removed_count==1);
  DynamicFilterBudget limit;limit.max_triangle_tests=1;
  result=run(in,limit);CHECK(!result.filter.complete&&result.cloud.data.empty()&&result.retained_sample_ids.empty());
}

void geometry_worker_lifecycle() {
  std::thread::id helper_id;std::atomic<int> checked{0};
  {BodyGeometryWorker worker([&]{helper_id=std::this_thread::get_id();++checked;});
    CHECK(checked==1);CHECK(helper_id!=std::this_thread::get_id());
    auto frame=worker.begin_frame();CHECK(checked==2);
    auto rejected=std::async(std::launch::async,[&]{try{auto second=worker.begin_frame();(void)second;return false;}catch(const std::runtime_error&){return true;}});CHECK(rejected.get());
    std::atomic<bool> completed{false};bool caller_error=false;
    try{worker.run([&]{completed=true;},[]{throw std::runtime_error("caller_failure");});}
    catch(const std::runtime_error& e){caller_error=std::string(e.what())=="caller_failure";}
    CHECK(caller_error);CHECK(completed);CHECK(worker.frame_helper_cpu_ms()>=0.);
  }
  bool startup_failed=false;try{BodyGeometryWorker worker([]{throw std::runtime_error("startup_owner");});}
  catch(const std::runtime_error& e){startup_failed=std::string(e.what())=="startup_owner";}CHECK(startup_failed);
  {std::atomic<int> calls{0};BodyGeometryWorker worker([&]{if(++calls>1)throw std::runtime_error("owner_changed");});
    bool drift_failed=false;try{auto frame=worker.begin_frame();(void)frame;}catch(const std::runtime_error& e){drift_failed=std::string(e.what())=="owner_changed";}CHECK(drift_failed);CHECK(calls==2);
  }
  {BodyGeometryWorker worker([]{});auto frame=worker.begin_frame();bool failed=false;
    try{worker.run([]{throw std::runtime_error("helper_failure");},[]{});}catch(const std::runtime_error& e){failed=std::string(e.what())=="helper_failure";}CHECK(failed);
    bool closed=false;try{worker.run([]{},[]{});}catch(const std::runtime_error&){closed=true;}CHECK(closed);
  }
  // Two role owners must enter independently; no global lock or shared job slot.
  {BodyGeometryWorker belly([]{}),chest([]{});std::mutex mutex;std::condition_variable cv;int entered=0;bool release=false;
    const auto run=[&](BodyGeometryWorker& worker){auto frame=worker.begin_frame();worker.run([&]{std::unique_lock<std::mutex> lock(mutex);++entered;cv.notify_all();cv.wait(lock,[&]{return release;});},[]{});};
    auto a=std::async(std::launch::async,[&]{run(belly);});auto b=std::async(std::launch::async,[&]{run(chest);});
    bool independent;{std::unique_lock<std::mutex> lock(mutex);independent=cv.wait_for(lock,std::chrono::seconds(2),[&]{return entered==2;});release=true;}cv.notify_all();a.get();b.get();CHECK(independent);
  }
}

void self_support_only() {
  Model model;Input in(3,1);in.keep.assign(3,1);
  in.info.k={10,0,0,0,10,0,0,0,1};in.set(0,1000);in.set(1,900);in.set(2,1000);
  const auto result=compare(in,model.links);
  CHECK(result.filter.complete&&result.self_support.size()==2);
  CHECK(result.self_support[0].sample_id==0&&result.self_support[1].sample_id==2);
  CHECK(result.retained_sample_ids==std::vector<uint64_t>({1}));
  in.set(1,1000);const auto all=compare(in,model.links);
  CHECK(all.cloud.data.empty()&&all.self_support.size()==3);
  auto budget=DynamicFilterBudget{};budget.max_node_visits=1;
  const auto failed=compare(in,model.links,.02,budget);
  CHECK(!failed.filter.complete&&failed.self_support.empty());
}
int main(int argc,char** argv) {try {
  if(argc==2&&std::string(argv[1])=="--self-support-only") {
    self_support_only();std::cout<<"PASS "<<checks<<" self support fused checks\n";return 0;
  }
  self_support_only();
  geometry_worker_lifecycle();
  native_pixel_contract();
  Model model;
  // Returns on either finite body surface are Self. A farther retained return
  // remains ground-blocked by the measured visibility reference.
  dsg::Mesh calibrated;
  auto surfaces=plane(.97);const auto actual=plane(1.);
  surfaces.insert(surfaces.end(),actual.begin(),actual.end());
  for(auto& triangle:surfaces)for(auto& vertex:triangle.vertices){vertex.x*=.025;vertex.y*=.025;}
  CHECK(calibrated.build(surfaces,{}));CHECK(calibrated.configure_measured_surface(2,2));
  const std::vector<dsg::LinkObservation> calibrated_links{{&calibrated,"calibrated_foot",{},true,true,true}};
  Input authority(3,1);authority.keep.assign(3,1);authority.info.k={10,0,0,0,10,0,0,0,1};
  authority.set(0,970);authority.set(1,1000);authority.set(2,1030);
  const auto guarded=compare(authority,calibrated_links);
  CHECK(guarded.filter.complete && guarded.filter.removed_count==2 && guarded.filter.retained_count==1);
  CHECK(guarded.retained_sample_ids==std::vector<uint64_t>({2}));
  CHECK(guarded.filter.ground_blocked==std::vector<uint8_t>({1}));
  CHECK(guarded.filter.nominal_model_conflict==std::vector<uint8_t>({0}));
  CHECK(guarded.filter.classifications[static_cast<std::size_t>(dsg::Classification::ExternalSurface)]==0);
  CHECK(guarded.filter.classifications[static_cast<std::size_t>(dsg::Classification::ModelConflict)]==1);
  CHECK(calibrated.configure_measured_surface(0,0));
  const auto nominal=compare(authority,calibrated_links);
  CHECK(nominal.filter.complete && nominal.filter.nominal_model_conflict==std::vector<uint8_t>({1}));
  // A finite surface witness resolves the near-edge body return; retained
  // adjacent rays keep their independent nominal-conflict evidence.
  dsg::Mesh grazing;
  auto grazing_surfaces=plane(.7);
  grazing_surfaces.push_back({{{{1.e-8,-1.e-8,.8},{1.e-8,1.e-8,1.},{1.e-8,2.e-8,1.2}}}});
  CHECK(grazing.build(grazing_surfaces,{}));CHECK(grazing.configure_measured_surface(2,1));
  const std::vector<dsg::LinkObservation> grazing_links{{&grazing,"foot_edge",{},true,true,true}};
  authority.set(0,1000);
  const auto local_ambiguity=compare(authority,grazing_links);
  CHECK(local_ambiguity.filter.complete && local_ambiguity.filter.removed_count==1);
  CHECK(local_ambiguity.filter.measured_reference_ambiguous_count==0);
  CHECK(local_ambiguity.filter.classifications[static_cast<std::size_t>(dsg::Classification::Unobserved)]==0);
  CHECK(local_ambiguity.filter.ground_blocked==std::vector<uint8_t>({1,1}));
  CHECK(local_ambiguity.filter.nominal_model_conflict==std::vector<uint8_t>({1,1}));
  // Padded source rows, either byte order, component masks, exact range limits,
  // strides and a mixture of self/near/far/invalid depth. Compare all cloud
  // metadata and retained bytes, not only coordinates or point counts.
  for(bool big:{false,true})for(uint32_t stride:{1U,2U,3U,99U})
    for(double cutoff:{.5,1.,1.5,std::numeric_limits<double>::infinity()})
      for(bool masked:{false,true}) {
        Input in(19,11,big);in.stride=stride;in.mask_max=cutoff;
        compare(in,model.links,.02,{},masked);
      }
  // Deterministic varied intrinsics, depth scale, masks and nontrivial geometry.
  uint32_t state=0x17b31U;
  for(unsigned trial=0;trial<60;++trial) {
    Input in(1+trial%21,1+trial%9,trial%2);
    in.scale=.00025*(1+trial%4);in.stride=1+trial%4;
    in.info.k[0]=.1+trial*1.013;in.info.k[4]=.2+trial*.973;
    for(std::size_t i=0;i<in.keep.size();++i) {
      state=state*1664525U+1013904223U;in.set(i,state%10001);in.keep[i]=(state%3!=0);
    }
    compare(in,model.links);
  }
  // Removing all points is a successful empty result; projecting none is the
  // legacy empty-input failure even if the model is invalid.
  Input all_self(2,1);all_self.info.k={10,0,0,0,10,0,0,0,1};
  all_self.keep.assign(2,1);all_self.set(0,1000);all_self.set(1,1000);
  auto result=compare(all_self,model.links);
  CHECK(result.filter.complete);CHECK(result.filter.removed_count==2);CHECK(result.cloud.data.empty());
  auto no_points=all_self;no_points.set(0,0);no_points.set(1,0);
  auto invalid_links=model.links;invalid_links[0].tf_valid=false;
  result=compare(no_points,invalid_links);CHECK(result.filter.reason=="point_count_budget_or_empty");
  auto rejected=all_self;rejected.keep.assign(2,0);rejected.mask_max=2;
  CHECK(compare(rejected,model.links).filter.reason=="point_count_budget_or_empty");
  // Nonfinite projected FLOAT32 XYZ must not bypass model/TF validation.
  auto invalid_xyz=all_self;invalid_xyz.info.k[0]=std::numeric_limits<double>::min();
  invalid_xyz.info.k[2]=-1.;
  result=compare(invalid_xyz,model.links);CHECK(result.filter.classifications[2]==2);
  CHECK(!compare(invalid_xyz,invalid_links).filter.complete);
  auto stale_model=model.links;stale_model[0].covers_source_time=false;
  CHECK(!compare(all_self,stale_model).filter.complete);
  // Frame and per-query failures clear partial output while preserving all
  // counters accumulated before the failing point.
  Input partial(3,1);partial.keep.assign(3,1);partial.info.k={10,0,0,0,10,0,0,0,1};
  partial.set(0,900);partial.set(1,1000);partial.set(2,1100);
  result=compare(partial,model.links);
  CHECK(result.retained_sample_ids==std::vector<uint64_t>({0,2}));
  CHECK(result.filter.ground_blocked==std::vector<uint8_t>({0,1}));
  DynamicFilterBudget limits;limits.max_triangle_tests=5;
  result=compare(partial,model.links,.02,limits);
  CHECK(!result.filter.complete);CHECK(result.filter.retained_count==1);
  CHECK(result.filter.reason=="frame_query_budget");CHECK(result.cloud.data.empty());
  CHECK(result.retained_sample_ids.empty());CHECK(result.filter.ground_blocked.empty());
  limits={};limits.max_node_visits=2;CHECK(!compare(partial,model.links,.02,limits).filter.complete);
  limits={};limits.query.max_triangle_tests=1;CHECK(!compare(partial,model.links,.02,limits).filter.complete);
  limits={};limits.max_points=2;CHECK(compare(partial,invalid_links,.02,limits).filter.reason=="point_count_budget_or_empty");
  limits={};limits.max_bytes=35;CHECK(compare(partial,invalid_links,.02,limits).filter.reason=="invalid_pointcloud_layout");
  limits={};limits.max_points=0;CHECK(compare(partial,model.links,.02,limits).filter.reason=="invalid_frame_budget");
  CHECK(compare(partial,model.links,-.001).filter.reason=="invalid_self_tolerance");
  CHECK(compare(partial,model.links,.031).filter.reason=="invalid_self_tolerance");
  CHECK(compare(partial,model.links,NAN).filter.reason=="invalid_self_tolerance");
  // Public accumulator cannot be used to append beyond its declared record
  // budget or dereference a null record, even for invalid/unqueried XYZ.
  const dsg::PreparedModel bounded_model(model.links,{});
  const auto one_record=depth_xyz_bytes({{0,0,0}});
  DynamicFilterAccumulator bounded(bounded_model,.02,{},1,12);
  CHECK(bounded.ready());CHECK(bounded.append(one_record.data(),0,0,0));
  CHECK(!bounded.append(one_record.data(),0,0,0));
  auto over=bounded.finish();CHECK(!over.complete);CHECK(over.data.empty());
  CHECK(over.retained_count==1);CHECK(over.classifications[2]==1);
  DynamicFilterAccumulator null_record(bounded_model,.02,{},1,12);
  CHECK(!null_record.append(nullptr,0,0,0));
  auto null_result=null_record.finish();CHECK(null_result.reason=="null_point_record");
  CHECK(null_result.data.empty());CHECK(null_result.retained_count==0);
  // A shared snapshot never silently substitutes different per-ray limits.
  const dsg::PreparedModel shared(model.links,{});
  limits={};limits.query.max_node_visits=1;
  auto mismatch=project_depth_filter_dynamic_self(partial.depth,partial.info,.001,.1,8,1,shared,.02,limits);
  CHECK(!mismatch.filter.complete);CHECK(mismatch.filter.reason=="prepared_model_query_budget_mismatch");
  CHECK(mismatch.cloud.data.empty());CHECK(mismatch.filter.node_visits==0);
  // Same-frame snapshots detect sequential mesh replacement, including a frame
  // whose projected points are all invalid XYZ.
  CHECK(model.mesh.build(plane(1),{}));
  auto stale=project_depth_filter_dynamic_self(invalid_xyz.depth,invalid_xyz.info,.001,.1,8,1,shared,.02);
  CHECK(!stale.filter.complete);CHECK(stale.cloud.data.empty());
  // An edge deliberately lies between the raw pixel ray and float-rounded XYZ
  // ray. A fused implementation using the raw pixel ray changes this result.
  Input edge(1,1);edge.set(0,1000);edge.keep[0]=1;edge.scale=.001003;
  edge.info.k={.3,0,-.1,0,1,0,0,0,1};
  const auto projected=project_depth(edge.depth,edge.info,edge.scale,.1,8,1);
  float xf,zf;std::memcpy(&xf,projected.data.data(),4);std::memcpy(&zf,projected.data.data()+8,4);
  const double raw_ray=-edge.info.k[2]/edge.info.k[0];
  const double boundary=(double(xf)+raw_ray*double(zf))*.5;
  dsg::Mesh edge_mesh;
  CHECK(edge_mesh.build(double(xf)<raw_ray*double(zf)?plane(zf,-20,boundary):plane(zf,boundary,20),{}));
  std::vector<dsg::LinkObservation> edge_links{{&edge_mesh,"edge",{},true,true,true}};
  const dsg::PreparedModel edge_model(edge_links,{});
  auto rounded=dsg::classify({double(xf)/zf,0,1},zf,1e-10,edge_model);
  auto pixel=dsg::classify({raw_ray,0,1},zf,1e-10,edge_model);
  CHECK(rounded.complete&&pixel.complete);CHECK(rounded.classification!=pixel.classification);
  compare(edge,edge_links,1e-10);
  // Projection input failures retain exact exception type/message and precedence.
  auto bad=partial;bad.depth.encoding="32FC1";invalid_projection(bad,model.links);
  bad=partial;bad.depth.step=1;invalid_projection(bad,model.links);
  bad=partial;bad.depth.data.pop_back();invalid_projection(bad,model.links);
  bad=partial;bad.depth.width=0;invalid_projection(bad,model.links);
  bad=partial;bad.info.width++;invalid_projection(bad,model.links);
  bad=partial;bad.info.header.frame_id="different";invalid_projection(bad,model.links);
  bad=partial;bad.info.k[4]=NAN;invalid_projection(bad,model.links);
  bad=partial;bad.scale=0;invalid_projection(bad,model.links);
  bad=partial;bad.minimum=-1;invalid_projection(bad,model.links);
  bad=partial;bad.maximum=bad.minimum;invalid_projection(bad,model.links);
  bad=partial;bad.stride=0;invalid_projection(bad,model.links);
  bad=partial;bad.keep.pop_back();invalid_projection(bad,model.links);
  bad=partial;bad.mask_max=NAN;invalid_projection(bad,model.links);
  std::cout<<"PASS "<<checks<<" checks\n";return 0;
} catch(const std::exception& e) {std::cerr<<"FAIL "<<checks<<" "<<e.what()<<"\n";return 1;}}
