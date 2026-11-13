#include "sunhorizon_rgbd_adapter/body_geometry_cuda.hpp"
#include <iostream>
#include <future>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <string>
#ifdef CAMERA_CUDA_TEST_CONTEXT
#include <cuda.h>
#endif
namespace rgb=sunhorizon_rgbd_adapter;
namespace dg=sunhorizon_obstacle_coverage::dynamic_self_geometry;
void check(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
template<class F>void rejects(F f,const char* why){try{f();}catch(const std::exception& e){check(std::string(e.what()).find(why)!=std::string::npos,e.what());return;}throw std::runtime_error("expected refusal");}
void same(const dg::Result&a,const dg::Result&b){
 check(a.classification==b.classification&&a.complete==b.complete&&a.node_visits==b.node_visits&&a.triangle_tests==b.triangle_tests,"discrete query mismatch");
 check(std::string(a.reason)==b.reason,"reason mismatch");
 check(a.has_first_surface==b.has_first_surface&&a.link_index==b.link_index&&a.triangle_index==b.triangle_index&&a.has_measured_surface==b.has_measured_surface&&a.measured_reference_ambiguous==b.measured_reference_ambiguous,"ownership mismatch");
 check(a.measured_link_index==b.measured_link_index&&a.measured_triangle_index==b.measured_triangle_index&&a.ambiguous_measured_link_index==b.ambiguous_measured_link_index&&a.ambiguous_measured_triangle_index==b.ambiguous_measured_triangle_index&&a.finite_surface_link_index==b.finite_surface_link_index&&a.finite_surface_triangle_index==b.finite_surface_triangle_index,"surface ownership mismatch");
}

void independent_roles_and_stop(){
 dg::Mesh mesh;check(mesh.build({{{{{-2,-2,1},{2,-2,1},{2,2,1}}}}},{}),"dual role mesh");
 dg::PreparedModel model({{&mesh,"body",{},true,true,true}},{});
 std::vector<rgb::BodyGeometryCuda::Sample> points(63,{{0,0,.9f},0});
 rgb::BodyGeometryCuda belly,chest;std::promise<void> belly_entered,chest_entered,release;
 auto belly_ready=belly_entered.get_future(),chest_ready=chest_entered.get_future();auto unblock=release.get_future().share();
#ifdef CAMERA_CUDA_TEST_CONTEXT
 CUcontext belly_context=nullptr,chest_context=nullptr;
#endif
 const auto run=[&](rgb::BodyGeometryCuda& backend,std::promise<void>& entered,bool is_belly){
   try{backend.query(model,.02,nullptr,points,[&](std::size_t i,const dg::Result&r){
     same(dg::classify_body_surface({0,0,1},double(.9f),.02,model),r);
     if(!i){
#ifdef CAMERA_CUDA_TEST_CONTEXT
       CUcontext current=nullptr;check(cuCtxGetCurrent(&current)==CUDA_SUCCESS,"dual context get");
       if(is_belly)belly_context=current;else chest_context=current;
#else
       (void)is_belly;
#endif
       entered.set_value();unblock.wait();
     }return true;});return false;
   }catch(const std::exception&e){check(std::string(e.what()).find("stopped")!=std::string::npos,e.what());return true;}
 };
 auto a=std::async(std::launch::async,[&]{return run(belly,belly_entered,true);});
 auto c=std::async(std::launch::async,[&]{return run(chest,chest_entered,false);});
 const bool both=belly_ready.wait_for(std::chrono::seconds(5))==std::future_status::ready&&chest_ready.wait_for(std::chrono::seconds(5))==std::future_status::ready;
 belly.request_stop();release.set_value();const bool a_stopped=a.get(),c_stopped=c.get();
 check(both&&a_stopped&&!c_stopped,"dual roles coupled by stop or lock");
#ifdef CAMERA_CUDA_TEST_CONTEXT
 check(belly_context&&chest_context&&belly_context!=chest_context,"roles share CUDA context");
#endif
 std::size_t consumed=0;chest.query(model,.02,nullptr,points,[&](auto,const auto&){++consumed;return true;});check(consumed==points.size(),"other role lost after stop");
}
void stop_while_event_pending(){
 // Real bounded query work, no artificial sleep kernel or computation timeout.
 // Identical overlapping faces force substantial actual BVH/triangle work.
 dg::Mesh mesh;std::vector<dg::Triangle> triangles(256,{{{{-2,-2,1},{2,-2,1},{2,2,1}}}});
 check(mesh.build(triangles,{}),"pending mesh");dg::PreparedModel model({{&mesh,"body",{},true,true,true}},{});
 std::vector<rgb::BodyGeometryCuda::Sample> points(307200,{{0,0,.9f},0});
 auto backend=std::make_unique<rgb::BodyGeometryCuda>();std::atomic<std::size_t> consumed{0};
 auto result=std::async(std::launch::async,[&]{try{backend->query(model,.02,nullptr,points,[&](auto,const auto&){++consumed;return true;});return false;}catch(const std::exception&e){check(std::string(e.what()).find("stopped")!=std::string::npos,e.what());return true;}});
 bool pending=false;const auto observation_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
 while(std::chrono::steady_clock::now()<observation_deadline){
   if(backend->has_pending_device_work()){pending=true;break;}
   if(result.wait_for(std::chrono::seconds(0))==std::future_status::ready)break;
   std::this_thread::yield();
 }
 backend->request_stop();const bool still_pending=backend->has_pending_device_work();const bool stopped=result.get();
 check(pending&&still_pending,"test did not observe outstanding GPU work at stop");
 check(stopped&&consumed.load()==0,"pending stop committed output");
 rejects([&]{backend->query(model,.02,nullptr,points,[](auto,const auto&){return true;});},"stopped");
 // Query has returned while its event was incomplete at stop. Destruction must
 // drain this owned stream before freeing pinned/device buffers or the context.
 backend.reset();
}
int main(){try{
 if(!rgb::BodyGeometryCuda::compiled()){
  rejects([]{rgb::BodyGeometryCuda backend;},"camera_body_cuda_not_compiled");
  std::cout<<"CPU-only explicit CUDA refusal passed; GPU contracts not run\n";return 0;
 }
#ifdef CAMERA_CUDA_TEST_CONTEXT
 CUcontext caller=nullptr,current=nullptr;CUdevice device{};
 check(cuInit(0)==CUDA_SUCCESS&&cuDeviceGet(&device,0)==CUDA_SUCCESS,"driver init");
 check(cuCtxCreate(&caller,CU_CTX_SCHED_AUTO,device)==CUDA_SUCCESS,"test caller context");
#endif
 {
 rgb::BodyGeometryCuda backend;
#ifdef CAMERA_CUDA_TEST_CONTEXT
 check(cuCtxGetCurrent(&current)==CUDA_SUCCESS&&current==caller,"constructor changed caller context");
#endif
 dg::Mesh mesh;check(mesh.build({{{{{-2,-2,1},{2,-2,1},{2,2,1}}}},{{{{-2,-2,1},{2,2,1},{-2,2,1}}}}},{}),"mesh build");
 std::vector<dg::LinkObservation> links{{&mesh,"body",{},true,true,true}};
 std::vector<rgb::BodyGeometryCuda::Sample> points;
 for(std::size_t i=0;i<63;++i)points.push_back({{float((int(i%7)-3)*.1),float((int(i/7)-4)*.1),float(.8+(i%5)*.1)},i});
 dg::NativePixelSupport pixel{7,9,0,0,10,10,3,4};
 for(bool native:{false,true})for(int repeat=0;repeat<3;++repeat){
  dg::PreparedModel model(links,{});std::size_t consumed=0;
  backend.query(model,.02,native?&pixel:nullptr,points,[&](std::size_t i,const dg::Result&r){
    auto p=pixel;p.u=points[i].sample_id%p.width;p.v=points[i].sample_id/p.width;
    const double x=points[i].xyz[0],y=points[i].xyz[1],z=points[i].xyz[2];
    same(native?dg::classify_native_body_filter({x/z,y/z,1},z,.02,model,p):dg::classify_body_surface({x/z,y/z,1},z,.02,model),r);++consumed;return true;
  });check(consumed==points.size(),"lost sample");
 }
 // Same source native stereo uses the same per-point counters on both eyes.
 {
  dg::Mesh rim;check(rim.build({dg::Triangle{{dg::Vec3{.055,-.02,.5},dg::Vec3{.065,-.02,.5},dg::Vec3{.06,.02,.5}}}},{}),"stereo mesh");
  std::vector<dg::LinkObservation> stereo_links{{&rim,"known_body",{},true,true,true}};
  dg::NativePixelSupport p{200,100,50,50,100,100,50,50,.12};
  const std::vector<rgb::BodyGeometryCuda::Sample> probes{{{0,0,1},10050},{{0,0,.3f},10050},{{.5f,0,1},10100}};
  for(std::size_t cap:{std::size_t(65536),std::size_t(1)}) {
   dg::QueryBudget q;q.max_node_visits=cap;dg::PreparedModel stereo(stereo_links,q);std::size_t seen=0;
   backend.query(stereo,.03,&p,probes,[&](std::size_t i,const dg::Result& actual){
    auto cell=p;cell.u=probes[i].sample_id%p.width;cell.v=probes[i].sample_id/p.width;
    const auto& v=probes[i].xyz;const auto expected=dg::classify_native_body_filter({double(v[0])/v[2],double(v[1])/v[2],1},v[2],.03,stereo,cell);
    same(expected,actual);
    if(cap==65536)check(dg::measurement_body_occluded(actual)==(i==0),"stereo foreground/visible object protection");
    else check(!dg::measurement_body_occluded(actual),"incomplete stereo granted rejection");
    ++seen;return true;
   });check(seen==probes.size(),"stereo probes missing");
  }
 }
 // Scratch storage must handle growth, shrinkage and byte changes without
 // treating reused addresses/revisions or old tail bytes as identity evidence.
 for(std::size_t count:{1U,3U,2U,1U}) {
   std::vector<dg::Triangle> triangles;
   for(std::size_t n=0;n<count;++n){const double z=.9+n*.07;triangles.push_back({{{{-2,-2,z},{2,-2,z},{2,2,z}}}});}
   check(mesh.build(triangles,{}),"changed mesh build");
   for(std::size_t link_count:{2U,1U}){
     auto varied=links;if(link_count==2)varied.push_back({&mesh,"body-copy",{},true,true,true});
     dg::PreparedModel changed(varied,{});
     backend.query(changed,.02,nullptr,points,[&](std::size_t i,const dg::Result&r){const double x=points[i].xyz[0],y=points[i].xyz[1],z=points[i].xyz[2];same(dg::classify_body_surface({x/z,y/z,1},z,.02,changed),r);return true;});
   }
 }
 dg::PreparedModel model(links,{});
 // Ordered consumer stops at the original prefix and cannot race buffer reuse.
 std::size_t consumed=0;backend.query(model,.02,&pixel,points,[&](std::size_t,const dg::Result&){return ++consumed<3;});check(consumed==3,"prefix commit ignored");
 std::promise<void> entered,release;auto done=release.get_future();
 auto worker=std::async(std::launch::async,[&]{backend.query(model,.02,&pixel,points,[&](std::size_t i,const dg::Result&){if(!i){entered.set_value();done.wait();}return true;});});
 entered.get_future().wait();rejects([&]{backend.query(model,.02,&pixel,points,[](auto,const auto&){return true;});},"concurrent_frame");release.set_value();worker.get();
#ifdef CAMERA_CUDA_TEST_CONTEXT
 check(cuCtxGetCurrent(&current)==CUDA_SUCCESS&&current==caller,"query changed caller context");
#endif
 backend.request_stop();rejects([&]{backend.query(model,.02,&pixel,points,[](auto,const auto&){return true;});},"stopped");
 // Rebuilt source invalidates old model before upload; errors poison that owner.
 rgb::BodyGeometryCuda invalidated;check(mesh.build({{{{{-2,-2,2},{2,-2,2},{2,2,2}}}}},{}),"rebuild");
 rejects([&]{invalidated.query(model,.02,&pixel,points,[](auto,const auto&){return true;});},"model_revision");
 rejects([&]{invalidated.query(model,.02,&pixel,points,[](auto,const auto&){return true;});},"failed_or_pending");
 }
 independent_roles_and_stop();stop_while_event_pending();
#ifdef CAMERA_CUDA_TEST_CONTEXT
 check(cuCtxGetCurrent(&current)==CUDA_SUCCESS&&current==caller,"destructor changed caller context");check(cuCtxDestroy(caller)==CUDA_SUCCESS,"caller context cleanup");
#endif
 std::cout<<"CUDA query parity, prefix, one-inflight, model revision, independent role stop and pending-event drain passed\n";return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
