#include "sunhorizon_rgbd_adapter/body_geometry_cuda.hpp"
#include <sunhorizon_obstacle_coverage/dynamic_self_geometry_cuda_contract.hpp>
#include <cuda.h>
#include <cuda_runtime.h>
#include <chrono>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include "device_geometry.cuh"
#if !defined(__aarch64__) || __GNUC__ != 11 || __GNUC_MINOR__ != 4 || __GNUC_PATCHLEVEL__ != 0
#error "Camera CUDA arithmetic requires reviewed GCC 11.4.0 AArch64 host compiler"
#endif
#if __CUDACC_VER_MAJOR__ != 12 || __CUDACC_VER_MINOR__ != 6
#error "Camera CUDA arithmetic requires reviewed CUDA 12.6 compiler"
#endif
namespace rgb=sunhorizon_rgbd_adapter;
namespace dg=sunhorizon_obstacle_coverage::dynamic_self_geometry;
namespace gg=gpu_geometry;
namespace {
double wall_ms(){return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();}
constexpr std::size_t kMaximumPoints=307200;
constexpr std::size_t kMaximumMeshBytes=256u*1024u*1024u;
void require(bool v,const char* why){if(!v)throw std::runtime_error(why);}
void check(cudaError_t e){if(e!=cudaSuccess)throw std::runtime_error(std::string("camera_body_cuda:")+cudaGetErrorString(e));}
void check_driver(CUresult e){if(e!=CUDA_SUCCESS){const char* why=nullptr;cuGetErrorString(e,&why);throw std::runtime_error(std::string("camera_body_cuda_context:")+(why?why:"unknown"));}}
template<class T,class U> T pod(const U& value){static_assert(sizeof(T)==sizeof(U));static_assert(std::is_trivially_copyable_v<T>&&std::is_trivially_copyable_v<U>);T out;std::memcpy(&out,&value,sizeof(out));return out;}
struct ContextScope {
  CUcontext context;
  explicit ContextScope(CUcontext c):context(c){check_driver(cuCtxPushCurrent(context));}
  ~ContextScope(){CUcontext popped=nullptr;(void)cuCtxPopCurrent(&popped);}
};
struct Snapshot {
  std::vector<unsigned char> bytes;
  std::size_t used=0;
  std::vector<gg::Mesh> meshes;
  std::vector<gg::PreparedModel::Link> links;
  gg::PreparedModel model;
  template<class T,class U> gg::View<T> append(const std::vector<U>& values){
    static_assert(sizeof(T)==sizeof(U));static_assert(alignof(T)==alignof(U));
    const auto at=(used+alignof(T)-1)&~(alignof(T)-1);
    require(values.size()<=kMaximumMeshBytes/sizeof(U)&&at<=kMaximumMeshBytes-values.size()*sizeof(U),"camera_body_cuda_mesh_capacity");
    const auto end=at+values.size()*sizeof(U);
    // Reuse owned scratch storage. Existing bytes are overwritten completely;
    // no mesh identity check or byte comparison is skipped on a warm frame.
    if(end>bytes.size())bytes.resize(end);
    if(at>used)std::memset(bytes.data()+used,0,at-used);
    used=end;
    if(!values.empty())std::memcpy(bytes.data()+at,values.data(),values.size()*sizeof(U));
    // Offset only. Converted into an owned device address before upload.
    return {reinterpret_cast<T*>(at),values.size()};
  }
};
struct Row {gg::Result result;char reason[96];};
__global__ void query_body(const rgb::BodyGeometryCuda::Sample* samples,std::size_t count,
    const gg::PreparedModel* model,double tolerance,gg::NativePixelSupport pixel,bool native,Row* rows){
  const auto at=std::size_t(blockIdx.x)*blockDim.x+threadIdx.x;if(at>=count)return;
  const auto& sample=samples[at];const double x=sample.xyz[0],y=sample.xyz[1],z=sample.xyz[2];
  auto& row=rows[at];row.result=gg::Result{};
  if(std::isfinite(x)&&std::isfinite(y)&&std::isfinite(z)&&z>0){
    if(native){pixel.u=sample.sample_id%pixel.width;pixel.v=sample.sample_id/pixel.width;}
    row.result=native?gg::classify_native_body_filter({x/z,y/z,1.},z,tolerance,*model,pixel):
      gg::classify_body_surface({x/z,y/z,1.},z,tolerance,*model);
  }
  unsigned i=0;for(;i+1<sizeof(row.reason)&&row.result.reason[i];++i)row.reason[i]=row.result.reason[i];row.reason[i]=0;
}
}
namespace sunhorizon_obstacle_coverage::dynamic_self_geometry {
// The sole runtime friend snapshots immutable model storage. It does not grant
// identity/TF/expiry and cannot mutate Mesh or PreparedModel.
class CudaGeometryAccess {
 public:
  static void snapshot(const PreparedModel& input,Snapshot& out){
    require(input.ready(),"camera_body_cuda_model_not_ready");
    require(!input.links_.empty()&&input.links_.size()<=32,"camera_body_cuda_link_capacity");
    out.used=0;out.links.clear();
    if(out.meshes.size()!=input.links_.size())out.meshes=std::vector<gg::Mesh>(input.links_.size());
    out.links.reserve(input.links_.size());
    for(std::size_t i=0;i<input.links_.size();++i){
      const auto& link=input.links_[i];require(link.mesh&&link.mesh->ready_&&link.revision==link.mesh->revision_,"camera_body_cuda_model_revision");
      const auto& source=*link.mesh;auto& target=out.meshes[i];
      target.ready_=source.ready_;target.revision_=source.revision_;
      target.triangles_=out.append<gg::Triangle>(source.triangles_);
      target.order_=out.append<std::size_t>(source.order_);
      target.nodes_=out.append<gg::Mesh::Node>(source.nodes_);
      target.boundary_edges_=out.append<gg::Mesh::BoundaryEdge>(source.boundary_edges_);
      target.boundary_bounds_=pod<gg::Mesh::Bounds>(source.boundary_bounds_);
      target.boundary_first_=source.boundary_first_;target.boundary_count_=source.boundary_count_;target.boundary_tolerance_m_=source.boundary_tolerance_m_;
      target.measured_first_=source.measured_first_;target.measured_count_=source.measured_count_;target.measured_bounds_=pod<gg::Mesh::Bounds>(source.measured_bounds_);
      auto uploaded=pod<gg::PreparedModel::Link>(link);uploaded.mesh=reinterpret_cast<const gg::Mesh*>(i);out.links.push_back(uploaded);
    }
    out.model.surface_bounds_min_=pod<gg::Vec3>(input.surface_bounds_min_);out.model.surface_bounds_max_=pod<gg::Vec3>(input.surface_bounds_max_);
    out.model.surface_bounds_padding_=input.surface_bounds_padding_;out.model.surface_bounds_valid_=input.surface_bounds_valid_;
    out.model.budget_=pod<gg::QueryBudget>(input.budget_);out.model.reason_=nullptr;out.bytes.resize(out.used);
  }
};
}
namespace sunhorizon_rgbd_adapter {
struct BodyGeometryCuda::Impl {
  struct Buffer {
    void* device=nullptr;void* host=nullptr;std::size_t capacity=0;
    void ensure(std::size_t count){
      if(count<=capacity)return;
      if(device){check(cudaFree(device));device=nullptr;}if(host){check(cudaFreeHost(host));host=nullptr;}capacity=0;
      check(cudaMalloc(&device,count));check(cudaMallocHost(&host,count));capacity=count;
    }
    void release()noexcept{if(device)(void)cudaFree(device);if(host)(void)cudaFreeHost(host);device=host=nullptr;capacity=0;}
    void upload(std::size_t count,cudaStream_t stream){if(count)check(cudaMemcpyAsync(device,host,count,cudaMemcpyHostToDevice,stream));}
  } mesh_bytes,meshes,links,model,samples,results;
  CUdevice device{};CUcontext context=nullptr;cudaStream_t stream=nullptr;cudaEvent_t done=nullptr,profile_events[3]{};
  bool profile=false;Timings timings;
  std::mutex mutex;std::atomic<bool> stopped{false},submitted{false};bool failed=false,pending=false;
  std::vector<unsigned char> cached_bytes,cached_meshes;
  Snapshot snapshot_storage;
  explicit Impl(bool enable_profile):profile(enable_profile){
    require(dg::camera_cuda_arithmetic_contract_supported(),"camera_body_cuda_cpu_arithmetic_contract_missing");
    try{
      check_driver(cuInit(0));check_driver(cuDeviceGet(&device,0));
      // Each role owns a distinct context; creation pushes it on this thread.
      // Pop immediately so every later scope restores the caller's context.
      check_driver(cuCtxCreate(&context,0,device));CUcontext created=nullptr;
      check_driver(cuCtxPopCurrent(&created));require(created==context,"camera_body_cuda_context_create_stack");
      ContextScope guard(context);cudaDeviceProp properties{};check(cudaGetDeviceProperties(&properties,0));
      require(properties.major==8&&properties.minor==7,"camera_body_cuda_requires_sm87");
      check(cudaStreamCreateWithFlags(&stream,cudaStreamNonBlocking));check(cudaEventCreateWithFlags(&done,profile?cudaEventDefault:cudaEventDisableTiming));
      if(profile)for(auto& event:profile_events)check(cudaEventCreate(&event));
    }catch(...){release();throw;}
  }
  void release()noexcept{
    if(!context)return;
    if(cuCtxPushCurrent(context)!=CUDA_SUCCESS)return; // Never free another owner's context resources.
    if(stream)(void)cudaStreamSynchronize(stream);
    for(auto* b:{&mesh_bytes,&meshes,&links,&model,&samples,&results})b->release();
    for(auto event:profile_events)if(event)(void)cudaEventDestroy(event);
    if(done)(void)cudaEventDestroy(done);if(stream)(void)cudaStreamDestroy(stream);
    CUcontext popped=nullptr;(void)cuCtxPopCurrent(&popped);(void)cuCtxDestroy(context);context=nullptr;
  }
  ~Impl(){release();}
  void check_admission(){require(!stopped.load(),"camera_body_cuda_stopped");require(!failed&&!pending,"camera_body_cuda_failed_or_pending");}
  void wait(){
    for(;;){
      // Cancellation discards this source's output. The instance is poisoned
      // and all device/pinned buffers stay alive until release() drains.
      require(!stopped.load(),"camera_body_cuda_stopped");
      const auto status=cudaEventQuery(done);if(status==cudaSuccess){pending=false;submitted.store(false,std::memory_order_release);return;}
      if(status!=cudaErrorNotReady)check(status);
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  }
};
bool BodyGeometryCuda::compiled()noexcept{return true;}
BodyGeometryCuda::BodyGeometryCuda(bool profile):impl_(std::make_unique<Impl>(profile)){}
bool BodyGeometryCuda::profiling()const noexcept{return impl_->profile;}
BodyGeometryCuda::Timings BodyGeometryCuda::last_timings()const{
  std::unique_lock<std::mutex> lock(impl_->mutex,std::try_to_lock);require(lock.owns_lock(),"camera_body_cuda_concurrent_timing_read");return impl_->timings;
}
bool BodyGeometryCuda::has_pending_device_work()const{
  if(!impl_->submitted.load(std::memory_order_acquire))return false;
  ContextScope context(impl_->context);const auto status=cudaEventQuery(impl_->done);
  if(status==cudaErrorNotReady)return true;check(status);return false;
}
BodyGeometryCuda::~BodyGeometryCuda()=default;
void BodyGeometryCuda::request_stop()noexcept{impl_->stopped.store(true);}
void BodyGeometryCuda::query(const Model& input,double tolerance,const Pixel* pixel,
    const std::vector<Sample>& input_samples,const std::function<bool(std::size_t,const Result&)>& consume){
  auto& p=*impl_;std::unique_lock<std::mutex> lock(p.mutex,std::try_to_lock);
  require(lock.owns_lock(),"camera_body_cuda_concurrent_frame");p.check_admission();
  require(!input_samples.empty()&&input_samples.size()<=kMaximumPoints&&bool(consume),"camera_body_cuda_input_capacity");
  try{
    ContextScope context(p.context);
    const auto start=p.profile?wall_ms():0.;auto at=start;p.timings={};
    auto& state=p.snapshot_storage;dg::CudaGeometryAccess::snapshot(input,state);
    if(p.profile){p.timings.snapshot_ms=wall_ms()-at;at=wall_ms();}
    // Exact byte comparison includes topology/order, patch metadata and mesh
    // revisions, so allocator address reuse cannot manufacture a cache hit.
    const bool changed=state.bytes!=p.cached_bytes||state.meshes.size()*sizeof(gg::Mesh)!=p.cached_meshes.size()||
      std::memcmp(state.meshes.data(),p.cached_meshes.data(),state.meshes.size()*sizeof(gg::Mesh))!=0;
    if(p.profile){p.timings.cache_check_ms=wall_ms()-at;at=wall_ms();p.timings.mesh_uploaded=changed;p.timings.mesh_bytes=state.bytes.size();}
    p.mesh_bytes.ensure(state.bytes.size());p.meshes.ensure(state.meshes.size()*sizeof(gg::Mesh));
    p.links.ensure(state.links.size()*sizeof(gg::PreparedModel::Link));p.model.ensure(sizeof(gg::PreparedModel));
    p.samples.ensure(input_samples.size()*sizeof(Sample));p.results.ensure(input_samples.size()*sizeof(Row));
    if(changed){
      p.cached_bytes=state.bytes;p.cached_meshes.resize(state.meshes.size()*sizeof(gg::Mesh));
      std::memcpy(p.cached_meshes.data(),state.meshes.data(),p.cached_meshes.size());
      if(!state.bytes.empty())std::memcpy(p.mesh_bytes.host,state.bytes.data(),state.bytes.size());
      for(auto& mesh:state.meshes){
        const auto relocate=[&](auto& view){using Pointer=decltype(view.pointer);view.pointer=reinterpret_cast<Pointer>(static_cast<unsigned char*>(p.mesh_bytes.device)+reinterpret_cast<std::uintptr_t>(view.pointer));};
        relocate(mesh.triangles_);relocate(mesh.order_);relocate(mesh.nodes_);relocate(mesh.boundary_edges_);
      }
      std::memcpy(p.meshes.host,state.meshes.data(),state.meshes.size()*sizeof(gg::Mesh));
    }
    for(std::size_t i=0;i<state.links.size();++i)state.links[i].mesh=static_cast<gg::Mesh*>(p.meshes.device)+i;
    state.model.links_={static_cast<gg::PreparedModel::Link*>(p.links.device),state.links.size()};
    std::memcpy(p.links.host,state.links.data(),state.links.size()*sizeof(gg::PreparedModel::Link));
    std::memcpy(p.model.host,&state.model,sizeof(state.model));std::memcpy(p.samples.host,input_samples.data(),input_samples.size()*sizeof(Sample));
    p.check_admission(); // Stop during host packing must not enqueue another source.
    if(p.profile){p.timings.buffer_prepare_ms=wall_ms()-at;at=wall_ms();}
    p.pending=true;
    if(p.profile)check(cudaEventRecord(p.profile_events[0],p.stream));
    if(changed){p.mesh_bytes.upload(state.bytes.size(),p.stream);p.meshes.upload(state.meshes.size()*sizeof(gg::Mesh),p.stream);}
    p.links.upload(state.links.size()*sizeof(gg::PreparedModel::Link),p.stream);p.model.upload(sizeof(state.model),p.stream);p.samples.upload(input_samples.size()*sizeof(Sample),p.stream);
    if(p.profile)check(cudaEventRecord(p.profile_events[1],p.stream));
    const auto native=pixel?pod<gg::NativePixelSupport>(*pixel):gg::NativePixelSupport{};
    query_body<<<(input_samples.size()+127)/128,128,0,p.stream>>>(static_cast<Sample*>(p.samples.device),input_samples.size(),static_cast<gg::PreparedModel*>(p.model.device),tolerance,native,pixel!=nullptr,static_cast<Row*>(p.results.device));
    check(cudaGetLastError());
    if(p.profile)check(cudaEventRecord(p.profile_events[2],p.stream));
    check(cudaMemcpyAsync(p.results.host,p.results.device,input_samples.size()*sizeof(Row),cudaMemcpyDeviceToHost,p.stream));
    check(cudaEventRecord(p.done,p.stream));
    p.submitted.store(true,std::memory_order_release);
    if(p.profile){p.timings.enqueue_ms=wall_ms()-at;at=wall_ms();}
    p.wait();
    if(p.profile){
      p.timings.wait_ms=wall_ms()-at;
      check(cudaEventElapsedTime(&p.timings.h2d_ms,p.profile_events[0],p.profile_events[1]));
      check(cudaEventElapsedTime(&p.timings.kernel_ms,p.profile_events[1],p.profile_events[2]));
      check(cudaEventElapsedTime(&p.timings.d2h_ms,p.profile_events[2],p.done));
      p.timings.h2d_bytes=(changed?state.bytes.size()+state.meshes.size()*sizeof(gg::Mesh):0)+state.links.size()*sizeof(gg::PreparedModel::Link)+sizeof(state.model)+input_samples.size()*sizeof(Sample);
      p.timings.d2h_bytes=input_samples.size()*sizeof(Row);at=wall_ms();
    }
    const auto* out=static_cast<const Row*>(p.results.host);
    for(std::size_t i=0;i<input_samples.size();++i){
      require(!p.stopped.load(),"camera_body_cuda_stopped");
      auto value=pod<Result>(out[i].result);value.reason=out[i].reason;
      if(!consume(i,value))break;
    }
    if(p.profile){p.timings.consume_ms=wall_ms()-at;p.timings.total_ms=wall_ms()-start;}
  }catch(...){p.failed=true;throw;}
}
}
