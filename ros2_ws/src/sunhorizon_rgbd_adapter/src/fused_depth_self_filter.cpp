#include "sunhorizon_rgbd_adapter/fused_depth_self_filter.hpp"
#include <utility>
#include <optional>
#include <chrono>
namespace sunhorizon_rgbd_adapter {
// Only this same-frame consumer may supply precomputed geometry. It cannot be
// called by an external pointcloud admission path with unauthenticated results.
class DynamicFilterPreparedConsumer {
 public:
  static bool reserve(const DynamicFilterAccumulator& a,std::size_t count,const DynamicFilterBudget& b) {
    return count && b.query.max_node_visits<a.remaining_nodes()/count &&
      b.query.max_triangle_tests<a.remaining_triangles()/count;
  }
  static bool append(DynamicFilterAccumulator& a,const uint8_t* bytes,
      const std::array<float,3>& xyz,
      const sunhorizon_obstacle_coverage::dynamic_self_geometry::Result& result) {
    return a.append_precomputed(bytes,double(xyz[0]),double(xyz[1]),double(xyz[2]),result);
  }
};
bool qualified_native_depth_raster(const sensor_msgs::msg::Image& depth,
    const sensor_msgs::msg::CameraInfo& info,double scale) {
  // Only the original unbinned rectified native depth raster. A valid legacy
  // projection with absent or transformed calibration still uses center rays.
  if(depth.encoding!="16UC1"||!std::isfinite(scale)||!(scale>0)||
      depth.header.frame_id.empty()||depth.header.frame_id!=info.header.frame_id||
      depth.header.stamp.sec!=info.header.stamp.sec||depth.header.stamp.nanosec!=info.header.stamp.nanosec||
      depth.width!=info.width||depth.height!=info.height||!depth.width||!depth.height||
      info.binning_x||info.binning_y||info.roi.x_offset||info.roi.y_offset||
      info.roi.width||info.roi.height||info.roi.do_rectify||
      info.distortion_model!="plumb_bob"||info.d.size()!=5)return false;
  for(const double d:info.d)if(d!=0.)return false;
  for(const double k:info.k)if(!std::isfinite(k))return false;
  if(!(info.k[0]>0)||!(info.k[4]>0)||info.k[1]!=0||info.k[3]!=0||
      info.k[6]!=0||info.k[7]!=0||info.k[8]!=1)return false;
  const std::array<double,9> identity{{1,0,0,0,1,0,0,0,1}};
  const std::array<double,12> projection{{info.k[0],0,info.k[2],0,0,info.k[4],info.k[5],0,0,0,1,0}};
  return info.r==identity&&info.p==projection;
}

FusedDepthSelfFilterResult project_depth_filter_dynamic_self(
    const sensor_msgs::msg::Image& depth,const sensor_msgs::msg::CameraInfo& info,
    double scale,double minimum,double maximum,uint32_t stride,
    const sunhorizon_obstacle_coverage::dynamic_self_geometry::PreparedModel& model,
    double tolerance,const DynamicFilterBudget& budget,
    const std::vector<uint8_t>* keep,double mask_max_depth,BodyGeometryWorker* geometry_worker,BodyGeometryCuda* geometry_cuda,double stereo_baseline_m) {
  const bool profile=geometry_cuda&&geometry_cuda->profiling();
  const auto profile_start=profile?std::chrono::steady_clock::now():std::chrono::steady_clock::time_point{};
  const DepthProjectionTraversal projection(depth,info,scale,minimum,maximum,stride,keep,mask_max_depth);
  // Count without storing records: the legacy filter rejects over-budget input
  // before model validation/classification, even if most points would be self.
  const auto count=projection.point_count();
  DynamicFilterAccumulator accumulator(model,tolerance,budget,count,12);
  FusedDepthSelfFilterResult result;
  if(!accumulator.ready()){result.filter=accumulator.finish();return result;}
  result.retained_sample_ids.reserve(count);
  const bool native=qualified_native_depth_raster(depth,info,scale);
  sunhorizon_obstacle_coverage::dynamic_self_geometry::NativePixelSupport pixel;
  if(native)pixel={depth.width,depth.height,0,0,info.k[0],info.k[4],info.k[2],info.k[5],stereo_baseline_m};
  if(geometry_cuda) {
    if(geometry_worker)throw std::invalid_argument("body_geometry_backend_conflict");
    // Projection stays on the original CPU traversal, including FLOAT32 rounding,
    // stride, mask cutoff, sample identity and invalid-input precedence.
    std::vector<BodyGeometryCuda::Sample> samples;samples.reserve(count);
    projection.for_each_sample([&](const std::array<float,3>& xyz,std::size_t id){samples.push_back({xyz,id});return true;});
    if(profile)result.cuda_projection_pack_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-profile_start).count();
    geometry_cuda->query(model,tolerance,native?&pixel:nullptr,samples,[&](std::size_t i,const auto& computed){
      const auto& sample=samples.at(i);const auto bytes=depth_xyz_bytes(sample.xyz);
      const auto before=accumulator.retained_count(),removed=accumulator.removed_count();
      const bool complete=DynamicFilterPreparedConsumer::append(accumulator,bytes.data(),sample.xyz,computed);
      if(complete&&accumulator.retained_count()!=before)result.retained_sample_ids.push_back(sample.sample_id);
      if(complete&&accumulator.removed_count()!=removed)result.self_support.push_back({sample.xyz,sample.sample_id});
      return complete;
    });
    result.filter=accumulator.finish();
    if(result.filter.complete){result.cloud=projection.empty_cloud();result.cloud.width=static_cast<uint32_t>(result.filter.retained_count);result.cloud.row_step=result.cloud.width*result.cloud.point_step;result.cloud.data=std::move(result.filter.data);}
    else{result.retained_sample_ids.clear();result.self_support.clear();}
    return result;
  }
  std::optional<BodyGeometryWorker::Frame> lease;
  if(geometry_worker)lease.emplace(geometry_worker->begin_frame());
  struct Sample {std::array<float,3> xyz;std::size_t id;};
  constexpr std::size_t chunk=128;
  std::array<Sample,chunk> samples{};
  std::array<sunhorizon_obstacle_coverage::dynamic_self_geometry::Result,chunk> results{};
  std::size_t used=0;
  const auto append_one=[&](const Sample& sample,const auto* computed) {
    const auto bytes=depth_xyz_bytes(sample.xyz);const auto before=accumulator.retained_count();const auto removed=accumulator.removed_count();
    if(native){pixel.u=sample.id%depth.width;pixel.v=sample.id/depth.width;}
    const bool complete=computed ? DynamicFilterPreparedConsumer::append(accumulator,bytes.data(),sample.xyz,*computed) :
      accumulator.append(bytes.data(),double(sample.xyz[0]),double(sample.xyz[1]),double(sample.xyz[2]),native?&pixel:nullptr);
    if(complete&&accumulator.retained_count()!=before)result.retained_sample_ids.push_back(sample.id);
    if(complete&&accumulator.removed_count()!=removed)result.self_support.push_back({sample.xyz,sample.id});
    return complete;
  };
  const auto flush=[&] {
    // Reserve the worst case (including a failed-query boundary step) before
    // any speculative work. A short remaining budget uses the original serial
    // path, with its original first failure and partial-counter precedence.
    const bool parallel=geometry_worker&&DynamicFilterPreparedConsumer::reserve(accumulator,used,budget);
    if(parallel){
      const auto query_range=[&](std::size_t first,std::size_t last){
        for(auto i=first;i<last;++i){const auto& sample=samples[i];
          const double x=sample.xyz[0],y=sample.xyz[1],z=sample.xyz[2];
          if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)||z<=0)continue;
          auto cell=pixel;cell.u=sample.id%depth.width;cell.v=sample.id/depth.width;
          results[i]=native ? sunhorizon_obstacle_coverage::dynamic_self_geometry::classify_native_body_filter(
            {x/z,y/z,1},z,tolerance,model,cell) :
            sunhorizon_obstacle_coverage::dynamic_self_geometry::classify_body_surface({x/z,y/z,1},z,tolerance,model);
        }
      };
      geometry_worker->run([&]{query_range(used/2,used);},[&]{query_range(0,used/2);});
    }
    for(std::size_t i=0;i<used;++i)if(!append_one(samples[i],parallel?&results[i]:nullptr)){used=0;return false;}
    used=0;return true;
  };
  bool traversal_complete=true;
  projection.for_each_sample([&](const std::array<float,3>& xyz,std::size_t sample_id) {
    if(!geometry_worker)return append_one({xyz,sample_id},static_cast<const sunhorizon_obstacle_coverage::dynamic_self_geometry::Result*>(nullptr));
    samples[used++]={xyz,sample_id};
    if(used==chunk&&!flush()){traversal_complete=false;return false;}return true;
  });
  if(traversal_complete&&used)flush();
  if(geometry_worker)result.geometry_helper_cpu_ms=geometry_worker->frame_helper_cpu_ms();
  result.filter=accumulator.finish();
  if(result.filter.complete) {
    result.cloud=projection.empty_cloud();
    result.cloud.width=static_cast<uint32_t>(result.filter.retained_count);
    result.cloud.row_step=result.cloud.width*result.cloud.point_step;
    result.cloud.data=std::move(result.filter.data);
  }else {result.retained_sample_ids.clear();result.self_support.clear();}
  return result;
}
}
