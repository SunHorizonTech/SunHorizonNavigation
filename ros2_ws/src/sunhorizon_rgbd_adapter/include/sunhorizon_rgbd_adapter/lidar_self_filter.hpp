#pragma once
#include <cmath>
#include <algorithm>
#include <utility>
#include <cstring>
#include <limits>
#include <map>
#include <unordered_set>
#include <stdexcept>
#include "sunhorizon_rgbd_adapter/pointcloud_self_exclusion_direct.hpp"

namespace sunhorizon_rgbd_adapter {
struct LidarSelfFilterResult : ExclusionResult {
  std::size_t unique_prepared_models{0};
  std::shared_ptr<const LidarSelfAudit> audit;
  std::size_t self_surface_count{0},external_count{0},conflict_count{0};
  std::size_t unobserved_count{0},invalid_count{0},source_time_groups{0},outside_cad_envelope_count{0};
};
// The accepted compact Airy schema carries FLOAT32 seconds from the raw scan
// header. Rebasing that header changes the absolute point time by the SAME
// offset, so normalized_header + relative_time is the original source query.
// No time binning, latest-time FK, optical Z or fixed ray-origin substitution.
// Diagnostic mode retains every record. In removal mode only complete nearest
// SelfSurface matches are deleted; unavailable/conflicting rays remain returns.
inline LidarSelfFilterResult filter_lidar_self_points(const SelfExclusionDirectInput& input,
    const LidarModelProvider& provide,double tolerance,bool remove_self_surfaces,
    double surface_radius_bound=std::numeric_limits<double>::infinity(), bool capture_audit=false) {
  namespace dg=sunhorizon_obstacle_coverage::dynamic_self_geometry;
  if(!input.cloud || !provide || !std::isfinite(tolerance) || tolerance<0.)
    throw std::invalid_argument("lidar_self_filter_invalid_contract");
  const auto& cloud=*input.cloud;
  const auto count=std::size_t(cloud.width)*cloud.height;
  if(cloud.is_bigendian || !cloud.width || !cloud.height ||
      !cloud.point_step || std::size_t(cloud.row_step)<std::size_t(cloud.width)*cloud.point_step ||
      cloud.data.size()!=std::size_t(cloud.row_step)*cloud.height ||
      !input.ray_origins || input.ray_origins->size()!=count)
    throw std::invalid_argument("lidar_self_filter_layout_or_ray_ownership");
  std::array<std::size_t,4> offsets{};
  for(std::size_t axis=0;axis<4;++axis){
    const std::string name=axis<3?std::string(1,"xyz"[axis]):"timestamp";
    unsigned found=0;
    for(const auto& field:cloud.fields)if(field.name==name){
      if(++found!=1 || field.datatype!=sensor_msgs::msg::PointField::FLOAT32 || field.count!=1 ||
          field.offset>cloud.point_step || cloud.point_step-field.offset<sizeof(float))
        throw std::invalid_argument("lidar_self_filter_expected_compact_relative_xyzirt");
      offsets[axis]=field.offset;
    }
    if(found!=1)throw std::invalid_argument("lidar_self_filter_missing_field");
    for(std::size_t previous=0;previous<axis;++previous)
      if(offsets[axis]<offsets[previous]+4 && offsets[previous]<offsets[axis]+4)
        throw std::invalid_argument("lidar_self_filter_overlapping_fields");
  }
  LidarSelfFilterResult result;result.input_count=count;
  std::array<std::vector<LidarSelfAuditPoint>,3> audit_groups;
  bool audit_ok=true;
  const auto audit_key=[](const LidarSelfAuditPoint& p){return std::pair<double,uint64_t>{
    double(p.position[0])*p.position[0]+double(p.position[1])*p.position[1],p.sample_id};};
  const auto audit=[&](std::size_t i,int64_t ns,const std::array<float,4>& xyz,
      const std::array<float,3>& origin,const dg::Result& item,const dg::PreparedModel& model){
    if(!capture_audit || xyz[2]>=0.f || !item.complete)return;
    const int bucket=item.classification==dg::Classification::SelfSurface?0:
      item.classification==dg::Classification::ExternalSurface?1:
      item.classification==dg::Classification::ModelConflict?2:-1;
    if(bucket<0)return;
    auto& selected=audit_groups[std::size_t(bucket)];
    const auto key=std::pair<double,uint64_t>{double(xyz[0])*xyz[0]+double(xyz[1])*xyz[1],i};
    auto replace=selected.end();
    if(selected.size()==32){replace=std::max_element(selected.begin(),selected.end(),
      [&](const auto& a,const auto& b){return audit_key(a)<audit_key(b);});
      if(!(key<audit_key(*replace)))return;}
    LidarSelfAuditPoint point;point.sample_id=i;point.source_ns=ns;
    point.position={xyz[0],xyz[1],xyz[2]};point.ray_origin=origin;point.result=item;
    point.links=model.diagnostic_links();
    if(replace==selected.end())selected.push_back(std::move(point));else *replace=std::move(point);
  };
  auto output=cloud;output.data.clear();output.data.reserve(count*cloud.point_step);
  std::map<int64_t,std::shared_ptr<const dg::PreparedModel>> prepared;
  std::vector<uint8_t> original_ground_blocked;original_ground_blocked.reserve(count);
  result.ground_blocked.reserve(count);
  for(std::size_t i=0;i<count;++i){
    const auto* record=cloud.data.data()+(i/cloud.width)*cloud.row_step+(i%cloud.width)*cloud.point_step;
    std::array<float,4> values{};
    for(std::size_t axis=0;axis<4;++axis)std::memcpy(&values[axis],record+offsets[axis],4);
    const auto& origin=(*input.ray_origins)[i];
    bool self=false,ground_blocked=true;
    if(!std::isfinite(values[0]) || !std::isfinite(values[1]) || !std::isfinite(values[2])){
      ++result.invalid_count;
    }else if(!std::isfinite(values[3]) || values[3]<0. || values[3]>.2 ||
        !std::isfinite(origin[0]) || !std::isfinite(origin[1]) || !std::isfinite(origin[2])){
      ++result.unobserved_count; // No geometry identity: retain, never infer an origin.
    }else{
      // A far endpoint cannot be Self, but that says nothing about visibility.
      // Its actual source-time ray still needs the shared CAD/BVH query before
      // the endpoint can support Ground (a body-occluded return stays blocked).
      if(std::hypot(std::hypot(double(values[0]),double(values[1])),double(values[2]))>
          std::nextafter(surface_radius_bound+tolerance,std::numeric_limits<double>::infinity()))
        ++result.outside_cad_envelope_count;
      const auto offset_ns=std::llround(double(values[3])*1.e9);
      if(input.stamp.source_ns<=0 || input.stamp.source_ns>std::numeric_limits<int64_t>::max()-offset_ns)
        throw std::invalid_argument("lidar_self_filter_source_arithmetic");
      const int64_t source_ns=input.stamp.source_ns+offset_ns;
      auto found=prepared.find(source_ns);
      if(found==prepared.end())found=prepared.emplace(source_ns,provide(cloud.header.frame_id,source_ns)).first;
      if(!found->second){++result.unobserved_count;}
      else{
        const dg::Vec3 ray{double(values[0])-origin[0],double(values[1])-origin[1],double(values[2])-origin[2]};
        const auto range=std::hypot(std::hypot(ray.x,ray.y),ray.z);
        if(!std::isfinite(range) || range<=0.)++result.invalid_count;
        else{
          const auto item=dg::classify_body_range({origin[0],origin[1],origin[2]},
            {ray.x/range,ray.y/range,ray.z/range},range,tolerance,*found->second);
          if(capture_audit && audit_ok)
            audit_ok=try_lidar_diagnostic([&]{audit(i,source_ns,values,origin,item,*found->second);});
          if(!item.complete)++result.unobserved_count;
          else if(item.classification==dg::Classification::SelfSurface){++result.self_surface_count;self=true;}
          else if(item.classification==dg::Classification::ModelConflict)++result.conflict_count;
          else if(item.classification==dg::Classification::ExternalSurface){++result.external_count;ground_blocked=false;}
          else ++result.invalid_count;
        }
      }
    }
    original_ground_blocked.push_back(ground_blocked);
    if(remove_self_surfaces && self)++result.removed_count;
    else{output.data.insert(output.data.end(),record,record+cloud.point_step);result.ground_blocked.push_back(ground_blocked);
      result.ray_origins.push_back(origin);result.sample_ids.push_back(i);}
  }
  if(capture_audit && audit_ok)(void)try_lidar_diagnostic([&]{
    auto captured=std::make_shared<LidarSelfAudit>();
    for(auto& group:audit_groups)for(auto& point:group)captured->points.push_back(std::move(point));
    std::sort(captured->points.begin(),captured->points.end(),[](const auto& a,const auto& b){return a.sample_id<b.sample_id;});
    result.audit=std::move(captured);
  });
  result.source_time_groups=prepared.size();
  std::unordered_set<const dg::PreparedModel*> unique_models;unique_models.reserve(prepared.size());
  for(const auto& item:prepared)if(item.second)unique_models.insert(item.second.get());
  result.unique_prepared_models=unique_models.size();
  // An incomplete model evaluation cannot publish a partially deleted frame.
  if(remove_self_surfaces && result.unobserved_count){
    result.removed_count=0;result.retained_count=count;result.cloud=cloud;
    result.ground_blocked=std::move(original_ground_blocked);
    result.ray_origins=*input.ray_origins;result.sample_ids.resize(count);
    for(std::size_t i=0;i<count;++i)result.sample_ids[i]=i;
    result.reason="unobserved_retained";return result;
  }
  result.retained_count=count-result.removed_count;
  output.height=1;output.width=static_cast<uint32_t>(result.retained_count);
  output.row_step=output.width*output.point_step;
  result.cloud=std::move(output);result.reason="filtered";return result;
}
} // namespace sunhorizon_rgbd_adapter
