#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <functional>
#include <map>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <nav2_util/direct_cloud.hpp>
#include "sunhorizon_navigation_runtime/detail/ground_clearance_packet.hpp"
#include <sunhorizon_perception_runtime/ground_reference_owner.hpp>
#include <sunhorizon_perception_runtime/registration_ground_geometry.hpp>

namespace sunhorizon_navigation_runtime::detail {
struct ClassifiedAiryPacket {
  nav2_util::DirectCloud frame;
  std::size_t input_points{0},finite_points{0},invalid_points{0};
  bool deliverable{false};
  // Whole-frame qualification failure is source unavailability, not a
  // successful all-Uncertain observation. Keep the frame for diagnostics only.
  bool ground_unavailable{false};
  sunhorizon_perception_runtime::ground_relative::ClassifiedObstacleFrame::ConstSharedPtr classification;
};
struct AiryGroundContext {
  sunhorizon_perception_runtime::ground_relative::GroundReferenceOwner::ConstSharedPtr owner;
  sunhorizon_perception_runtime::ground_relative::SourceTimeTransform source_to_reference;
  std::string calibration_id;
  int64_t expiry_budget_ns{0};
  // Original point acquisition time, not scan completion/header time. Missing
  // geometry leaves the measured endpoint Uncertain; no latest-TF fallback.
  std::function<sunhorizon_perception_runtime::ground_relative::SourceTimeTransform(int64_t)>
    source_transform_at;
  sunhorizon_perception_runtime::RegistrationGroundGeometryProvider registration_geometry;
};
inline std::string classified_airy_packet_failure(const nav2_util::DirectCloud& input,
    const char* reason,int64_t finite=-1,int64_t invalid=-1) {
  std::ostringstream out;
  out<<"classified_airy_packet_invalid:"<<reason<<" finite="<<finite<<" invalid="<<invalid
     <<" seq="<<input.metadata.sequence<<" epoch="<<input.metadata.epoch.substr(0,64);
  if(!input.cloud)return out.str()+" cloud=null";
  const auto& c=*input.cloud;
  out<<" width="<<c.width<<" height="<<c.height<<" step="<<c.point_step
     <<" row="<<c.row_step<<" bytes="<<c.data.size()<<" bigendian="<<c.is_bigendian
     <<" source="<<input.metadata.source_ros_ns<<" header="<<c.header.stamp.sec<<":"<<c.header.stamp.nanosec
     <<" frame_match="<<(c.header.frame_id==input.metadata.frame_id);
  for(const auto* name:{"x","y","z"}) {
    out<<" "<<name<<"=";unsigned found=0;
    for(const auto& field:c.fields)if(field.name==name) {
      if(found++<2)out<<field.offset<<":"<<unsigned(field.datatype)<<":"<<field.count<<",";
    }
    out<<"count"<<found;
  }
  return out.str().substr(0,512);
}
// Airy/self-filter retains NaN no-return records. These are not endpoints and
// carry no mark, clear, ray or freshness credit. As in the shared conservative
// classifier, finite nonself returns retain their conservative point classes.
// Whole-frame ground failure is withheld from every consumer; it is neither
// free space nor a new occupied observation. Original metadata is never renewed.
inline ClassifiedAiryPacket make_classified_airy_packet(const nav2_util::DirectCloud& input,
    const AiryGroundContext& ground = {},
    const std::shared_ptr<const std::vector<uint8_t>>& ground_blocked = {},
    const std::shared_ptr<const std::vector<uint64_t>>& sample_ids = {}) {
  // Native no-return records have NaN XYZ AND no physical ray origin. Validate
  // the cloud layout first, then compact these records and their side arrays
  // together before enforcing the strict finite consumer-origin contract.
  auto layout = input;
  layout.ray_origins.reset();
  if(input.kind!=nav2_util::DirectCloudKind::RetainedIndependentSource ||
      !nav2_util::directCloudMatchesMetadata(layout) ||
      (input.ray_origins && input.ray_origins->size()!=
        std::size_t(input.cloud->width)*input.cloud->height))
    throw std::invalid_argument(classified_airy_packet_failure(input,"layout_or_metadata"));
  ClassifiedAiryPacket result;
  const auto& cloud=*input.cloud;
  result.input_points=std::size_t(cloud.width)*cloud.height;
  if(ground_blocked && (ground_blocked->size()!=result.input_points ||
      std::any_of(ground_blocked->begin(),ground_blocked->end(),[](auto value){return value>1;})))
    throw std::invalid_argument("classified_airy_ground_block_mask_layout");
  if(sample_ids && sample_ids->size()!=result.input_points)
    throw std::invalid_argument("classified_airy_sample_id_layout");
  std::vector<uint8_t> finite_ground_blocked;finite_ground_blocked.reserve(result.input_points);
  std::vector<uint64_t> finite_sample_ids;finite_sample_ids.reserve(result.input_points);
  std::array<std::size_t,3> offsets{};
  for(std::size_t axis=0;axis<3;++axis)
    for(const auto& field:cloud.fields)
      if(field.name==std::string(1,"xyz"[axis]))offsets[axis]=field.offset;
  const auto finite=[&](std::size_t i) {
    for(const auto offset:offsets) {
      float value;std::memcpy(&value,cloud.data.data()+i*cloud.point_step+offset,sizeof(value));
      if(!std::isfinite(value))return false;
    }
    return true;
  };
  for(std::size_t i=0;i<result.input_points;++i)if(finite(i)){
    if(input.ray_origins && std::any_of((*input.ray_origins)[i].begin(),
        (*input.ray_origins)[i].end(),[](float value){return !std::isfinite(value);}))
      throw std::invalid_argument(classified_airy_packet_failure(input,"finite_return_origin"));
    ++result.finite_points;
    finite_ground_blocked.push_back(ground_blocked?(*ground_blocked)[i]:1);
    finite_sample_ids.push_back(sample_ids?(*sample_ids)[i]:i);
  }
  result.invalid_points=result.input_points-result.finite_points;
  // In particular, an all-invalid scan is not a complete empty Ground frame.
  // The caller must not admit it or refresh an older consumer observation.
  if(!result.finite_points)return result;
  result.frame=input;
  if(result.invalid_points) {
    auto compact=std::make_shared<sensor_msgs::msg::PointCloud2>();
    compact->header=cloud.header;compact->fields=cloud.fields;
    compact->height=1;compact->width=static_cast<uint32_t>(result.finite_points);
    compact->point_step=cloud.point_step;compact->row_step=compact->width*compact->point_step;
    compact->is_bigendian=cloud.is_bigendian;compact->is_dense=true;
    compact->data.reserve(compact->row_step);
    for(std::size_t i=0;i<result.input_points;++i)if(finite(i)) {
      const auto* begin=cloud.data.data()+i*cloud.point_step;
      compact->data.insert(compact->data.end(),begin,begin+cloud.point_step);
    }
    result.frame.cloud=std::move(compact);
  }
  if(input.ray_origins && result.invalid_points) {
    auto origins=std::make_shared<nav2_util::DirectRayOrigins>();origins->reserve(result.finite_points);
    for(std::size_t i=0;i<result.input_points;++i)if(finite(i))origins->push_back((*input.ray_origins)[i]);
    result.frame.ray_origins=std::move(origins);
  }
  result.frame.kind=nav2_util::DirectCloudKind::ClassifiedObstacleFrame;
  result.frame.classes=std::make_shared<const std::vector<uint8_t>>(
    result.finite_points,uint8_t(nav2_util::DirectPointClass::Uncertain));
  result.frame.complete=true;result.frame.ground_qualified=false;
  if(ground.owner) {
    namespace gr=sunhorizon_perception_runtime::ground_relative;
    gr::SourceFrame source;
    // Endpoint segmentation is not a physical sensor/registration error bound.
    source.point_error_bound_m=gr::kUnspecified;
    source.transform_translation_error_bound_m=gr::kUnspecified;
    source.transform_angle_error_bound_rad=gr::kUnspecified;
    const auto& id=input.metadata;
    source.identity={id.source_id,id.epoch,ground.calibration_id,id.frame_id,
      ground.owner->reference_frame(),id.sequence,id.source_ros_ns,id.first_receipt_ros_ns,
      id.first_receipt_steady_ns,ground.expiry_budget_ns,id.raw_source_ns,
      id.normalization_start_steady_ns,id.normalization_end_steady_ns,id.normalization_thread_cpu_ns};
    source.to_common=ground.source_to_reference;
    std::optional<std::size_t> time_offset;
    for(const auto& field:cloud.fields)if(field.name=="timestamp" &&
        field.datatype==sensor_msgs::msg::PointField::FLOAT32 && field.count==1 &&
        field.offset<=cloud.point_step && cloud.point_step-field.offset>=sizeof(float)) {
      if(time_offset)throw std::invalid_argument("classified_airy_duplicate_point_time");
      time_offset=field.offset;
    }
    std::map<int64_t,gr::SourceTimeTransform> transforms;
    std::vector<int64_t> query_times{id.source_ros_ns};
    query_times.reserve(result.finite_points+1);
    // No header-origin substitute for an omnidirectional rotating LiDAR.
    // Missing real origins retains endpoints as Uncertain, never Ground rays.
    bool all_point_times_valid=bool(input.ray_origins);
    source.points.reserve(result.finite_points);
    for(std::size_t i=0;i<result.finite_points;++i) {
      gr::PointReturn point;point.kind=gr::ReturnKind::Measured;
      point.ground_blocked=finite_ground_blocked[i];
      point.sample_id=finite_sample_ids[i];
      for(std::size_t axis=0;axis<3;++axis) {
        float value;std::memcpy(&value,result.frame.cloud->data.data()+i*cloud.point_step+offsets[axis],4);
        point.position[axis]=value;
      }
      float seconds=std::numeric_limits<float>::quiet_NaN();
      if(time_offset)std::memcpy(&seconds,result.frame.cloud->data.data()+i*cloud.point_step+*time_offset,4);
      if(!std::isfinite(seconds) || seconds<0.f || seconds>.2f) {
        // Retain the original finite return for diagnostics. A malformed/absent
        // acquisition time cannot qualify this frame for consumer delivery.
        source.to_common.qualified=false;
        all_point_times_valid=false;
      } else {
        point.source_offset_ns=std::llround(double(seconds)*1.e9);
        if(point.source_offset_ns>0) {
          if(id.source_ros_ns>std::numeric_limits<int64_t>::max()-point.source_offset_ns)
            throw std::invalid_argument("classified_airy_point_time_overflow");
          const auto point_ns=id.source_ros_ns+point.source_offset_ns;
          // Preserve the original exact stamps. Native scan groups are already
          // ordered; adjacent repetitions need no tree insertion per return.
          if(query_times.back()!=point_ns)query_times.push_back(point_ns);
          if(ground.source_transform_at && !transforms.count(point_ns))
            transforms.emplace(point_ns,ground.source_transform_at(point_ns));
        }
      }
      source.points.push_back(point);
    }
    gr::RegistrationMapSourceBinding map_binding;
    if(ground.registration_geometry) {
      if(!std::is_sorted(query_times.begin(),query_times.end()))
        std::sort(query_times.begin(),query_times.end());
      query_times.erase(std::unique(query_times.begin(),query_times.end()),query_times.end());
      auto geometry=ground.registration_geometry(source.identity,query_times,true);
      source.transform_diagnostic=std::move(geometry.diagnostic);
      if(geometry.transforms.size()==query_times.size()) {
        source.to_common=geometry.transforms.front();
        source.to_common.qualified=source.to_common.qualified&&all_point_times_valid;
        source.source_time_transforms=std::move(geometry.transforms);
        map_binding=std::move(geometry.map_binding);
      }else source.to_common.qualified=false;
    }else for(auto& item:transforms)source.source_time_transforms.push_back(std::move(item.second));
    source.to_common.qualified=source.to_common.qualified&&all_point_times_valid;
    result.classification=ground.owner->classify(std::move(source),std::move(map_binding));
    auto classes=std::make_shared<std::vector<uint8_t>>();classes->reserve(result.finite_points);
    for(const auto point:result.classification->classes()) {
      if(point!=gr::PointClass::Ground && point!=gr::PointClass::Obstacle && point!=gr::PointClass::Uncertain)
        throw std::runtime_error("classified_airy_unmeasured_output");
      classes->push_back(static_cast<uint8_t>(point));
    }
    result.frame.classes=std::move(classes);
    result.frame.ground_qualified=result.classification->ground().safety_qualified;
    result.frame.ground_basis=direct_ground_basis(result.classification->ground());
    attach_ground_validity(result.frame,result.classification);
  }
  if(!nav2_util::directCloudMatchesMetadata(result.frame))
    throw std::invalid_argument(classified_airy_packet_failure(input,"compacted_packet",
      result.finite_points,result.invalid_points));
  // A source-level geometry failure does not establish that every endpoint is
  // an obstacle. Do not feed mark/clear or renew any downstream source lease.
  // Within a qualified frame, only Obstacle supplies obstacle endpoints;
  // Uncertain retains its label and supplies no clearing credit.
  result.ground_unavailable=!result.frame.ground_qualified;
  result.deliverable=!result.ground_unavailable;return result;
}
} // namespace sunhorizon_navigation_runtime::detail
