#pragma once
#include <cstring>
#include <algorithm>
#include <map>
#include <nlohmann/json.hpp>
#include <nav2_costmap_2d/observation_buffer.hpp>
#include <sunhorizon_rgbd_adapter/pointcloud_self_exclusion_direct.hpp>
#include <sunhorizon_perception_runtime/classified_obstacle_frame.hpp>
namespace sunhorizon_navigation_runtime::detail {
// Called only on the existing asynchronous timing timer. No TF/model lookup, raycast, consumer
// lease operation, mutable Observation or raw buffer pointer is used here.
inline std::string serialize_airy_geometry_audit(
    const sunhorizon_perception_runtime::ground_relative::ClassifiedObstacleFrame& frame,
    const sunhorizon_rgbd_adapter::SelfExclusionDirectOutput& filtered,
    const nav2_costmap_2d::DirectProjectionSnapshot* projection) {
  using Json=nlohmann::json;
  const auto& id=frame.source().identity;const auto& stamp=filtered.input->stamp;
  if(id.source_id!=stamp.source_identity || id.epoch!=stamp.epoch || id.sequence!=stamp.sequence ||
      id.source_ns!=stamp.source_ns)throw std::invalid_argument("airy_audit_producer_frame_identity");
  Json out={{"schema","airy-geometry-audit-v1"},{"purpose","diagnostic_only"},
    {"source_id",id.source_id},{"epoch",id.epoch},{"sequence",id.sequence},{"source_ns",id.source_ns},
    {"source_frame",id.source_frame},{"maximum_points",96},
    {"selection","nearest source-horizontal lower-hemisphere points, max32 each Self/External/Conflict"},
    {"model_link_index_contract","actual PreparedModel query order; matches configured CAD link manifest"},
    {"producer_audit_available",bool(filtered.lidar_audit)},{"consumer",nullptr},{"points",Json::array()}};
  if(!filtered.lidar_audit)out["producer_audit_unavailable_reason"]="capture_failed_or_attempt_budget_exhausted";
  if(projection) {
    const auto& m=projection->metadata;
    if(m.source_id!=id.source_id||m.epoch!=id.epoch||m.sequence!=id.sequence||m.source_ros_ns!=id.source_ns||m.frame_id!=id.source_frame||
        m.first_receipt_ros_ns!=id.first_receipt_ros_ns||m.first_receipt_steady_ns!=id.first_receipt_steady_ns||
        !projection->cloud||!projection->source_cloud||
        std::size_t(projection->cloud->width)*projection->cloud->height!=frame.source().points.size())
      throw std::invalid_argument("airy_audit_consumer_frame_identity");
    const auto& t=projection->source_to_global;const auto& v=t.transform;
    if(t.header.frame_id!=projection->cloud->header.frame_id ||
        t.child_frame_id!=projection->source_cloud->header.frame_id ||
        projection->cloud->header.stamp!=projection->source_cloud->header.stamp)
      throw std::invalid_argument("airy_audit_actual_consumer_transform_frame");
    out["consumer"]={{"accepted_projection",true},{"source_id",m.source_id},{"epoch",m.epoch},
      {"sequence",m.sequence},{"source_ns",m.source_ros_ns},{"first_receipt_ros_ns",m.first_receipt_ros_ns},
      {"first_receipt_steady_ns",m.first_receipt_steady_ns},{"frame",projection->cloud->header.frame_id},
      {"actual_transform",{{"parent",t.header.frame_id},{"child",t.child_frame_id},
        {"stamp_sec",t.header.stamp.sec},{"stamp_nanosec",t.header.stamp.nanosec},
        {"translation",{v.translation.x,v.translation.y,v.translation.z}},
        {"rotation_xyzw",{v.rotation.x,v.rotation.y,v.rotation.z,v.rotation.w}}}}};
  }else out["consumer_unavailable_reason"]="same_frame_accepted_global_projection_not_observed_within_2s";
  std::map<uint64_t,std::size_t> indices;
  for(std::size_t i=0;i<frame.source().points.size();++i)
    if(!indices.emplace(frame.source().points[i].sample_id,i).second)
      throw std::invalid_argument("airy_audit_duplicate_raw_sample_id");
  if(filtered.lidar_audit)for(const auto& p:filtered.lidar_audit->points) {
    if(out["points"].size()>=96)throw std::invalid_argument("airy_audit_capacity");
    Json row={{"raw_sample_id",p.sample_id},{"point_source_ns",p.source_ns},
      {"source_xyz",p.position},{"sdk_ray_origin",p.ray_origin},
      {"cad",{{"classification",int(p.result.classification)},{"complete",p.result.complete},
        {"reason",p.result.reason},{"has_first_surface",p.result.has_first_surface},
        {"first_range_m",p.result.has_first_surface?Json(p.result.first_axial_depth_m):Json(nullptr)},
        {"link_index",p.result.has_first_surface?Json(p.result.link_index):Json(nullptr)},
        {"triangle_index",p.result.has_first_surface?Json(p.result.triangle_index):Json(nullptr)},
        {"node_visits",p.result.node_visits},{"triangle_tests",p.result.triangle_tests}}},
      {"prepared_links",Json::array()},{"classification_index",nullptr},{"consumer_xyz",nullptr}};
    for(const auto& link:p.links)row["prepared_links"].push_back({{"link_index",link.link_index},
      {"triangle_count",link.triangle_count},{"mesh_revision",link.mesh_revision},
      {"source_ns",p.source_ns},{"source_from_link",{{"rotation",link.optical_from_link.rotation},
        {"translation",{link.optical_from_link.translation.x,link.optical_from_link.translation.y,
          link.optical_from_link.translation.z}}}}});
    const auto found=indices.find(p.sample_id);
    if(found!=indices.end()) {
      const auto i=found->second;const auto& original=frame.source().points[i];
      if(p.source_ns<id.source_ns || p.source_ns-id.source_ns!=original.source_offset_ns)
        throw std::invalid_argument("airy_audit_point_source_time_mismatch");
      for(std::size_t axis=0;axis<3;++axis)if(original.position[axis]!=double(p.position[axis]))
        throw std::invalid_argument("airy_audit_original_point_mismatch");
      row["classification_index"]=i;row["point_class"]=int(frame.classes()[i]);
      row["ground_blocked"]=original.ground_blocked;
      if(projection) {
        const auto& cloud=*projection->cloud;std::array<float,3> xyz{};
        for(std::size_t axis=0;axis<3;++axis) {
          const auto name=std::string(1,"xyz"[axis]);const auto f=std::find_if(cloud.fields.begin(),cloud.fields.end(),
            [&](const auto& field){return field.name==name;});
          if(f==cloud.fields.end()||f->offset+4>cloud.point_step||(i/cloud.width)*cloud.row_step+(i%cloud.width)*cloud.point_step+f->offset+4>cloud.data.size())
            throw std::invalid_argument("airy_audit_consumer_layout");
          std::memcpy(&xyz[axis],cloud.data.data()+(i/cloud.width)*cloud.row_step+(i%cloud.width)*cloud.point_step+f->offset,4);
        }
        row["consumer_xyz"]=xyz;
        if(projection->ray_origins) {
          if(i>=projection->ray_origins->size())throw std::invalid_argument("airy_audit_consumer_origins");
          row["consumer_ray_origin"]=(*projection->ray_origins)[i];
        }
      }
    }
    out["points"].push_back(std::move(row));
  }
  return out.dump();
}
}
