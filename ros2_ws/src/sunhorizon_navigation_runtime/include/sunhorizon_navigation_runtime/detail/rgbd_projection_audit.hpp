#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <memory>
#include <unordered_set>
#include <nlohmann/json.hpp>
#include <nav2_costmap_2d/observation_buffer.hpp>
#include <sunhorizon_perception_runtime/ground_classification_snapshot.hpp>
#include <sunhorizon_perception_runtime/perception_engine.hpp>

namespace sunhorizon_navigation_runtime::detail {
namespace gr=sunhorizon_perception_runtime::ground_relative;
enum class RgbdProjectionConsumer {Global,Local};
struct RgbdProjectionAuditFrame {
  nav2_util::DirectObservationMetadata metadata;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr source_cloud;
  gr::ClassifiedObstacleFrame::ConstSharedPtr classification;
  std::shared_ptr<const nav2_costmap_2d::DirectProjectionSnapshot> projection;
  int64_t selected_steady_ns{0};
  std::shared_ptr<const sunhorizon_perception_runtime::BodyModelDiagnostic> body_model;
  std::string request_id;
  RgbdProjectionConsumer consumer{RgbdProjectionConsumer::Global};
  std::shared_ptr<const std::vector<std::size_t>> near_body_indices;

};
inline bool same_projection_source_owner(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& a,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& b) {
  return a && b && a==b && !a.owner_before(b) && !b.owner_before(a);
}
// External timing_mutex protects the four per-consumer/per-role bounded slots. Disabled
// capture allocates/copies nothing; every method is diagnostic-only/noexcept.
class RgbdProjectionCapture {
public:
  explicit RgbdProjectionCapture(bool enabled=false,
      RgbdProjectionConsumer consumer=RgbdProjectionConsumer::Global):enabled_(enabled),consumer_(consumer) {}
  // At most four explicit late captures. An accepted source owner cannot be
  // replaced by a request; the original matching/timeout must finish first.
  bool rearm(const std::string& request) noexcept {
    if(!enabled_||pending_||(!done_&&!exhausted())||request.empty()||request.size()>64||
        request_count_==requests_.size())return false;
    for(char c:request)if(!((c>='a'&&c<='z')||(c>='A'&&c<='Z')||
        (c>='0'&&c<='9')||c=='_'||c=='-'))return false;
    for(unsigned i=0;i<request_count_;++i)if(requests_[i]==request)return false;
    try {requests_[request_count_]=request;request_id_=request;}
    catch(...) {return false;}
    ++request_count_;done_=false;attempts_=0;return true;
  }
  bool offer(const nav2_util::DirectObservationMetadata& metadata,
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& source,
      const gr::ClassifiedObstacleFrame::ConstSharedPtr& classification,int64_t now,
      std::shared_ptr<const sunhorizon_perception_runtime::BodyModelDiagnostic> body_model={},
      std::shared_ptr<const std::vector<std::size_t>> near_body_indices={}) noexcept {
    if(!enabled_||done_||pending_||attempts_>=8||!source||!classification||
        !classification->classification_complete()||!classification->ground().safety_qualified||
        classification->ground().result!=gr::Qualification::Qualified)return false;
    ++attempts_;
    try {pending_=std::make_unique<RgbdProjectionAuditFrame>(RgbdProjectionAuditFrame{
      metadata,source,classification,{},now,std::move(body_model),request_id_,consumer_,std::move(near_body_indices)});return true;} catch(...) {return false;}
  }
  bool wants(const nav2_util::DirectObservationMetadata& metadata,
      RgbdProjectionConsumer consumer=RgbdProjectionConsumer::Global)const noexcept {
    return consumer==consumer_&&enabled_&&pending_&&!pending_->projection&&nav2_util::directMetadataEqual(pending_->metadata,metadata);
  }
  bool receive(const nav2_costmap_2d::DirectProjectionSnapshot& projection,
      RgbdProjectionConsumer consumer=RgbdProjectionConsumer::Global) noexcept {
    if(!wants(projection.metadata,consumer)||!same_projection_source_owner(pending_->source_cloud,projection.source_cloud))return false;
    try {pending_->projection=std::make_shared<const nav2_costmap_2d::DirectProjectionSnapshot>(projection);return true;}
    catch(...) {return false;}
  }
  std::unique_ptr<RgbdProjectionAuditFrame> take(int64_t now) noexcept {
    if(!pending_)return {};
    if(pending_->projection){done_=true;return std::move(pending_);}
    if(now>=pending_->selected_steady_ns && now-pending_->selected_steady_ns>=2000000000LL)pending_.reset();
    return {}; // An unaccepted frame is never serialized as a matching pair.
  }
  unsigned attempts()const noexcept{return attempts_;}
  bool done()const noexcept{return done_;}
  bool exhausted()const noexcept{return enabled_&&!done_&&!pending_&&attempts_>=8;}
private:
  bool enabled_{false},done_{false};unsigned attempts_{0};
  RgbdProjectionConsumer consumer_{RgbdProjectionConsumer::Global};
  std::array<std::string,4> requests_;
  unsigned request_count_{0};std::string request_id_;
  std::unique_ptr<RgbdProjectionAuditFrame> pending_;
};
inline std::array<std::size_t,3> rgbd_audit_layout(const sensor_msgs::msg::PointCloud2& c) {
  if(!c.width||!c.height||c.point_step<12||c.row_step<std::size_t(c.width)*c.point_step||
      c.data.size()!=std::size_t(c.row_step)*c.height)throw std::invalid_argument("rgbd_audit_cloud_layout");
  std::array<std::size_t,3> offsets{};
  for(std::size_t axis=0;axis<3;++axis){unsigned count=0;
    for(const auto& field:c.fields)if(field.name==std::string(1,"xyz"[axis])){
      ++count;if(field.datatype!=sensor_msgs::msg::PointField::FLOAT32||field.count!=1||
          std::size_t(field.offset)+4>c.point_step)throw std::invalid_argument("rgbd_audit_xyz_field");
      offsets[axis]=field.offset;
    }
    if(count!=1)throw std::invalid_argument("rgbd_audit_xyz_count");
  }
  for(std::size_t i=0;i<3;++i)for(std::size_t j=i+1;j<3;++j)
    if(offsets[i]<offsets[j]+4&&offsets[j]<offsets[i]+4)throw std::invalid_argument("rgbd_audit_xyz_overlap");
  return offsets;
}
inline std::array<float,3> rgbd_audit_xyz(const sensor_msgs::msg::PointCloud2& c,
    const std::array<std::size_t,3>& offsets,std::size_t i) {
  std::array<float,3> out{};const auto* p=c.data.data()+(i/c.width)*c.row_step+(i%c.width)*c.point_step;
  for(std::size_t axis=0;axis<3;++axis){std::array<uint8_t,4> bytes{};std::memcpy(bytes.data(),p+offsets[axis],4);
    const uint16_t endian_test=1;const bool native_big=*reinterpret_cast<const uint8_t*>(&endian_test)==0;
    if(c.is_bigendian!=native_big)std::reverse(bytes.begin(),bytes.end());
    std::memcpy(&out[axis],bytes.data(),4);if(!std::isfinite(out[axis]))throw std::invalid_argument("rgbd_audit_nonfinite_xyz");
  }return out;
}
inline uint8_t rgbd_audit_direct_class(gr::PointClass point,
    const gr::SourceFrame& source,std::size_t index,
    gr::PointDecision decision=gr::PointDecision::NotRecorded) {
  switch(point){case gr::PointClass::Ground:return uint8_t(nav2_util::DirectPointClass::Ground);
    case gr::PointClass::Obstacle:return uint8_t(nav2_util::DirectPointClass::Obstacle);
    case gr::PointClass::Uncertain:return uint8_t(nav2_util::DirectPointClass::Uncertain);
    case gr::PointClass::Self:
    case gr::PointClass::Masked:
      // Native model edges are Self, with their original mixed-depth record
      // retained. Legacy Masked edge records remain readable. Neither is a
      // physical body hit: reject the endpoint, without Self denominator or
      // free-ray credit. Ordinary Self and arbitrary Masked cannot borrow this.
      if(decision==gr::PointDecision::ModelProjectionEdge && index<source.points.size() &&
          ((point==gr::PointClass::Self && source.points[index].kind==gr::ReturnKind::Self) ||
           (point==gr::PointClass::Masked && source.points[index].kind==gr::ReturnKind::Masked)) &&
          source.points[index].ground_blocked &&
          !source.points[index].model_conflict && !source.points[index].measurement_unreliable)
        return uint8_t(nav2_util::DirectPointClass::Rejected);
      throw std::invalid_argument("rgbd_self_or_masked_without_model_edge_source");
    case gr::PointClass::Invalid:
      if(gr::measurement_rejection_proven(source,index)||
          (decision==gr::PointDecision::MeasurementBodyOccluded&&gr::body_occlusion_rejection_proven(source,index)))
        return uint8_t(nav2_util::DirectPointClass::Rejected);
      throw std::invalid_argument("rgbd_audit_unproven_measurement_rejection");
    default:throw std::invalid_argument("rgbd_audit_unmeasured_point_class");}
}
// Only the asynchronous timing timer serializes. There is no TF/model lookup,
// mutable ObservationBuffer, raycast, lease refresh or costmap modification here.
inline std::string serialize_rgbd_projection_audit(const RgbdProjectionAuditFrame& captured,
    const gr::GroundReferenceOwner* owner=nullptr,bool require_qualified_ground=true) {
  using Json=nlohmann::json;
  if(!captured.classification||!captured.projection)throw std::invalid_argument("rgbd_audit_missing_accepted_pair");
  const auto& frame=*captured.classification;const auto& id=frame.source().identity;
  const auto& p=*captured.projection;const auto& m=p.metadata;
  if(!nav2_util::directMetadataEqual(captured.metadata,m)||
      !same_projection_source_owner(captured.source_cloud,p.source_cloud)||!p.cloud||!p.classes||
      id.source_id!=m.source_id||id.epoch!=m.epoch||id.sequence!=m.sequence||id.source_ns!=m.source_ros_ns||
      id.source_frame!=m.frame_id||id.first_receipt_ros_ns!=m.first_receipt_ros_ns||
      id.first_receipt_steady_ns!=m.first_receipt_steady_ns||(require_qualified_ground&&!frame.ground().safety_qualified))
    throw std::invalid_argument("rgbd_audit_pair_identity");
  const auto& source=*p.source_cloud;const auto& cloud=*p.cloud;const auto n=frame.source().points.size();
  if(n>307200||n!=frame.classes().size()||n!=p.classes->size()||n!=std::size_t(source.width)*source.height||
      source.width!=cloud.width||source.height!=cloud.height||source.fields!=cloud.fields||
      source.point_step!=cloud.point_step||source.row_step!=cloud.row_step||source.is_bigendian!=cloud.is_bigendian||
      source.header.frame_id!=id.source_frame||source.header.stamp!=cloud.header.stamp||
      int64_t(source.header.stamp.sec)*1000000000LL+source.header.stamp.nanosec!=id.source_ns)
    throw std::invalid_argument("rgbd_audit_point_layout_or_stamp");
  const auto source_offsets=rgbd_audit_layout(source),global_offsets=rgbd_audit_layout(cloud);
  const auto& t=p.source_to_global;const auto& v=t.transform;
  if(t.header.frame_id!=cloud.header.frame_id||t.child_frame_id!=source.header.frame_id)
    throw std::invalid_argument("rgbd_audit_consumer_transform_frames");
  if(captured.consumer!=RgbdProjectionConsumer::Global&&captured.consumer!=RgbdProjectionConsumer::Local)
    throw std::invalid_argument("rgbd_audit_consumer_identity");
  const bool local=captured.consumer==RgbdProjectionConsumer::Local;
  Json out={{"schema",local?"rgbd-local-projection-audit-v1":"rgbd-global-projection-audit-v1"},{"purpose","diagnostic_only"}};
  out["capture_request_id"]=captured.request_id.empty()?"startup":captured.request_id;
  out["consumer"]={{"stage",local?"local.observation_buffer_committed":"global.observation_buffer_committed"},{"accepted_projection",true},
    {"not_final_voxel_cost_evidence",true},{"source_id",m.source_id},{"epoch",m.epoch},
    {"sequence",m.sequence},{"source_ns",m.source_ros_ns},{"source_frame",source.header.frame_id},
    {local?"local_frame":"global_frame",cloud.header.frame_id},{"first_receipt_ros_ns",m.first_receipt_ros_ns},
    {"first_receipt_steady_ns",m.first_receipt_steady_ns}};
  out["consumer"]["actual_header_transform"]={{"parent",t.header.frame_id},{"child",t.child_frame_id},
    {"stamp_sec",t.header.stamp.sec},{"stamp_nanosec",t.header.stamp.nanosec},
    {"translation",{v.translation.x,v.translation.y,v.translation.z}},
    {"rotation_xyzw",{v.rotation.x,v.rotation.y,v.rotation.z,v.rotation.w}}};
  if(captured.near_body_indices) out["near_body_excluded_classification_indices"]=*captured.near_body_indices;
  out["body_model"]=nullptr;
  if(captured.body_model) {
    const auto& model=*captured.body_model;const auto& m=model.identity;
    if(model.source_frame!=id.source_frame||m.source_id!=id.source_id||m.epoch!=id.epoch||m.calibration_id!=id.calibration_id||
        m.sequence!=id.sequence||m.source_stamp_ros_ns!=id.source_ns||
        m.receipt_ros_ns!=id.first_receipt_ros_ns||m.receipt_steady_ns!=id.first_receipt_steady_ns||model.links.size()!=12||
        !std::isfinite(model.self_tolerance_m)||model.self_tolerance_m<0.)
      throw std::invalid_argument("rgbd_audit_body_model_identity");
    Json links=Json::array();
    for(std::size_t i=0;i<model.links.size();++i) {
      const auto& link=model.links[i];
      if(link.link_index!=i||!link.triangle_count||!link.mesh_revision)
        throw std::invalid_argument("rgbd_audit_body_model_link_identity");
      links.push_back({{"link_index",i},{"triangle_count",link.triangle_count},{"mesh_revision",link.mesh_revision},
        {"boundary_triangle_begin",link.boundary_triangle_begin},{"boundary_triangle_count",link.boundary_triangle_count},
        {"boundary_tolerance_m",link.boundary_tolerance_m},
        {"measured_triangle_begin",link.measured_triangle_begin},{"measured_triangle_count",link.measured_triangle_count},
        {"optical_from_link",{{"rotation",link.optical_from_link.rotation},
          {"translation",{link.optical_from_link.translation.x,link.optical_from_link.translation.y,link.optical_from_link.translation.z}}}}});
    }
    out["body_model"]={{"source_id",m.source_id},{"role",m.source_id},{"source_frame",model.source_frame},{"epoch",m.epoch},
      {"calibration_id",m.calibration_id},{"sequence",m.sequence},{"source_ns",m.source_stamp_ros_ns},
      {"first_receipt_ros_ns",m.receipt_ros_ns},{"first_receipt_steady_ns",m.receipt_steady_ns},
      {"self_tolerance_m",model.self_tolerance_m},{"query_budget",{{"max_links",model.query_budget.max_links},
        {"max_node_visits",model.query_budget.max_node_visits},{"max_triangle_tests",model.query_budget.max_triangle_tests}}},
      {"model_link_index_contract","configured twelve-link mesh manifest order"},{"links",std::move(links)}};
  }else out["body_model_unavailable_reason"]="diagnostic_disabled_or_capture_allocation_failed";
  out["classification"]=Json::parse(sunhorizon_perception_runtime::detail::serialize_ground_classification_snapshot(frame,owner));
  out["consumer_point_columns"]={"classification_index","raw_sample_id","source_x","source_y","source_z",local?"local_x":"global_x",local?"local_y":"global_y",local?"local_z":"global_z","direct_class","ground_blocked"};
  out["consumer_points"]=Json::array();
  std::unordered_set<uint64_t> ids;ids.reserve(n);
  for(std::size_t i=0;i<n;++i){const auto& q=frame.source().points[i];
    if(!ids.insert(q.sample_id).second)throw std::invalid_argument("rgbd_audit_duplicate_sample_id");
    const bool excluded=captured.near_body_indices && std::binary_search(
      captured.near_body_indices->begin(),captured.near_body_indices->end(),i);
    const auto expected=excluded ? uint8_t(nav2_util::DirectPointClass::Rejected) :
      rgbd_audit_direct_class(frame.classes()[i],frame.source(),i,frame.decisions().at(i));
    if((*p.classes)[i]!=expected)throw std::invalid_argument("rgbd_audit_consumer_classes");
    const auto a=rgbd_audit_xyz(source,source_offsets,i),b=rgbd_audit_xyz(cloud,global_offsets,i);
    for(std::size_t axis=0;axis<3;++axis)if(double(a[axis])!=q.position[axis])throw std::invalid_argument("rgbd_audit_source_point_identity");
    out["consumer_points"].push_back({i,q.sample_id,a[0],a[1],a[2],b[0],b[1],b[2],(*p.classes)[i],q.ground_blocked});
  }return out.dump();
}
} // namespace sunhorizon_navigation_runtime::detail
