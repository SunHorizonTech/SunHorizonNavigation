#include "sunhorizon_perception_runtime/ground_classification_snapshot.hpp"
#include <nlohmann/json.hpp>
#include "sunhorizon_perception_runtime/runtime.hpp"
#include "sunhorizon_perception_runtime/runtime_config.hpp"
#include <cstring>
#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <system_error>
#include <fcntl.h>
#include <unistd.h>
#include <openssl/sha.h>
#include <sstream>
#include <iomanip>

namespace sunhorizon_perception_runtime::detail {
namespace {
using Json=nlohmann::json;
namespace gr=ground_relative;
Json transform(const gr::SourceTimeTransform& t) {
  return {{"rotation",t.rotation},{"translation",t.translation},{"source_frame",t.source_frame},
    {"common_frame",t.common_frame},{"source_ns",t.source_ns},{"common_ns",t.common_ns},{"qualified",t.qualified}};
}
Json map_identity(const gr::RegistrationMapIdentity& m) {
  return {{"pcd_sha256",m.pcd_sha256},{"datum_id",m.datum_id}};
}
}
std::string ground_snapshot_directory_from_environment() {
  const auto* value=std::getenv("SUNHORIZON_GROUND_SNAPSHOT_DIR");
  return value?value:"";
}
std::string serialize_ground_classification_snapshot(
    const gr::ClassifiedObstacleFrame& frame,const gr::GroundReferenceOwner* owner) {
  const auto& source=frame.source();const auto& id=source.identity;const auto& g=frame.ground();
  Json j={{"schema","ground-classification-snapshot-v1"},{"purpose","diagnostic_only"},
    {"nonfinite_numbers","null"},{"point_columns",{"x","y","z","return_kind","ground_blocked","source_offset_ns","sample_id","model_conflict","measurement_unreliable","measurement_body_occluded"}},
    {"return_kinds",{"Measured","Invalid","Self","Masked"}},
    {"point_classes",{"Ground","Obstacle","Uncertain","Invalid","Self","Masked"}}};
  j["source"]["identity"]={{"source_id",id.source_id},{"epoch",id.epoch},{"calibration_id",id.calibration_id},
    {"source_frame",id.source_frame},{"common_frame",id.common_frame},{"sequence",id.sequence},
    {"source_ns",id.source_ns},{"first_receipt_ros_ns",id.first_receipt_ros_ns},
    {"first_receipt_steady_ns",id.first_receipt_steady_ns},{"expiry_budget_ns",id.expiry_budget_ns},
    {"raw_source_ns",id.raw_source_ns},{"normalization_start_steady_ns",id.normalization_start_steady_ns},
    {"normalization_end_steady_ns",id.normalization_end_steady_ns},{"normalization_thread_cpu_ns",id.normalization_thread_cpu_ns}};
  auto& s=j["source"];s["to_common"]=transform(source.to_common);
  s["self_support_complete"]=source.self_support_complete;
  s["original_projected_samples"]=source.original_projected_samples;
  s["self_support_quality_rejected_samples"]=source.self_support_quality_rejected_samples;
  s["self_support_columns"]={"sample_id","x","y","z"};s["self_support"]=Json::array();
  for(const auto& p:source.self_support)s["self_support"].push_back({p.sample_id,p.position[0],p.position[1],p.position[2]});
  s["raster_geometry"]={{"width",source.raster_geometry.width},{"height",source.raster_geometry.height},{"stride",source.raster_geometry.stride}};
  s["transform_diagnostic"]={{"provider_state",source.transform_diagnostic.provider_state},
    {"reason",source.transform_diagnostic.reason}};
  s["source_time_transforms"]=Json::array();for(const auto& t:source.source_time_transforms)s["source_time_transforms"].push_back(transform(t));
  s["measurement_qualified"]=source.measurement_qualified;s["point_error_bound_m"]=source.point_error_bound_m;
  s["transform_translation_error_bound_m"]=source.transform_translation_error_bound_m;
  s["transform_angle_error_bound_rad"]=source.transform_angle_error_bound_rad;
  s["points"]=Json::array();for(const auto& p:source.points)s["points"].push_back(
    Json::array({p.position[0],p.position[1],p.position[2],int(p.kind),p.ground_blocked,p.source_offset_ns,p.sample_id,p.model_conflict,p.measurement_unreliable,p.measurement_body_occluded}));
  s["body_occlusion"]=nullptr;
  if(source.body_occlusion.complete){const auto& proof=source.body_occlusion;const auto& p=proof.identity;
    s["body_occlusion"]={{"complete",proof.complete},{"same_sdk_geometry_verified",proof.same_sdk_geometry_verified},
      {"width",proof.width},{"height",proof.height},{"baseline_m",proof.baseline_m},{"sample_ids",proof.sample_ids},
      {"identity",{{"source_id",p.source_id},{"epoch",p.epoch},{"calibration_id",p.calibration_id},
        {"source_frame",p.source_frame},{"common_frame",p.common_frame},{"sequence",p.sequence},{"source_ns",p.source_ns},
        {"first_receipt_ros_ns",p.first_receipt_ros_ns},{"first_receipt_steady_ns",p.first_receipt_steady_ns},
        {"expiry_budget_ns",p.expiry_budget_ns},{"raw_source_ns",p.raw_source_ns},
        {"normalization_start_steady_ns",p.normalization_start_steady_ns},{"normalization_end_steady_ns",p.normalization_end_steady_ns},
        {"normalization_thread_cpu_ns",p.normalization_thread_cpu_ns}}}};
  }
  s["measurement_quality"]=nullptr;
  if(source.measurement_quality.present){const auto& q=source.measurement_quality;
    const bool counts_valid=gr::measurement_rejection_counts_valid(q);
    const Json direct_count=counts_valid?Json(q.unreliable_sample_ids.size()-q.component_sample_ids.size()-q.stereo_mismatch_sample_ids.size()):Json(nullptr);
    s["measurement_quality"]={{"schema","depth-measurement-quality-v1"},{"algorithm",q.algorithm},
      {"exact_rectified_pair",q.exact_rectified_pair},{"role",q.role},{"source_frame",q.source_frame},
      {"source_epoch",q.source_epoch},{"calibration_id",q.calibration_id},{"opened_serial",q.opened_serial},
      {"capture_epoch",q.capture_epoch},{"reason",q.reason},{"source_ns",q.source_ns},{"sequence",q.sequence},
      {"width",q.width},{"height",q.height},{"device_us",q.device_us},{"global_us",q.global_us},
      {"frame_indices",q.frame_indices},{"depth_pixel_type",q.depth_pixel_type},{"sdk_depth_scale_m",q.sdk_depth_scale_m},
      {"stereo_mismatch_sample_ids",q.stereo_mismatch_sample_ids},{"stereo_match_complete",q.stereo_match_complete},
      {"stereo_match_reason",q.stereo_match_reason},{"stereo_match_checks",q.stereo_match_checks},{"stereo_mismatch_pixels",q.stereo_mismatch_sample_ids.size()},
      {"component_sample_ids",q.component_sample_ids},{"component_extension_complete",q.component_extension_complete},
      {"component_extension_reason",q.component_extension_reason},{"component_checks",q.component_checks},
      {"rejection_counts_valid",counts_valid},{"rejection_counts_error",counts_valid?Json(nullptr):Json("invalid_rejection_partition")},
      {"direct_saturated_pixels",direct_count},{"component_rejected_pixels",q.component_sample_ids.size()},
      {"unassessed_pixels",q.unassessed_pixels},{"unreliable_sample_ids",q.unreliable_sample_ids}};
  }
  j["ground"]={{"basis",int(g.basis)},{"result",int(g.result)},{"reference_geometry_valid",g.reference_geometry_valid},
    {"support_valid",g.support_valid},{"safety_qualified",g.safety_qualified},{"observation_only",g.observation_only},
    {"candidate_ground_points",g.candidate_ground_points},{"candidate_obstacle_points",g.candidate_obstacle_points},
    {"candidate_uncertain_points",g.candidate_uncertain_points},{"measured_points",g.measured_points},
    {"support_checks",g.support_checks},{"provider_state",g.transform_diagnostic.provider_state},
    {"transform_reason",g.transform_diagnostic.reason}};
  const auto& fit=g.measured_plane_fit;
  j["ground"]["measured_plane_fit"]={{"result",int(fit.result)},
    {"unit_normal",fit.unit_normal},{"anchor",fit.anchor},
    {"training_points",fit.training_points},{"holdout_points",fit.holdout_points},
    {"training_cells",fit.training_cells},{"retained_training_cells",fit.retained_training_cells},
    {"training_span_x_m",fit.training_span_x_m},{"training_span_y_m",fit.training_span_y_m},
    {"training_double_area_m2",fit.training_double_area_m2},
    {"holdout_cell_median_m",fit.holdout_cell_median_m},{"holdout_cell_mad_m",fit.holdout_cell_mad_m}};
  auto& cells=j["ground"]["cells"];cells=Json::array();
  for(const auto& c:g.cells)cells.push_back({{"x",c.cell.x},{"y",c.cell.y},{"holdout",c.holdout},
    {"passed",c.passed},{"failure",int(c.failure)},{"samples",c.samples},{"mean_measured_xy",c.mean_measured_xy},
    {"minimum_signed_distance_m",c.minimum_signed_distance_m},{"maximum_signed_distance_m",c.maximum_signed_distance_m},
    {"minimum_lower_bound_m",c.minimum_lower_bound_m},{"maximum_upper_bound_m",c.maximum_upper_bound_m},
    {"training_samples",c.training_samples},{"holdout_samples",c.holdout_samples},
    {"training_median_residual_m",c.training_median_residual_m},{"holdout_median_residual_m",c.holdout_median_residual_m},
    {"training_mad_m",c.training_mad_m},{"holdout_mad_m",c.holdout_mad_m}});
  j["decision_schema"]="measured-support-branch-v3";
  j["decision_names"]={"NotRecorded","NonMeasured","MeasurementUnreliable","ModelBlocked",
    "Unsupported","BelowGroundBand","SeedOutsideBand","GroundContinuous","GroundReconciled",
    "ObstacleContinuous","ObstacleAnchored","SaturationDominatedComponent","StereoMismatch",
    "ModelProjectionEdge","MeasurementBodyOccluded"};
  j["decisions"]=frame.decisions();
  // decisions[i], classes[i], source.points[i] and sample_id share this frame's
  // source identity. NotRecorded explicitly covers global failures/other modes.
  j["classes"]=frame.classes();j["relative_heights_m"]=frame.relative_heights_m();j["error_bounds_m"]=frame.error_bounds_m();
  j["seed_relative_heights_m"]=frame.seed_relative_heights_m();
  j["marking_indices"]=frame.marking_indices();j["clearing_indices"]=frame.clearing_indices();
  j["protected_hit_indices"]=frame.protected_hit_indices();
  j["reference"]=nullptr;
  if(frame.reference()) {
    const auto& r=*frame.reference();auto& ref=j["reference"];
    ref={{"lifetime",int(r.lifetime)},{"map_asset",map_identity(r.map_asset)},{"provenance_id",r.provenance_id},
      {"calibration_id",r.calibration_id},{"common_frame",r.common_frame},{"expected_source_id",r.expected_source_id},
      {"expected_source_epoch",r.expected_source_epoch},{"expected_source_calibration_id",r.expected_source_calibration_id},
      {"valid_for_source_ns",r.valid_for_source_ns},{"valid_for_common_ns",r.valid_for_common_ns},
      {"source_ns",r.source_ns},{"first_receipt_ros_ns",r.first_receipt_ros_ns},
      {"first_receipt_steady_ns",r.first_receipt_steady_ns},{"expiry_budget_ns",r.expiry_budget_ns},
      {"independently_qualified",r.independently_qualified},{"source_time_transform_qualified",r.source_time_transform_qualified},
      {"unit_normal",r.unit_normal},{"anchor",r.anchor},{"plane_offset_error_bound_m",r.plane_offset_error_bound_m},
      {"plane_angle_error_bound_rad",r.plane_angle_error_bound_rad}};
    ref["support"]={{"origin_x_m",r.support.origin_x_m},{"origin_y_m",r.support.origin_y_m},
      {"resolution_m",r.support.resolution_m},{"width",r.support.width},{"height",r.support.height},{"required_cells",Json::array()}};
    for(const auto& c:r.support.required_cells)ref["support"]["required_cells"].push_back({{"x",c.x},{"y",c.y}});
    ref["map_binding"]={{"source_id",r.map_binding.source_id},{"source_epoch",r.map_binding.source_epoch},
      {"source_ns",r.map_binding.source_ns},{"alignment",nullptr}};
    if(const auto& a=r.map_binding.alignment)ref["map_binding"]["alignment"]={
      {"identity",map_identity(a->identity())},{"reset_epoch",a->reset_epoch()},
      {"geometry_valid_until_ros_ns",a->geometry_valid_until_ros_ns()},
      {"geometry_valid_until_steady_ns",a->geometry_valid_until_steady_ns()},
      {"current_at_export",a->current()}};
  }
  if(owner) {
    j["owner"]={{"supported_segmentation",owner->supported_segmentation()},{"observation_only",owner->observation_only()},
      {"map_identity",map_identity(owner->map_identity())}};
    const auto& c=owner->segmentation_config();
    j["owner"]["segmentation"]={
      {"maximum_point_residual_m",c.maximum_point_residual_m},
      {"maximum_cell_median_residual_m",c.maximum_cell_median_residual_m},{"maximum_cell_mad_m",c.maximum_cell_mad_m},
      {"maximum_partition_median_difference_m",c.maximum_partition_median_difference_m},
      {"maximum_normal_tilt_rad",c.maximum_normal_tilt_rad},{"minimum_support_span_x_m",c.minimum_support_span_x_m},
      {"minimum_support_span_y_m",c.minimum_support_span_y_m},{"minimum_support_double_area_m2",c.minimum_support_double_area_m2},
      {"minimum_samples_per_partition",c.minimum_samples_per_partition},{"minimum_supported_cells",c.minimum_supported_cells},
      {"continuous_support_radius_cells",c.continuous_support_radius_cells},{"maximum_points",c.maximum_points},
      {"maximum_support_cells",c.maximum_support_cells},{"maximum_support_checks",c.maximum_support_checks}};
  }
  return j.dump();
}
namespace {
std::string write_snapshot_bytes(const std::string& directory,const std::string& name,const std::string& bytes) {
  namespace fs=std::filesystem;
  const auto dir=fs::canonical(directory);const fs::path root="/data/navigation/evidence";
  const auto relative=dir.lexically_relative(root);
  if(!dir.is_absolute()||relative.empty()||*relative.begin()==".."||!fs::is_directory(dir))
    throw std::invalid_argument("ground_snapshot_outside_evidence_directory");
  const int directory_fd=::open(dir.c_str(),O_RDONLY|O_DIRECTORY|O_NOFOLLOW);
  if(directory_fd<0)throw std::system_error(errno,std::generic_category(),"ground_snapshot_directory_open");
  struct Close {int fd;~Close(){if(fd>=0)::close(fd);}} close_dir{directory_fd};
  const auto partial=name+".partial";
  const int fd=::openat(directory_fd,partial.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_NOFOLLOW,0600);
  if(fd<0)throw std::system_error(errno,std::generic_category(),"ground_snapshot_create");
  Close close_file{fd};
  try {
    std::size_t at=0;
    while(at<bytes.size()) {
      const auto count=::write(fd,bytes.data()+at,bytes.size()-at);
      if(count<0&&errno==EINTR)continue;
      if(count<=0)throw std::system_error(count<0?errno:EIO,std::generic_category(),"ground_snapshot_write");
      at+=static_cast<std::size_t>(count);
    }
    if(::fsync(fd)!=0)throw std::system_error(errno,std::generic_category(),"ground_snapshot_sync");
    // Atomic publication with no replacement of any previous evidence file.
    if(::linkat(directory_fd,partial.c_str(),directory_fd,name.c_str(),0)!=0)
      throw std::system_error(errno,std::generic_category(),"ground_snapshot_publish");
  }catch(...) {::unlinkat(directory_fd,partial.c_str(),0);throw;}
  ::unlinkat(directory_fd,partial.c_str(),0);
  return (dir/name).string();
}
}
std::string write_ground_classification_snapshot(const std::string& directory,const std::string& role,
    const gr::ClassifiedObstacleFrame& frame,const gr::GroundReferenceOwner* owner) {
  if(directory.empty()||(role!="belly"&&role!="chest"&&role!="airy"))
    throw std::invalid_argument("ground_snapshot_directory_or_role");
  return write_snapshot_bytes(directory,role+"-classification-"+
    std::to_string(frame.source().identity.sequence)+".json",serialize_ground_classification_snapshot(frame,owner));
}
std::string write_airy_geometry_audit_snapshot(const std::string& directory,uint64_t sequence,const std::string& bytes) {
  if(directory.empty()||!sequence)throw std::invalid_argument("airy_geometry_audit_snapshot_identity");
  return write_snapshot_bytes(directory,"airy-geometry-audit-"+std::to_string(sequence)+".json",bytes);
}

std::string write_rgbd_projection_audit_snapshot(const std::string& directory,const std::string& role,
    uint64_t sequence,const std::string& bytes,const std::string& consumer) {
  if(directory.empty()||!sequence||(role!="belly"&&role!="chest")||
      (consumer!="global"&&consumer!="local"))
    throw std::invalid_argument("rgbd_projection_audit_snapshot_identity");
  return write_snapshot_bytes(directory,role+"-"+consumer+"-projection-audit-"+std::to_string(sequence)+".json",bytes);
}

namespace {
std::string writer_prefix(const std::string& role,uint64_t sequence,
    const std::string& request,const std::string& consumer) {
  if((role!="belly"&&role!="chest")||(consumer!="local"&&consumer!="global")||
      !sequence||request.empty()||request.size()>64)throw std::invalid_argument("writer_snapshot_identity");
  for(const char c:request)if(!((c>='a'&&c<='z')||(c>='A'&&c<='Z')||
      (c>='0'&&c<='9')||c=='_'||c=='-'))throw std::invalid_argument("writer_snapshot_request");
  return role+"-"+consumer+"-writer-projection-audit-"+request+"-"+std::to_string(sequence);
}
std::string bytes_sha256(const std::string& bytes) {
  unsigned char digest[SHA256_DIGEST_LENGTH];SHA256(reinterpret_cast<const unsigned char*>(bytes.data()),bytes.size(),digest);
  std::ostringstream out;for(auto b:digest)out<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(b);return out.str();
}
}
static nlohmann::json write_primary_snapshot(const std::string& directory,const std::string& prefix,
    const sensor_msgs::msg::Image& image,const sensor_msgs::msg::CameraInfo& info) {
  if(image.encoding!="16UC1"||!image.width||!image.height||uint64_t(image.step)<uint64_t(image.width)*2||
     image.header.frame_id!=info.header.frame_id||image.header.stamp.sec!=info.header.stamp.sec||
     image.header.stamp.nanosec!=info.header.stamp.nanosec||image.width!=info.width||image.height!=info.height||
     uint64_t(image.step)*image.height!=image.data.size())throw std::invalid_argument("writer_primary_image_info_identity");
  const std::string raw(reinterpret_cast<const char*>(image.data.data()),image.data.size());
  const auto raw_path=write_snapshot_bytes(directory,prefix+"-primary.bin",raw);
  Json camera={{"header_ns",int64_t(info.header.stamp.sec)*1000000000+info.header.stamp.nanosec},
    {"frame_id",info.header.frame_id},{"width",info.width},{"height",info.height},
    {"distortion_model",info.distortion_model},{"d",info.d},{"k",info.k},{"r",info.r},{"p",info.p},
    {"binning_x",info.binning_x},{"binning_y",info.binning_y},
    {"roi",{{"x_offset",info.roi.x_offset},{"y_offset",info.roi.y_offset},{"height",info.roi.height},
      {"width",info.roi.width},{"do_rectify",info.roi.do_rectify}}}};
  const auto camera_bytes=camera.dump();const auto info_path=write_snapshot_bytes(directory,prefix+"-primary-camera-info.json",camera_bytes);
  return {{"scope","Original retained CameraFrame primary depth/Info; native and dual-IR payloads not embedded"},
    {"path",raw_path},{"sha256",bytes_sha256(raw)},{"bytes",raw.size()},
    {"encoding",image.encoding},{"is_bigendian",image.is_bigendian},{"step",image.step},
    {"width",image.width},{"height",image.height},{"header_ns",camera["header_ns"]},{"frame_id",image.header.frame_id},
    {"camera_info_path",info_path},{"camera_info_sha256",bytes_sha256(camera_bytes)}};
}
nlohmann::json write_rgbd_writer_primary_snapshot(const std::string& directory,
    const std::string& role,uint64_t sequence,const std::string& request,const std::string& consumer,
    const sensor_msgs::msg::Image& image,const sensor_msgs::msg::CameraInfo& info) {
  return write_primary_snapshot(directory,writer_prefix(role,sequence,request,consumer),image,info);
}
std::string write_rgbd_writer_status_snapshot(const std::string& directory,
    const std::string& request,std::size_t slot,const std::string& bytes) {
  (void)writer_prefix("belly",1,request,"local");
  if(slot>=4)throw std::invalid_argument("writer_status_slot");
  return write_snapshot_bytes(directory,"rgbd-writer-audit-status-"+request+"-"+std::to_string(slot)+".json",bytes);
}
std::string write_rgbd_writer_classification_snapshot(const std::string& directory,
    const std::string& role,uint64_t sequence,const std::string& request,const std::string& consumer,const std::string& bytes) {
  return write_snapshot_bytes(directory,writer_prefix(role,sequence,request,consumer)+".json",bytes);
}

namespace {
int64_t body_stamp(const std_msgs::msg::Header& h) {
  return int64_t(h.stamp.sec)*1000000000LL+h.stamp.nanosec;
}
Json body_header(const std_msgs::msg::Header& h) {
  return {{"header_ns",body_stamp(h)},{"frame_id",h.frame_id}};
}
template<class T> std::string body_bytes(const std::vector<T>& v) {
  return v.empty()?std::string{}:std::string(reinterpret_cast<const char*>(v.data()),v.size()*sizeof(T));
}
Json body_camera_info(const sensor_msgs::msg::CameraInfo& i) {
  auto j=body_header(i.header);
  j.update({{"width",i.width},{"height",i.height},{"distortion_model",i.distortion_model},
    {"d",i.d},{"k",i.k},{"r",i.r},{"p",i.p},{"binning_x",i.binning_x},{"binning_y",i.binning_y},
    {"roi",{{"x_offset",i.roi.x_offset},{"y_offset",i.roi.y_offset},{"height",i.roi.height},
      {"width",i.roi.width},{"do_rectify",i.roi.do_rectify}}}});
  return j;
}
std::pair<std::vector<uint64_t>,std::vector<float>> body_support(const BodyReplayDiagnostic& r) {
  std::pair<std::vector<uint64_t>,std::vector<float>> result;
  for(const auto& s:r.self_support) {
    result.first.push_back(s.sample_id);
    result.second.insert(result.second.end(),s.xyz.begin(),s.xyz.end());
  }
  return result;
}
Json body_query_budget(const dg::QueryBudget& b) {
  return {{"max_links",b.max_links},{"max_node_visits",b.max_node_visits},{"max_triangle_tests",b.max_triangle_tests}};
}
}
nlohmann::json serialize_body_replay_snapshot(const PerceptionOutput& output,const RuntimeConfig& runtime) {
  const auto replay=output.frame.body_replay_diagnostic;
  if(!replay||!replay->model||!replay->image||!replay->info||!output.input)
    throw std::invalid_argument("body_replay_missing_original_owner");
  const auto& r=*replay;const auto& m=*r.model;const auto& id=m.identity;
  const auto& image=*r.image;const auto& info=*r.info;const auto& c=r.config;
  const auto& f=output.frame.body_filter;
  const auto role=id.source_id=="belly"?0:id.source_id=="chest"?1:2;
  if(role==2||runtime.sources[role].name!=id.source_id||runtime.sources[role].epoch!=id.epoch||
     runtime.sources[role].calibration!=id.calibration_id||!id.sequence||id.source_stamp_ros_ns<=0||
     id.receipt_ros_ns<=0||id.receipt_steady_ns<=0||id.expiry_budget_ns<=0||
     output.input->role!=id.source_id||output.input->sequence!=id.sequence||!output.input->source_validated||
     output.input->first_sdk_callback_ros_ns!=id.receipt_ros_ns||
     output.input->first_sdk_callback_steady_ns!=id.receipt_steady_ns||
     int64_t(output.input->source_stamp.sec)*1000000000LL+output.input->source_stamp.nanosec!=id.source_stamp_ros_ns||
     r.image!=output.input->depth||r.info!=output.input->depth_info||r.measurement_quality!=output.input->measurement_quality||
     body_stamp(image.header)!=id.source_stamp_ros_ns||image.header.frame_id!=m.source_frame||
     body_stamp(info.header)!=id.source_stamp_ros_ns||info.header.frame_id!=m.source_frame||
     body_stamp(output.frame.body_cloud.header)!=id.source_stamp_ros_ns||
     output.frame.body_cloud.header.frame_id!=m.source_frame||
     image.encoding!="16UC1"||!image.width||!image.height||image.step<uint64_t(image.width)*2||
     uint64_t(image.step)*image.height!=image.data.size()||image.width!=info.width||image.height!=info.height||
     !f.complete||r.retained_sample_ids.size()!=f.retained_count||
     f.ground_blocked.size()!=f.retained_count||f.nominal_model_conflict.size()!=f.retained_count||
     r.query_ns<=0||m.links.size()!=12||runtime.model_specs.size()!=12||r.keep_present!=(c.component_minimum_pixels!=0)||
     (r.keep_present&&r.keep.size()!=std::size_t(image.width)*image.height)||
     (r.cuda_backend&&r.parallel_cpu)||m.self_tolerance_m!=c.self_tolerance_m||
     m.query_budget.max_links!=c.body_budget.query.max_links||
     m.query_budget.max_node_visits!=c.body_budget.query.max_node_visits||
     m.query_budget.max_triangle_tests!=c.body_budget.query.max_triangle_tests)
    throw std::invalid_argument("body_replay_original_identity_or_layout");
  if(!std::isfinite(r.stereo_baseline_m)||r.stereo_baseline_m<0. ||
      (r.stereo_baseline_m>0. && (!c.body_stereo_occlusion_enabled||!r.measurement_quality||
       r.measurement_quality->body_occlusion_baseline(info.k,output.input->depth_time.device_us,
         output.input->depth_time.global_us,output.input->depth_frame_index,output.input->sdk_depth_scale_m)!=r.stereo_baseline_m)))
    throw std::invalid_argument("body_replay_stereo_geometry_binding_mismatch");
  uint16_t endian=1;
  if(*reinterpret_cast<uint8_t*>(&endian)!=1||sizeof(float)!=4||sizeof(uint64_t)!=8||sizeof(std::size_t)!=8)
    throw std::invalid_argument("body_replay_requires_original_little_endian_bytes");
  for(const auto v:r.keep)if(v>1)throw std::invalid_argument("body_replay_keep_value");
  Json identity={{"source_id",id.source_id},{"source_frame",m.source_frame},{"epoch",id.epoch},
    {"calibration_id",id.calibration_id},{"sequence",id.sequence},{"source_ns",id.source_stamp_ros_ns},
    {"first_receipt_ros_ns",id.receipt_ros_ns},{"first_receipt_steady_ns",id.receipt_steady_ns},
    {"expiry_budget_ns",id.expiry_budget_ns},{"required_for_evaluation",id.required_for_evaluation}};
  Json model=identity;model.update({{"role",id.source_id},{"query_ns",r.query_ns},
    {"prepared_model_ready",true},{"self_tolerance_m",m.self_tolerance_m},
    {"query_budget",body_query_budget(m.query_budget)},
    {"model_link_index_contract","configured twelve-link mesh manifest order"},{"links",Json::array()}});
  Json manifest={{"model_link_names",Json::array()},{"model_mesh_paths",Json::array()},
    {"model_mesh_sha256",Json::array()},{"model_triangle_counts",Json::array()},
    {"self_tolerance_m",m.self_tolerance_m}};
  for(std::size_t i=0;i<12;++i) {
    const auto& l=m.links[i];const auto& a=runtime.model_specs[i];
    if(l.link_index!=i||!l.mesh_revision||!l.triangle_count||l.triangle_count!=a.triangle_count||
       l.boundary_triangle_begin!=a.boundary_triangle_begin||l.boundary_triangle_count!=a.boundary_triangle_count||
       l.boundary_tolerance_m!=a.boundary_tolerance_m||l.measured_triangle_begin!=a.measured_triangle_begin||
       l.measured_triangle_count!=a.measured_triangle_count||!sunhorizon_rgbd_adapter::dynamic_self_model::sha_valid(a.sha256))
      throw std::invalid_argument("body_replay_model_manifest_mismatch");
    model["links"].push_back({{"link_index",i},{"triangle_count",l.triangle_count},{"mesh_revision",l.mesh_revision},
      {"boundary_triangle_begin",l.boundary_triangle_begin},{"boundary_triangle_count",l.boundary_triangle_count},
      {"boundary_tolerance_m",l.boundary_tolerance_m},{"measured_triangle_begin",l.measured_triangle_begin},
      {"measured_triangle_count",l.measured_triangle_count},
      {"optical_from_link",{{"rotation",l.optical_from_link.rotation},
        {"translation",{l.optical_from_link.translation.x,l.optical_from_link.translation.y,l.optical_from_link.translation.z}}}}});
    manifest["model_link_names"].push_back(a.link_name);manifest["model_mesh_paths"].push_back(a.path);
    manifest["model_mesh_sha256"].push_back(a.sha256);manifest["model_triangle_counts"].push_back(a.triangle_count);
  }
  const auto support=body_support(r);
  Json canonical={{"complete",f.complete},{"reason",f.reason},{"input",f.input_count},{"removed",f.removed_count},
    {"retained",f.retained_count},{"ambiguous",f.measured_reference_ambiguous_count},{"classifications",f.classifications},
    {"nodes",f.node_visits},{"triangles",f.triangle_tests},
    {"xyz_sha256",bytes_sha256(body_bytes(output.frame.body_cloud.data))},
    {"sample_ids_sha256",bytes_sha256(body_bytes(r.retained_sample_ids))},
    {"blocked_sha256",bytes_sha256(body_bytes(f.ground_blocked))},
    {"conflict_sha256",bytes_sha256(body_bytes(f.nominal_model_conflict))},
    {"support_ids_sha256",bytes_sha256(body_bytes(support.first))},
    {"support_xyz_sha256",bytes_sha256(body_bytes(support.second))}};
  Json cfg={{"depth_scale_m",c.depth_scale_m},{"minimum_depth_m",c.minimum_depth_m},{"maximum_depth_m",c.maximum_depth_m},
    {"body_stride",c.body_stride},{"self_tolerance_m",c.self_tolerance_m},
    {"body_stereo_occlusion_enabled",c.body_stereo_occlusion_enabled},{"stereo_body_baseline_m",r.stereo_baseline_m},
    {"component_minimum_pixels",c.component_minimum_pixels},{"component_neighbor_delta_m",c.component_neighbor_delta_m},
    {"component_maximum_depth_m",c.component_maximum_depth_m},{"near_component_minimum_pixels",c.near_component_minimum_pixels},
    {"near_component_maximum_depth_m",c.near_component_maximum_depth_m},
    {"body_geometry_compute_backend",r.cuda_backend?"cuda_arm11":"cpu"},{"body_geometry_parallel_enabled",r.parallel_cpu},
    {"body_budget",{{"max_points",c.body_budget.max_points},{"max_bytes",c.body_budget.max_bytes},
      {"max_node_visits",c.body_budget.max_node_visits},{"max_triangle_tests",c.body_budget.max_triangle_tests},
      {"query",body_query_budget(c.body_budget.query)}}}};
  const auto& q=r.calibration;
  Json cloud={{"header",body_header(output.frame.body_cloud.header)},{"width",output.frame.body_cloud.width},
    {"height",output.frame.body_cloud.height},{"point_step",output.frame.body_cloud.point_step},
    {"row_step",output.frame.body_cloud.row_step},{"is_bigendian",bool(output.frame.body_cloud.is_bigendian)},
    {"is_dense",output.frame.body_cloud.is_dense},{"fields",Json::array()}};
  for(const auto& field:output.frame.body_cloud.fields)cloud["fields"].push_back(
    {{"name",field.name},{"offset",field.offset},{"datatype",field.datatype},{"count",field.count}});
  Json result={{"schema","body_replay_snapshot_v1"},{"purpose","diagnostic_only_original_BODY_no_ground_authority"},
    {"source",{{"identity",identity}}},{"body_model",model},{"model_manifest",manifest},
    {"body_filter_config",cfg},{"camera_info",body_camera_info(info)},{"runtime_body",canonical},
    {"runtime_body_cloud",cloud},{"body_keep",{{"present",r.keep_present},{"bytes",r.keep.size()}}},
    {"qualification",{{"source_validated",output.input->source_validated},{"rectified_pinhole_verified",q.rectified_pinhole_verified},
      {"metric_axial_depth_verified",q.metric_axial_depth_verified},{"depth_intrinsics_registration_verified",q.depth_intrinsics_registration_verified},
      {"optical_to_base_extrinsics_verified",q.optical_to_base_extrinsics_verified},
      {"source_time_and_transform_verified",q.source_time_and_transform_verified}}},
    {"raw_capture",{{"depth_header",body_header(output.input->depth->header)},{"depth_scale_m",output.input->depth_scale_m},
      {"sdk_depth_scale_m",output.input->sdk_depth_scale_m},{"color_header",nullptr},
      {"depth_time",{{"device_us",output.input->depth_time.device_us},{"global_us",output.input->depth_time.global_us},{"sdk_system_us",output.input->depth_time.sdk_system_us}}}}},
    {"byte_contract","canonical hashes use original uint8/uint64LE/float32LE bytes; signed zero preserved; no transformed or Ground-classified substitute"}};
  result["stereo_geometry"]=nullptr;
  if(r.measurement_quality){const auto& geometry=*r.measurement_quality;
    result["stereo_geometry"]={{"verified",geometry.stereo_body_geometry_verified},
      {"reason",geometry.stereo_body_geometry_reason},{"source_ns",geometry.source_ns},{"sequence",geometry.sequence},
      {"source_frame",geometry.source_frame},{"opened_serial",geometry.opened_serial},{"capture_epoch",geometry.capture_epoch},
      {"depth_device_us",geometry.geometry_depth_device_us},{"depth_global_us",geometry.geometry_depth_global_us},
      {"depth_frame_index",geometry.geometry_depth_frame_index},{"sdk_depth_scale_m",geometry.geometry_depth_scale_m},
      {"depth_pixel_type",geometry.depth_pixel_type},{"baseline_m",geometry.stereo_baseline_m},
      {"right_from_depth_translation_m",geometry.right_from_depth_translation_m},{"depth_k",geometry.rectified_depth_k},
      {"exact_ir_pair",geometry.exact_rectified_pair},{"effective_baseline_m",r.stereo_baseline_m}};
  }
  if(output.input->color)result["raw_capture"]["color_header"]=body_header(output.input->color->header);
  if(output.input->source_identity)result["raw_capture"]["source_identity"]={
    {"owner_fqn",output.input->source_identity->owner_fqn},{"opened_serial",output.input->source_identity->opened_serial},
    {"capture_epoch",output.input->source_identity->capture_epoch}};
  return result;
}
std::string write_body_replay_snapshot(const std::string& directory,const PerceptionOutput& output,const RuntimeConfig& runtime) {
  auto j=serialize_body_replay_snapshot(output,runtime); // Validate before any I/O.
  const auto& r=*output.frame.body_replay_diagnostic;const auto& f=output.frame.body_filter;
  const auto prefix="body-replay-"+r.model->identity.source_id+"-"+std::to_string(r.model->identity.sequence);
  j["primary_depth"]=write_primary_snapshot(directory,prefix,*r.image,*r.info);
  j["primary_depth"]["scope"]="Exact immutable Image and CameraInfo used by original BODY";
  j["primary_depth"]["is_bigendian"]=bool(r.image->is_bigendian);
  const auto save=[&](const std::string& suffix,const std::string& bytes) {
    return Json{{"path",write_snapshot_bytes(directory,prefix+"-"+suffix+".bin",bytes)},
      {"sha256",bytes_sha256(bytes)},{"bytes",bytes.size()}};
  };
  if(r.keep_present){j["body_keep"]=save("keep",body_bytes(r.keep));j["body_keep"]["present"]=true;}
  const auto support=body_support(r);
  j["runtime_body_artifacts"]={{"cloud",save("body-cloud",body_bytes(output.frame.body_cloud.data))},
    {"retained_sample_ids",save("retained-ids",body_bytes(r.retained_sample_ids))},
    {"ground_blocked",save("blocked",body_bytes(f.ground_blocked))},
    {"nominal_model_conflict",save("conflict",body_bytes(f.nominal_model_conflict))},
    {"self_support_ids",save("support-ids",body_bytes(support.first))},
    {"self_support_xyz",save("support-xyz",body_bytes(support.second))}};
  return write_snapshot_bytes(directory,prefix+".json",j.dump());
}
}
