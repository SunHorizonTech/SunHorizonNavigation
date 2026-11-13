#include "sunhorizon_perception_runtime/runtime_config.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <set>
#include <stdexcept>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include "sunhorizon_rgbd_adapter/region_depth_producer.hpp"
#include "sunhorizon_perception_runtime/joint_startup_contract.hpp"

namespace sunhorizon_perception_runtime {
namespace dm = sunhorizon_rgbd_adapter::dynamic_self_model;
namespace {
template<class T> T parameter(rclcpp::Node& node,const std::string& name,const T& fallback) {
  if (node.has_parameter(name)) return node.get_parameter(name).get_value<T>();
  rcl_interfaces::msg::ParameterDescriptor descriptor;descriptor.read_only=true;
  return node.declare_parameter<T>(name,fallback,descriptor);
}
void identity(const std::string& value) {
  if (value.empty() || value.size()>128 || std::any_of(value.begin(),value.end(),
      [](unsigned char c){return c<33 || c>126;}))
    throw std::invalid_argument("invalid_spatial_identity");
}
std::size_t count_parameter(rclcpp::Node& node,const std::string& name,int64_t fallback,
    std::size_t maximum,bool zero_allowed) {
  const auto value=parameter<int64_t>(node,name,fallback);
  if (value<(zero_allowed?0:1) || uint64_t(value)>maximum)
    throw std::invalid_argument("invalid_runtime_count_parameter:"+name);
  return std::size_t(value);
}
RuntimeSourceConfig source_config(rclcpp::Node& node,std::size_t index,
    double tolerance,double margin) {
  RuntimeSourceConfig source;source.name=index==0?"belly":"chest";
  const auto& name=source.name;
  source.frame=parameter<std::string>(node,name+"_frame","");
  source.epoch=parameter<std::string>(node,name+"_epoch","");
  source.calibration=parameter<std::string>(node,name+"_calibration","");
  // Sensor ownership is the connected SDK factory, not an obsolete DDS GID.
  identity(source.frame);identity(source.epoch);
  if(source.calibration.empty() || source.calibration.size()>8192)
    throw std::invalid_argument("spatial_source_manifest");
  auto& engine=source.engine;
  engine.classified_obstacles_enabled=parameter<bool>(node,"classified_obstacles_enabled",false);
  engine.depth_scale_m=parameter<double>(node,name+"_depth_scale_m",0.);
  engine.minimum_depth_m=parameter<double>(node,name+"_minimum_depth_m",0.);
  engine.maximum_depth_m=parameter<double>(node,name+"_maximum_depth_m",0.);
  if (!std::isfinite(engine.depth_scale_m) || engine.depth_scale_m<=0. ||
      !std::isfinite(engine.minimum_depth_m) || engine.minimum_depth_m<=0. ||
      !std::isfinite(engine.maximum_depth_m) || engine.maximum_depth_m<=engine.minimum_depth_m)
    throw std::invalid_argument("spatial_source_manifest");
  auto& accepted=source.attestations;
  accepted.rectified_pinhole_verified=parameter<bool>(node,name+"_rectified_pinhole_verified",false);
  accepted.metric_axial_depth_verified=parameter<bool>(node,name+"_metric_axial_depth_verified",false);
  accepted.depth_intrinsics_registration_verified=parameter<bool>(node,name+"_depth_intrinsics_registration_verified",false);
  accepted.optical_to_base_extrinsics_verified=parameter<bool>(node,name+"_optical_to_base_extrinsics_verified",false);
  accepted.source_time_and_transform_verified=parameter<bool>(node,name+"_source_time_and_transform_verified",false);
  engine.coverage_enabled=accepted.rectified_pinhole_verified && accepted.metric_axial_depth_verified &&
      accepted.depth_intrinsics_registration_verified && accepted.optical_to_base_extrinsics_verified &&
      accepted.source_time_and_transform_verified;
  if (index==0 && !engine.coverage_enabled) throw std::invalid_argument("spatial_source_attestation_missing");
  source.body_enabled=parameter<bool>(node,name+"_body_enabled",false);
  source.projection_accepted=parameter<bool>(node,name+"_projection_accepted",false);
  if (!source.body_enabled || !source.projection_accepted)
    throw std::invalid_argument("runtime_body_or_projection_not_accepted:"+name);
  if (index==0) {
    source.self_filter_enabled=parameter<bool>(node,name+"_self_filter_enabled",false);
    source.self_exclusion_mode=parameter<std::string>(node,name+"_self_exclusion_mode","");
    if (!source.self_filter_enabled || source.self_exclusion_mode!="dynamic-mesh-v1")
      throw std::invalid_argument("runtime_requires_already_selected_dynamic_body_profile");
  }
  engine.map_body_proximity_filter_enabled=parameter<bool>(node,name+"_map_body_proximity_filter_enabled",false);
  engine.body_proximity_filter_enabled=parameter<bool>(node,name+"_body_proximity_filter_enabled",false);
  if((engine.body_proximity_filter_enabled || engine.map_body_proximity_filter_enabled) &&
      !engine.classified_obstacles_enabled)
    throw std::invalid_argument("body_proximity_requires_classified_obstacles");
  engine.self_tolerance_m=tolerance;engine.clear_margin_m=margin;
  source.max_removed_fraction=parameter<double>(node,name+"_max_removed_fraction",.02);
  if (!std::isfinite(source.max_removed_fraction) || source.max_removed_fraction<=0. || source.max_removed_fraction>.10)
    throw std::invalid_argument("invalid_runtime_max_removed_fraction");
  engine.body_stride=uint32_t(count_parameter(node,name+"_pointcloud_pixel_stride",2,16,false));
  const bool components=parameter<bool>(node,name+"_component_filter_enabled",false);
  const auto component_minimum=count_parameter(node,name+"_component_min_pixels",0,
      sunhorizon_rgbd_adapter::kMaxDepthComponentPixels,true);
  if (components && !component_minimum) throw std::invalid_argument("enabled_runtime_component_filter_has_no_threshold");
  engine.component_minimum_pixels=components?component_minimum:0;
  engine.component_neighbor_delta_m=parameter<double>(node,name+"_component_neighbor_delta_m",.05);
  engine.component_maximum_depth_m=parameter<double>(node,name+"_component_filter_max_depth_m",engine.maximum_depth_m);
  const auto near_minimum=count_parameter(node,name+"_near_component_min_pixels",0,
      sunhorizon_rgbd_adapter::kMaxDepthComponentPixels,true);
  if (!components && near_minimum) throw std::invalid_argument("near_component_filter_requires_component_filter");
  engine.near_component_minimum_pixels=near_minimum;
  engine.near_component_maximum_depth_m=parameter<double>(node,name+"_near_component_max_depth_m",1.5);
  engine.raw_projection_enabled=parameter<bool>(node,name+"_raw_projection_enabled",true);
  engine.body_stereo_occlusion_enabled=parameter<bool>(node,name+"_body_stereo_occlusion_enabled",false);
  if(engine.body_stereo_occlusion_enabled && (name!="belly" || engine.raw_projection_enabled))
    throw std::invalid_argument("stereo_body_occlusion_requires_belly_raw_disabled");
  if(engine.classified_obstacles_enabled && engine.raw_projection_enabled)
    throw std::invalid_argument("classified_obstacles_require_raw_projection_disabled");
  engine.raw_projection_stride=uint32_t(count_parameter(node,"rgbd.pointcloud_pixel_stride",2,16,false));
  const auto pair_delta=parameter<double>(node,"rgbd.max_pair_delta_sec",.005);
  if(!std::isfinite(pair_delta) || pair_delta<0. || pair_delta>.005)
    throw std::invalid_argument("raw_projection_original_rgbd_pair_bound_changed");
  engine.raw_projection_max_pair_delta_ns=static_cast<int64_t>(pair_delta*1e9);
  const auto raw_age=parameter<double>(node,"rgbd.freshness_timeout_sec",.3);
  if(!std::isfinite(raw_age) || raw_age<=0. || raw_age>1.)
    throw std::invalid_argument("raw_projection_original_age_out_of_supported_bound");
  engine.raw_projection_expiry_budget_ns=static_cast<int64_t>(raw_age*1e9);
  if(engine.raw_projection_enabled &&
     (engine.depth_scale_m!=parameter<double>(node,"rgbd.depth_scale_m",.001) ||
      engine.minimum_depth_m!=parameter<double>(node,"rgbd.min_depth_m",.2) ||
      engine.maximum_depth_m!=parameter<double>(node,"rgbd.max_depth_m",8.)))
    throw std::invalid_argument("raw_projection_original_depth_geometry_contract_mismatch");
  engine.raw_projection_use_component_mask=index==1 && parameter<bool>(node,"rgbd.chest_component_filter_enabled",false);
  if(engine.raw_projection_enabled && engine.raw_projection_use_component_mask){
    // Reuse exactly the original chest quality mask. If the body profile has
    // diverged, reject instead of silently applying its different algorithm.
    if(engine.component_minimum_pixels!=count_parameter(node,"rgbd.chest_component_min_pixels",0,
         sunhorizon_rgbd_adapter::kMaxDepthComponentPixels,true) ||
       engine.near_component_minimum_pixels!=count_parameter(node,"rgbd.chest_near_component_min_pixels",0,
         sunhorizon_rgbd_adapter::kMaxDepthComponentPixels,true) ||
       engine.component_neighbor_delta_m!=parameter<double>(node,"rgbd.chest_component_neighbor_delta_m",.05) ||
       engine.near_component_maximum_depth_m!=parameter<double>(node,"rgbd.chest_near_component_max_depth_m",1.5))
      throw std::invalid_argument("raw_chest_projection_original_quality_mask_contract_mismatch");
  }
  // Reuse the actual computation object's scalar/budget validation. No input,
  // mesh preparation, geometry computation, or subscription happens here.
  const PerceptionEngine validation(engine);(void)validation;
  const auto prefix=name+"_region.";
  if (parameter<bool>(node,prefix+"region_support_enabled",false)) {
    sunhorizon_rgbd_adapter::RegionProducerOptions o;
    o.region_id=parameter<std::string>(node,prefix+"region_id","");
    o.base_frame=parameter<std::string>(node,prefix+"region_base_frame","base_footprint");
    o.optical_frame=parameter<std::string>(node,prefix+"region_optical_frame","");
    o.depth_topic=parameter<std::string>(node,prefix+"depth_topic","");
    o.info_topic=parameter<std::string>(node,prefix+"camera_info_topic","");
    o.diagnostics_topic=parameter<std::string>(node,prefix+"region_support_diagnostics_topic","");
    o.expected_calibration=parameter<std::string>(node,prefix+"region_expected_calibration","");
    o.expected_contract=parameter<std::string>(node,prefix+"region_expected_contract","");
    const auto minimum=parameter<std::vector<double>>(node,prefix+"region_box_min_xyz",{});
    const auto maximum=parameter<std::vector<double>>(node,prefix+"region_box_max_xyz",{});
    if (minimum.size()!=3 || maximum.size()!=3) throw std::invalid_argument("invalid_region_box_size");
    std::copy(minimum.begin(),minimum.end(),o.box_min.begin());
    std::copy(maximum.begin(),maximum.end(),o.box_max.begin());
    o.clear_margin_m=parameter<double>(node,prefix+"region_clear_margin_m",.02);
    o.tf_wait_sec=parameter<double>(node,prefix+"region_tf_wait_sec",0.);
    o.shadow_only_enabled=parameter<bool>(node,prefix+"region_shadow_only_enabled",false);
    o.hold_complete_measurement_enabled=parameter<bool>(node,prefix+"region_hold_complete_measurement_enabled",false);
    o.typed_evaluation_diagnostics_enabled=parameter<bool>(node,prefix+"region_typed_evaluation_diagnostics_enabled",false);
    o.depth_scale_m=engine.depth_scale_m;o.minimum_depth_m=engine.minimum_depth_m;
    o.maximum_depth_m=engine.maximum_depth_m;o.max_age_sec=parameter<double>(node,prefix+"max_cloud_age_sec",.3);
    o.accepted.rectified_pinhole_verified=parameter<bool>(node,prefix+"region_rectified_pinhole_accepted",false);
    o.accepted.metric_axial_depth_verified=parameter<bool>(node,prefix+"region_metric_axial_depth_accepted",false);
    o.accepted.depth_intrinsics_registration_verified=parameter<bool>(node,prefix+"region_depth_intrinsics_registration_accepted",false);
    o.accepted.optical_to_base_extrinsics_verified=parameter<bool>(node,prefix+"region_optical_to_base_extrinsics_accepted",false);
    o.accepted.source_time_and_transform_verified=parameter<bool>(node,prefix+"region_source_time_transform_accepted",false);
    source.region_producer_node_name=parameter<std::string>(node,name+"_region_producer_node_name","");
    identity(source.region_producer_node_name);
    if (o.optical_frame!=source.frame || o.expected_calibration!=source.calibration ||
        o.expected_contract!=sunhorizon_rgbd_adapter::region_producer_contract(o))
      throw std::invalid_argument("runtime_region_original_contract_mismatch");
    source.region=std::move(o);
  }
  return source;
}
std::string checked_urdf(const std::string& filename,const std::string& hash) {
  if (!dm::sha_valid(hash)) throw std::invalid_argument("runtime_urdf_requires_sha256");
  const std::filesystem::path path(filename);
  if (!path.is_absolute() || std::filesystem::canonical(path)!=path ||
      !std::filesystem::is_regular_file(std::filesystem::symlink_status(path)))
    throw std::invalid_argument("runtime_urdf_not_canonical_regular");
  const auto size=std::filesystem::file_size(path);const auto time=std::filesystem::last_write_time(path);
  if (!size || size>16*1024*1024) throw std::invalid_argument("runtime_urdf_size");
  std::ifstream stream(path,std::ios::binary);std::string bytes(std::size_t(size),'\0');
  if (!stream.read(bytes.data(),std::streamsize(bytes.size())) || stream.peek()!=std::char_traits<char>::eof() ||
      std::filesystem::file_size(path)!=size || std::filesystem::last_write_time(path)!=time || dm::sha256(bytes)!=hash)
    throw std::runtime_error("runtime_urdf_changed_or_sha_mismatch");
  return bytes;
}
}  // namespace

SpatialQueryPolicy load_spatial_query_policy(rclcpp::Node& node) {
  SpatialQueryPolicy policy;
  policy.mode=parameter<std::string>(node,"spatial_query_mode","full-region-v1");
  if (policy.height_limited()) {
    policy.collision_min_height_m=parameter<double>(node,"collision_min_height_m",-.75);
    policy.collision_max_height_m=parameter<double>(node,"collision_max_height_m",2.);
  }
  detail::validate_spatial_query_policy(policy);return policy;
}
RuntimeConfig load_runtime_config(rclcpp::Node& node) {
  RuntimeConfig out;out.enabled=parameter<bool>(node,"enabled",false);
  if (!out.enabled) return out;
  out.camera_maximum_age_ns=sunhorizon_obstacle_coverage::camera_freshness_age_ns(
    parameter<std::string>(node,"camera_freshness_profile",sunhorizon_obstacle_coverage::kStrictCameraFreshness));
  if(out.camera_maximum_age_ns>sv::kMaximumEvidenceAgeNs &&
      parameter<std::string>(node,"kinematics_tf_mode","")!="attended_measured_direct")
    throw std::invalid_argument("extended_camera_age_requires_measured_direct_runtime");
  if (node.get_parameter("use_sim_time").as_bool()) throw std::invalid_argument("spatial_node_identity_or_clock");
  out.contract=parameter<std::string>(node,"contract","");
  out.query_digest=parameter<std::string>(node,"query_digest","");
  out.producer_epoch=parameter<std::string>(node,"producer_epoch","");
  out.clock_boot_id=parameter<std::string>(node,"clock_boot_id","");
  out.common_frame=parameter<std::string>(node,"common_frame","");
  if (!dm::sha_valid(out.contract) || !dm::sha_valid(out.query_digest))
    throw std::invalid_argument("spatial_contract_requires_sha256");
  identity(out.producer_epoch);identity(out.clock_boot_id);identity(out.common_frame);
  if(out.camera_maximum_age_ns>sv::kMaximumEvidenceAgeNs &&
      parameter<double>(node,"rgbd.freshness_timeout_sec",.3)!=double(out.camera_maximum_age_ns)/1e9)
    throw std::invalid_argument("extended_camera_age_requires_matching_rgbd_health_deadline");
  out.query_policy=load_spatial_query_policy(node);
  std::ifstream boot_file("/proc/sys/kernel/random/boot_id");std::string actual_boot;std::getline(boot_file,actual_boot);
  if (actual_boot!=out.clock_boot_id) throw std::invalid_argument("spatial_boot_mismatch");
  const auto tolerance=parameter<double>(node,"self_tolerance_m",0.);
  const auto margin=parameter<double>(node,"clear_margin_m",0.);
  if (!std::isfinite(tolerance) || tolerance<=0. || tolerance>sunhorizon_obstacle_coverage::dynamic_self_geometry::kSelfSurfaceToleranceM || !std::isfinite(margin) || margin<0.)
    throw std::invalid_argument("spatial_tolerance_or_margin");
  const auto mins=parameter<std::vector<double>>(node,"cell_min",{});
  const auto maxs=parameter<std::vector<double>>(node,"cell_max",{});
  if (mins.empty() || mins.size()!=maxs.size() || mins.size()%3 || mins.size()/3>sv::kMaximumCells)
    throw std::invalid_argument("spatial_cell_count");
  std::vector<SpatialQueryBox> original_boxes;
  for (std::size_t i=0;i<mins.size()/3;++i) {
    sv::AxisAlignedCell cell;cell.id=uint32_t(i+1);
    for (std::size_t j=0;j<3;++j) {
      cell.min[j]=mins[i*3+j];cell.max[j]=maxs[i*3+j];
      if (!std::isfinite(cell.min[j]) || !std::isfinite(cell.max[j]) || cell.min[j]>=cell.max[j])
        throw std::invalid_argument("spatial_cell_bounds");
    }
    out.query.cells.push_back(cell);
    original_boxes.push_back({cell.min,cell.max});
  }
  const auto canonical=detail::spatial_query_canonical(out.query_policy,out.common_frame,original_boxes);
  const auto effective=detail::effective_spatial_boxes(out.query_policy,out.common_frame,original_boxes);
  for (std::size_t i=0;i<effective.size();++i) {
    out.query.cells[i].min=effective[i].min;out.query.cells[i].max=effective[i].max;
  }
  if (dm::sha256(canonical)!=out.query_digest) throw std::invalid_argument("spatial_query_digest_mismatch");
  out.query.common_frame=out.common_frame;out.query.query_id=out.query_digest;out.query.contract_id=out.contract;
  const auto paths=parameter<std::vector<std::string>>(node,"model_mesh_paths",{});
  const auto hashes=parameter<std::vector<std::string>>(node,"model_mesh_sha256",{});
  const auto counts=parameter<std::vector<int64_t>>(node,"model_triangle_counts",{});
  const auto links=parameter<std::vector<std::string>>(node,"model_link_names",{});
  const auto parents=parameter<std::vector<std::string>>(node,"model_parent_frames",{});
  const auto boundary_begin=parameter<std::vector<int64_t>>(node,"model_boundary_triangle_begin",std::vector<int64_t>(12,0));
  const auto boundary_count=parameter<std::vector<int64_t>>(node,"model_boundary_triangle_count",std::vector<int64_t>(12,0));
  const auto boundary_tolerance=parameter<std::vector<double>>(node,"model_boundary_tolerance_m",std::vector<double>(12,0.));
  const auto measured_begin=parameter<std::vector<int64_t>>(node,"model_measured_triangle_begin",std::vector<int64_t>(12,0));
  const auto measured_count=parameter<std::vector<int64_t>>(node,"model_measured_triangle_count",std::vector<int64_t>(12,0));
  if (paths.size()!=12 || hashes.size()!=12 || counts.size()!=12 || links.size()!=12 || parents.size()!=12)
    throw std::invalid_argument("spatial_model_manifest_length");
  if(boundary_begin.size()!=12 || boundary_count.size()!=12 || boundary_tolerance.size()!=12)
    throw std::invalid_argument("spatial_model_boundary_manifest_length");
  if(measured_begin.size()!=12 || measured_count.size()!=12)
    throw std::invalid_argument("spatial_model_measured_manifest_length");
  std::set<std::string> unique_links;std::size_t total_triangles=0;
  for (std::size_t i=0;i<12;++i) {
    identity(links[i]);identity(parents[i]);
    if (!unique_links.insert(links[i]).second || links[i]==parents[i] || !dm::sha_valid(hashes[i]) ||
        counts[i]<=0 || counts[i]>1000000 || total_triangles>1000000-std::size_t(counts[i]))
      throw std::invalid_argument("spatial_model_triangle_count_or_identity");
    total_triangles+=std::size_t(counts[i]);
    if(boundary_begin[i]<0 || boundary_count[i]<0)throw std::invalid_argument("spatial_model_boundary_negative_range");
    if(measured_begin[i]<0 || measured_count[i]<0)throw std::invalid_argument("spatial_model_measured_negative_range");
    dm::AssetSpec spec{paths[i],hashes[i],std::size_t(counts[i]),links[i],parents[i],
      std::size_t(boundary_begin[i]),std::size_t(boundary_count[i]),boundary_tolerance[i],
      std::size_t(measured_begin[i]),std::size_t(measured_count[i])};
    if(!dm::boundary_policy_valid(spec))throw std::invalid_argument("spatial_model_boundary_authorization");
    if(!dm::measured_policy_valid(spec))throw std::invalid_argument("spatial_model_measured_authorization");
    out.model_specs.push_back(std::move(spec));
  }
  out.include_chest=parameter<bool>(node,"include_chest",false);
  for (std::size_t i=0;i<out.sources.size();++i) {
    out.sources[i]=source_config(node,i,tolerance,margin);
    if (i==0 || out.include_chest)
      out.query.source_contracts.push_back({out.sources[i].name,out.sources[i].epoch,i==0});
  }
  if(out.sources[0].engine.body_stereo_occlusion_enabled &&
      (out.sources[0].engine.raw_projection_enabled||out.sources[1].engine.raw_projection_enabled))
    throw std::invalid_argument("stereo_body_occlusion_requires_both_raw_disabled");
  out.urdf_path=parameter<std::string>(node,"urdf_path","");
  out.urdf_sha256=parameter<std::string>(node,"urdf_sha256","");
  out.joint_kinematics.urdf_xml=checked_urdf(out.urdf_path,out.urdf_sha256);
  auto& joints=out.joint_kinematics;
  joints.expected_joint_names=parameter<std::vector<std::string>>(node,"kinematics_expected_joint_names",{});
  if (joints.expected_joint_names.size()!=13 || std::set<std::string>(joints.expected_joint_names.begin(),
      joints.expected_joint_names.end()).size()!=joints.expected_joint_names.size())
    throw std::invalid_argument("runtime_requires_13_expected_joint_names");
  for (const auto& name:joints.expected_joint_names) identity(name);
  joints.target_frames=links;
  joints.base_frame=parameter<std::string>(node,"kinematics_base_frame","base_link");
  const auto startup_options=load_joint_startup_options(node);
  const std::string required_root=startup_options.tf_mode!="production_direct" ? "base_link" : "base_footprint";
  joints.root_frame=parameter<std::string>(node,"kinematics_root_frame",required_root);
  if (joints.root_frame!=required_root)
    throw std::invalid_argument("runtime_joint_root_does_not_match_explicit_tf_mode");
  identity(joints.base_frame);identity(joints.root_frame);
  if (joints.base_frame!="base_link") throw std::invalid_argument("runtime_model_requires_base_link_fk");
  joints.cache_time_ns=parameter<int64_t>(node,"kinematics_cache_time_ns",2000000000LL);
  joints.max_history_samples=count_parameter(node,"kinematics_max_history_samples",512,4096,false);
  joints.maximum_age_ns=parameter<int64_t>(node,"kinematics_maximum_age_ns",250000000LL);
  joints.future_tolerance_ns=parameter<int64_t>(node,"kinematics_future_tolerance_ns",20000000LL);
  const auto joint_age_limit_ns = out.camera_maximum_age_ns == sunhorizon_obstacle_coverage::kOneSecondCameraAgeNs
      ? 500000000LL : sv::kMaximumEvidenceAgeNs;
  if (joints.cache_time_ns<=0 || joints.cache_time_ns>10000000000LL ||
      joints.maximum_age_ns<=0 || joints.maximum_age_ns>joint_age_limit_ns ||
      joints.future_tolerance_ns<0 || joints.future_tolerance_ns>50000000LL)
    throw std::invalid_argument("invalid_runtime_joint_cache_or_age");
  const auto validated=validate_joint_startup(startup_options,out.urdf_path,out.urdf_sha256);
  if (validated.output_joint_names!=joints.expected_joint_names ||
      std::set<std::string>(validated.leg_target_frames.begin(),validated.leg_target_frames.end())!=
      std::set<std::string>(links.begin(),links.end()) ||
      validated.waist_source_joint_name!="waist_yaw_joint" || validated.waist_output_joint_name!="waist_yaw")
    throw std::invalid_argument("runtime_joint_model_mapping_or_targets_mismatch");
  // Only FK sees the merged model. Mesh loading, canonical asset identity and
  // archived evidence retain the original manufacturer URDF's bytes and hash.
  joints.urdf_xml=validated.merged_urdf_xml;
  joints.source_receipt_sha256=startup_options.source_receipt_sha256;
  joints.mapping_receipt_sha256=startup_options.mapping_receipt_sha256;
  joints.production_tf_receipt_sha256=startup_options.production_tf_receipt_sha256;
  joints.tf_mode=startup_options.tf_mode;
  joints.attended_usage=startup_options.attended_usage;
  joints.attended_tf_asset_sha256=startup_options.attended_tf_asset_sha256;
  joints.attended_tf_receipt_sha256=startup_options.attended_tf_receipt_sha256;
  joints.attended_static_edges=validated.attended_static_edges;
  joints.legacy_urdf_sha256=out.urdf_sha256;
  joints.merged_urdf_sha256=validated.merged_urdf_sha256;
  return out;
}

bool runtime_camera_info_matches(const RuntimeSourceConfig& source,const sensor_msgs::msg::CameraInfo& info) {
  if (info.header.frame_id!=source.frame || !info.width || !info.height || info.width>640 || info.height>480 ||
      !std::isfinite(info.k[0]) || info.k[0]<=0. || !std::isfinite(info.k[4]) || info.k[4]<=0. ||
      !std::isfinite(info.k[2]) || !std::isfinite(info.k[5])) return false;
  try {if (sunhorizon_rgbd_adapter::region_calibration_contract(info)!=source.calibration) return false;}
  catch (const std::exception&) {return false;}
  // Legacy body projection uses K under its own explicit accepted contract.
  // Matching an original distorted CameraInfo cannot grant spatial pinhole
  // qualification, but must not silently change that accepted body's pixels.
  if (!source.engine.coverage_enabled) return source.projection_accepted;
  if (
      !(info.distortion_model=="plumb_bob" || info.distortion_model=="rational_polynomial") ||
      info.binning_x>1 || info.binning_y>1 || info.roi.do_rectify ||
      info.roi.x_offset || info.roi.y_offset || info.roi.width || info.roi.height) return false;
  for (const double value:info.d) if (value!=0.) return false;
  for (std::size_t i=0;i<9;++i) if (info.r[i]!=(i%4==0?1.:0.)) return false;
  if (info.k[1]!=0. || info.k[3]!=0. || info.k[6]!=0. || info.k[7]!=0. || info.k[8]!=1. ||
      !std::isfinite(info.k[0]) || info.k[0]<=0. || !std::isfinite(info.k[4]) || info.k[4]<=0.) return false;
  for (std::size_t i=0;i<3;++i) {
    for (std::size_t j=0;j<3;++j) if (info.p[i*4+j]!=info.k[i*3+j]) return false;
    if (info.p[i*4+3]!=0.) return false;
  }
  return true;
}
}  // namespace sunhorizon_perception_runtime
