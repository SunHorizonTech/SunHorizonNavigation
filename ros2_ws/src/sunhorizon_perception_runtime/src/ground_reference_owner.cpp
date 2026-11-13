#include "sunhorizon_perception_runtime/ground_reference_owner.hpp"

#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>

namespace sunhorizon_perception_runtime::ground_relative {
namespace {
using Json=nlohmann::json;
void require(bool condition,const char* reason) {
  if(!condition)throw std::invalid_argument(std::string("ground_reference_")+reason);
}
bool sha_valid(const std::string& value) {
  return value.size()==64&&std::all_of(value.begin(),value.end(),[](char c){return (c>='0'&&c<='9')||(c>='a'&&c<='f');});
}
std::string digest(const std::string& bytes) {
  unsigned char hash[EVP_MAX_MD_SIZE];unsigned int size=0;
  require(EVP_Digest(bytes.data(),bytes.size(),hash,&size,EVP_sha256(),nullptr)==1&&size==32,"sha_failure");
  std::ostringstream out;out<<std::hex<<std::setfill('0');
  for(unsigned int i=0;i<size;++i)out<<std::setw(2)<<static_cast<unsigned int>(hash[i]);
  return out.str();
}
std::string text(const Json& object,const char* key,std::size_t limit=16384) {
  const auto value=object.at(key).get<std::string>();
  require(!value.empty()&&value.size()<=limit&&std::all_of(value.begin(),value.end(),
    [](unsigned char c){return c>=33&&c<=126;}),"invalid_text");return value;
}
double number(const Json& object,const char* key) {
  require(object.at(key).is_number(),"number_required");
  const auto result=object.at(key).get<double>();require(std::isfinite(result),"nonfinite_number");return result;
}
double optional_number(const Json& object,const char* key,bool allow_unspecified) {
  if(allow_unspecified&&(!object.contains(key)||object.at(key).is_null()))return kUnspecified;
  return number(object,key);
}
int64_t clock(const Json& object,const char* key) {
  require(object.at(key).is_number_integer(),"integer_clock_required");
  const auto value=object.at(key).get<int64_t>();require(value>0,"positive_clock_required");return value;
}
std::size_t count(const Json& object,const char* key,std::size_t maximum) {
  require(object.at(key).is_number_integer(),"integer_count_required");
  const auto value=object.at(key).get<int64_t>();require(value>0&&static_cast<uint64_t>(value)<=maximum,"count_limit");
  return static_cast<std::size_t>(value);
}
std::array<double,3> vector3(const Json& object,const char* key) {
  const auto& values=object.at(key);require(values.is_array()&&values.size()==3,"vector3_required");
  std::array<double,3> result{};
  for(std::size_t i=0;i<3;++i){require(values[i].is_number(),"vector_number_required");result[i]=values[i].get<double>();require(std::isfinite(result[i]),"vector_nonfinite");}
  return result;
}
void validate_partial_config(const ClassifierConfig& c) {
  constexpr double pi=3.14159265358979323846;
  const double nonnegative[]={c.maximum_support_residual_m,c.maximum_point_error_m,
    c.maximum_transform_translation_error_m,c.maximum_transform_angle_error_rad,
    c.maximum_plane_offset_error_m,c.maximum_plane_angle_error_rad,c.maximum_normal_tilt_rad};
  for(const auto value:nonnegative)require(std::isnan(value)||value>=0.,"negative_explicit_bound");
  require((std::isnan(c.ground_band_min_m)||c.ground_band_min_m<=0.)&&
    (std::isnan(c.ground_band_max_m)||c.ground_band_max_m>=0.)&&
    (std::isnan(c.ground_band_min_m)||std::isnan(c.ground_band_max_m)||c.ground_band_min_m<c.ground_band_max_m),"ground_band");
  for(const auto value:{c.rotation_tolerance,c.normal_norm_tolerance})
    require(std::isnan(value)||(value>0.&&value<=1.e-6),"numeric_tolerance");
  for(const auto value:{c.minimum_support_span_x_m,c.minimum_support_span_y_m,c.minimum_support_double_area_m2})
    require(std::isnan(value)||value>0.,"support_geometry_bound");
  for(const auto value:{c.maximum_transform_angle_error_rad,c.maximum_plane_angle_error_rad})
    require(std::isnan(value)||value<=pi,"angle_bound");
  require((std::isnan(c.maximum_normal_tilt_rad)||c.maximum_normal_tilt_rad<pi/2.)&&
    (!c.minimum_training_cells||c.minimum_training_cells>=3)&&
    (!c.minimum_holdout_cells||c.minimum_holdout_cells>=3),"support_configuration");
}
}
GroundReferenceOwner::ConstSharedPtr GroundReferenceOwner::load(const std::string& filename,
  const std::string& expected_sha,const std::string& boot_id,
  const std::optional<RegistrationMapIdentity>& actual_registration_map) {
  if(filename.empty()&&expected_sha.empty())return {};
  require(!filename.empty()&&sha_valid(expected_sha),"file_requires_sha256");
  const std::filesystem::path path(filename);
  require(path.is_absolute()&&std::filesystem::canonical(path)==path&&
    std::filesystem::is_regular_file(std::filesystem::symlink_status(path)),"canonical_regular_file_required");
  const auto size=std::filesystem::file_size(path);const auto time=std::filesystem::last_write_time(path);
  require(size>0&&size<=1024*1024,"file_size");
  std::ifstream stream(path,std::ios::binary);std::string bytes(static_cast<std::size_t>(size),'\0');
  require(bool(stream.read(bytes.data(),static_cast<std::streamsize>(bytes.size())))&&
    stream.peek()==std::char_traits<char>::eof()&&std::filesystem::file_size(path)==size&&
    std::filesystem::last_write_time(path)==time&&digest(bytes)==expected_sha,"file_changed_or_sha_mismatch");
  // Reject duplicate keys rather than silently accepting the last qualifier.
  std::vector<std::set<std::string>> keys;
  const auto doc=Json::parse(bytes,[&](int,Json::parse_event_t event,Json& parsed){
    if(event==Json::parse_event_t::object_start)keys.emplace_back();
    if(event==Json::parse_event_t::key)require(keys.back().insert(parsed.get<std::string>()).second,"duplicate_json_key");
    if(event==Json::parse_event_t::object_end)keys.pop_back();
    return true;
  });
  const auto schema=text(doc,"schema");
  const bool map_asset=schema=="ground-reference-map-asset-v1";
  require(map_asset||schema=="ground-reference-owner-v1","schema");
  const auto mode=text(doc,"mode");
  const bool segmentation=mode=="supported-segmentation";
  require(mode=="observe"||mode=="qualified"||segmentation,"mode");
  require(!segmentation||map_asset,"segmentation_requires_map_asset");
  if(!map_asset)require(!boot_id.empty()&&text(doc,"clock_boot_id")==boot_id,"boot_mismatch");
  else require(!doc.contains("clock_boot_id"),"map_asset_has_no_boot_clock");
  auto owner=std::shared_ptr<GroundReferenceOwner>(new GroundReferenceOwner);
  owner->observation_only_=mode=="observe";
  owner->supported_segmentation_=segmentation;
  if(!map_asset)owner->clock_boot_id_=boot_id;
  const auto& j=doc.at("reference");auto& ref=owner->reference_;
  const bool allow_unspecified=map_asset&&(owner->observation_only_||segmentation);
  if(map_asset) {
    ref.lifetime=GroundReference::Lifetime::RegistrationMapAsset;
    const auto& asset=doc.at("asset");
    ref.map_asset={text(asset,"pcd_sha256",64),text(asset,"datum_id",128)};
    require(sha_valid(ref.map_asset.pcd_sha256)&&actual_registration_map&&
      actual_registration_map->pcd_sha256==ref.map_asset.pcd_sha256&&
      actual_registration_map->datum_id==ref.map_asset.datum_id,"actual_registration_map_mismatch");
    for(const auto* key:{"source_ns","first_receipt_ros_ns","first_receipt_steady_ns","expiry_budget_ns"})
      require(!j.contains(key),"map_asset_has_no_observation_clocks");
  }
  ref.provenance_id=text(j,"provenance_id");ref.calibration_id=text(j,"calibration_id");
  ref.common_frame=text(j,"common_frame",128);
  // A fixed plane cannot be loaded in a moving robot/body frame. Additional
  // fixed-frame conventions require an explicit schema revision and review.
  require(map_asset?ref.common_frame=="registration_map":
    (ref.common_frame=="odom"||ref.common_frame=="map"),"fixed_reference_frame_required");
  if(!map_asset) {
    ref.source_ns=clock(j,"source_ns");ref.first_receipt_ros_ns=clock(j,"first_receipt_ros_ns");
    ref.first_receipt_steady_ns=clock(j,"first_receipt_steady_ns");ref.expiry_budget_ns=clock(j,"expiry_budget_ns");
    require(ref.first_receipt_ros_ns>=ref.source_ns,"reference_clock_order");
  }
  ref.independently_qualified=allow_unspecified?j.value("independently_qualified",false):j.at("independently_qualified").get<bool>();
  ref.unit_normal=vector3(j,"unit_normal");ref.anchor=vector3(j,"anchor");
  ref.plane_offset_error_bound_m=optional_number(j,"plane_offset_error_bound_m",allow_unspecified);
  ref.plane_angle_error_bound_rad=optional_number(j,"plane_angle_error_bound_rad",allow_unspecified);
  auto& c=owner->config_;const auto config=allow_unspecified?doc.value("classifier",Json::object()):doc.at("classifier");
#define GROUND_NUMBER(name) c.name=optional_number(config,#name,allow_unspecified)
  GROUND_NUMBER(ground_band_min_m);GROUND_NUMBER(ground_band_max_m);GROUND_NUMBER(maximum_support_residual_m);
  GROUND_NUMBER(maximum_point_error_m);GROUND_NUMBER(maximum_transform_translation_error_m);
  GROUND_NUMBER(maximum_transform_angle_error_rad);GROUND_NUMBER(maximum_plane_offset_error_m);
  GROUND_NUMBER(maximum_plane_angle_error_rad);GROUND_NUMBER(maximum_normal_tilt_rad);
  GROUND_NUMBER(rotation_tolerance);GROUND_NUMBER(normal_norm_tolerance);
  GROUND_NUMBER(minimum_support_span_x_m);GROUND_NUMBER(minimum_support_span_y_m);GROUND_NUMBER(minimum_support_double_area_m2);
#undef GROUND_NUMBER
  const auto config_count=[&](const char* key,std::size_t maximum) {
    if(allow_unspecified&&(!config.contains(key)||config.at(key).is_null()))return std::size_t(0);
    return count(config,key,maximum);
  };
  c.minimum_samples_per_cell=config_count("minimum_samples_per_cell",kMaximumClassifiedPoints);
  c.minimum_training_cells=config_count("minimum_training_cells",kMaximumSupportCells);
  c.minimum_holdout_cells=config_count("minimum_holdout_cells",kMaximumSupportCells);
  c.continuous_support_radius_cells=static_cast<uint32_t>(config_count("continuous_support_radius_cells",32));
  c.maximum_points=config_count("maximum_points",kMaximumClassifiedPoints);
  c.maximum_support_cells=config_count("maximum_support_cells",kMaximumSupportCells);
  c.maximum_support_checks=config_count("maximum_support_checks",16*1024*1024);
  if(allow_unspecified)validate_partial_config(c);
  if(segmentation) {
    const auto& item=doc.at("segmentation");auto& s=owner->segmentation_config_;
    require(!item.contains("reconcile_model_conflicts"),"retired_ground_reconciliation_parameter");
#define SEGMENT_NUMBER(name) s.name=number(item,#name)
    SEGMENT_NUMBER(maximum_point_residual_m);SEGMENT_NUMBER(maximum_cell_median_residual_m);
    SEGMENT_NUMBER(maximum_cell_mad_m);SEGMENT_NUMBER(maximum_partition_median_difference_m);
    SEGMENT_NUMBER(maximum_normal_tilt_rad);SEGMENT_NUMBER(minimum_support_span_x_m);
    SEGMENT_NUMBER(minimum_support_span_y_m);SEGMENT_NUMBER(minimum_support_double_area_m2);
#undef SEGMENT_NUMBER
    s.minimum_samples_per_partition=count(item,"minimum_samples_per_partition",kMaximumClassifiedPoints);
    s.minimum_supported_cells=count(item,"minimum_supported_cells",kMaximumSupportCells);
    s.continuous_support_radius_cells=static_cast<uint32_t>(count(item,"continuous_support_radius_cells",32));
    s.maximum_points=count(item,"maximum_points",kMaximumClassifiedPoints);
    s.maximum_support_cells=count(item,"maximum_support_cells",kMaximumSupportCells);
    s.maximum_support_checks=count(item,"maximum_support_checks",16*1024*1024);
  }
  const auto& support=j.at("support");auto& grid=ref.support;
  grid.origin_x_m=number(support,"origin_x_m");grid.origin_y_m=number(support,"origin_y_m");
  grid.resolution_m=number(support,"resolution_m");
  grid.width=static_cast<uint32_t>(count(support,"width",4096));grid.height=static_cast<uint32_t>(count(support,"height",4096));
  require(grid.resolution_m>0.&&std::isfinite(grid.origin_x_m+grid.resolution_m*grid.width)&&
    std::isfinite(grid.origin_y_m+grid.resolution_m*grid.height),"grid_geometry");
  const auto& cells=support.at("required_cells");require(cells.is_array()&&!cells.empty()&&
    cells.size()<=(c.maximum_support_cells?c.maximum_support_cells:kMaximumSupportCells),"support_cells");
  std::set<std::pair<uint32_t,uint32_t>> unique_cells;
  for(const auto& cell:cells){
    require(cell.at("x").is_number_integer()&&cell.at("y").is_number_integer(),"cell_integer");
    const auto x=cell.at("x").get<int64_t>(),y=cell.at("y").get<int64_t>();
    require(x>=0&&y>=0&&x<grid.width&&y<grid.height,"cell_bounds");
    const auto pair=std::make_pair(static_cast<uint32_t>(x),static_cast<uint32_t>(y));
    require(unique_cells.insert(pair).second,"duplicate_cell");grid.required_cells.push_back({pair.first,pair.second});
  }
  const auto training=std::count_if(grid.required_cells.begin(),grid.required_cells.end(),
    [](const SupportCell& cell){return (cell.x+cell.y)%2==0;});
  require(static_cast<std::size_t>(training)>=c.minimum_training_cells&&
    grid.required_cells.size()-static_cast<std::size_t>(training)>=c.minimum_holdout_cells,"insufficient_selected_support_cells");
  const auto normal_length=std::hypot(ref.unit_normal[0],ref.unit_normal[1],ref.unit_normal[2]);
  const auto bound_valid=[&](double value,double maximum) {
    if(allow_unspecified&&std::isnan(value))return true;
    return std::isfinite(value)&&value>=0.&&
      ((allow_unspecified&&std::isnan(maximum))||value<=maximum);
  };
  require(std::abs(normal_length-1.)<=(std::isfinite(c.normal_norm_tolerance)?c.normal_norm_tolerance:1.e-6)&&
    ref.unit_normal[2]>0.&&((allow_unspecified&&std::isnan(c.maximum_normal_tilt_rad))||
    ref.unit_normal[2]/normal_length>=std::cos(c.maximum_normal_tilt_rad))&&
    bound_valid(ref.plane_offset_error_bound_m,c.maximum_plane_offset_error_m)&&
    bound_valid(ref.plane_angle_error_bound_rad,c.maximum_plane_angle_error_rad),"plane_geometry_or_error");
  require(std::isnan(ref.plane_angle_error_bound_rad)||ref.plane_angle_error_bound_rad<=3.14159265358979323846,"plane_angle_bound");
  const auto& sources=doc.at("sources");require(sources.is_array()&&!sources.empty()&&sources.size()<=16,"source_bindings");
  for(const auto& source:sources){
    const auto id=text(source,"source_id",128);Binding b;
    if(map_asset)require(!source.contains("epoch"),"map_asset_has_no_producer_epoch");
    else b.epoch=text(source,"epoch",128);
    b.calibration_id=text(source,"calibration_id");
    b.source_frame=text(source,"source_frame",128);
    if(!allow_unspecified||source.contains("measurement_provenance_id"))b.measurement_provenance_id=text(source,"measurement_provenance_id");
    b.measurement_qualified=allow_unspecified?source.value("measurement_qualified",false):source.at("measurement_qualified").get<bool>();
    b.transform_qualified=allow_unspecified?source.value("source_time_transform_qualified",false):source.at("source_time_transform_qualified").get<bool>();
    b.point_error_m=optional_number(source,"point_error_bound_m",allow_unspecified);
    b.translation_error_m=optional_number(source,"transform_translation_error_bound_m",allow_unspecified);
    b.angle_error_rad=optional_number(source,"transform_angle_error_bound_rad",allow_unspecified);
    require(bound_valid(b.point_error_m,c.maximum_point_error_m)&&
      bound_valid(b.translation_error_m,c.maximum_transform_translation_error_m)&&
      bound_valid(b.angle_error_rad,c.maximum_transform_angle_error_rad),"source_error_bounds");
    require(std::isnan(b.angle_error_rad)||b.angle_error_rad<=3.14159265358979323846,"source_angle_bound");
    // Exercise the classifier's full config validator without granting any source credit.
    SourceFrame probe;probe.identity={id,map_asset?"probe":b.epoch,b.calibration_id,"probe",ref.common_frame,1,1,1,1,1};
    probe.point_error_bound_m=b.point_error_m;probe.transform_translation_error_bound_m=b.translation_error_m;
    probe.transform_angle_error_bound_rad=b.angle_error_rad;
    if(segmentation)(void)classify_supported_ground_frame(std::move(probe),owner->segmentation_config_,ref);
    else (void)classify_obstacle_frame(std::move(probe),c,ref,owner->observation_only_&&map_asset);
    if(!owner->observation_only_&&!segmentation)
      require(b.measurement_qualified&&b.transform_qualified,"qualified_source_evidence_required");
    require(owner->bindings_.emplace(id,std::move(b)).second,"duplicate_source_binding");
  }
  if(!owner->observation_only_&&!segmentation){
    require(ref.independently_qualified,"independent_qualification_required");
    const auto& evidence=doc.at("qualification");
    require(text(evidence,"approved_usage")=="production-ground-classification","qualification_usage");
    for(const auto* key:{"independent_ground_evidence_sha256","measurement_error_evidence_sha256",
      "transform_error_evidence_sha256","clock_alignment_evidence_sha256","support_selection_evidence_sha256"})
      require(sha_valid(text(evidence,key)),"qualification_evidence_sha256");
  }
  if(doc.contains("near_ground_clearance")&&!segmentation) {
    const auto& item=doc.at("near_ground_clearance");near_ground_clearance::TileGrid grid;
    grid.contract_id=text(item,"contract_id",128);
    grid.origin_x_m=number(item,"origin_x_m");grid.origin_y_m=number(item,"origin_y_m");
    grid.resolution_m=number(item,"resolution_m");
    grid.width=count(item,"width",4096);grid.height=count(item,"height",4096);
    grid.coarse_origin_z_m=number(item,"coarse_origin_z_m");
    grid.coarse_resolution_z_m=number(item,"coarse_resolution_z_m");
    require(item.at("coarse_z_index").is_number_integer(),"coarse_z_index_integer");
    const auto zi=item.at("coarse_z_index").get<int64_t>();require(zi>=0&&zi<16,"coarse_z_index_range");
    grid.coarse_z_index=static_cast<uint32_t>(zi);
    grid.maximum_tiles=count(item,"maximum_tiles",near_ground_clearance::sv::kMaximumCells);
    SourceFrame probe;probe.identity={"grid-probe","probe","probe","probe",ref.common_frame,1,1,1,1,1};
    const auto frame=conservative_unqualified_frame(std::move(probe));
    (void)near_ground_clearance::make_query(*frame,c,grid); // Validates hard geometry/budget even without qualification.
    owner->clearance_grid_=std::move(grid);
  }
  return owner;
}
ClassifiedObstacleFrame::ConstSharedPtr GroundReferenceOwner::classify(SourceFrame source,
    RegistrationMapSourceBinding map_binding) const {
  auto ref=reference_;
  const bool map_asset=ref.lifetime==GroundReference::Lifetime::RegistrationMapAsset;
  if(map_asset)ref.map_binding=std::move(map_binding);
  const auto found=bindings_.find(source.identity.source_id);
  if(found!=bindings_.end()){
    const auto& b=found->second;
    ref.expected_source_id=found->first;
    ref.expected_source_epoch=map_asset?ref.map_binding.source_epoch:b.epoch;
    ref.expected_source_calibration_id=b.calibration_id;
    if(!supported_segmentation_) {
      source.point_error_bound_m=b.point_error_m;source.transform_translation_error_bound_m=b.translation_error_m;
      source.transform_angle_error_bound_rad=b.angle_error_rad;
      source.measurement_qualified=b.measurement_qualified&&source.identity.source_frame==b.source_frame;
      ref.source_time_transform_qualified=b.transform_qualified;
    }
    if(source.identity.source_frame!=b.source_frame)source.to_common.qualified=false;
  }else source.measurement_qualified=false;
  // These are the only reference time fields bound to this frame. Baseline
  // acquisition/receipt clocks and expiry above remain immutable evidence.
  ref.valid_for_source_ns=source.identity.source_ns;ref.valid_for_common_ns=source.to_common.common_ns;
  source.to_common.qualified=source.to_common.qualified&&source.to_common.common_ns==source.identity.source_ns;
  if(supported_segmentation_)
    return classify_supported_ground_frame(std::move(source),segmentation_config_,std::move(ref));
  return classify_obstacle_frame(std::move(source),config_,std::move(ref),observation_only_);
}
}
