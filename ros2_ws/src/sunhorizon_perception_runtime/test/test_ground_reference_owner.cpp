#include "sunhorizon_perception_runtime/ground_reference_owner.hpp"
#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include <filesystem>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace g=sunhorizon_perception_runtime::ground_relative;
using Json=nlohmann::json;
namespace {
std::size_t checks=0;
void check(bool value){++checks;if(!value)throw std::runtime_error("ground owner check "+std::to_string(checks));}
template<class F>void rejects(F fn){bool rejected=false;try{fn();}catch(const std::exception&){rejected=true;}check(rejected);}
std::string sha(const std::string& value){
  unsigned char bytes[32];unsigned int length=0;check(EVP_Digest(value.data(),value.size(),bytes,&length,EVP_sha256(),nullptr)==1&&length==32);
  std::ostringstream out;out<<std::hex<<std::setfill('0');for(auto b:bytes)out<<std::setw(2)<<unsigned(b);return out.str();
}
// Synthetic fixture only. These deliberately convenient numbers are not a
// robot calibration, production profile, or a physical qualification claim.
Json fixture(){
  Json c={{"ground_band_min_m",-.01},{"ground_band_max_m",.01},{"maximum_support_residual_m",.05},
    {"maximum_point_error_m",.1},{"maximum_transform_translation_error_m",.1},{"maximum_transform_angle_error_rad",.2},
    {"maximum_plane_offset_error_m",.1},{"maximum_plane_angle_error_rad",.2},{"maximum_normal_tilt_rad",.2},
    {"rotation_tolerance",1.e-12},{"normal_norm_tolerance",1.e-12},{"minimum_support_span_x_m",.1},
    {"minimum_support_span_y_m",.1},{"minimum_support_double_area_m2",.005},{"minimum_samples_per_cell",1},
    {"minimum_training_cells",3},{"minimum_holdout_cells",3},{"continuous_support_radius_cells",1},
    {"maximum_points",1000},{"maximum_support_cells",100},{"maximum_support_checks",10000}};
  Json cells=Json::array();for(unsigned y=0;y<5;++y)for(unsigned x=0;x<5;++x)cells.push_back({{"x",x},{"y",y}});
  return {{"schema","ground-reference-owner-v1"},{"mode","observe"},{"clock_boot_id","synthetic-boot"},{"classifier",c},
    {"reference",{{"provenance_id","synthetic-independent"},{"calibration_id","synthetic-ground"},{"common_frame","odom"},
      {"source_ns",900000000},{"first_receipt_ros_ns",901000000},{"first_receipt_steady_ns",4900000000LL},
      {"expiry_budget_ns",600000000},{"independently_qualified",false},{"unit_normal",{0.,0.,1.}},
      {"anchor",{0.,0.,-.9}},{"plane_offset_error_bound_m",0.},{"plane_angle_error_bound_rad",0.},
      {"support",{{"origin_x_m",0.},{"origin_y_m",0.},{"resolution_m",.1},{"width",5},{"height",5},{"required_cells",cells}}}}},
    {"sources",Json::array({{{"source_id","belly"},{"epoch","camera-epoch"},{"calibration_id","measured-calibration"},
      {"source_frame","optical"},{"measurement_provenance_id","synthetic-measurement"},{"measurement_qualified",false},{"source_time_transform_qualified",false},
      {"point_error_bound_m",0.},{"transform_translation_error_bound_m",0.},{"transform_angle_error_bound_rad",0.}}})}};
}
g::SourceFrame frame(){
  g::SourceFrame s;s.identity={"belly","camera-epoch","measured-calibration","optical","odom",42,
    1000000000,1001000000,5000000000LL,300000000};
  s.to_common.source_frame="optical";s.to_common.common_frame="odom";
  s.to_common.source_ns=s.to_common.common_ns=s.identity.source_ns;s.to_common.qualified=true;
  for(unsigned y=0;y<5;++y)for(unsigned x=0;x<5;++x)s.points.push_back({{(x+.5)*.1,(y+.5)*.1,-.9},g::ReturnKind::Measured});
  return s;
}
struct TempProfile {
  std::filesystem::path path;
  TempProfile():path(std::filesystem::canonical(std::filesystem::temp_directory_path())/std::filesystem::path("ground-owner-test-"+std::to_string(
    std::chrono::steady_clock::now().time_since_epoch().count())+".json")){}
  ~TempProfile(){std::filesystem::remove(path);}
  g::GroundReferenceOwner::ConstSharedPtr load(const Json& doc,
      std::optional<g::RegistrationMapIdentity> actual=std::nullopt,const std::string& boot="synthetic-boot"){
    const auto bytes=doc.dump();{std::ofstream file(path);file<<bytes;}
    return g::GroundReferenceOwner::load(path.string(),sha(bytes),boot,actual);
  }
};
void conservative(const g::ClassifiedObstacleFrame::ConstSharedPtr& value){
  check(!value->ground().safety_qualified);check(value->clearing_indices().empty());
  check(value->marking_indices().empty());check(value->protected_hit_indices().size()==25);
  for(const auto point:value->classes())check(point==g::PointClass::Uncertain);
}
void map_assets(){
  // Synthetic PCD identity/bounds, never a measured robot profile.
  const g::RegistrationMapIdentity actual{std::string(64,'b'),"synthetic-pcd-datum"};
  auto doc=fixture();doc["schema"]="ground-reference-map-asset-v1";doc.erase("clock_boot_id");
  doc["asset"]={{"pcd_sha256",actual.pcd_sha256},{"datum_id",actual.datum_id}};
  auto& reference=doc["reference"];reference["common_frame"]="registration_map";
  for(const auto* key:{"source_ns","first_receipt_ros_ns","first_receipt_steady_ns","expiry_budget_ns"})reference.erase(key);
  doc["sources"][0].erase("epoch");
  auto s=frame();s.identity.common_frame=s.to_common.common_frame="registration_map";
  auto reset=std::make_shared<std::atomic<bool>>(false);
  constexpr int64_t ros_deadline=1301000000,steady_deadline=5300000000LL;
  auto alignment=std::make_shared<g::RegistrationMapAlignment>(actual,7,reset,ros_deadline,steady_deadline);
  g::RegistrationMapSourceBinding binding{alignment,s.identity.source_id,s.identity.epoch,s.identity.source_ns};
  TempProfile file;
  rejects([&]{file.load(doc);});
  rejects([&]{file.load(doc,g::RegistrationMapIdentity{std::string(64,'c'),actual.datum_id});});
  rejects([&]{file.load(doc,g::RegistrationMapIdentity{actual.pcd_sha256,"public-planar-map"});});
  auto owner=file.load(doc,actual,"entirely-new-boot");check(owner->registration_map_asset());
  check(owner->clock_boot_id().empty());check(owner->map_identity().pcd_sha256==actual.pcd_sha256);
  const auto observed=owner->classify(s,binding);conservative(observed);
  check(observed->ground().candidate_ground_points==25);
  check(observed->reference()->source_ns==0&&observed->reference()->first_receipt_ros_ns==0&&
    observed->reference()->first_receipt_steady_ns==0&&observed->reference()->expiry_budget_ns==0);
  conservative(owner->classify(s)); // Loading the asset does not create alignment.
  auto wrong=binding;wrong.source_epoch="older-producer";conservative(owner->classify(s,wrong));
  wrong=binding;--wrong.source_ns;conservative(owner->classify(s,wrong));
  wrong=binding;wrong.alignment=std::make_shared<g::RegistrationMapAlignment>(
    g::RegistrationMapIdentity{std::string(64,'c'),actual.datum_id},7);
  conservative(owner->classify(s,wrong));
  auto broken=doc;broken["clock_boot_id"]="old-boot";rejects([&]{file.load(broken,actual);});
  broken=doc;broken["reference"]["source_ns"]=1;rejects([&]{file.load(broken,actual);});
  broken=doc;broken["sources"][0]["epoch"]="old-epoch";rejects([&]{file.load(broken,actual);});
  broken=doc;broken["reference"]["common_frame"]="map";rejects([&]{file.load(broken,actual);});
  auto unbounded=doc;unbounded.erase("classifier");
  for(const auto* key:{"plane_offset_error_bound_m","plane_angle_error_bound_rad","independently_qualified"})
    unbounded["reference"].erase(key);
  for(const auto* key:{"point_error_bound_m","transform_translation_error_bound_m","transform_angle_error_bound_rad",
      "measurement_qualified","source_time_transform_qualified","measurement_provenance_id"})unbounded["sources"][0].erase(key);
  unbounded["reference"]["plane_offset_error_bound_m"]=nullptr;
  unbounded["sources"][0]["point_error_bound_m"]=nullptr;
  owner=file.load(unbounded,actual,"");auto geometry=owner->classify(s,binding);conservative(geometry);
  check(geometry->ground().result==g::Qualification::Unbounded);
  check(geometry->ground().reference_geometry_valid&&!geometry->ground().support_valid);
  check(geometry->ground().candidate_ground_points==0&&geometry->ground().candidate_uncertain_points==25);
  for(std::size_t i=0;i<25;++i){check(std::abs(geometry->relative_heights_m()[i])<1.e-12);check(std::isnan(geometry->error_bounds_m()[i]));}
  check(geometry->source().identity.first_receipt_ros_ns==s.identity.first_receipt_ros_ns);
  check(geometry->source().identity.first_receipt_steady_ns==s.identity.first_receipt_steady_ns);
  check(std::isnan(geometry->reference()->plane_offset_error_bound_m));
  check(std::isnan(geometry->source().point_error_bound_m));
  broken=unbounded;broken["classifier"]={{"maximum_point_error_m",-.1}};
  rejects([&]{file.load(broken,actual);});
  broken=unbounded;broken["sources"][0]["point_error_bound_m"]=-.1;
  rejects([&]{file.load(broken,actual);});
  broken=unbounded;broken["mode"]="qualified";rejects([&]{file.load(broken,actual);});
  // Reset permanently invalidates old objects, including a newly wrapped old
  // snapshot carrying the same revoked manager token.
  reset->store(true);conservative(owner->classify(s,binding));
  auto rewrapped=std::make_shared<g::RegistrationMapAlignment>(actual,7,reset,ros_deadline,steady_deadline);
  wrong=binding;wrong.alignment=rewrapped;check(!rewrapped->current());conservative(owner->classify(s,wrong));
  alignment=std::make_shared<g::RegistrationMapAlignment>(actual,8,std::make_shared<std::atomic<bool>>(false),ros_deadline,steady_deadline);
  binding.alignment=alignment;check(owner->classify(s,binding)->ground().result==g::Qualification::Unbounded);
  alignment->revoke();check(!alignment->current());conservative(owner->classify(s,binding));
  // Static assets do not prohibit qualified operation when genuine explicit
  // evidence is supplied. These test hashes/bounds are synthetic only.
  doc["mode"]="qualified";doc["reference"]["independently_qualified"]=true;
  doc["sources"][0]["measurement_qualified"]=true;doc["sources"][0]["source_time_transform_qualified"]=true;
  doc["qualification"]={{"approved_usage","production-ground-classification"}};
  for(const auto* key:{"independent_ground_evidence_sha256","measurement_error_evidence_sha256","transform_error_evidence_sha256",
    "clock_alignment_evidence_sha256","support_selection_evidence_sha256"})doc["qualification"][key]=std::string(64,'a');
  owner=file.load(doc,actual,"second-reboot");
  binding.alignment=std::make_shared<g::RegistrationMapAlignment>(actual,9,nullptr,ros_deadline,steady_deadline);
  s.identity.epoch="new-camera-after-reboot";binding.source_epoch=s.identity.epoch;
  s.identity.first_receipt_steady_ns=123456789;
  const auto qualified=owner->classify(s,binding);check(qualified->ground().safety_qualified);
  check(qualified->clearing_indices().size()==25);check(qualified->reference()->first_receipt_steady_ns==0);
  // Immutable map geometry does not make a dynamic registration immortal.
  // These synthetic absolute limits must cover acquisition/receipt. A shorter
  // registration lease is carried separately to consumers, never enlarged to
  // cover the sensor's future budget.
  s=frame();s.identity.common_frame=s.to_common.common_frame="registration_map";
  binding.source_epoch=s.identity.epoch;
  for(const auto limits:{std::pair<int64_t,int64_t>{s.identity.first_receipt_ros_ns-1,steady_deadline},
      {ros_deadline,s.identity.first_receipt_steady_ns-1},{0,0}}) {
    binding.alignment=std::make_shared<g::RegistrationMapAlignment>(actual,10,nullptr,limits.first,limits.second);
    const auto expired=owner->classify(s,binding);conservative(expired);
    check(expired->ground().result==g::Qualification::ReferenceExpired);
  }
  binding.alignment=std::make_shared<g::RegistrationMapAlignment>(actual,11,nullptr,
    s.identity.first_receipt_ros_ns+1000000,s.identity.first_receipt_steady_ns+1000000);
  check(owner->classify(s,binding)->ground().safety_qualified);
  auto delayed=s;delayed.identity.first_receipt_ros_ns+=1000001;
  conservative(owner->classify(delayed,binding));
  s.identity.expiry_budget_ns=std::numeric_limits<int64_t>::max();
  binding.alignment=std::make_shared<g::RegistrationMapAlignment>(actual,12,nullptr,
    std::numeric_limits<int64_t>::max(),std::numeric_limits<int64_t>::max());
  const auto long_sensor_lease=owner->classify(s,binding);
  check(long_sensor_lease->ground().safety_qualified);
  check(long_sensor_lease->source().identity.expiry_budget_ns==std::numeric_limits<int64_t>::max());
  doc["mode"]="observe";owner=file.load(doc,actual);
  s=frame();s.identity.common_frame=s.to_common.common_frame="registration_map";
  binding.alignment=std::make_shared<g::RegistrationMapAlignment>(actual,13);
  const auto untimed=owner->classify(s,binding);conservative(untimed);
  check(untimed->ground().result==g::Qualification::Unbounded);
  check(untimed->ground().candidate_ground_points==0);
  for(const auto h:untimed->relative_heights_m())check(std::abs(h)<1.e-12);
}
void supported_segmentation(){
  TempProfile file;auto doc=fixture();doc["schema"]="ground-reference-map-asset-v1";
  doc["mode"]="supported-segmentation";doc.erase("clock_boot_id");doc.erase("classifier");
  const g::RegistrationMapIdentity actual{std::string(64,'b'),"synthetic-registration-map"};
  doc["asset"]={{"pcd_sha256",actual.pcd_sha256},{"datum_id",actual.datum_id}};
  auto& r=doc["reference"];r["common_frame"]="registration_map";
  for(const auto* key:{"source_ns","first_receipt_ros_ns","first_receipt_steady_ns","expiry_budget_ns"})r.erase(key);
  r["plane_offset_error_bound_m"]=nullptr;r["plane_angle_error_bound_rad"]=nullptr;
  auto& binding=doc["sources"][0];binding.erase("epoch");binding.erase("measurement_provenance_id");
  for(const auto* key:{"point_error_bound_m","transform_translation_error_bound_m","transform_angle_error_bound_rad"})binding[key]=nullptr;
  doc["segmentation"]={{"maximum_point_residual_m",.04},{"maximum_cell_median_residual_m",.03},
    {"maximum_cell_mad_m",.015},{"maximum_partition_median_difference_m",.02},{"maximum_normal_tilt_rad",.2},
    {"minimum_support_span_x_m",.1},{"minimum_support_span_y_m",.1},{"minimum_support_double_area_m2",.005},
    {"minimum_samples_per_partition",1},{"minimum_supported_cells",6},{"continuous_support_radius_cells",1},
    {"maximum_points",4000},{"maximum_support_cells",100},{"maximum_support_checks",100000}};
  auto owner=file.load(doc,actual,"");check(owner->supported_segmentation()&&!owner->observation_only());
  check(!owner->clearance_grid());check(std::isnan(owner->config().maximum_point_error_m));
  // Retired compensation cannot be silently re-enabled by an old owner bundle.
  for(const auto value:{true,false}) {
    auto retired=doc;retired["segmentation"]["reconcile_model_conflicts"]=value;
    rejects([&]{file.load(retired,actual,"");});
  }
  auto input=frame();input.identity.common_frame=input.to_common.common_frame="registration_map";
  input.measurement_qualified=false;input.point_error_bound_m=input.transform_translation_error_bound_m=
    input.transform_angle_error_bound_rad=g::kUnspecified;
  const auto originals=input.points;input.points.clear();
  for(const auto& p:originals)for(unsigned i=0;i<16;++i){auto point=p;point.sample_id=input.points.size();input.points.push_back(point);}
  auto reset=std::make_shared<std::atomic<bool>>(false);
  g::RegistrationMapSourceBinding map_binding{
    std::make_shared<g::RegistrationMapAlignment>(actual,17,reset,1301000000,5300000000LL),
    input.identity.source_id,input.identity.epoch,input.identity.source_ns};
  const auto result=owner->classify(input,map_binding);
  check(result->ground().safety_qualified&&result->ground().basis==g::GroundQualification::Basis::MeasuredSupport);
  check(result->clearing_indices().size()==400&&!result->source().measurement_qualified);
  check(!result->reference()->independently_qualified&&!result->reference()->source_time_transform_qualified);
  check(std::isnan(result->source().point_error_bound_m)&&std::isnan(result->source().transform_translation_error_bound_m));
  check(std::isnan(result->reference()->plane_offset_error_bound_m));
  for(const auto value:result->error_bounds_m())check(std::isnan(value));
  reset->store(true);check(!owner->classify(input,map_binding)->ground().safety_qualified);
  auto wrong=input;wrong.identity.source_frame=wrong.to_common.source_frame="wrong";
  check(!owner->classify(wrong,map_binding)->ground().safety_qualified);
  rejects([&]{file.load(doc);});
  auto broken=doc;broken["segmentation"]["maximum_point_residual_m"]=-.01;
  rejects([&]{file.load(broken,actual);});
  broken=doc;broken["mode"]="qualified";rejects([&]{file.load(broken,actual);});
  broken=fixture();broken["mode"]="supported-segmentation";rejects([&]{file.load(broken);});
}
void run(){
  TempProfile file;auto doc=fixture();auto owner=file.load(doc);check(owner->reference_frame()=="odom");check(owner->observation_only());
  const auto observed=owner->classify(frame());conservative(observed);
  check(observed->ground().result==g::Qualification::ObserveOnly);check(observed->ground().support_valid);
  check(observed->ground().candidate_ground_points==25);check(observed->ground().candidate_obstacle_points==0);
  check(!observed->reference()->independently_qualified);check(observed->reference()->source_ns==900000000);
  check(observed->reference()->first_receipt_ros_ns==901000000);check(observed->reference()->first_receipt_steady_ns==4900000000LL);
  check(observed->reference()->valid_for_source_ns==1000000000);check(observed->source().identity.first_receipt_ros_ns==1001000000);
  {std::ofstream changed(file.path);changed<<"{}";}
  // The immutable owner never reopens a changed pathname during classification.
  const auto retained=owner->classify(frame());conservative(retained);check(retained->ground().candidate_ground_points==25);
  for(const auto* field:{"epoch","calibration_id","source_id"}){
    auto s=frame();if(std::string(field)=="epoch")s.identity.epoch="wrong";
    else if(std::string(field)=="source_id")s.identity.source_id="airy";else s.identity.calibration_id="wrong";
    const auto result=owner->classify(s);conservative(result);check(result->ground().candidate_ground_points==0);
  }
  auto s=frame();s.to_common.qualified=false;conservative(owner->classify(s));
  s=frame();s.to_common.common_ns+=1;conservative(owner->classify(s));
  s=frame();s.identity.first_receipt_ros_ns+=300000000;s.identity.first_receipt_steady_ns+=300000000;
  const auto expired=owner->classify(s);conservative(expired);check(expired->ground().result==g::Qualification::ReferenceExpired);
  s=frame();s.identity.first_receipt_steady_ns=4800000000LL;conservative(owner->classify(s));
  s=frame();s.identity.common_frame="base_link";conservative(owner->classify(s));
  s=frame();s.identity.source_frame=s.to_common.source_frame="wrong_optical";conservative(owner->classify(s));
  auto broken=doc;broken["classifier"].erase("maximum_point_error_m");rejects([&]{file.load(broken);});
  broken=doc;broken["clock_boot_id"]="another-boot";rejects([&]{file.load(broken);});
  broken=doc;broken["reference"]["first_receipt_ros_ns"]=1;rejects([&]{file.load(broken);});
  broken=doc;broken["mode"]="qualified";rejects([&]{file.load(broken);});
  broken=doc;broken["classifier"]["rotation_tolerance"]=0.;rejects([&]{file.load(broken);});
  broken=doc;broken["reference"]["common_frame"]="base_link";rejects([&]{file.load(broken);});
  broken=doc;broken["classifier"]["minimum_training_cells"]=100;rejects([&]{file.load(broken);});
  broken=doc;broken["reference"]["support"]["required_cells"].push_back({{"x",0},{"y",0}});rejects([&]{file.load(broken);});
  owner=file.load(doc);rejects([&]{g::GroundReferenceOwner::load(file.path.string(),std::string(64,'0'),"synthetic-boot");});
  check(!g::GroundReferenceOwner::load("","","synthetic-boot"));
  rejects([&]{g::GroundReferenceOwner::load("",std::string(64,'0'),"synthetic-boot");});
  doc["mode"]="qualified";doc["reference"]["independently_qualified"]=true;
  doc["sources"][0]["measurement_qualified"]=true;doc["sources"][0]["source_time_transform_qualified"]=true;
  doc["qualification"]={{"approved_usage","production-ground-classification"}};
  for(const auto* key:{"independent_ground_evidence_sha256","measurement_error_evidence_sha256","transform_error_evidence_sha256",
    "clock_alignment_evidence_sha256","support_selection_evidence_sha256"})doc["qualification"][key]=std::string(64,'a');
  owner=file.load(doc);const auto qualified=owner->classify(frame());check(qualified->ground().safety_qualified);check(qualified->clearing_indices().size()==25);
  // Reference age is evaluated on every frame; successful prior frames cannot renew it.
  conservative(owner->classify(s=[]{auto value=frame();value.identity.first_receipt_ros_ns+=300000000;return value;}()));
  s=frame();s.points[0].position[2]=-.74;auto obstacle=owner->classify(s);check(obstacle->clearing_indices().size()<25);
  check(!obstacle->ground().cells.empty());
  doc["mode"]="observe";owner=file.load(doc);const auto still_observe=owner->classify(frame());conservative(still_observe);
  check(still_observe->ground().candidate_ground_points==25);
  // Owner binding cannot renew a near-expiry reference to match fresh source
  // receipt clocks. Both modes require enough remaining reference lifetime for
  // the complete source lease, with an exact nanosecond boundary.
  for(const auto* mode:{"observe","qualified"}){
    auto boundary=doc;boundary["mode"]=mode;
    boundary["reference"]["expiry_budget_ns"]=401000000;
    owner=file.load(boundary);const auto exact=owner->classify(frame());
    check(exact->ground().result==(std::string(mode)=="observe"?g::Qualification::ObserveOnly:g::Qualification::Qualified));
    boundary["reference"]["expiry_budget_ns"]=400999999;
    owner=file.load(boundary);const auto too_short=owner->classify(frame());conservative(too_short);
    check(too_short->ground().result==g::Qualification::ReferenceExpired);
    boundary["reference"]["expiry_budget_ns"]=101000001;
    owner=file.load(boundary);const auto near_expiry=owner->classify(frame());conservative(near_expiry);
    check(near_expiry->ground().result==g::Qualification::ReferenceExpired);
    check(near_expiry->reference()->first_receipt_ros_ns==901000000);
    check(near_expiry->reference()->first_receipt_steady_ns==4900000000LL);
  }
}
}
int main(){try{run();map_assets();supported_segmentation();std::cout<<"ground reference owner "<<checks<<" checks passed\n";return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
