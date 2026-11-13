// Offline only. Original CPU production code is linked without a ROS node.
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include <sunhorizon_rgbd_adapter/fused_depth_self_filter.hpp>
#include <sunhorizon_rgbd_adapter/depth_components.hpp>
#ifndef ARM_NOMINAL_CONTRACT
#define ARM_NOMINAL_CONTRACT 0
#endif
namespace dg=sunhorizon_obstacle_coverage::dynamic_self_geometry;
namespace rgb=sunhorizon_rgbd_adapter;
using Json=nlohmann::json;
void need(bool v,const char* why){if(!v)throw std::runtime_error(why);}
Json read(const char* path){std::ifstream f(path);need(bool(f),"input_open_failed");Json j;f>>j;return j;}
std::string sha(const std::string& bytes){unsigned char d[32];unsigned n=0;need(EVP_Digest(bytes.data(),bytes.size(),d,&n,EVP_sha256(),nullptr)==1&&n==32,"sha_failure");std::ostringstream s;for(auto x:d)s<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(x);return s.str();}
template<class T> std::string hashvec(const std::vector<T>& a){return sha(std::string(reinterpret_cast<const char*>(a.data()),a.size()*sizeof(T)));}
double ms(clockid_t c=CLOCK_MONOTONIC){timespec t{};clock_gettime(c,&t);return t.tv_sec*1000.+t.tv_nsec*1e-6;}
Json canonical(const rgb::FusedDepthSelfFilterResult& r){const auto& f=r.filter;
 std::vector<std::size_t> support;std::vector<float> supportxyz;for(auto& s:r.self_support){support.push_back(s.sample_id);supportxyz.insert(supportxyz.end(),s.xyz.begin(),s.xyz.end());}
 return {{"complete",f.complete},{"reason",f.reason},{"input",f.input_count},{"removed",f.removed_count},{"retained",f.retained_count},{"ambiguous",f.measured_reference_ambiguous_count},{"classifications",f.classifications},{"nodes",f.node_visits},{"triangles",f.triangle_tests},{"xyz_sha256",hashvec(r.cloud.data)},{"sample_ids_sha256",hashvec(r.retained_sample_ids)},{"blocked_sha256",hashvec(f.ground_blocked)},{"conflict_sha256",hashvec(f.nominal_model_conflict)},{"support_ids_sha256",hashvec(support)},{"support_xyz_sha256",hashvec(supportxyz)}};
}
int main(int argc,char** argv){try{
 const bool offline_flag=argc==8&&std::string(argv[7])=="--offline-candidate";
 need(argc==7||offline_flag,"audit manifest camera-info raw-depth mesh-directory rounds [--offline-candidate]");
 const auto in=read(argv[1]),manifest=read(argv[2]);
 const auto& model=in.at("body_model");need(!model.is_null(),"same_frame_model_snapshot_required");
 const std::string schema=in.contains("schema")&&in.at("schema").is_string()?in.at("schema").get<std::string>():std::string{};
 const bool offline=schema=="body_replay_offline_fk_v1";
 need(offline==offline_flag,"offline_fk_requires_explicit_mode_no_live_golden_bypass");
 if(offline)need(in.contains("offline_provenance")&&!in.contains("runtime_body"),"offline_fk_provenance_required_no_fake_live_golden");
 const bool live_captured=schema=="body_replay_snapshot_v1";
 const bool captured=live_captured||offline;
 need(captured||(!in.contains("body_filter_config")&&in.contains("classification")),"unsupported_body_replay_schema");
 const auto& identity=captured?in.at("source").at("identity"):in.at("classification").at("source").at("identity");
 const Json cfg=captured?in.at("body_filter_config"):Json::object();
 if(captured)for(const char* key:{"model_link_names","model_mesh_paths","model_mesh_sha256","model_triangle_counts","self_tolerance_m"})need(in.at("model_manifest").at(key)==manifest.at(key),"captured_model_manifest_mismatch");
 const double scale=captured?cfg.at("depth_scale_m").get<double>():.001;
 const double minimum=captured?cfg.at("minimum_depth_m").get<double>():.2;
 const double maximum=captured?cfg.at("maximum_depth_m").get<double>():8.;
 const uint32_t stride=captured?cfg.at("body_stride").get<uint32_t>():2;
 const std::size_t component_min=captured?cfg.at("component_minimum_pixels").get<std::size_t>():200;
 const double neighbor=captured?cfg.at("component_neighbor_delta_m").get<double>():.05;
 const double component_max=captured?cfg.at("component_maximum_depth_m").get<double>():1.;
 const std::size_t near_min=captured?cfg.at("near_component_minimum_pixels").get<std::size_t>():0;
 const double near_max=captured?cfg.at("near_component_maximum_depth_m").get<double>():1.5;
 const bool parallel=captured?cfg.at("body_geometry_parallel_enabled").get<bool>():true;
 const std::string live_backend=captured?cfg.at("body_geometry_compute_backend").get<std::string>():"legacy_not_recorded";
 if(captured)need(live_backend=="cpu"||live_backend=="cuda_arm11","captured_backend_invalid");
 need(std::isfinite(scale)&&scale>0.&&std::isfinite(minimum)&&std::isfinite(maximum)&&minimum>=0.&&maximum>minimum&&stride>0,"captured_projection_config_invalid");
 need(std::isfinite(neighbor)&&neighbor>=0.&&std::isfinite(component_max)&&component_max>=minimum&&std::isfinite(near_max)&&near_max>=0.,"captured_component_config_invalid");
 for(const auto* key:{"source_id","source_frame","epoch","calibration_id","sequence","source_ns","first_receipt_ros_ns","first_receipt_steady_ns"})need(model.at(key)==identity.at(key),"model_source_identity_mismatch");
 need(model.at("role")==identity.at("source_id"),"model_role_mismatch");
 if(captured)need(model.at("expiry_budget_ns")==identity.at("expiry_budget_ns"),"model_expiry_identity_mismatch");
 if(captured)for(const char* key:{"sequence","source_ns","first_receipt_ros_ns","first_receipt_steady_ns","expiry_budget_ns"})need(model.at(key).is_number_integer()&&identity.at(key).is_number_integer(),"exact_integer_identity_required");
 const double tolerance=model.at("self_tolerance_m");need(std::isfinite(tolerance)&&tolerance>=0.,"invalid_tolerance");
 need(manifest.at("self_tolerance_m")==model.at("self_tolerance_m"),"deployment_tolerance_mismatch");
 if(captured)need(cfg.at("self_tolerance_m")==model.at("self_tolerance_m"),"captured_tolerance_mismatch");
 const auto& links=model.at("links");need(links.size()==12&&manifest.at("model_mesh_paths").size()==12,"twelve_links_required");
 dg::QueryBudget budget;const auto& b=model.at("query_budget");budget.max_links=b.at("max_links");budget.max_node_visits=b.at("max_node_visits");budget.max_triangle_tests=b.at("max_triangle_tests");
 std::vector<dg::Mesh> meshes(12);std::vector<dg::LinkObservation> observations;Json assets=Json::array();
 for(std::size_t i=0;i<12;++i){
  const auto& link=links.at(i);need(link.at("link_index")==i,"model_order_mismatch");
  const std::string path=(std::filesystem::path(argv[5])/std::filesystem::path(manifest.at("model_mesh_paths").at(i).get<std::string>()).filename()).string();std::ifstream f(path,std::ios::binary);need(bool(f),"mesh_open_failed");const std::string bytes{std::istreambuf_iterator<char>(f),{}};
  const auto hash=sha(bytes);need(hash==manifest.at("model_mesh_sha256").at(i),"mesh_sha_mismatch");
  const std::size_t triangles=manifest.at("model_triangle_counts").at(i);need(link.at("triangle_count")==triangles,"triangle_count_mismatch");std::string error;
  need(meshes[i].load_binary_stl(reinterpret_cast<const uint8_t*>(bytes.data()),bytes.size(),triangles,{},&error),error.c_str());
  const auto boundary_begin=captured?link.at("boundary_triangle_begin").get<std::size_t>():link.value("boundary_triangle_begin",std::size_t(0));
  const auto boundary_count=captured?link.at("boundary_triangle_count").get<std::size_t>():link.value("boundary_triangle_count",std::size_t(0));
  const auto boundary_tolerance=captured?link.at("boundary_tolerance_m").get<double>():link.value("boundary_tolerance_m",0.);
  if(boundary_begin||boundary_count||boundary_tolerance!=0.)
    need(meshes[i].configure_boundary_patch(boundary_begin,boundary_count,boundary_tolerance,&error),error.c_str());
  need(link.contains("measured_triangle_begin")==link.contains("measured_triangle_count"),"measured_surface_metadata_incomplete");
  const auto measured_begin=captured?link.at("measured_triangle_begin").get<std::size_t>():link.value("measured_triangle_begin",std::size_t(0));
  const auto measured_count=captured?link.at("measured_triangle_count").get<std::size_t>():link.value("measured_triangle_count",std::size_t(0));
  if(measured_begin||measured_count) {
    const bool configured=meshes[i].configure_measured_surface(measured_begin,measured_count,&error);
    need(configured,error.c_str());
  }
  dg::RigidTransform transform;transform.rotation=link.at("optical_from_link").at("rotation").get<std::array<double,9>>();const auto t=link.at("optical_from_link").at("translation").get<std::array<double,3>>();transform.translation={t[0],t[1],t[2]};
  const std::string name=manifest.at("model_link_names").at(i);observations.push_back({&meshes[i],name,transform,true,true,true});
  assets.push_back({{"link_index",i},{"link_name",name},{"sha256",hash},{"triangle_count",triangles},{"captured_mesh_revision",link.at("mesh_revision")},
    {"boundary_triangle_begin",boundary_begin},{"boundary_triangle_count",boundary_count},{"boundary_tolerance_m",boundary_tolerance},
    {"measured_triangle_begin",measured_begin},{"measured_triangle_count",measured_count}});
 }

 const auto info=read(argv[3]);
 dg::PreparedModel prepared(observations,budget);need(prepared.ready(),prepared.reason());
 const auto width=info.at("width").get<std::size_t>(),height=info.at("height").get<std::size_t>();
 need(info.at("header_ns")==identity.at("source_ns")&&info.at("frame_id")==identity.at("source_frame"),"camera_source_identity_mismatch");
 std::ifstream rawfile(argv[4],std::ios::binary);std::vector<uint8_t> raw((std::istreambuf_iterator<char>(rawfile)),{});const auto& primary=captured?in.at("primary_depth"):Json::object();
 const uint32_t step=captured?primary.at("step").get<uint32_t>():uint32_t(width*2);
 const bool bigendian=captured?primary.at("is_bigendian").get<bool>():false;
 need(step>=width*2&&raw.size()==uint64_t(step)*height,"depth_size");
 if(captured){need(primary.at("encoding")=="16UC1"&&primary.at("width")==width&&primary.at("height")==height,"captured_depth_layout_mismatch");
   need(primary.at("header_ns")==identity.at("source_ns")&&primary.at("frame_id")==identity.at("source_frame"),"captured_normalized_depth_identity_mismatch");
   need(primary.at("bytes")==raw.size()&&primary.at("sha256")==sha(std::string(reinterpret_cast<const char*>(raw.data()),raw.size())),"captured_raw_depth_hash_mismatch");
   std::ifstream inf(argv[3],std::ios::binary);const std::string ib{std::istreambuf_iterator<char>(inf),{}};need(primary.at("camera_info_sha256")==sha(ib),"captured_camera_info_hash_mismatch");}
 sensor_msgs::msg::Image image;image.header.frame_id=info.at("frame_id");image.header.stamp.sec=info.at("header_ns").get<int64_t>()/1000000000;image.header.stamp.nanosec=info.at("header_ns").get<int64_t>()%1000000000;image.width=width;image.height=height;image.step=step;image.is_bigendian=bigendian;image.encoding="16UC1";image.data=raw;
 sensor_msgs::msg::CameraInfo camera;camera.header=image.header;camera.width=width;camera.height=height;camera.distortion_model=info.at("distortion_model");camera.k=info.at("k").get<decltype(camera.k)>();camera.r=info.at("r").get<decltype(camera.r)>();camera.p=info.at("p").get<decltype(camera.p)>();camera.d=info.at("d").get<decltype(camera.d)>();
 if(captured){camera.binning_x=info.at("binning_x");camera.binning_y=info.at("binning_y");const auto& roi=info.at("roi");camera.roi.x_offset=roi.at("x_offset");camera.roi.y_offset=roi.at("y_offset");camera.roi.width=roi.at("width");camera.roi.height=roi.at("height");camera.roi.do_rectify=roi.at("do_rectify");}
 need(rgb::qualified_native_depth_raster(image,camera,scale),"native_raster_required_no_attestation_promoted");

 rgb::BodyGeometryCuda backend(true);rgb::BodyGeometryWorker worker([]{});rgb::DepthComponentWorkspace workspace;
 rgb::DynamicFilterBudget limits;limits.query=budget;
 std::vector<uint8_t> captured_keep;
 if(captured){const auto& b=cfg.at("body_budget");limits.max_points=b.at("max_points");limits.max_bytes=b.at("max_bytes");limits.max_node_visits=b.at("max_node_visits");limits.max_triangle_tests=b.at("max_triangle_tests");
   need(b.at("query")==model.at("query_budget"),"captured_model_frame_query_budget_mismatch");
   const auto& pin=in.at("body_keep");need(pin.at("present").get<bool>()==(component_min>0),"captured_component_mask_presence_mismatch");
   if(component_min){const auto path=std::filesystem::path(argv[1]).parent_path()/std::filesystem::path(pin.at("path").get<std::string>()).filename();std::ifstream f(path,std::ios::binary);need(bool(f),"captured_keep_open_failed");captured_keep.assign(std::istreambuf_iterator<char>(f),{});
     need(captured_keep.size()==width*height&&pin.at("bytes")==captured_keep.size()&&pin.at("sha256")==sha(std::string(reinterpret_cast<const char*>(captured_keep.data()),captured_keep.size())),"captured_keep_hash_or_size_mismatch");}
 }
 Json live;
 if(live_captured){const auto& r=in.at("runtime_body");const auto& c=r.contains("canonical")?r.at("canonical"):r;
   for(const char* key:{"complete","reason","input","removed","retained","ambiguous","classifications","nodes","triangles","xyz_sha256","sample_ids_sha256","blocked_sha256","conflict_sha256","support_ids_sha256","support_xyz_sha256"})live[key]=c.at(key);}

 double quality_ms=0.;
 const auto run=[&](bool gpu,const rgb::DynamicFilterBudget& lim){
   const auto quality_start=ms();
   const std::vector<uint8_t>* keep=nullptr;
   if(component_min){const auto& mask=rgb::depth_component_mask({raw.data(),raw.size(),uint32_t(width),uint32_t(height),step,bigendian},scale,minimum,maximum,neighbor,component_min,workspace,near_min,near_max);need(mask.valid,"mask_invalid");
     if(captured){need(mask.keep==captured_keep,"recomputed_component_mask_differs_from_live");keep=&captured_keep;}else keep=&mask.keep;}
   quality_ms=ms()-quality_start;
   return rgb::project_depth_filter_dynamic_self(image,camera,scale,minimum,maximum,stride,prepared,tolerance,lim,keep,component_min?component_max:std::numeric_limits<double>::infinity(),gpu?nullptr:(parallel?&worker:nullptr),gpu?&backend:nullptr,captured&&cfg.value("body_stereo_occlusion_enabled",false)?cfg.at("stereo_body_baseline_m").get<double>():0.);
 };
 Json pairs=Json::array();const unsigned rounds=std::stoul(argv[6]);need(rounds>0&&rounds<=100,"rounds_range");
 for(unsigned i=0;i<rounds;++i){Json pair;
   for(bool gpu:{false,true}){const double wall=ms(),cpu=ms(CLOCK_PROCESS_CPUTIME_ID);const auto result=run(gpu,limits);pair[gpu?"cuda":"cpu"]={{"measurement_body_occluded_samples",std::count(result.filter.ground_blocked.begin(),result.filter.ground_blocked.end(),uint8_t(3))},{"wall_ms",ms()-wall},{"process_cpu_ms",ms(CLOCK_PROCESS_CPUTIME_ID)-cpu},{"output",canonical(result)}};if(gpu){const auto t=backend.last_timings();pair["cuda"]["stages"]={{"quality_ms",quality_ms},{"projection_pack_ms",result.cuda_projection_pack_ms},{"snapshot_ms",t.snapshot_ms},{"cache_check_ms",t.cache_check_ms},{"buffer_prepare_ms",t.buffer_prepare_ms},{"enqueue_ms",t.enqueue_ms},{"wait_ms",t.wait_ms},{"consume_ms",t.consume_ms},{"backend_total_ms",t.total_ms},{"h2d_ms",t.h2d_ms},{"kernel_ms",t.kernel_ms},{"d2h_ms",t.d2h_ms},{"mesh_uploaded",t.mesh_uploaded},{"mesh_bytes",t.mesh_bytes},{"h2d_bytes",t.h2d_bytes},{"d2h_bytes",t.d2h_bytes}};}}
   pair["equal"]=pair["cpu"]["output"]==pair["cuda"]["output"];
   if(live_captured)pair["cpu_matches_live_runtime"]=pair["cpu"]["output"]==live;
   pairs.push_back(pair);
 }
 Json cuts=Json::array();
 for(std::size_t n:{0,1,2,3,5,10,100,1000,10000})for(bool nodes:{false,true}){
   auto b=limits;if(nodes)b.max_node_visits=n;else b.max_triangle_tests=n;
   const auto a=canonical(run(false,b)),c=canonical(run(true,b));cuts.push_back({{"limit",n},{"nodes",nodes},{"equal",a==c},{"cpu",a},{"cuda",c}});
 }
 for(std::size_t n:{1,24}){auto q=budget;q.max_node_visits=n;q.max_triangle_tests=n;prepared=dg::PreparedModel(observations,q);auto b=limits;b.query=q;const auto a=canonical(run(false,b)),c=canonical(run(true,b));cuts.push_back({{"query_limit",n},{"equal",a==c},{"cpu",a},{"cuda",c}});}
 bool equal=true;for(auto& p:pairs)equal=equal&&p["equal"].get<bool>()&&(!live_captured||p["cpu_matches_live_runtime"].get<bool>());for(auto& c:cuts)equal=equal&&c["equal"].get<bool>();
 std::cout<<Json{{"schema",1},{"scope","installed camera runtime BODY backend; original CPU projection/quality included; no ROS owner"},{"source_identity",identity},{"input_schema",captured?in.at("schema"):Json("legacy_belly_hardcoded_v0")},{"validation_mode",offline?"offline_fk_cpu_gpu_no_live_golden":"original_live_or_legacy"},{"captured_config",cfg},{"live_runtime_backend",live_backend},{"cpu_counterpart_parallel_helper",parallel},{"live_runtime_canonical",live},{"assets",assets},{"pairs",pairs},{"budget_cases",cuts},{"all_exact",equal}}.dump(2)<<'\n';return equal?0:2;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
