#include "sunhorizon_perception_runtime/runtime_config.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

#include "sunhorizon_rgbd_adapter/region_depth_producer.hpp"

using namespace sunhorizon_perception_runtime;
namespace dm = sunhorizon_rgbd_adapter::dynamic_self_model;
namespace {
int checks=0;
#define CHECK(x) do {++checks;if (!(x)) throw std::runtime_error(#x);} while (false)
template<class Function> void rejects(Function&& function) {
  bool rejected=false;try {function();} catch (const std::exception&) {rejected=true;}CHECK(rejected);
}
struct Files {
  std::filesystem::path directory;
  std::string urdf,stl;
  Files() {
    directory=std::filesystem::temp_directory_path()/("perception-config-test-"+
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(directory);directory=std::filesystem::canonical(directory);
    urdf="<robot name=\"fixture\"><link name=\"base_link\"/></robot>";
    std::ofstream(directory/"robot.urdf",std::ios::binary)<<urdf;
    stl.assign(134,'\0');stl[80]=1;
    const float vertices[]{0,0,1,1,0,1,0,1,1};
    std::memcpy(stl.data()+96,vertices,sizeof(vertices));
    std::ofstream output(directory/"mesh.stl",std::ios::binary);output.write(stl.data(),std::streamsize(stl.size()));
  }
  ~Files() {std::error_code error;std::filesystem::remove_all(directory,error);}
};
sensor_msgs::msg::CameraInfo info(const std::string& source) {
  sensor_msgs::msg::CameraInfo value;value.header.frame_id=source+"_optical";
  value.width=value.height=3;value.distortion_model="plumb_bob";value.d.assign(5,0.);
  value.k={10,0,1,0,10,1,0,0,1};value.r={1,0,0,0,1,0,0,0,1};
  value.p={10,0,1,0,0,10,1,0,0,0,1,0};return value;
}
using Parameters=std::vector<rclcpp::Parameter>;
void replace(Parameters& parameters,const rclcpp::Parameter& replacement) {
  const auto found=std::find_if(parameters.begin(),parameters.end(),
      [&](const rclcpp::Parameter& value){return value.get_name()==replacement.get_name();});
  if (found==parameters.end()) parameters.push_back(replacement);else *found=replacement;
}
Parameters parameters(const Files& files) {
  std::ifstream boot_file("/proc/sys/kernel/random/boot_id");std::string boot;std::getline(boot_file,boot);
  if (boot.empty()) throw std::runtime_error("test_runtime_config_requires_linux_boot_id");
  std::string canonical="spatial-query-v1;";
  auto append=[&](const std::string& value){canonical+=std::to_string(value.size())+":"+value;};
  append("base_footprint");
  for (const auto pair:{std::pair<double,double>{-.1,.1},{-.1,.1},{.3,.4}}) {
    sunhorizon_rgbd_adapter::RegionCanonical numbers;numbers.number(pair.first).number(pair.second);append(numbers.str());
  }
  const std::vector<double> cell_min{-.1,-.1,.3},cell_max{.1,.1,.4};
  const std::vector<std::string> paths(12,(files.directory/"mesh.stl").string()),hashes(12,dm::sha256(files.stl)),parents(12,"base_link");
  const std::vector<int64_t> counts(12,1);
  std::vector<std::string> links,joints;
  for (unsigned i=0;i<12;++i) links.push_back("link_"+std::to_string(i));
  for (unsigned i=0;i<13;++i) joints.push_back("joint_"+std::to_string(i));
  Parameters values{
    {"enabled",true},
    {"contract",std::string(64,'a')},{"query_digest",dm::sha256(canonical)},
    {"producer_epoch","producer-1"},{"clock_boot_id",boot},{"common_frame","base_footprint"},
    {"self_tolerance_m",.03},{"clear_margin_m",.02},{"cell_min",cell_min},{"cell_max",cell_max},
    {"model_mesh_paths",paths},{"model_mesh_sha256",hashes},{"model_triangle_counts",counts},
    {"model_link_names",links},{"model_parent_frames",parents},{"include_chest",false},
    {"urdf_path",(files.directory/"robot.urdf").string()},{"urdf_sha256",dm::sha256(files.urdf)},
    {"kinematics_expected_joint_names",joints}};
  for (const std::string source:{"belly","chest"}) {
    values.emplace_back(source+"_frame",source+"_optical");
    values.emplace_back(source+"_epoch",source+"-epoch");
    values.emplace_back(source+"_calibration",sunhorizon_rgbd_adapter::region_calibration_contract(info(source)));
    values.emplace_back(source+"_depth_scale_m",.001);
    values.emplace_back(source+"_minimum_depth_m",.2);
    values.emplace_back(source+"_maximum_depth_m",8.);
    values.emplace_back(source+"_body_enabled",true);
    values.emplace_back(source+"_projection_accepted",true);
    for (const auto* flag:{"rectified_pinhole_verified","metric_axial_depth_verified","depth_intrinsics_registration_verified",
        "optical_to_base_extrinsics_verified","source_time_and_transform_verified"}) values.emplace_back(source+"_"+flag,true);
  }
  values.emplace_back("belly_self_filter_enabled",true);
  values.emplace_back("belly_self_exclusion_mode","dynamic-mesh-v1");
  return values;
}
RuntimeConfig load(const Parameters& values) {
  rclcpp::Node node("test_perception_runtime_config",rclcpp::NodeOptions().use_global_arguments(false)
      .parameter_overrides(values).start_parameter_services(false).start_parameter_event_publisher(false));
  return load_runtime_config(node);
}
void run() {
  SpatialQueryPolicy limited;limited.mode="planar-collision-height-v1";
  const std::vector<SpatialQueryBox> region{{{.2,.15,-.8},{.3,.25,-.7}}};
  CHECK(dm::sha256(detail::spatial_query_canonical({},"base_footprint",region))==
    "d6a95477214b1c62a412f6185b82cf28bea6577a0e6c9bfd7a1f6fbe30c73f17");
  CHECK(dm::sha256(detail::spatial_query_canonical(limited,"base_footprint",region))==
    "0816359e21dedcb32fcf052556cf87318333d042c57b85930b7dfa56db6f09b5");
  rclcpp::Node policy_node("height_policy",rclcpp::NodeOptions().use_global_arguments(false)
    .parameter_overrides({{"spatial_query_mode","planar-collision-height-v1"},
      {"collision_min_height_m",-.75},{"collision_max_height_m",2.}}));
  CHECK(load_spatial_query_policy(policy_node)==limited);
  CHECK(policy_node.describe_parameter("spatial_query_mode").read_only);
  CHECK(policy_node.describe_parameter("collision_min_height_m").read_only);
  CHECK(!policy_node.set_parameter({"collision_min_height_m",-.74}).successful);
  CHECK(!load({}).enabled);
  Files files;const auto values=parameters(files);
  dm::AssetSpec off;off.triangle_count=1;CHECK(dm::boundary_policy_valid(off));
  CHECK(dm::measured_policy_valid(off));
  auto measured=off;measured.measured_triangle_count=1;CHECK(dm::measured_policy_valid(measured));
  measured.measured_triangle_begin=1;CHECK(!dm::measured_policy_valid(measured));
  measured.measured_triangle_count=0;CHECK(!dm::measured_policy_valid(measured));
  measured=off;measured.measured_triangle_count=2;CHECK(!dm::measured_policy_valid(measured));
  measured=off;measured.measured_triangle_begin=std::numeric_limits<std::size_t>::max();
  measured.measured_triangle_count=1;CHECK(!dm::measured_policy_valid(measured));
  auto edge=off;edge.boundary_triangle_count=1;edge.boundary_tolerance_m=.003;
  CHECK(dm::boundary_policy_valid(edge));edge.boundary_triangle_begin=1;CHECK(!dm::boundary_policy_valid(edge));
  edge.boundary_triangle_begin=0;edge.boundary_tolerance_m=.00301;CHECK(!dm::boundary_policy_valid(edge));
  edge.boundary_tolerance_m=0.;CHECK(!dm::boundary_policy_valid(edge));
  edge=off;edge.boundary_tolerance_m=.001;CHECK(!dm::boundary_policy_valid(edge));
  edge=off;edge.boundary_tolerance_m=std::numeric_limits<double>::quiet_NaN();CHECK(!dm::boundary_policy_valid(edge));
  // A syntactically plausible manifest and accept booleans cannot authorize a
  // fake robot URDF or missing signed source/mapping/production receipts.
  rejects([&]{load(values);});
  RuntimeSourceConfig qualified;
  qualified.name="chest";qualified.frame="chest_optical";
  qualified.calibration=sunhorizon_rgbd_adapter::region_calibration_contract(info("chest"));
  qualified.engine.coverage_enabled=true;qualified.projection_accepted=true;
  CHECK(runtime_camera_info_matches(qualified,info("chest")));
  auto wrong=info("chest");wrong.k[0]=11.;
  CHECK(!runtime_camera_info_matches(qualified,wrong));
  wrong=info("chest");wrong.d[0]=.01;CHECK(!runtime_camera_info_matches(qualified,wrong));
  wrong=info("chest");wrong.roi.do_rectify=true;CHECK(!runtime_camera_info_matches(qualified,wrong));
  auto changed=values;replace(changed,{"include_chest",true});
  rejects([&]{load(changed);});
  changed=values;replace(changed,{"chest_source_time_and_transform_verified",false});
  rejects([&]{load(changed);});
  auto original_info=info("chest");original_info.d[0]=.01;
  auto original_body=qualified;original_body.engine.coverage_enabled=false;
  original_body.calibration=sunhorizon_rgbd_adapter::region_calibration_contract(original_info);
  CHECK(runtime_camera_info_matches(original_body,original_info));
  changed=values;replace(changed,{"belly_source_time_and_transform_verified",false});rejects([&]{load(changed);});
  changed=values;replace(changed,{"query_digest",std::string(64,'b')});rejects([&]{load(changed);});
  changed=values;replace(changed,{"clock_boot_id","wrong-boot"});rejects([&]{load(changed);});
  changed=values;replace(changed,{"urdf_sha256",std::string(64,'b')});rejects([&]{load(changed);});
  changed=values;replace(changed,{"chest_component_filter_enabled",true});rejects([&]{load(changed);});
  changed=values;replace(changed,{"belly_pointcloud_pixel_stride",int64_t(0)});rejects([&]{load(changed);});
  changed=values;replace(changed,{"belly_projection_accepted",false});rejects([&]{load(changed);});
  changed=values;replace(changed,{"belly_self_exclusion_mode","aabb-v1"});rejects([&]{load(changed);});
  changed=values;replace(changed,{"chest_max_removed_fraction",.11});rejects([&]{load(changed);});
  changed=values;replace(changed,{"kinematics_future_tolerance_ns",int64_t(50000000LL)});
  rejects([&]{load(changed);});
  replace(changed,{"kinematics_future_tolerance_ns",int64_t(50000001LL)});rejects([&]{load(changed);});
  std::vector<dm::AssetSpec> specs;
  for (unsigned i=0;i<12;++i) specs.push_back({(files.directory/"mesh.stl").string(),
    dm::sha256(files.stl),1,"link_"+std::to_string(i),"base_link"});
  dm::Model model(specs);std::vector<dg::RigidTransform> legs(12);dg::RigidTransform camera;
  auto measured_specs=specs;measured_specs[0].measured_triangle_count=1;
  dm::Model measured_model(measured_specs);
  const dg::PreparedModel measured_prepared(measured_model.observations(camera,legs,1000000000LL),dg::QueryBudget{});
  const auto measured_diagnostic=measured_prepared.diagnostic_links();
  CHECK(measured_diagnostic[0].measured_triangle_count==1 && measured_diagnostic[0].measured_triangle_begin==0);
  CHECK(measured_diagnostic[1].measured_triangle_count==0 && measured_diagnostic[1].measured_triangle_begin==0);
  measured_specs[0].measured_triangle_begin=1;rejects([&]{dm::Model invalid(measured_specs);});
  auto boundary_specs=specs;boundary_specs[0].boundary_triangle_count=1;
  boundary_specs[0].boundary_tolerance_m=.003;dm::Model boundary_model(boundary_specs);
  const auto boundary_observations=boundary_model.observations(camera,legs,1000000000LL);
  const dg::PreparedModel boundary_prepared(boundary_observations,dg::QueryBudget{});
  const auto boundary_result=dg::classify({-.002,.2,1},1.,.02,boundary_prepared);
  CHECK(boundary_result.complete && boundary_result.classification==dg::Classification::SelfSurface);
  const auto boundary_diagnostic=boundary_prepared.diagnostic_links();
  CHECK(boundary_diagnostic[0].boundary_triangle_count==1 && boundary_diagnostic[0].boundary_tolerance_m==.003);
  CHECK(boundary_diagnostic[1].boundary_triangle_count==0 && boundary_diagnostic[1].boundary_tolerance_m==0.);
  boundary_specs[0].boundary_tolerance_m=.004;rejects([&]{dm::Model invalid(boundary_specs);});
  camera.translation.x=1.;legs[0].translation.y=2.;
  const auto observations=model.observations(camera,legs,1000000000LL);
  CHECK(observations.size()==12 && observations[0].link_name=="link_0");
  CHECK(observations[0].optical_from_link.translation.x==1. && observations[0].optical_from_link.translation.y==2.);
  CHECK(observations[0].identity_valid && observations[0].tf_valid && observations[0].covers_source_time);
  legs.pop_back();rejects([&]{model.observations(camera,legs,1000000000LL);});legs.resize(12);
  rejects([&]{model.observations(camera,legs,0);});
  camera.rotation[0]=2.;legs[0].rotation[0]=.5;
  rejects([&]{model.observations(camera,legs,1000000000LL);}); // Invalid factors cannot cancel into a rigid product.
}
}  // namespace
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {run();rclcpp::shutdown();std::cout<<"PASS "<<checks<<" runtime config checks\n";return 0;}
  catch (const std::exception& error) {rclcpp::shutdown();std::cerr<<"FAIL "<<checks<<" "<<error.what()<<'\n';return 1;}
}
