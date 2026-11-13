#include "sunhorizon_perception_runtime/ground_classification_snapshot.hpp"
#include "sunhorizon_perception_runtime/runtime.hpp"
#include "sunhorizon_perception_runtime/runtime_config.hpp"
#include <cstring>
#include <iostream>
#include <stdexcept>
namespace p=sunhorizon_perception_runtime;
namespace {
void check(bool x){if(!x)throw std::runtime_error("body replay snapshot contract");}
template<class F> void rejects(F f){bool caught=false;try{f();}catch(const std::invalid_argument&){caught=true;}check(caught);}
struct Fixture {
  p::RuntimeConfig config;
  p::PerceptionOutput output;
  std::shared_ptr<p::BodyReplayDiagnostic> replay=std::make_shared<p::BodyReplayDiagnostic>();
  std::shared_ptr<p::BodyModelDiagnostic> model=std::make_shared<p::BodyModelDiagnostic>();
  std::shared_ptr<sunhorizon_camera_runtime::CameraFrame> frame=std::make_shared<sunhorizon_camera_runtime::CameraFrame>();
  Fixture(){
    const int64_t source=1793356673253032000LL;
    model->identity={"chest","epoch","calibration",42,source,source+29000000,8661492804591LL,500000000,false};
    model->source_frame="chest_optical";model->self_tolerance_m=.02;
    config.sources[1].name="chest";config.sources[1].epoch="epoch";config.sources[1].calibration="calibration";
    auto image=std::make_shared<sensor_msgs::msg::Image>();
    image->header.frame_id=model->source_frame;image->header.stamp.sec=source/1000000000;
    image->header.stamp.nanosec=source%1000000000;
    image->width=1;image->height=1;image->step=4;image->encoding="16UC1";
    image->data={0xe8,0x03,0xaa,0xbb}; // Preserve padding, never repack pixels.
    auto info=std::make_shared<sensor_msgs::msg::CameraInfo>();
    info->header=image->header;info->width=1;info->height=1;info->distortion_model="plumb_bob";
    info->k={100,0,0,0,100,0,0,0,1};info->r={1,0,0,0,1,0,0,0,1};
    info->p={100,0,0,0,0,100,0,0,0,0,1,0};info->d={0,0,0,0,0};
    frame->role="chest";frame->sequence=42;frame->source_stamp=image->header.stamp;
    frame->first_sdk_callback_ros_ns=model->identity.receipt_ros_ns;
    frame->first_sdk_callback_steady_ns=model->identity.receipt_steady_ns;
    frame->source_validated=true;frame->depth=image;frame->depth_info=info;
    auto color=std::make_shared<sensor_msgs::msg::Image>();color->header=image->header;
    color->header.stamp.nanosec+=1000;frame->color=color;
    replay->image=image;replay->info=info;replay->model=model;replay->query_ns=source+77;
    replay->config.component_minimum_pixels=65;replay->config.near_component_minimum_pixels=160;
    replay->config.near_component_maximum_depth_m=.6;replay->config.body_stride=2;
    replay->config.self_tolerance_m=.02;replay->config.depth_scale_m=.001;
    replay->parallel_cpu=true;replay->keep_present=true;replay->keep={1};
    replay->retained_sample_ids={9007199254740993ULL};
    replay->self_support.push_back({{-0.f,1.f,2.f},7});
    model->query_budget=replay->config.body_budget.query;
    for(std::size_t i=0;i<12;++i){
      p::dg::PreparedModel::DiagnosticLink link{};link.link_index=i;link.triangle_count=1;link.mesh_revision=i+1;
      link.optical_from_link.rotation={1,0,0,0,1,0,0,0,1};
      link.optical_from_link.translation.x=.12345678901234567;
      model->links.push_back(link);
      sunhorizon_rgbd_adapter::dynamic_self_model::AssetSpec asset;
      asset.path="/pin/"+std::to_string(i)+".stl";asset.sha256=std::string(64,'a');
      asset.triangle_count=1;asset.link_name="link"+std::to_string(i);config.model_specs.push_back(asset);
    }
    output.input=frame;output.frame.body_replay_diagnostic=replay;output.frame.body_model_diagnostic=model;
    auto& f=output.frame.body_filter;f.complete=true;f.reason="filtered";f.input_count=2;f.removed_count=1;f.retained_count=1;
    f.ground_blocked={0};f.nominal_model_conflict={0};f.classifications={0,1,0,0,1};
    auto& cloud=output.frame.body_cloud;cloud.header=image->header;cloud.width=1;cloud.height=1;
    cloud.point_step=12;cloud.row_step=12;cloud.data.resize(12);
    const float xyz[3]={0.f,-0.f,2.f};std::memcpy(cloud.data.data(),xyz,sizeof(xyz));
  }
};
}
int main(){
  Fixture f;
  const auto j=p::detail::serialize_body_replay_snapshot(f.output,f.config);
  check(j["schema"]=="body_replay_snapshot_v1");
  check(j["source"]["identity"]["source_ns"].is_number_integer());
  const auto round=nlohmann::json::parse(j.dump());
  check(round["source"]["identity"]["source_ns"]==1793356673253032000LL);
  check(round["body_model"]["query_ns"]==1793356673253032077LL);
  check(round["body_filter_config"]["component_minimum_pixels"]==65);
  check(round["body_filter_config"]["near_component_minimum_pixels"]==160);
  check(round["body_filter_config"]["near_component_maximum_depth_m"]==.6);
  check(round["body_filter_config"]["body_geometry_compute_backend"]=="cpu");
  check(round["body_filter_config"]["body_geometry_parallel_enabled"]==true);
  check(round["body_model"]["links"][0]["optical_from_link"]["translation"][0].get<double>()==.12345678901234567);
  check(round["raw_capture"]["color_header"]["header_ns"]==1793356673253033000LL);
  check(round["camera_info"]["header_ns"]==1793356673253032000LL);
  check(round["runtime_body"]["support_xyz_sha256"]=="6cef3bbe6241dbd25571987a126f216d0b87ef2dc992d8ca087ef9e019633120");
  check(round["runtime_body"]["xyz_sha256"]=="13dc4c3825c96d76bb0bf3b1fc54e1ff0e8bc7f5c5a27c8308494610980dae73");
  check(round["runtime_body"]["sample_ids_sha256"]=="1fd91507df9f27061eba22953b65842e60b01e11297b0c0441ef932713d9d5d0");
  check(!round["qualification"]["rectified_pinhole_verified"].get<bool>()); // No qualification promotion.
  f.replay->self_support[0].xyz[0]=0.f;
  check(p::detail::serialize_body_replay_snapshot(f.output,f.config)["runtime_body"]["support_xyz_sha256"]==
    "75664b4da1c08de9e8fad52303cc458b3e420edde6591e58761e138cc5e3f163");
  {Fixture bad;bad.replay->image=std::make_shared<const sensor_msgs::msg::Image>(*bad.replay->image);
    rejects([&]{p::detail::serialize_body_replay_snapshot(bad.output,bad.config);});}
  {Fixture bad;bad.frame->sequence++;rejects([&]{p::detail::serialize_body_replay_snapshot(bad.output,bad.config);});}
  {Fixture bad;bad.config.sources[1].calibration="other";rejects([&]{p::detail::serialize_body_replay_snapshot(bad.output,bad.config);});}
  {Fixture bad;bad.model->identity.source_stamp_ros_ns++;rejects([&]{p::detail::serialize_body_replay_snapshot(bad.output,bad.config);});}
  {Fixture bad;bad.config.model_specs[0].triangle_count++;rejects([&]{p::detail::serialize_body_replay_snapshot(bad.output,bad.config);});}
  {Fixture bad;bad.replay->keep[0]=2;rejects([&]{p::detail::serialize_body_replay_snapshot(bad.output,bad.config);});}
  {Fixture bad;bad.replay->keep.clear();rejects([&]{p::detail::serialize_body_replay_snapshot(bad.output,bad.config);});}
  {Fixture bad;bad.replay->retained_sample_ids.clear();rejects([&]{p::detail::serialize_body_replay_snapshot(bad.output,bad.config);});}
  {Fixture bad;bad.replay->cuda_backend=true;rejects([&]{p::detail::serialize_body_replay_snapshot(bad.output,bad.config);});}
  {Fixture off;off.replay->keep_present=false;off.replay->keep.clear();off.replay->config.component_minimum_pixels=0;
    check(!p::detail::serialize_body_replay_snapshot(off.output,off.config)["body_keep"]["present"].get<bool>());}
  std::cout<<"BODY snapshot identity/config/canonical byte contract passed\n";
}
