#include "sunhorizon_navigation_runtime/detail/rgbd_projection_audit.hpp"
#include "sunhorizon_navigation_runtime/detail/rgbd_writer_audit.hpp"
#include "sunhorizon_navigation_runtime/detail/classified_airy_packet.hpp"
#include "sunhorizon_navigation_runtime/detail/airy_geometry_audit.hpp"
#include "sunhorizon_navigation_runtime/detail/ground_clearance_packet.hpp"
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <rcl/time.h>
#include <nav2_costmap_2d/observation_buffer.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_collision_monitor/pointcloud.hpp>

namespace {
void require(bool value) {if(!value)throw std::runtime_error("classified Airy packet check failed");}
nav2_util::DirectCloud source(const std::vector<std::array<float,3>>& points) {
  auto cloud=std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud->header.frame_id="rslidar";cloud->header.stamp.sec=10;
  cloud->height=1;cloud->width=points.size();cloud->point_step=19;cloud->row_step=19*points.size();
  for(std::size_t axis=0;axis<3;++axis) {
    sensor_msgs::msg::PointField field;field.name=std::string(1,"xyz"[axis]);
    field.offset=4*axis;field.datatype=field.FLOAT32;field.count=1;cloud->fields.push_back(field);
  }
  sensor_msgs::msg::PointField time;
  time.name="timestamp";time.offset=15;time.datatype=time.FLOAT32;time.count=1;
  cloud->fields.push_back(time);
  cloud->data.resize(cloud->row_step);
  for(std::size_t i=0;i<points.size();++i) {
    auto* record=cloud->data.data()+i*19;std::memcpy(record,points[i].data(),12);
    for(std::size_t j=12;j<19;++j)record[j]=static_cast<uint8_t>(i*19+j);
    const float offset=0.f;std::memcpy(record+15,&offset,4);
  }
  nav2_util::DirectCloud result{cloud,{"airy#points","epoch","rslidar",7,10000000000LL,10010000000LL,
    20000000000LL,9990000000LL,19990000000LL,19995000000LL,120000},
    nav2_util::DirectCloudKind::RetainedIndependentSource};
  result.ray_origins=std::make_shared<const nav2_util::DirectRayOrigins>(points.size(),std::array<float,3>{0.f,0.f,0.f});
  return result;
}
void mixed_returns() {
  const auto nan=std::numeric_limits<float>::quiet_NaN(),inf=std::numeric_limits<float>::infinity();
  auto input=source({{.2f,.1f,-.82f},{nan,0,1},{.3f,.1f,-.9f},{0,inf,1},{.4f,.2f,.5f},{1,2,-inf}});
  auto actual_origins=std::make_shared<nav2_util::DirectRayOrigins>();
  for(std::size_t i=0;i<6;++i)actual_origins->push_back(i%2 ?
    std::array<float,3>{nan,nan,nan} : std::array<float,3>{float(i),float(i+.1),float(i+.2)});
  input.ray_origins=actual_origins;
  const auto before=input.cloud->data;
  // This is the former production bridge, and the strict consumer rejects it.
  auto old=input;old.kind=nav2_util::DirectCloudKind::ClassifiedObstacleFrame;
  old.classes=std::make_shared<const std::vector<uint8_t>>(6,uint8_t(nav2_util::DirectPointClass::Uncertain));
  old.complete=true;require(!nav2_util::directCloudMatchesMetadata(old));
  const auto result=sunhorizon_navigation_runtime::detail::make_classified_airy_packet(input);
  require(result.input_points==6 && result.finite_points==3 && result.invalid_points==3);
  require(!result.deliverable && result.ground_unavailable && result.frame.complete && !result.frame.ground_qualified);
  require(nav2_util::directCloudMatchesMetadata(result.frame));
  require(input.cloud->data==before && result.frame.cloud->header==input.cloud->header);
  require(result.frame.cloud->fields==input.cloud->fields && result.frame.cloud->point_step==19);
  require(result.frame.cloud->width==3 && result.frame.cloud->row_step==57);
  const std::array<std::size_t,3> kept{0,2,4};
  for(std::size_t i=0;i<kept.size();++i) {
    require(std::memcmp(result.frame.cloud->data.data()+19*i,before.data()+19*kept[i],19)==0);
    require((*result.frame.ray_origins)[i]==(*input.ray_origins)[kept[i]]);
    require((*result.frame.classes)[i]==uint8_t(nav2_util::DirectPointClass::Uncertain));
  }
  const auto& a=input.metadata;const auto& b=result.frame.metadata;
  require(a.source_id==b.source_id && a.epoch==b.epoch && a.frame_id==b.frame_id && a.sequence==b.sequence);
  require(a.source_ros_ns==b.source_ros_ns && a.raw_source_ns==b.raw_source_ns);
  require(a.first_receipt_ros_ns==b.first_receipt_ros_ns && a.first_receipt_steady_ns==b.first_receipt_steady_ns);
  require(a.normalization_start_steady_ns==b.normalization_start_steady_ns &&
    a.normalization_end_steady_ns==b.normalization_end_steady_ns &&
    a.normalization_thread_cpu_ns==b.normalization_thread_cpu_ns);
}
void finite_and_missing() {
  const auto finite=source({{1,2,-.85f},{2,3,-.9f}});
  const auto result=sunhorizon_navigation_runtime::detail::make_classified_airy_packet(finite);
  require(!result.deliverable && result.ground_unavailable && result.invalid_points==0 && result.frame.cloud==finite.cloud);
  auto invalid_origin=finite;
  auto origins=std::make_shared<nav2_util::DirectRayOrigins>(*finite.ray_origins);
  (*origins)[0][1]=std::numeric_limits<float>::quiet_NaN();invalid_origin.ray_origins=origins;
  bool rejected=false;
  try{(void)sunhorizon_navigation_runtime::detail::make_classified_airy_packet(invalid_origin);}
  catch(const std::invalid_argument& e){rejected=std::string(e.what()).find("finite_return_origin")!=std::string::npos;}
  require(rejected);  // A finite measured hit must never be dropped to repair its origin.
  const auto nan=std::numeric_limits<float>::quiet_NaN();
  auto no_returns=source({{nan,1,2},{1,nan,2}});
  no_returns.ray_origins=std::make_shared<const nav2_util::DirectRayOrigins>(2,std::array<float,3>{nan,nan,nan});
  const auto missing=sunhorizon_navigation_runtime::detail::make_classified_airy_packet(no_returns);
  require(!missing.deliverable && !missing.ground_unavailable && !missing.frame.cloud && !missing.frame.complete && !missing.frame.classes);
  require(missing.input_points==2 && missing.invalid_points==2 && missing.finite_points==0);
}
void malformed() {
  const auto original=source({{1,2,3}});
  for(int which=0;which<7;++which) {
    auto input=original;auto cloud=std::make_shared<sensor_msgs::msg::PointCloud2>(*original.cloud);input.cloud=cloud;
    if(which==0)cloud->row_step++;
    if(which==1)cloud->fields.erase(cloud->fields.begin()+2);
    if(which==2)cloud->fields.push_back(cloud->fields.front());
    if(which==3)cloud->is_bigendian=true;
    if(which==4)input.metadata.source_ros_ns++;
    if(which==5)cloud->width=0;
    if(which==6)cloud->width=1048577;
    bool rejected=false;
    try{(void)sunhorizon_navigation_runtime::detail::make_classified_airy_packet(input);}
    catch(const std::invalid_argument& error) {
      const std::string text=error.what();rejected=text.find("classified_airy_packet_invalid:layout_or_metadata")==0 &&
        text.find("width=")!=std::string::npos && text.find("finite=-1 invalid=-1")!=std::string::npos && text.size()<=512;
    }
    require(rejected);
  }
}
// Exercise the real consumer implementations, not an empty-cloud substitute.
// A rejected producer packet makes no submission; the last qualified evidence
// remains usable only until its unchanged original geometry deadline.
void consumer_leases(const sunhorizon_navigation_runtime::detail::ClassifiedAiryPacket& qualified,
    const sunhorizon_navigation_runtime::detail::ClassifiedAiryPacket& unavailable) {
  require(qualified.deliverable && !unavailable.deliverable && unavailable.ground_unavailable);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"airy.input_mode","direct-classified-v1"},{"airy.direct_source_id","airy#points"},
    {"airy.direct_epoch","epoch"},{"airy.direct_frame","rslidar"},
    {"airy.direct_maximum_age_sec",.3}});
  auto node=std::make_shared<nav2_util::LifecycleNode>("airy_unavailable_consumers","",options);
  auto set_ros=[&](int64_t ns){
    require(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle())==RCL_RET_OK);
    require(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),ns)==RCL_RET_OK);
  };
  set_ros(10020000000LL);
  auto tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp=node->now();transform.header.frame_id="odom";
  transform.child_frame_id="rslidar";transform.transform.rotation.w=1.;
  require(tf->setTransform(transform,"synthetic",true));
  nav2_collision_monitor::PointCloud collision(node,"airy",tf,"rslidar","odom",
    tf2::durationFromSec(0.),rclcpp::Duration::from_seconds(.3),true);
  collision.configure();
  nav2_costmap_2d::ObservationBuffer costmap(node,"airy",0.,0.,-1.,1.,100.,0.,100.,0.,
    *tf,"odom","rslidar",tf2::durationFromSec(0.));
  costmap.enableDirectInput({"airy#points","epoch","rslidar",300000000LL},
    nav2_util::DirectCloudKind::ClassifiedObstacleFrame);
  // Deliberately all-free static content cannot establish a required source lease.
  nav2_costmap_2d::Costmap2D static_grid(10,10,.1,0.,0.,nav2_costmap_2d::FREE_SPACE);
  require(static_grid.getCost(0,0)==nav2_costmap_2d::FREE_SPACE && !costmap.isCurrent());
  std::vector<nav2_collision_monitor::Point> collision_points;
  require(!collision.getData(node->now(),collision_points));
  auto input=qualified.frame;
  // Rebase only the synthetic test's steady clock; never done to a live packet.
  input.metadata.first_receipt_steady_ns=nav2_util::directSteadyNowNs();
  input.ground_validity=std::make_shared<const nav2_util::DirectGroundValidity>(
    nav2_util::DirectGroundValidity{10040000000LL,nav2_util::directSteadyNowNs()+1000000000LL,[]{return true;}});
  require(costmap.bufferDirectCloud(input) && collision.submitDirectCloud(input));
  std::vector<nav2_costmap_2d::Observation> observations;
  require(costmap.getObservations(observations) && observations.size()==1);
  // Only Obstacle contributes a collision endpoint; Uncertain retains its lease semantics.
  require(collision.getData(node->now(),collision_points) && collision_points.size()==1);
  require(costmap.isCurrent() && collision.directSnapshotCurrent());
  unsigned submissions=0;
  // Same guard as assembly, for every mark/clear/collision consumer.
  if(unavailable.deliverable) {
    ++submissions;
    (void)costmap.bufferDirectCloud(unavailable.frame);
    (void)collision.submitDirectCloud(unavailable.frame);
  }
  require(submissions==0);
  set_ros(10040000001LL); // geometry expires before the .3 s sensor budget
  observations.clear();
  require(!costmap.isCurrent() && !costmap.getObservations(observations) && observations.empty());
  collision_points={{12.,34.}};
  require(!collision.getData(node->now(),collision_points) && !collision.directSnapshotCurrent());
  require(collision_points.size()==1); // failure, never a successful empty/free source
  require(static_grid.getCost(0,0)==nav2_costmap_2d::FREE_SPACE);
}

void rgbd_projection_audit(const sunhorizon_navigation_runtime::detail::ClassifiedAiryPacket& packet,
    const sunhorizon_perception_runtime::ground_relative::ClassifierConfig& config) {
  namespace detail=sunhorizon_navigation_runtime::detail;
  const auto& metadata=packet.frame.metadata;
  const auto original_bytes=packet.frame.cloud->data;const auto original_classes=*packet.frame.classes;
  nav2_costmap_2d::DirectProjectionSnapshot p;
  p.metadata=metadata;p.source_cloud=packet.frame.cloud;p.classes=packet.frame.classes;
  auto cloud=std::make_shared<sensor_msgs::msg::PointCloud2>(*packet.frame.cloud);
  cloud->header.frame_id="actual_global_map";p.cloud=cloud;
  p.source_to_global.header.frame_id=cloud->header.frame_id;
  p.source_to_global.header.stamp.sec=8; // Preserve the actual TF stamp, never rewrite to source/latest.
  p.source_to_global.child_frame_id=packet.frame.cloud->header.frame_id;
  p.source_to_global.transform.rotation.w=1.;
  detail::RgbdProjectionCapture disabled;
  require(!disabled.rearm("late-1"));
  require(!disabled.offer(metadata,packet.frame.cloud,packet.classification,1));
  require(disabled.attempts()==0&&!disabled.wants(metadata)&&!disabled.receive(p)&&!disabled.take(10));
  detail::RgbdProjectionCapture capture(true);
  require(capture.offer(metadata,packet.frame.cloud,packet.classification,1));
  require(!capture.offer(metadata,packet.frame.cloud,packet.classification,2));
  require(!capture.rearm("late-1")); // Never replace a selected source owner.
  auto wrong=p;wrong.metadata.epoch="other-epoch";
  require(!capture.wants(wrong.metadata)&&!capture.receive(wrong));
  wrong=p;wrong.metadata.normalization_thread_cpu_ns++;
  require(!capture.receive(wrong)); // All metadata fields, not just quartet.
  wrong=p;wrong.source_cloud=std::make_shared<const sensor_msgs::msg::PointCloud2>(*packet.frame.cloud);
  require(!capture.receive(wrong));
  wrong=p;wrong.source_cloud=sensor_msgs::msg::PointCloud2::ConstSharedPtr(packet.frame.cloud.get(),[](const auto*){});
  require(!capture.receive(wrong)); // Even same address must share the original control block.
  require(capture.receive(p)&&!capture.receive(p));
  auto matched=capture.take(2);require(bool(matched)&&capture.done()&&!capture.take(3));
  require(!capture.offer(metadata,packet.frame.cloud,packet.classification,3));
  require(!capture.rearm("bad token")&&!capture.rearm(std::string(65,'a')));
  for(unsigned i=1;i<=4;++i) {
    const auto request="late-"+std::to_string(i);
    require(capture.rearm(request)&&!capture.rearm(request));
    require(capture.offer(metadata,packet.frame.cloud,packet.classification,3));
    require(!capture.rearm("next-while-pending"));
    require(capture.receive(p));
    auto late=capture.take(4);
    require(bool(late)&&late->request_id==request&&capture.done());
    require(!capture.rearm(request)&&!capture.rearm("late-1"));
  }
  require(!capture.rearm("late-5"));
  auto model=std::make_shared<sunhorizon_perception_runtime::BodyModelDiagnostic>();
  const auto& source_identity=packet.classification->source().identity;
  model->source_frame=source_identity.source_frame;model->identity.source_id=source_identity.source_id;model->identity.epoch=source_identity.epoch;
  model->identity.calibration_id=source_identity.calibration_id;model->identity.sequence=source_identity.sequence;
  model->identity.source_stamp_ros_ns=source_identity.source_ns;model->self_tolerance_m=.02;
  model->identity.receipt_ros_ns=source_identity.first_receipt_ros_ns;
  model->identity.receipt_steady_ns=source_identity.first_receipt_steady_ns;
  for(std::size_t i=0;i<12;++i)model->links.push_back({i,2,1,{}});
  model->links[9].boundary_triangle_begin=1;
  model->links[9].boundary_triangle_count=1;
  model->links[9].boundary_tolerance_m=.003;
  model->links[9].measured_triangle_begin=1;
  model->links[9].measured_triangle_count=1;
  matched->body_model=model;
  // Actual writer capture must also seal conservative O/U frames: lack of a
  // ground qualification is evidence, never a reason to invent Ground/clearing.
  {
    namespace gr=sunhorizon_perception_runtime::ground_relative;
    auto source_frame=packet.classification->source();source_frame.identity.source_id="belly";
    auto classification=gr::conservative_unqualified_frame(std::move(source_frame));
    auto body=std::make_shared<sunhorizon_perception_runtime::PerceptionOutput>();
    body->frame.classified_obstacles=classification;body->frame.body_cloud=*packet.frame.cloud;
    auto original=std::make_shared<sunhorizon_camera_runtime::CameraFrame>();
    original->role="belly";original->sequence=metadata.sequence;
    auto primary=std::make_shared<sensor_msgs::msg::Image>();primary->header=packet.frame.cloud->header;
    primary->width=primary->height=16;primary->step=32;primary->encoding="16UC1";primary->data.resize(512);
    auto info=std::make_shared<sensor_msgs::msg::CameraInfo>();info->header=primary->header;info->width=info->height=16;
    original->depth=primary;original->depth_info=info;body->input=original;
    auto body_model=std::make_shared<sunhorizon_perception_runtime::BodyModelDiagnostic>(*model);
    body_model->identity.source_id="belly";body->frame.body_model_diagnostic=body_model;
    auto projection=std::make_shared<nav2_util::DirectProjectionSnapshot>(p);
    projection->metadata.source_id="belly";
    projection->source_cloud=sensor_msgs::msg::PointCloud2::ConstSharedPtr(body,&body->frame.body_cloud);
    projection->classes=std::make_shared<const std::vector<uint8_t>>(classification->classes().size(),uint8_t(nav2_util::DirectPointClass::Uncertain));
    auto projected=std::make_shared<sensor_msgs::msg::PointCloud2>(*p.cloud);projected->header.frame_id="map";
    projection->cloud=projected;projection->source_to_global.header.frame_id="map";
    auto request=std::make_shared<detail::WriterCaptureRequest>("actual-writer");
    auto queue=std::make_shared<detail::RgbdWriterQueue>();
    auto audit=std::make_shared<detail::RgbdWriterOwner>(body,projection->metadata,request,queue);
    nav2_util::DirectWriterHit hit;hit.point_index=0;hit.after_cost=253;
    const auto xyz=detail::rgbd_audit_xyz(*projected,detail::rgbd_audit_layout(*projected),0);
    hit.xyz={double(xyz[0]),double(xyz[1]),double(xyz[2])};
    audit->actualMark(projection,hit);require(request->states[0].load()==detail::WriterCaptureState::Armed);
    hit.measurement_support_available=true;hit.ground_obstacle_uncertain={2,3,1};
    hit.frame_supported=hit.persistence_confirmed=true;hit.previous_distinct_sequence=41;
    hit.previous_distinct_source_ns=900000000;
    hit.before_cost=hit.after_cost=254;hit.mark_threshold_reached=false; // Reinforcement is still a real write.
    audit->actualMark(projection,hit);audit->actualMark(projection,hit);
    require(request->states[0].load()==detail::WriterCaptureState::Queued);
    std::weak_ptr<const sunhorizon_perception_runtime::PerceptionOutput> retained=body;
    audit.reset();body.reset();original.reset();projection.reset();
    auto events=queue->take();require(bool(events[0])&&!events[1]&&!retained.expired());
    const auto captured=detail::writer_projection_frame(*events[0]);
    require(!captured.classification->ground().safety_qualified);
    bool rejected=false;try{(void)detail::serialize_rgbd_projection_audit(captured);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected); // Original projection-capture qualification contract stays strict.
    const auto sealed=nlohmann::json::parse(detail::serialize_rgbd_projection_audit(captured,nullptr,false));
    require(sealed["classification"]["ground"]["safety_qualified"]==false);
    require(sealed["classification"]["classes"][0]==int(gr::PointClass::Uncertain));
    require(detail::writer_hit_json(*events[0])["reinforcement_of_existing_254"]==true);
    const auto support=detail::writer_hit_json(*events[0])["measurement_support"];
    require(support["available"]==true&&support["ground_obstacle_uncertain"]==nlohmann::json::array({2,3,1}));
    require(support["frame_supported"]==true&&support["persistence_confirmed"]==true);
    require(support["previous_distinct_sequence"]==41&&support["previous_distinct_source_ns"]==900000000);
    require(detail::writer_hit_json(*events[0])["not_final_master_cost_evidence"]==true);
    // The actual owner (not just the queue abstraction) exposes queue-full as
    // a terminal failed request and never substitutes a new source afterward.
    for(unsigned i=0;i<4;++i)require(queue->tryPush(*events[0]));
    auto failed_request=std::make_shared<detail::WriterCaptureRequest>("full");
    detail::RgbdWriterOwner full(events[0]->body,events[0]->projection->metadata,failed_request,queue);
    full.actualMark(events[0]->projection,events[0]->hit);
    require(failed_request->states[0].load()==detail::WriterCaptureState::FailedQueue);
    (void)queue->take();full.actualMark(events[0]->projection,events[0]->hit);
    for(const auto& item:queue->take())require(!item);
    auto untouched=std::make_shared<detail::WriterCaptureRequest>("wrong-owner");
    detail::RgbdWriterOwner guarded(events[0]->body,events[0]->projection->metadata,untouched,queue);
    auto forged=std::make_shared<nav2_util::DirectProjectionSnapshot>(*events[0]->projection);
    forged->source_cloud=sensor_msgs::msg::PointCloud2::ConstSharedPtr(forged->source_cloud.get(),[](const auto*){});
    guarded.actualMark(forged,events[0]->hit);
    require(untouched->states[0].load()==detail::WriterCaptureState::Armed);
    for(int mismatch=0;mismatch<4;++mismatch) {
      auto wrong=*events[0];
      if(mismatch==0)wrong.hit.point_index=999999;
      if(mismatch==1)wrong.hit.xyz[0]+=.01;
      if(mismatch==2)wrong.hit.after_cost=253;
      if(mismatch==3){auto bad_projection=std::make_shared<nav2_util::DirectProjectionSnapshot>(*wrong.projection);bad_projection->classes=std::make_shared<const std::vector<uint8_t>>(classification->classes().size(),0);wrong.projection=bad_projection;}
      bool failed=false;try{(void)detail::writer_projection_frame(wrong);}catch(const std::invalid_argument&){failed=true;}
      require(failed);
    }
  }
  const auto good=nlohmann::json::parse(detail::serialize_rgbd_projection_audit(*matched));
  require(good["schema"]=="rgbd-global-projection-audit-v1");
  require(good["consumer"]["stage"]=="global.observation_buffer_committed");
  require(good["consumer"]["global_frame"]=="actual_global_map");
  require(good["consumer_point_columns"][5]=="global_x");
  detail::RgbdProjectionCapture local_capture(true,detail::RgbdProjectionConsumer::Local);
  require(local_capture.offer(metadata,packet.frame.cloud,packet.classification,1,model));
  require(!local_capture.wants(metadata)&&!local_capture.receive(p));
  require(local_capture.wants(metadata,detail::RgbdProjectionConsumer::Local));
  auto local_projection=p;
  auto local_cloud=std::make_shared<sensor_msgs::msg::PointCloud2>(*cloud);
  local_cloud->header.frame_id="actual_local_odom";
  local_projection.cloud=local_cloud;
  local_projection.source_to_global.header.frame_id=local_cloud->header.frame_id;
  detail::RgbdProjectionCapture global_scoped(true);
  require(global_scoped.offer(metadata,packet.frame.cloud,packet.classification,1));
  require(!global_scoped.wants(metadata,detail::RgbdProjectionConsumer::Local));
  require(!global_scoped.receive(local_projection,detail::RgbdProjectionConsumer::Local));
  require(global_scoped.receive(p));
  require(bool(global_scoped.take(2)));
  require(local_capture.receive(local_projection,detail::RgbdProjectionConsumer::Local));
  auto local_matched=local_capture.take(2);require(bool(local_matched));
  const auto local_doc=nlohmann::json::parse(detail::serialize_rgbd_projection_audit(*local_matched));
  require(local_doc["schema"]=="rgbd-local-projection-audit-v1");
  require(local_doc["consumer"]["stage"]=="local.observation_buffer_committed");
  require(local_doc["consumer"]["local_frame"]=="actual_local_odom");
  require(!local_doc["consumer"].contains("global_frame"));
  require(local_doc["consumer_point_columns"][5]=="local_x");
  require(local_doc["body_model"]==good["body_model"]);
  require(local_doc["classification"]==good["classification"]);
  require(local_doc["consumer"]["actual_header_transform"]["parent"]=="actual_local_odom");
  require(local_doc["consumer"]["actual_header_transform"]["stamp_sec"]==8);
  // A nonce is independently bounded per consumer and does not assert that
  // different consumers accept the same source frame or transformation.
  require(local_capture.rearm("late-1"));
  require(local_capture.offer(metadata,packet.frame.cloud,packet.classification,3,model));
  require(!local_capture.receive(local_projection));
  require(local_capture.receive(local_projection,detail::RgbdProjectionConsumer::Local));
  require(local_capture.take(4)->request_id=="late-1");
  require(good["body_model"]["links"].size()==12);
  require(good["body_model"]["links"][9]["boundary_triangle_begin"]==1);
  require(good["body_model"]["links"][9]["boundary_triangle_count"]==1);
  require(good["body_model"]["links"][9]["boundary_tolerance_m"]==.003);
  require(good["body_model"]["links"][0]["boundary_triangle_count"]==0);
  require(good["body_model"]["links"][9]["measured_triangle_begin"]==1);
  require(good["body_model"]["links"][9]["measured_triangle_count"]==1);
  require(good["body_model"]["links"][0]["measured_triangle_begin"]==0);
  require(good["body_model"]["links"][0]["measured_triangle_count"]==0);
  require(good["body_model"]["sequence"]==source_identity.sequence);
  require(good["body_model"]["self_tolerance_m"]==.02);
  for(int mismatch=0;mismatch<9;++mismatch) {
    auto wrong_model=std::make_shared<sunhorizon_perception_runtime::BodyModelDiagnostic>(*model);
    if(mismatch==0)++wrong_model->identity.sequence;
    if(mismatch==1)++wrong_model->identity.source_stamp_ros_ns;
    if(mismatch==2)wrong_model->identity.epoch+="wrong";
    if(mismatch==3)wrong_model->identity.source_id+="wrong";
    if(mismatch==4)wrong_model->identity.calibration_id+="wrong";
    if(mismatch==5)wrong_model->links[0].link_index=1;
    if(mismatch==6)++wrong_model->identity.receipt_ros_ns;
    if(mismatch==7)++wrong_model->identity.receipt_steady_ns;
    if(mismatch==8)wrong_model->source_frame+="wrong";
    matched->body_model=wrong_model;bool rejected=false;
    try{(void)detail::serialize_rgbd_projection_audit(*matched);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected);
  }
  matched->body_model=model;
  require(good["consumer"]["accepted_projection"]==true&&good["consumer"]["not_final_voxel_cost_evidence"]==true);
  require(good["consumer"]["actual_header_transform"]["stamp_sec"]==8);
  require(good["consumer_points"].size()==packet.classification->classes().size());
  for(std::size_t i=0;i<good["consumer_points"].size();++i) {
    require(good["consumer_points"][i][1]==packet.classification->source().points[i].sample_id);
    require(good["consumer_points"][i][8]==(*packet.frame.classes)[i]);
    require(good["consumer_points"][i][9]==packet.classification->source().points[i].ground_blocked);
  }

  // A rejected measurement keeps its original slot and pixel in the accepted
  // projection audit. It cannot be silently relabelled Ground or Uncertain.
  {
    namespace gr=sunhorizon_perception_runtime::ground_relative;
    auto rejected_source=packet.classification->source();
    const auto index=rejected_source.points.size();
    auto rejected_point=rejected_source.points.back();
    rejected_point.sample_id=1000;rejected_point.measurement_unreliable=true;
    rejected_source.points.push_back(rejected_point);
    rejected_source.raster_geometry={640,480,1};
    auto& proof=rejected_source.measurement_quality;
    const auto& id=rejected_source.identity;
    proof.present=proof.exact_rectified_pair=true;
    proof.algorithm="dual-ir-native-quality-v4";
    proof.stereo_match_complete=true;proof.stereo_match_reason="disabled_foreground_ambiguity";
    proof.role=id.source_id;proof.source_frame=id.source_frame;
    proof.source_epoch=id.epoch;proof.calibration_id=id.calibration_id;
    proof.source_ns=id.source_ns;proof.sequence=id.sequence;
    proof.opened_serial="synthetic-serial";proof.capture_epoch="synthetic-capture";
    proof.width=640;proof.height=480;proof.sdk_depth_scale_m=.001;proof.depth_pixel_type=0;
    proof.device_us={1234,1234,1234};
    proof.global_us={uint64_t(id.source_ns/1000),uint64_t(id.source_ns/1000),uint64_t(id.source_ns/1000)};
    proof.unreliable_sample_ids={rejected_point.sample_id};
    const auto classified=gr::classify_obstacle_frame(rejected_source,config,packet.classification->reference());
    require(classified->ground().safety_qualified&&classified->classes()[index]==gr::PointClass::Invalid);
    auto append_record=[](const sensor_msgs::msg::PointCloud2& original) {
      auto c=std::make_shared<sensor_msgs::msg::PointCloud2>(original);
      require(c->height==1&&c->row_step==c->width*c->point_step);
      const auto last=std::vector<uint8_t>(c->data.end()-c->point_step,c->data.end());
      c->data.insert(c->data.end(),last.begin(),last.end());
      ++c->width;c->row_step+=c->point_step;return c;
    };
    auto sealed=*matched;
    sealed.classification=classified;
    auto projection=std::make_shared<nav2_costmap_2d::DirectProjectionSnapshot>(*matched->projection);
    sealed.source_cloud=append_record(*matched->source_cloud);
    projection->source_cloud=sealed.source_cloud;
    projection->cloud=append_record(*matched->projection->cloud);
    auto classes=std::make_shared<std::vector<uint8_t>>(*matched->projection->classes);
    classes->push_back(uint8_t(nav2_util::DirectPointClass::Rejected));projection->classes=classes;
    sealed.projection=projection;
    const auto doc=nlohmann::json::parse(detail::serialize_rgbd_projection_audit(sealed));
    require(doc["consumer_points"].size()==index+1);
    require(doc["consumer_points"][index][1]==1000);
    require(doc["consumer_points"][index][8]==uint8_t(nav2_util::DirectPointClass::Rejected));
    (*classes)[index]=uint8_t(nav2_util::DirectPointClass::Ground);
    bool rejected=false;
    try{(void)detail::serialize_rgbd_projection_audit(sealed);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected);
  }

  // Use the original ObservationBuffer prepare/commit callback too; no producer
  // or latest-TF projection stands in for the actual consumer transformation.
  auto actual_source=packet.classification->source();
  auto actual_reference=*packet.classification->reference();
  const auto steady_delta=nav2_util::directSteadyNowNs()-actual_source.identity.first_receipt_steady_ns;
  actual_source.identity.first_receipt_steady_ns+=steady_delta;
  actual_source.identity.normalization_start_steady_ns+=steady_delta;
  actual_source.identity.normalization_end_steady_ns+=steady_delta;
  actual_reference.first_receipt_steady_ns+=steady_delta;
  const auto actual_classification=sunhorizon_perception_runtime::ground_relative::classify_obstacle_frame(
    actual_source,config,actual_reference);
  require(actual_classification->ground().safety_qualified);
  auto actual_input=packet.frame;
  actual_input.metadata.first_receipt_steady_ns+=steady_delta;
  actual_input.metadata.normalization_start_steady_ns+=steady_delta;
  actual_input.metadata.normalization_end_steady_ns+=steady_delta;
  actual_input.ground_validity=std::make_shared<const nav2_util::DirectGroundValidity>(
    nav2_util::DirectGroundValidity{10040000000LL,nav2_util::directSteadyNowNs()+1000000000LL,[]{return true;}});
  auto node=std::make_shared<nav2_util::LifecycleNode>("rgbd_projection_original_consumer","",rclcpp::NodeOptions());
  require(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle())==RCL_RET_OK);
  require(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),10020000000LL)==RCL_RET_OK);
  tf2_ros::Buffer tf(node->get_clock());geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp=node->now();transform.header.frame_id="public_global";
  transform.child_frame_id=actual_input.metadata.frame_id;transform.transform.rotation.w=1.;transform.transform.translation.x=3.;
  require(tf.setTransform(transform,"synthetic-original-consumer",true));
  nav2_costmap_2d::ObservationBuffer buffer(node,"rgbd",0.,0.,-1.,1.,100.,0.,100.,0.,tf,
    "public_global",actual_input.metadata.frame_id,tf2::durationFromSec(0.));
  buffer.enableDirectInput({actual_input.metadata.source_id,actual_input.metadata.epoch,actual_input.metadata.frame_id,300000000LL},
    nav2_util::DirectCloudKind::ClassifiedObstacleFrame);
  detail::RgbdProjectionCapture original_consumer(true);
  require(original_consumer.offer(actual_input.metadata,actual_input.cloud,actual_classification,nav2_util::directSteadyNowNs()));
  buffer.setDirectProjectionSink([&](const auto& m){return original_consumer.wants(m);},
    [&](const auto& snapshot){require(original_consumer.receive(snapshot));});
  require(buffer.bufferDirectCloud(actual_input)&&buffer.isCurrent());
  auto committed=original_consumer.take(nav2_util::directSteadyNowNs());require(bool(committed));
  const auto actual_doc=nlohmann::json::parse(detail::serialize_rgbd_projection_audit(*committed));
  require(actual_doc["consumer"]["global_frame"]=="public_global");
  require(std::abs(actual_doc["consumer_points"][0][5].get<double>()-
    actual_doc["consumer_points"][0][2].get<double>()-3.)<1.e-5);
  transform.header.frame_id="public_local";transform.transform.translation.x=7.;
  require(tf.setTransform(transform,"synthetic-original-local-consumer",true));
  nav2_costmap_2d::ObservationBuffer local_buffer(node,"rgbd-local",0.,0.,-1.,1.,100.,0.,100.,0.,tf,
    "public_local",actual_input.metadata.frame_id,tf2::durationFromSec(0.));
  local_buffer.enableDirectInput({actual_input.metadata.source_id,actual_input.metadata.epoch,actual_input.metadata.frame_id,300000000LL},
    nav2_util::DirectCloudKind::ClassifiedObstacleFrame);
  detail::RgbdProjectionCapture original_local_consumer(true,detail::RgbdProjectionConsumer::Local);
  require(original_local_consumer.offer(actual_input.metadata,actual_input.cloud,actual_classification,nav2_util::directSteadyNowNs()));
  local_buffer.setDirectProjectionSink([&](const auto& m){return original_local_consumer.wants(m,detail::RgbdProjectionConsumer::Local);},
    [&](const auto& snapshot){require(original_local_consumer.receive(snapshot,detail::RgbdProjectionConsumer::Local));});
  require(local_buffer.bufferDirectCloud(actual_input)&&local_buffer.isCurrent());
  auto local_committed=original_local_consumer.take(nav2_util::directSteadyNowNs());require(bool(local_committed));
  const auto actual_local_doc=nlohmann::json::parse(detail::serialize_rgbd_projection_audit(*local_committed));
  require(actual_local_doc["consumer"]["local_frame"]=="public_local");
  require(actual_local_doc["consumer"]["actual_header_transform"]["parent"]=="public_local");
  require(std::abs(actual_local_doc["consumer_points"][0][5].get<double>()-
    actual_local_doc["consumer_points"][0][2].get<double>()-7.)<1.e-5);
  const auto rejects=[&](nav2_costmap_2d::DirectProjectionSnapshot changed){
    auto before=matched->projection;matched->projection=std::make_shared<const nav2_costmap_2d::DirectProjectionSnapshot>(std::move(changed));
    bool rejected=false;try{(void)detail::serialize_rgbd_projection_audit(*matched);}catch(const std::invalid_argument&){rejected=true;}
    matched->projection=before;require(rejected);
  };
  wrong=p;auto classes=std::make_shared<std::vector<uint8_t>>(*p.classes);(*classes)[0]=255;wrong.classes=classes;rejects(wrong);
  wrong=p;auto malformed=std::make_shared<sensor_msgs::msg::PointCloud2>(*cloud);malformed->is_bigendian=!cloud->is_bigendian;wrong.cloud=malformed;rejects(wrong);
  wrong=p;malformed=std::make_shared<sensor_msgs::msg::PointCloud2>(*cloud);malformed->fields[0].datatype=sensor_msgs::msg::PointField::FLOAT64;wrong.cloud=malformed;rejects(wrong);
  wrong=p;malformed=std::make_shared<sensor_msgs::msg::PointCloud2>(*cloud);malformed->row_step++;wrong.cloud=malformed;rejects(wrong);
  wrong=p;wrong.source_to_global.child_frame_id="other-source";rejects(wrong);
  auto duplicate_source=packet.classification->source();
  duplicate_source.points[1].sample_id=duplicate_source.points[0].sample_id;
  const auto duplicate=sunhorizon_perception_runtime::ground_relative::classify_obstacle_frame(
    duplicate_source,config,*packet.classification->reference());
  require(duplicate->ground().safety_qualified);
  auto original_frame=matched->classification;matched->classification=duplicate;
  bool duplicate_rejected=false;
  try{(void)detail::serialize_rgbd_projection_audit(*matched);}catch(const std::invalid_argument&){duplicate_rejected=true;}
  matched->classification=original_frame;require(duplicate_rejected);
  require(packet.frame.cloud->data==original_bytes&&*packet.frame.classes==original_classes);
  detail::RgbdProjectionCapture retries(true);
  for(unsigned i=0;i<8;++i) {
    const int64_t begin=int64_t(i)*3000000000LL+1;
    require(retries.offer(metadata,packet.frame.cloud,packet.classification,begin));
    require(!retries.take(begin+1999999999LL));
    require(!retries.take(begin+2000000000LL)); // No unpaired export.
  }
  require(retries.exhausted());
  require(retries.attempts()==8&&!retries.offer(metadata,packet.frame.cloud,packet.classification,30000000000LL));
  require(retries.rearm("retry-exhausted")&&retries.attempts()==0&&!retries.exhausted());
}

void geometry_audit(const sunhorizon_navigation_runtime::detail::ClassifiedAiryPacket& packet,
    const nav2_util::DirectCloud& raw) {
  namespace adapter=sunhorizon_rgbd_adapter;
  adapter::SelfExclusionDirectOutput filtered;
  auto input=std::make_shared<adapter::SelfExclusionDirectInput>();input->cloud=raw.cloud;
  const auto& m=packet.frame.metadata;
  input->stamp.source_identity=m.source_id;input->stamp.epoch=m.epoch;
  input->stamp.sequence=m.sequence;input->stamp.source_ns=m.source_ros_ns;
  filtered.input=input;
  auto audit=std::make_shared<adapter::LidarSelfAudit>();
  adapter::LidarSelfAuditPoint point;point.sample_id=126;point.source_ns=m.source_ros_ns;
  point.position={50.f,50.f,-.9f};point.ray_origin={.01f,.02f,.03f};
  point.result.complete=true;point.result.reason="no_self_intersection";
  audit->points.push_back(point);filtered.lidar_audit=audit;
  nav2_costmap_2d::DirectProjectionSnapshot projection;
  projection.metadata=m;projection.source_cloud=packet.frame.cloud;projection.classes=packet.frame.classes;
  auto cloud=std::make_shared<sensor_msgs::msg::PointCloud2>(*packet.frame.cloud);
  cloud->header.frame_id="actual_public_map";
  for(std::size_t i=0;i<cloud->width;++i){float x;std::memcpy(&x,cloud->data.data()+i*cloud->point_step,4);
    x+=3.f;std::memcpy(cloud->data.data()+i*cloud->point_step,&x,4);}
  projection.cloud=cloud;
  projection.source_to_global.header=cloud->header;
  projection.source_to_global.child_frame_id=packet.frame.cloud->header.frame_id;
  projection.source_to_global.transform.rotation.w=1.;projection.source_to_global.transform.translation.x=3.;
  projection.ray_origins=std::make_shared<const std::vector<std::array<double,3>>>(cloud->width,
    std::array<double,3>{3.01,.02,.03});
  const auto doc=nlohmann::json::parse(sunhorizon_navigation_runtime::detail::serialize_airy_geometry_audit(
    *packet.classification,filtered,&projection));
  require(doc["consumer"]["frame"]=="actual_public_map");
  require(doc["points"][0]["raw_sample_id"]==126 && doc["points"][0]["classification_index"]==25);
  require(doc["points"][0]["consumer_xyz"][0]==53.); // actual consumer values, not private registration XY
  require(doc["points"][0]["point_class"]==2);
  require(std::abs(doc["points"][0]["sdk_ray_origin"][0].get<double>()-.01)<1.e-6);
  const auto missing=nlohmann::json::parse(sunhorizon_navigation_runtime::detail::serialize_airy_geometry_audit(
    *packet.classification,filtered,nullptr));require(missing["consumer"].is_null());
  projection.metadata.epoch="other_epoch";bool rejected=false;
  try{(void)sunhorizon_navigation_runtime::detail::serialize_airy_geometry_audit(*packet.classification,filtered,&projection);}
  catch(const std::invalid_argument&){rejected=true;}require(rejected);
  projection.metadata=m;audit->points[0].source_ns++;rejected=false;
  try{(void)sunhorizon_navigation_runtime::detail::serialize_airy_geometry_audit(*packet.classification,filtered,&projection);}
  catch(const std::invalid_argument&){rejected=true;}require(rejected);
}
void shared_ground_owner() {
  using Json=nlohmann::json;
  namespace gr=sunhorizon_perception_runtime::ground_relative;
  Json c={{"ground_band_min_m",-.01},{"ground_band_max_m",.01},{"maximum_support_residual_m",.02},
    {"maximum_point_error_m",.01},{"maximum_transform_translation_error_m",.01},
    {"maximum_transform_angle_error_rad",.01},{"maximum_plane_offset_error_m",.01},
    {"maximum_plane_angle_error_rad",.01},{"maximum_normal_tilt_rad",.1},
    {"rotation_tolerance",1.e-12},{"normal_norm_tolerance",1.e-12},
    {"minimum_support_span_x_m",.1},{"minimum_support_span_y_m",.1},{"minimum_support_double_area_m2",.005},
    {"minimum_samples_per_cell",1},{"minimum_training_cells",3},{"minimum_holdout_cells",3},
    {"continuous_support_radius_cells",1},{"maximum_points",1000},{"maximum_support_cells",100},
    {"maximum_support_checks",10000}};
  Json cells=Json::array();std::vector<std::array<float,3>> points;
  for(unsigned y=0;y<5;++y)for(unsigned x=0;x<5;++x) {
    cells.push_back({{"x",x},{"y",y}});points.push_back({float((x+.5)*.1),float((y+.5)*.1),-.9f});
  }
  // Deliberately synthetic fixture: neither these bounds nor hashes qualify a robot.
  Json doc={{"schema","ground-reference-owner-v1"},{"mode","observe"},{"clock_boot_id","synthetic"},{"classifier",c},
    {"reference",{{"provenance_id","synthetic"},{"calibration_id","synthetic-floor"},{"common_frame","odom"},
      {"source_ns",9990000000LL},{"first_receipt_ros_ns",9991000000LL},{"first_receipt_steady_ns",19990000000LL},
      {"expiry_budget_ns",600000000},{"independently_qualified",false},{"unit_normal",{0.,0.,1.}},
      {"anchor",{0.,0.,-.9}},{"plane_offset_error_bound_m",0.},{"plane_angle_error_bound_rad",0.},
      {"support",{{"origin_x_m",0.},{"origin_y_m",0.},{"resolution_m",.1},{"width",5},{"height",5},{"required_cells",cells}}}}},
    {"sources",Json::array({{{"source_id","airy#points"},{"epoch","epoch"},{"calibration_id","synthetic-airy"},
      {"source_frame","rslidar"},{"measurement_provenance_id","synthetic"},{"measurement_qualified",false},
      {"source_time_transform_qualified",false},{"point_error_bound_m",0.},
      {"transform_translation_error_bound_m",0.},{"transform_angle_error_bound_rad",0.}}})}};
  const auto path=std::filesystem::canonical(std::filesystem::temp_directory_path()) /
    ("airy-ground-test-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".json");
  struct Cleanup {std::filesystem::path path;~Cleanup(){std::filesystem::remove(path);}} cleanup{path};
  auto load=[&] {
    const auto bytes=doc.dump();{std::ofstream f(path);f<<bytes;}
    unsigned char digest[32];unsigned int length=0;
    require(EVP_Digest(bytes.data(),bytes.size(),digest,&length,EVP_sha256(),nullptr)==1 && length==32);
    std::ostringstream out;out<<std::hex<<std::setfill('0');for(auto b:digest)out<<std::setw(2)<<unsigned(b);
    return gr::GroundReferenceOwner::load(path.string(),out.str(),"synthetic");
  };
  sunhorizon_navigation_runtime::detail::AiryGroundContext context;
  context.owner=load();context.calibration_id="synthetic-airy";context.expiry_budget_ns=300000000;
  context.source_to_reference.source_frame="rslidar";context.source_to_reference.common_frame="odom";
  context.source_to_reference.source_ns=context.source_to_reference.common_ns=10000000000LL;
  context.source_to_reference.qualified=true;
  const auto input=source(points);
  const auto visible=std::make_shared<const std::vector<uint8_t>>(points.size(),0);
  const auto observed=sunhorizon_navigation_runtime::detail::make_classified_airy_packet(input,context,visible);
  require(!observed.deliverable && observed.ground_unavailable && !observed.frame.ground_qualified && observed.frame.cloud==input.cloud);
  require(observed.classification && observed.classification->ground().candidate_ground_points==25);
  require(observed.classification->clearing_indices().empty());
  for(auto point:*observed.frame.classes)require(point==uint8_t(nav2_util::DirectPointClass::Uncertain));
  doc["mode"]="qualified";doc["reference"]["independently_qualified"]=true;
  doc["sources"][0]["measurement_qualified"]=true;doc["sources"][0]["source_time_transform_qualified"]=true;
  doc["qualification"]={{"approved_usage","production-ground-classification"}};
  for(const auto* key:{"independent_ground_evidence_sha256","measurement_error_evidence_sha256",
      "transform_error_evidence_sha256","clock_alignment_evidence_sha256","support_selection_evidence_sha256"})
    doc["qualification"][key]=std::string(64,'a');
  context.owner=load();
  const auto qualified=sunhorizon_navigation_runtime::detail::make_classified_airy_packet(input,context,visible);
  require(qualified.deliverable && !qualified.ground_unavailable && qualified.frame.ground_qualified && qualified.classification->clearing_indices().size()==25);
  require(qualified.frame.cloud==input.cloud && nav2_util::directCloudMatchesMetadata(qualified.frame));
  auto mixed_points=points;
  mixed_points.push_back({.25f,.25f,-.885f}); // above ground band, within support residual
  mixed_points.push_back({50.f,50.f,-.9f}); // outside qualified support, still a hit
  auto mixed_mask=std::make_shared<std::vector<uint8_t>>(mixed_points.size(),0);
  mixed_mask->back()=1;
  const auto conservative=sunhorizon_navigation_runtime::detail::make_classified_airy_packet(
    source(mixed_points),context,mixed_mask);
  require(conservative.deliverable && !conservative.ground_unavailable);
  require((*conservative.frame.classes)[25]==uint8_t(nav2_util::DirectPointClass::Obstacle));
  require((*conservative.frame.classes)[26]==uint8_t(nav2_util::DirectPointClass::Uncertain));
  // A static map asset can use a shorter registration lease without changing
  // the original sensor clocks. The same helper is used by RGB-D and Airy.
  auto map_source=qualified.classification->source();
  auto map_reference=*qualified.classification->reference();
  map_source.identity.common_frame=map_source.to_common.common_frame="registration_map";
  map_reference.common_frame="registration_map";
  map_reference.lifetime=gr::GroundReference::Lifetime::RegistrationMapAsset;
  map_reference.map_asset={std::string(64,'b'),"synthetic-independent-pcd"};
  auto reset=std::make_shared<std::atomic<bool>>(false);
  auto alignment=std::make_shared<gr::RegistrationMapAlignment>(map_reference.map_asset,1,reset,
    10100000000LL,20100000000LL);
  map_reference.map_binding={alignment,map_source.identity.source_id,map_source.identity.epoch,map_source.identity.source_ns};
  const auto map_frame=gr::classify_obstacle_frame(map_source,context.owner->config(),map_reference);
  require(map_frame->ground().safety_qualified);
  auto map_packet=qualified.frame;
  sunhorizon_navigation_runtime::detail::attach_ground_validity(map_packet,map_frame);
  require(map_packet.ground_validity && nav2_util::directCloudMatchesMetadata(map_packet));
  require(nav2_util::directMetadataEqual(map_packet.metadata,input.metadata));
  require(map_packet.ground_validity->valid_until_ros_ns==10100000000LL);
  require(map_packet.ground_validity->valid_until_steady_ns==20100000000LL);
  require(nav2_util::directGroundValidityFresh(*map_packet.ground_validity,10020000000LL,20020000000LL));
  require(!nav2_util::directGroundValidityFresh(*map_packet.ground_validity,10100000001LL,20020000000LL));
  require(!nav2_util::directGroundValidityFresh(*map_packet.ground_validity,10020000000LL,20100000001LL));
  reset->store(true);
  require(!nav2_util::directGroundValidityFresh(*map_packet.ground_validity,10020000000LL,20020000000LL));
  const auto no_cad_visibility=sunhorizon_navigation_runtime::detail::make_classified_airy_packet(input,context);
  require(no_cad_visibility.classification->clearing_indices().empty());
  // The same valid endpoint detector basis cannot attach bounded marking or
  // visibility-volume evidence, even with an independently qualified owner.
  auto support_source=qualified.classification->source();
  const auto single_samples=support_source.points;support_source.points.clear();
  support_source.measurement_qualified=false;
  support_source.point_error_bound_m=support_source.transform_translation_error_bound_m=
    support_source.transform_angle_error_bound_rad=gr::kUnspecified;
  for(const auto& point:single_samples)for(unsigned repeat=0;repeat<16;++repeat){
    auto sample=point;sample.sample_id=support_source.points.size();support_source.points.push_back(sample);}
  auto support_reference=*qualified.classification->reference();
  support_reference.independently_qualified=support_reference.source_time_transform_qualified=false;
  support_reference.plane_offset_error_bound_m=support_reference.plane_angle_error_bound_rad=gr::kUnspecified;
  gr::SegmentationConfig support_config;support_config.maximum_point_residual_m=.04;
  support_config.maximum_cell_median_residual_m=.03;support_config.maximum_cell_mad_m=.015;
  support_config.maximum_partition_median_difference_m=.02;support_config.maximum_normal_tilt_rad=.2;
  support_config.minimum_support_span_x_m=support_config.minimum_support_span_y_m=.1;
  support_config.minimum_support_double_area_m2=.005;support_config.minimum_samples_per_partition=1;
  support_config.minimum_supported_cells=6;support_config.continuous_support_radius_cells=1;
  support_config.maximum_points=1000;support_config.maximum_support_cells=100;support_config.maximum_support_checks=10000;
  const auto supported=gr::classify_supported_ground_frame(support_source,support_config,support_reference);
  require(supported->ground().safety_qualified&&supported->ground().basis==gr::GroundQualification::Basis::MeasuredSupport);
  auto supported_packet=qualified.frame;
  supported_packet.ground_basis=sunhorizon_navigation_runtime::detail::direct_ground_basis(supported->ground());
  sunhorizon_navigation_runtime::detail::attach_ground_clearance(supported_packet,supported,context.owner);
  require(!supported_packet.mark_error_bound_qualified&&std::isnan(supported_packet.mark_error_bound_m)&&
    !supported_packet.ground_clearance&&nav2_util::directCloudMatchesMetadata(supported_packet));
  // Timed references cannot gain empirical correction by borrowing a sensor
  // lease. The current candidate is strictly map-asset + attached geometry lease.
  auto timed_empirical=supported_packet;
  timed_empirical.ground_validity=std::make_shared<const nav2_util::DirectGroundValidity>(
    nav2_util::DirectGroundValidity{10100000000LL,20100000000LL,[]{return true;}});
  require(supported->reference()->lifetime==gr::GroundReference::Lifetime::TimedObservation);
  sunhorizon_navigation_runtime::detail::attach_empirical_ground_support(timed_empirical,supported,context.owner);
  require(!timed_empirical.empirical_ground_support);
  auto empirical_source=support_source;
  empirical_source.identity.common_frame=empirical_source.to_common.common_frame="registration_map";
  empirical_source.raster_geometry={400,1,1};
  auto empirical_reference=support_reference;
  empirical_reference.common_frame="registration_map";
  empirical_reference.lifetime=gr::GroundReference::Lifetime::RegistrationMapAsset;
  empirical_reference.map_asset=map_reference.map_asset;
  auto empirical_alignment=std::make_shared<gr::RegistrationMapAlignment>(
    empirical_reference.map_asset,2,nullptr,10100000000LL,20100000000LL);
  empirical_reference.map_binding={empirical_alignment,empirical_source.identity.source_id,
    empirical_source.identity.epoch,empirical_source.identity.source_ns};
  auto empirical_frame=gr::classify_supported_ground_frame(empirical_source,support_config,empirical_reference);
  require(empirical_frame->ground().safety_qualified);
  auto empirical_packet=supported_packet;empirical_packet.ground_validity.reset();
  auto empirical_cloud=std::make_shared<sensor_msgs::msg::PointCloud2>(*empirical_packet.cloud);
  empirical_cloud->width=empirical_source.points.size();
  empirical_cloud->row_step=empirical_cloud->point_step*empirical_cloud->width;
  empirical_cloud->data.assign(empirical_cloud->row_step,0);
  auto empirical_classes=std::make_shared<std::vector<uint8_t>>();
  auto empirical_origins=std::make_shared<nav2_util::DirectRayOrigins>();
  for(std::size_t i=0;i<empirical_source.points.size();++i){
    for(unsigned axis=0;axis<3;++axis){const float value=empirical_source.points[i].position[axis];
      std::memcpy(empirical_cloud->data.data()+i*empirical_cloud->point_step+axis*4,&value,4);}
    empirical_classes->push_back(static_cast<uint8_t>(empirical_frame->classes()[i]));
    empirical_origins->push_back((*qualified.frame.ray_origins)[i/16]);
  }
  empirical_packet.cloud=empirical_cloud;empirical_packet.classes=empirical_classes;
  // The expanded source must carry one matching origin per expanded point.
  empirical_packet.ray_origins=empirical_origins;
  require(empirical_packet.ray_origins->size()==empirical_cloud->width);
  sunhorizon_navigation_runtime::detail::attach_empirical_ground_support(empirical_packet,empirical_frame,context.owner);
  require(!empirical_packet.empirical_ground_support);
  sunhorizon_navigation_runtime::detail::attach_ground_validity(empirical_packet,empirical_frame);
  sunhorizon_navigation_runtime::detail::attach_empirical_ground_support(empirical_packet,empirical_frame,context.owner);
  require(empirical_packet.empirical_ground_support && empirical_packet.ground_validity);
  require(nav2_util::directCloudMatchesMetadata(empirical_packet));
  const auto& empirical_geometry=empirical_packet.empirical_ground_support->geometry_identity;
  require(empirical_geometry && empirical_geometry->valid());
  require(empirical_geometry->map_pcd_sha256==empirical_reference.map_asset.pcd_sha256 &&
    empirical_geometry->map_datum_id==empirical_reference.map_asset.datum_id &&
    empirical_geometry->common_frame==empirical_reference.common_frame &&
    empirical_geometry->alignment_reset_epoch==2 &&
    empirical_geometry->source_frame==empirical_source.identity.source_frame &&
    empirical_geometry->source_calibration_id==empirical_source.identity.calibration_id);
  for(int geometry_case=0;geometry_case<4;++geometry_case) {
    auto bad_packet=empirical_packet;
    auto bad_support=std::make_shared<nav2_util::DirectEmpiricalGroundSupport>(*bad_packet.empirical_ground_support);
    if(geometry_case==0)bad_support->geometry_identity->map_pcd_sha256.clear();
    if(geometry_case==1)bad_support->geometry_identity->source_frame="foreign_camera";
    if(geometry_case==2)bad_support->geometry_identity->source_calibration_id="foreign_K";
    if(geometry_case==3)bad_support->geometry_identity->map_datum_id.clear();
    bad_packet.empirical_ground_support=bad_support;
    require(!nav2_util::directCloudMatchesMetadata(bad_packet));
  }
  auto legacy_empirical=empirical_packet;
  auto legacy_support=std::make_shared<nav2_util::DirectEmpiricalGroundSupport>(*legacy_empirical.empirical_ground_support);
  legacy_support->geometry_identity.reset();legacy_empirical.empirical_ground_support=legacy_support;
  require(nav2_util::directCloudMatchesMetadata(legacy_empirical));
  require(nav2_util::directGroundValidityFresh(*empirical_packet.ground_validity,10020000000LL,20020000000LL));
  require(!nav2_util::directGroundValidityFresh(*empirical_packet.ground_validity,10100000001LL,20020000000LL));
  empirical_alignment->revoke();
  require(!nav2_util::directGroundValidityFresh(*empirical_packet.ground_validity,10020000000LL,20020000000LL));
  bool revoked_empirical_rejected=false;
  try {sunhorizon_navigation_runtime::detail::attach_empirical_ground_support(empirical_packet,empirical_frame,context.owner);}
  catch(const std::invalid_argument&) {revoked_empirical_rejected=true;}
  require(revoked_empirical_rejected);
  auto no_origins=input;no_origins.ray_origins.reset();
  const auto unseen=sunhorizon_navigation_runtime::detail::make_classified_airy_packet(no_origins,context,visible);
  require(!unseen.deliverable && unseen.ground_unavailable && !unseen.frame.ground_qualified && unseen.classification->clearing_indices().empty());
  for(auto kind:*unseen.frame.classes)require(kind==uint8_t(nav2_util::DirectPointClass::Uncertain));
  auto moving=input;auto moving_cloud=std::make_shared<sensor_msgs::msg::PointCloud2>(*input.cloud);
  moving.cloud=moving_cloud;
  const float rolling_offset=.01f,shifted_z=-.91f;
  for(std::size_t i=0;i<points.size();++i) {
    std::memcpy(moving_cloud->data.data()+19*i+8,&shifted_z,4);
    std::memcpy(moving_cloud->data.data()+19*i+15,&rolling_offset,4);
  }
  const auto unavailable=sunhorizon_navigation_runtime::detail::make_classified_airy_packet(moving,context,visible);
  require(!unavailable.deliverable && unavailable.ground_unavailable && !unavailable.frame.ground_qualified && unavailable.classification->clearing_indices().empty());
  int point_time_queries=0;
  context.source_transform_at=[&](int64_t original_ns) {
    ++point_time_queries;auto transform=context.source_to_reference;
    require(original_ns==10010000000LL);
    transform.source_ns=transform.common_ns=original_ns;transform.translation[2]=.01;
    return transform;
  };
  const auto compensated=sunhorizon_navigation_runtime::detail::make_classified_airy_packet(moving,context,visible);
  require(compensated.deliverable && !compensated.ground_unavailable && compensated.frame.ground_qualified && compensated.classification->clearing_indices().size()==25);
  require(point_time_queries==1 && compensated.classification->source().source_time_transforms.size()==1);
  require(compensated.classification->source().points.front().source_offset_ns==10000000);
  require(compensated.frame.cloud==moving.cloud && moving.metadata.source_ros_ns==input.metadata.source_ros_ns);
  // Native ordered groups and an out-of-order source must give the same
  // exact ascending unique provider list, retaining original point records and
  // first-encounter callback order (including nonadjacent duplicate stamps).
  for(bool ordered:{false,true}) {
    auto grouped=input;auto records=std::make_shared<sensor_msgs::msg::PointCloud2>(*input.cloud);grouped.cloud=records;
    std::vector<float> offsets(points.size());
    for(std::size_t i=0;i<offsets.size();++i)offsets[i]=i%4==0?.02f:i%4==1?.01f:i%4==2?.02f:0.f;
    if(ordered)std::sort(offsets.begin(),offsets.end());
    for(std::size_t i=0;i<offsets.size();++i){
      const float z=-.9f-offsets[i];std::memcpy(records->data.data()+19*i+8,&z,4);
      std::memcpy(records->data.data()+19*i+15,&offsets[i],4);
    }
    std::vector<int64_t> callbacks,queried;
    context.source_transform_at=[&](int64_t ns){callbacks.push_back(ns);auto transform=context.source_to_reference;
      transform.source_ns=transform.common_ns=ns;transform.translation[2]=double(ns-10000000000LL)/1.e9;return transform;};
    context.registration_geometry=[&](const gr::SourceIdentity& identity,const std::vector<int64_t>& times,bool scan){
      require(scan && identity.source_ns==input.metadata.source_ros_ns);queried=times;
      sunhorizon_perception_runtime::RegistrationGroundGeometryBatch batch;
      for(auto ns:times){auto transform=context.source_to_reference;transform.source_ns=transform.common_ns=ns;
        transform.translation[2]=double(ns-10000000000LL)/1.e9;batch.transforms.push_back(transform);}return batch;};
    const auto packet=sunhorizon_navigation_runtime::detail::make_classified_airy_packet(grouped,context,visible);
    require(queried==std::vector<int64_t>({10000000000LL,10010000000LL,10020000000LL}));
    require(callbacks==(ordered?std::vector<int64_t>({10010000000LL,10020000000LL}):
      std::vector<int64_t>({10020000000LL,10010000000LL})));
    require(packet.frame.cloud==grouped.cloud && packet.frame.ground_qualified);
    require(packet.classification->clearing_indices().size()==points.size());
    for(std::size_t i=0;i<points.size();++i){const auto& point=packet.classification->source().points[i];
      require(point.source_offset_ns==std::llround(double(offsets[i])*1.e9));
      require(point.sample_id==i && !point.ground_blocked);
      require((*packet.frame.ray_origins)[i]==(*grouped.ray_origins)[i]);}
  }
  context.registration_geometry={};
  context.source_transform_at={};
  // A retained distant endpoint must preserve its range-dependent source error
  // even while excluded from support/clearing. Synthetic bounds only.
  doc["sources"][0]["transform_angle_error_bound_rad"]=.005;context.owner=load();
  auto duplicate_points=points;duplicate_points.push_back({NAN,0,0});duplicate_points.push_back({50.,50.,-.9f});
  const auto with_mask=source(duplicate_points);
  auto mask=std::make_shared<std::vector<uint8_t>>(duplicate_points.size(),0);mask->back()=1;
  auto original_ids=std::make_shared<std::vector<uint64_t>>();
  for(std::size_t i=0;i<duplicate_points.size();++i)original_ids->push_back(100+i);
  const auto blocked=sunhorizon_navigation_runtime::detail::make_classified_airy_packet(with_mask,context,mask,original_ids);
  require(blocked.deliverable && !blocked.ground_unavailable && blocked.frame.ground_qualified && blocked.classification->clearing_indices().size()==25);
  require(blocked.invalid_points==1 && blocked.finite_points==26);
  require(blocked.classification->source().points.back().kind==gr::ReturnKind::Measured);
  require(blocked.classification->source().points.back().ground_blocked);
  require(blocked.classification->source().points.back().sample_id==126);
  require(blocked.classification->classes().back()==gr::PointClass::Uncertain);
  require(blocked.classification->marking_indices().empty());
  require(blocked.classification->protected_hit_indices()==std::vector<std::size_t>({25}));
  const auto& endpoint=blocked.classification->source().points.back().position;
  const double expected_error=2.*std::hypot(endpoint[0],endpoint[1],endpoint[2])*std::sin(.005/2.);
  require(blocked.classification->error_bounds_m().back()>=expected_error);
  auto packet=blocked.frame;
  sunhorizon_navigation_runtime::detail::attach_ground_clearance(packet,blocked.classification,context.owner);
  require(packet.mark_error_bound_qualified && packet.mark_error_bound_m>=expected_error);
  require(blocked.frame.classes->back()==uint8_t(nav2_util::DirectPointClass::Uncertain));
  geometry_audit(blocked,with_mask);
  rgbd_projection_audit(blocked,context.owner->config());
  context.source_to_reference.qualified=false;
  const auto lost=sunhorizon_navigation_runtime::detail::make_classified_airy_packet(input,context,visible);
  require(!lost.deliverable && lost.ground_unavailable && !lost.frame.ground_qualified && lost.classification->clearing_indices().empty());
  context.registration_geometry=[](const gr::SourceIdentity&,const std::vector<int64_t>&,bool) {
    sunhorizon_perception_runtime::RegistrationGroundGeometryBatch failed;
    failed.diagnostic={5,"pcd_registration_original_tracking_evidence_expired"};return failed;
  };
  const auto provider_lost=sunhorizon_navigation_runtime::detail::make_classified_airy_packet(input,context,visible);
  require(!provider_lost.deliverable && provider_lost.ground_unavailable && !provider_lost.frame.ground_qualified && provider_lost.classification->clearing_indices().empty());
  require(provider_lost.classification->ground().result==gr::Qualification::InvalidTransform);
  require(provider_lost.classification->ground().transform_diagnostic.provider_state==5);
  require(provider_lost.classification->ground().transform_diagnostic.reason==
    "pcd_registration_original_tracking_evidence_expired");
  context.registration_geometry=[](const gr::SourceIdentity&,const std::vector<int64_t>&,bool) {
    sunhorizon_perception_runtime::RegistrationGroundGeometryBatch failed;
    failed.diagnostic={6,"pcd_query_original_measurement_expired"};return failed;
  };
  const auto expired=sunhorizon_navigation_runtime::detail::make_classified_airy_packet(input,context,visible);
  require(!expired.deliverable && expired.ground_unavailable && !expired.frame.ground_validity);
  require(expired.frame.cloud==input.cloud && nav2_util::directMetadataEqual(expired.frame.metadata,input.metadata));
  require(expired.classification->ground().transform_diagnostic.provider_state==6);
  consumer_leases(conservative,expired);
}
}
void writer_capture_region() {
  std::vector<geometry_msgs::msg::Point> footprint(4);
  footprint[0].x=-.224771;footprint[0].y=-.355515;
  footprint[1].x=-.224771;footprint[1].y=.355515;
  footprint[2].x=.25005;footprint[2].y=.355515;
  footprint[3].x=.25005;footprint[3].y=-.355515;
  const auto selected=[&](double x,double y,double yaw){return nav2_util::directWriterFootprintRegion(x,y,0,0,yaw,.05,footprint);};
  require(selected(.133,.272,0)); // L1 was outside the old +/-10cm hook.
  require(selected(-.272,.133,1.5707963267948966)); // Same physical point after a quarter turn.
  require(!selected(.133,.58,0)); // Distant left cluster cannot consume the token.
  require(selected(.27,0,0)&&!selected(.28,0,0)); // Half-cell boundary coverage.
  require(!nav2_util::directWriterFootprintRegion(0,0,0,0,0,.05,{}));
  require(!nav2_util::directWriterFootprintRegion(0,0,0,0,0,0,footprint));
}
int main() {
  rclcpp::init(0,nullptr);
  struct Shutdown {~Shutdown(){rclcpp::shutdown();}} shutdown;
  try{writer_capture_region();mixed_returns();finite_and_missing();malformed();shared_ground_owner();std::cout<<"PASS classified Airy finite records and original identity\n";return 0;}
  catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
