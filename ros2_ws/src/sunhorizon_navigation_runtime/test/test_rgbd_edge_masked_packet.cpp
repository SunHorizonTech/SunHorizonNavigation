#include "sunhorizon_navigation_runtime/detail/rgbd_projection_audit.hpp"
#include <nav2_costmap_2d/voxel_measurement_support.hpp>
#include <nav2_costmap_2d/observation_buffer.hpp>
#include "sunhorizon_navigation_runtime/detail/near_body_endpoints.hpp"
#include <nav2_collision_monitor/pointcloud.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <stdexcept>
#include <string>

namespace gr=sunhorizon_perception_runtime::ground_relative;
namespace detail=sunhorizon_navigation_runtime::detail;
namespace {
void check_at(bool value,int line) {
  if(!value)throw std::runtime_error("RGBD packet check failed at line "+std::to_string(line));
}
#define check(value) check_at((value),__LINE__)
template<class F> void rejects(F&& f) {
  bool rejected=false;
  try {f();}catch(const std::invalid_argument&){rejected=true;}
  check(rejected);
}
}
void body_occluded_packet() {
  gr::SourceFrame source;source.identity={"belly","epoch","calibration","camera_optical","map",1,1000000000,1000000000,5000000000,500000000};
  source.raster_geometry={640,480,1};
  gr::PointReturn external{{.1,.1,1.},gr::ReturnKind::Measured};external.sample_id=1;
  auto rejected=external;rejected.sample_id=2;rejected.kind=gr::ReturnKind::Invalid;
  rejected.ground_blocked=rejected.measurement_body_occluded=true;
  auto other=external;other.sample_id=3;source.points={external,rejected,other};
  source.body_occlusion={true,true,source.identity,640,480,.05,{2}};
  const auto sealed=gr::conservative_unqualified_frame(source);
  check(!sealed->source().measurement_quality.present); // no IR pair is required or invented
  const auto code=detail::rgbd_audit_direct_class(sealed->classes()[1],sealed->source(),1,sealed->decisions()[1]);
  check(code==uint8_t(nav2_util::DirectPointClass::Rejected));
  check(sealed->decisions()[1]==gr::PointDecision::MeasurementBodyOccluded);
  check(sealed->source().self_support.empty()&&sealed->marking_indices().empty()&&sealed->clearing_indices().empty());
  rejects([&]{detail::rgbd_audit_direct_class(gr::PointClass::Invalid,sealed->source(),1);});
  const auto refused=[&](const gr::SourceFrame& bad){rejects([&]{gr::conservative_unqualified_frame(bad);});};
  auto bad=source;bad.body_occlusion={};refused(bad);
  bad=source;bad.body_occlusion.identity.epoch+="bad";refused(bad);
  bad=source;bad.points[1].sample_id=4;refused(bad);
  bad=source;bad.points[1].source_offset_ns=1;refused(bad);
  bad=source;bad.points[1].measurement_unreliable=true;refused(bad); // missing IR proof cannot borrow body proof
  // Actual buffer consumes the same unified mapping. Two independently measured
  // obstacle samples share a voxel with the rejected endpoint; it contributes no vote/clear.
  rclcpp::init(0,nullptr);
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"near.input_mode","direct-classified-v1"},{"near.direct_source_id","belly"},
      {"near.direct_epoch","epoch"},{"near.direct_frame","camera_optical"},
      {"near.direct_maximum_age_sec",.5}});
    auto node=std::make_shared<nav2_util::LifecycleNode>("body_occlusion_packet_test","",options);
    const auto now=node->now().nanoseconds();source.identity.source_ns=now-1000000;
    source.identity.first_receipt_ros_ns=now;source.identity.first_receipt_steady_ns=nav2_util::directSteadyNowNs();
    source.body_occlusion.identity=source.identity;const auto current=gr::conservative_unqualified_frame(source);
    auto cloud=std::make_shared<sensor_msgs::msg::PointCloud2>();cloud->header.frame_id="camera_optical";
    cloud->header.stamp=rclcpp::Time(source.identity.source_ns,node->get_clock()->get_clock_type());
    sensor_msgs::PointCloud2Modifier mod(*cloud);mod.setPointCloud2FieldsByString(1,"xyz");mod.resize(3);
    sensor_msgs::PointCloud2Iterator<float> x(*cloud,"x"),y(*cloud,"y"),z(*cloud,"z");
    for(int i=0;i<3;++i){*x=.1F*(i+1);*y=.1F;*z=1.F;++x;++y;++z;}
    auto classes=std::make_shared<std::vector<uint8_t>>();
    // Explicit independent obstacle input labels are not derived from the rejected sample.
    classes->push_back(detail::rgbd_audit_direct_class(gr::PointClass::Obstacle,current->source(),0));
    classes->push_back(detail::rgbd_audit_direct_class(current->classes()[1],current->source(),1,current->decisions()[1]));
    classes->push_back(detail::rgbd_audit_direct_class(gr::PointClass::Obstacle,current->source(),2));
    nav2_util::DirectCloud packet{cloud,{"belly","epoch","camera_optical",1,source.identity.source_ns,now,source.identity.first_receipt_steady_ns}};
    packet.kind=nav2_util::DirectCloudKind::ClassifiedObstacleFrame;packet.classes=classes;packet.complete=true;
    auto tf_owner=std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto& tf=*tf_owner;tf.setUsingDedicatedThread(true);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id="map";transform.child_frame_id="camera_optical";transform.header.stamp=node->now();transform.transform.rotation.w=1.;
    check(tf.setTransform(transform,"test",true));
    nav2_costmap_2d::ObservationBuffer buffer(node,"body",0.,0.,.2,2.,4.,.1,5.,.1,tf,"map","camera_optical",tf2::durationFromSec(0.));
    buffer.enableDirectInput({"belly","epoch","camera_optical",500000000LL},packet.kind);
    check(buffer.bufferDirectCloud(packet));std::vector<nav2_costmap_2d::Observation> observations;
    check(buffer.getObservations(observations)&&observations.size()==1);
    const auto& obs=observations[0];check(obs.cloud_->width==3);
    check(obs.shouldMark(0)&&obs.shouldMark(2));
    check(!obs.shouldMark(1)&&!obs.shouldClear(1)&&!obs.shouldProtectEndpoint(1));
    // Both consumers get the very same labels and lease. Point 0 has a body
    // witness, point 1 was already rejected, and point 2 remains an obstacle.
    auto filtered=detail::exclude_near_body_endpoints(packet,{0});
    check(filtered.cloud==packet.cloud && filtered.ground_validity==packet.ground_validity);
    check(nav2_util::directMetadataEqual(filtered.metadata,packet.metadata));
    check(detail::exclude_near_body_endpoints(packet,{}).classes==packet.classes);
    check(packet.classes->at(0)==uint8_t(nav2_util::DirectPointClass::Obstacle));
    check(filtered.classes->at(0)==uint8_t(nav2_util::DirectPointClass::Rejected));
    nav2_collision_monitor::PointCloud collision(node,"near",tf_owner,"map","map",
      tf2::durationFromSec(0.),rclcpp::Duration::from_seconds(.5),true);
    collision.configure();
    nav2_costmap_2d::ObservationBuffer masked(node,"masked",0.,0.,.2,2.,4.,.1,5.,.1,tf,"map","camera_optical",tf2::durationFromSec(0.));
    masked.enableDirectInput({"belly","epoch","camera_optical",500000000LL},packet.kind);
    check(masked.bufferDirectCloud(filtered) && collision.submitDirectCloud(filtered));
    std::vector<nav2_costmap_2d::Observation> after;
    check(masked.getObservations(after) && after.size()==1);
    check(!after[0].shouldMark(0) && !after[0].shouldClear(0) && after[0].shouldMark(2));
    std::vector<nav2_collision_monitor::Point> collision_points;
    check(collision.getData(node->now(),collision_points) && collision_points.size()==1);
    check(std::abs(collision_points.front().x-.3)<1.e-6);
    nav2_costmap_2d::voxel_support::Frame baseline,votes;
    check(baseline.add(7,classes->at(0),1)&&baseline.add(7,classes->at(2),3));
    const auto before=baseline.cells.at(7);check(before.supported());
    check(votes.add(7,classes->at(0),1));
    check(votes.add(7,classes->at(1),2));
    check(votes.add(7,classes->at(2),3));
    check(votes.cells.at(7).obstacle==before.obstacle&&votes.cells.at(7).valid()==before.valid()&&
      votes.cells.at(7).self==before.self&&votes.cells.at(7).supported()==before.supported());
  }
  rclcpp::shutdown();
}
int main() {
  body_occluded_packet();
  gr::SourceFrame source;
  source.identity={"belly","camera-epoch","calibration","camera_optical","base_footprint",
    42,1000000000,1001000000,5000000000,500000000};
  source.raster_geometry={640,480,1};
  gr::PointReturn measured{{.1,.1,1.},gr::ReturnKind::Measured};
  measured.sample_id=1;
  auto edge=measured;edge.kind=gr::ReturnKind::Self;edge.ground_blocked=true;edge.sample_id=2;
  source.points={measured,edge};
  source.self_support_complete=true;source.original_projected_samples=2;
  const auto frame=gr::conservative_unqualified_frame(source);
  check(frame->classification_complete() && frame->source().self_support.empty());
  check(frame->classes()[0]==gr::PointClass::Uncertain && frame->classes()[1]==gr::PointClass::Self);
  check(frame->source().points.size()==2 && frame->source().points[1].sample_id==2 &&
    frame->source().points[1].position==edge.position);
  check(frame->marking_indices().empty() && frame->clearing_indices().empty());
  check(frame->protected_hit_indices()==std::vector<std::size_t>{0});
  check(frame->decisions()[1]==gr::PointDecision::ModelProjectionEdge);
  const auto rejected=detail::rgbd_audit_direct_class(frame->classes()[1],frame->source(),1,frame->decisions()[1]);
  check(rejected==uint8_t(nav2_util::DirectPointClass::Rejected));
  nav2_costmap_2d::Observation observation;
  observation.direct_kind_=nav2_util::DirectCloudKind::ClassifiedObstacleFrame;
  observation.classes_=std::make_shared<const std::vector<uint8_t>>(1,rejected);
  check(!observation.shouldMark(0) && !observation.shouldClear(0) && !observation.shouldProtectEndpoint(0));
  nav2_costmap_2d::voxel_support::Frame votes;
  check(votes.add(7,rejected,edge.sample_id) && votes.valid() && votes.cells.empty());
  // Ordinary Self, mismatched source class, and an absent original record
  // cannot borrow the native model-edge path. Quality rejection stays separate.
  const auto map_edge=[&](const gr::SourceFrame& value,std::size_t index=1) {
    return detail::rgbd_audit_direct_class(gr::PointClass::Self,value,index,gr::PointDecision::ModelProjectionEdge);
  };
  auto changed=source;changed.points[1].ground_blocked=false;
  rejects([&]{map_edge(changed);});
  changed=source;changed.points[1].kind=gr::ReturnKind::Measured;
  rejects([&]{map_edge(changed);});
  changed=source;changed.points[1].model_conflict=true;
  rejects([&]{map_edge(changed);});
  changed=source;changed.points[1].measurement_unreliable=true;
  rejects([&]{map_edge(changed);});
  rejects([&]{map_edge(source,source.points.size());});
  rejects([&]{detail::rgbd_audit_direct_class(gr::PointClass::Masked,source,1);});
  rejects([&]{detail::rgbd_audit_direct_class(gr::PointClass::Invalid,source,1);});
  rejects([&]{detail::rgbd_audit_direct_class(gr::PointClass::Self,source,1);});
  // Even far-away mixed depths cannot add physical Self denominator samples,
  // clear a ray, or suppress an independent obstacle in the same voxel.
  check(votes.add(7,uint8_t(nav2_util::DirectPointClass::Obstacle),3));
  check(votes.add(7,uint8_t(nav2_util::DirectPointClass::Obstacle),4));
  const auto before=votes.cells.at(7);
  for(uint64_t id=10;id<60;++id)check(votes.add(7,rejected,id));
  check(votes.cells.size()==1 && votes.cells.at(7).supported() &&
    votes.cells.at(7).obstacle==before.obstacle && votes.cells.at(7).valid()==before.valid() &&
    votes.cells.at(7).self==0);
  auto legacy=source;legacy.points[1].kind=gr::ReturnKind::Masked;
  const auto legacy_frame=gr::conservative_unqualified_frame(legacy);
  check(detail::rgbd_audit_direct_class(legacy_frame->classes()[1],legacy_frame->source(),1,
    legacy_frame->decisions()[1])==rejected);
  check(detail::rgbd_audit_direct_class(gr::PointClass::Uncertain,source,0)==
    uint8_t(nav2_util::DirectPointClass::Uncertain));
  std::cout<<"RGBD edge Self packet and legacy masked checks passed\n";
}
