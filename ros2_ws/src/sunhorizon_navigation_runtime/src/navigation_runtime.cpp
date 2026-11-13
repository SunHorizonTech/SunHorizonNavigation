#include "sunhorizon_navigation_runtime/navigation_runtime.hpp"
#include "sunhorizon_navigation_runtime/bounded_lane.hpp"
#include "sunhorizon_navigation_runtime/detail/executor_options.hpp"
#include "sunhorizon_navigation_runtime/detail/joint_context.hpp"
#include "sunhorizon_navigation_runtime/detail/direct_camera_identity.hpp"
#include "sunhorizon_navigation_runtime/detail/original_source_geometry_wait.hpp"
#include "sunhorizon_navigation_runtime/detail/registration_ground_geometry.hpp"
#include "sunhorizon_navigation_runtime/detail/collision_height_parameters.hpp"
#include "sunhorizon_camera_runtime/latest_frame_worker.hpp"
#include "sunhorizon_camera_runtime/direct_frame.hpp"
#include "sunhorizon_perception_runtime/joint_ingress.hpp"
#include "sunhorizon_perception_runtime/measured_waist_geometry.hpp"
#include "sunhorizon_rgbd_adapter/dynamic_self_model.hpp"
#include <fast_livo/direct_runtime.hpp>
#include <epoch_timing.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rslidar_sdk/direct_driver.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <fstream>
#include <sstream>
#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>
#include <utility>
#include <variant>
#include "sunhorizon_navigation_runtime/vendor_admission.hpp"
#include <unistd.h>

namespace sunhorizon_navigation_runtime {
namespace airy=rslidar_sdk_direct;
namespace normalization=sunhorizon_airy_adapter;
namespace vendor=fast_livo_direct;
namespace livo=sunhorizon_livo_bringup;
namespace ndt=sunhorizon_3d_localizer;
namespace camera=sunhorizon_camera_runtime;
namespace manager=sunhorizon_localization_manager;
namespace perception=sunhorizon_perception_runtime;
namespace {
using Camera=camera::CameraFrame::ConstSharedPtr;
struct LivoCandidate {nav_msgs::msg::Odometry::ConstSharedPtr message;livo::LivoDirectStamp stamp;};
struct BackendStatus {sunhorizon_navigation_interfaces::msg::LocalizationStatus::ConstSharedPtr message;manager::LocalizationInputStamp stamp;};
struct NdtDelivery {std::shared_ptr<const ndt::NdtDirectResult> result;int64_t enqueued_ns{0};std::size_t pending_before_admit{0};};
using ManagerInput=std::variant<LivoCandidate,NdtDelivery,BackendStatus>;
using CanonicalOutput=std::variant<std::shared_ptr<const manager::CanonicalLocalizationState>,std::shared_ptr<const manager::CanonicalLocalizationHealth>>;
using VendorOutput=std::variant<vendor::CorrectedState::ConstSharedPtr,vendor::PropagatedState::ConstSharedPtr>;
struct VendorDelivery {
  VendorOutput output;
  int64_t enqueued_steady_ns{};
};
std::int64_t ns(const builtin_interfaces::msg::Time & t){return std::int64_t(t.sec)*1000000000LL+t.nanosec;}
std::string process_startticks(){
  std::ifstream stream("/proc/self/stat");std::string line,field;
  if(!std::getline(stream,line))throw std::runtime_error("direct_owner_process_stat_missing");
  const auto end=line.rfind(')');
  if(end==std::string::npos)throw std::runtime_error("direct_owner_process_stat_invalid");
  std::istringstream fields(line.substr(end+2));
  for(unsigned i=0;i<20;++i)if(!(fields>>field))throw std::runtime_error("direct_owner_process_startticks_missing");
  if(field.empty()||field.find_first_not_of("0123456789")!=std::string::npos||field=="0")
    throw std::runtime_error("direct_owner_process_startticks_invalid");
  return field;
}
std::string gid_hex(const uint8_t* data,std::size_t count){
  static constexpr char digits[]="0123456789abcdef";std::string value;value.reserve(count*2);
  for(std::size_t i=0;i<count;++i){value.push_back(digits[data[i]>>4]);value.push_back(digits[data[i]&15]);}
  return value;
}
rclcpp::NodeOptions observer_option(rclcpp::NodeOptions options,const char * name,bool enabled){
  auto parameters=options.parameter_overrides();
  parameters.erase(std::remove_if(parameters.begin(),parameters.end(),[name](const auto & p){return p.get_name()==name;}),parameters.end());
  parameters.emplace_back(name,enabled);options.parameter_overrides(parameters);return options;
}
rclcpp::NodeOptions kinematics_options(rclcpp::NodeOptions options,const std::string & mode){
  if(mode!="production_direct"&&mode!="attended_direct"&&mode!="attended_measured_direct")
    throw std::invalid_argument("unified runtime requires explicit production_direct or receipted attended_direct TF mode");
  auto parameters=options.parameter_overrides();
  for(const auto & p:parameters)if(p.get_name()=="kinematics_tf_mode"&&p.as_string()!=mode)
    throw std::invalid_argument("unified runtime TF mode conflicts with component NodeOptions");
  parameters.erase(std::remove_if(parameters.begin(),parameters.end(),[](const auto & p){return p.get_name()=="kinematics_tf_mode";}),parameters.end());
  parameters.emplace_back("kinematics_tf_mode",mode);
  options.parameter_overrides(parameters);return options;
}
vendor::InputStamp vendor_stamp(const livo::LivoDirectStamp & s){
  return {s.source_stamp_ns,s.receipt_ros_ns,s.receipt_steady_ns,s.sequence,s.source_identity};
}
ndt::NdtDirectStamp ndt_stamp(const livo::LivoDirectStamp & s,const std::string & owner){
  return {s.source_stamp_ns,s.receipt_ros_ns,s.receipt_steady_ns,s.sequence,owner};
}
}
struct NavigationRuntime::Impl {
  NavigationRuntimeOptions options;NavigationRuntimeSinks sinks;
  rclcpp::Node::SharedPtr control,camera_owner;
  std::shared_ptr<tf2_ros::Buffer> transforms;
  std::shared_ptr<perception::PerceptionRuntime> perception_owner;
  bool collision_height_bound{false};
  int64_t original_tracking_maximum_age_ns{0};
  std::shared_ptr<perception::JointIngress> joints;
  std::shared_ptr<sunhorizon_obstacle_coverage::CoverageDirectRuntime> coverage_owner;
  struct CoverageCloud {sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;int64_t receipt_ros_ns;};
  std::unique_ptr<camera::LatestFrameWorker<CoverageCloud>> coverage_belly,coverage_airy;
  std::unique_ptr<detail::BoundedLane<std::shared_ptr<const perception::RegionDiagnosticOutput>>> coverage_region;
  std::shared_ptr<vendor::DirectRuntime> vendor_owner;
  std::shared_ptr<livo::LivoDirectAdapter> adapter;
  std::shared_ptr<ndt::NdtDirectRuntime> ndt_owner;
  std::shared_ptr<manager::LocalizationDirectRuntime> manager_owner;
  std::shared_ptr<airy::DirectDriver> airy_owner;
  std::shared_ptr<normalization::AiryDirectAdapter> airy_adapter;
  std::unique_ptr<detail::BoundedLane<airy::CloudFrame::ConstSharedPtr>> point_lane;
  std::unique_ptr<detail::BoundedLane<airy::ImuFrame::ConstSharedPtr>> imu_lane;
  std::unique_ptr<detail::BoundedLane<Camera>> image_lane;
  std::unique_ptr<detail::BoundedLane<VendorDelivery>> vendor_output_lane;
  std::unique_ptr<detail::BoundedLane<ManagerInput>> manager_lane;
  std::unique_ptr<detail::BoundedLane<CanonicalOutput>> canonical_lane;
  struct NdtDeliveryTiming {
    int64_t sequence{},source_ns{},input_receipt_ns{},result_receipt_ns{},enqueue_ns{},dequeue_ns{},
      registration_end_ns{},waist_end_ns{},map_pose_end_ns{};
    int64_t input_receipt_ros_ns{},result_receipt_ros_ns{};
    std::size_t pending_before_admit{},pending_at_dequeue{};
    int waist_lookup_status{-1};std::size_t waist_transform_count{0};
    int waist_status{-1};bool registration_admitted{false},map_pose_admitted{false};
  };
  std::mutex ndt_delivery_timing_mutex;
  std::array<NdtDeliveryTiming,16> ndt_delivery_timings{};
  std::size_t ndt_delivery_timing_next{0};
  detail::DirectCameraIdentity camera_evidence;
  // Retain only completed immutable outputs. Formatting stays on the 1 Hz
  // observer; delivery never waits for ROS diagnostics or copies point arrays.
  struct VendorTiming {
    vendor::CorrectedState::ConstSharedPtr corrected;
    vendor::PropagatedState::ConstSharedPtr propagated;
    std::int64_t corrected_enter_ns{0},corrected_exit_ns{0};
    std::int64_t propagated_enter_ns{0},propagated_exit_ns{0};
  };
  std::mutex vendor_timing_mutex;
  VendorTiming vendor_timing;
  // Bounded evidence only: no point arrays retained, no formatting on input owners.
  struct AiryInputTiming {
    std::uint64_t sequence{},sdk_sequence{};
    int64_t source_ns{},receipt_ros_ns{},receipt_steady_ns{},conversion_begin_ns{},conversion_end_ns{};
    sunhorizon_timing::Sample dispatch,completed;
  };
  struct AiryNormalizationTiming {
    normalization::AiryDirectStamp stamp;
    int64_t fanout_begin_ns{},livo_begin_ns{},livo_end_ns{};
  };
  std::mutex airy_timing_mutex;
  std::array<AiryInputTiming,16> airy_input_timings{};
  std::array<AiryNormalizationTiming,16> airy_normalization_timings{};
  std::size_t airy_input_next{},airy_normalization_next{};
  std::array<vendor::CorrectedState::ConstSharedPtr,16> corrected_frame_timings{};
  std::size_t corrected_frame_next{};
  diagnostic_msgs::msg::DiagnosticStatus identity_static;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr identity_publisher;
  std::unique_ptr<camera::LatestFrameWorker<diagnostic_msgs::msg::DiagnosticArray>> identity_observer;
  rclcpp::TimerBase::SharedPtr identity_timer;
  std::vector<rclcpp::Node::SharedPtr> control_nodes;
  std::vector<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr> extra_control_nodes;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> joint_executor;
  std::thread control_thread,joint_thread;
  std::string airy_cloud_identity,airy_imu_identity,camera_identity,corrected_identity,propagated_identity,candidate_identity;
  std::string ndt_identity,manager_identity,ndt_backend;
  std::atomic<std::uint64_t> ndt_status_sequence{0};
  bool livo_alignment_enabled{false};
  std::optional<geometry_msgs::msg::Transform> ndt_nominal_source_from_base;
  std::atomic<bool> inputs_ready{false},terminal{false},stopping{false};
  bool started{false},ndt_accepts_prior{true};
  mutable std::mutex reason_mutex;std::mutex lifecycle;
  std::string reason;
  Impl(NavigationRuntimeOptions config,NavigationRuntimeSinks output)
    :options(std::move(config)),sinks(std::move(output))
  {
    control=std::make_shared<rclcpp::Node>("sunhorizon_navigation_runtime",options.runtime_node);
    if(control->get_parameter("use_sim_time").as_bool())throw std::invalid_argument("navigation runtime requires realtime clocks");
    if(options.airy_config_path.empty())options.airy_config_path=control->declare_parameter<std::string>("airy_config_path","");
    if(options.airy_config_path.empty())throw std::invalid_argument("navigation runtime requires the accepted Airy SDK YAML");
    transforms=options.direct_tf_buffer?options.direct_tf_buffer:
      std::make_shared<tf2_ros::Buffer>(control->get_clock(),tf2::durationFromSec(2.));
    transforms->setUsingDedicatedThread(true);
    try{
      // This owner installs validated direct TF below. Do not start a temporary
      // ROS listener only to cancel/join it during the same construction.
      perception_owner=std::make_shared<perception::PerceptionRuntime>(
        observer_option(observer_option(kinematics_options(options.perception_node,options.kinematics_tf_mode),"publish_body_clouds",options.debug_publish_body_clouds),"publish_raw_projection_clouds",false),true,false);
      if(options.defer_spatial_evidence_until_startup)perception_owner->defer_spatial_startup();
      if(!(options.spatial_query==perception_owner->spatial_query_policy()))
        throw std::invalid_argument("entry_perception_spatial_query_policy_mismatch");
      if(options.spatial_query.height_limited()){
        const auto& joints=perception_owner->kinematics()->options();
        if(!options.defer_spatial_evidence_until_startup || joints.tf_mode!="attended_direct")
          throw std::invalid_argument("collision_height_requires_deferred_attended_unified_runtime");
        std::size_t identities=0;
        for(const auto& edge:joints.attended_static_edges)if(edge.child_frame_id=="base_link"){
          if(edge.header.frame_id!="base_footprint")
            throw std::invalid_argument("collision_height_requires_fixed_base_parent");
          const auto& t=edge.transform;
          perception::detail::require_exact_base_identity(t.translation.x,t.translation.y,t.translation.z,
            t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w);++identities;
        }
        if(identities!=1)throw std::invalid_argument("collision_height_requires_one_accepted_base_identity");
      }
      // Before JointIngress exists or camera construction can start acquisition.
      perception_owner->configure_direct_transforms(transforms,sinks.tf_compatibility);
      sunhorizon_obstacle_coverage::CoverageDirectOptions coverage;
      coverage.node_options=options.coverage_node;coverage.transforms=transforms;
      coverage.spatial_owner=perception_owner;coverage.spatial_publisher=perception_owner->spatial_publisher();
      coverage.producer_failed=[weak=std::weak_ptr<perception::PerceptionRuntime>(perception_owner)]{
        const auto owner=weak.lock();return !owner || owner->failed();};
      coverage_owner=sunhorizon_obstacle_coverage::make_coverage_direct_runtime(std::move(coverage));
      require_obstacle_coverage_profile(options.obstacle_source_mode,
        coverage_owner->node()->get_parameter("profile").as_string(),
        coverage_owner->node()->get_parameter("spatial_visibility_required").as_bool());
      auto coverage_failure=[this](std::exception_ptr error){try{if(error)std::rethrow_exception(error);}
        catch(const std::exception& e){fault(std::string("coverage:")+e.what());}catch(...){fault("coverage_delivery_failed");}};
      auto allowed=[this]{return !stopping.load()&&!terminal.load();};
      coverage_belly=std::make_unique<camera::LatestFrameWorker<CoverageCloud>>(
        [this](auto& input){coverage_owner->receive_belly(input.cloud,input.receipt_ros_ns);},allowed,coverage_failure);
      if(obstacle_uses_airy(options.obstacle_source_mode))coverage_airy=std::make_unique<camera::LatestFrameWorker<CoverageCloud>>(
        [this](auto& input){coverage_owner->receive_airy(input.cloud,input.receipt_ros_ns);},allowed,coverage_failure);
      coverage_region=std::make_unique<detail::BoundedLane<std::shared_ptr<const perception::RegionDiagnosticOutput>>>(16,
        [this](auto input){if(!stopping.load())coverage_owner->receive_region(input->diagnostics);},coverage_failure);
      perception_owner->set_health_output_sink([this](auto status){if(!stopping.load())coverage_owner->receive_rgbd(std::move(status));});
      perception_owner->set_spatial_output_sink([this](auto evidence){
        using State=sunhorizon_obstacle_coverage::SpatialDeliveryState;
        if(stopping.load())return sunhorizon_obstacle_coverage::SpatialDeliveryResult{
          State::MotionUnavailable,"navigation_stopping"};
        const auto result=coverage_owner->receive_spatial(std::move(evidence));
        // receive_spatial released the tracker lock before invoking any parent
        // fault path, so revocation cannot recursively acquire that same lock.
        sunhorizon_obstacle_coverage::propagate_spatial_internal_fault(result,
          [this](const std::string& reason){fault("coverage_spatial:"+reason);});
        return result;
      });
      perception_owner->set_output_sink([this](auto result){
        if(stopping.load()||terminal.load())return;
        if(result && result->input->role=="belly" && result->frame.body_filter.complete){
          auto cloud=std::shared_ptr<const sensor_msgs::msg::PointCloud2>(result,&result->frame.body_cloud);
          auto pending=std::make_unique<CoverageCloud>(CoverageCloud{cloud,result->input->first_sdk_callback_ros_ns});
          const auto* offered=pending.get();auto retired=coverage_belly->replace(std::move(pending));
          if(retired.get()==offered)throw std::runtime_error("coverage_belly_delivery_closed");
        }
        if(sinks.perception_output)sinks.perception_output(std::move(result));
      });
      perception_owner->set_region_output_sink([this](auto result){
        if(stopping.load()||terminal.load())return;
        if(!coverage_region->admit(result))throw std::runtime_error("coverage_region_delivery_overflow_or_closed");
        if(sinks.region_output)sinks.region_output(std::move(result));
      });
      auto failure=[this](std::exception_ptr error){try{if(error)std::rethrow_exception(error);}
        catch(const std::exception & e){fault(e.what());}catch(...){fault("unknown_navigation_lane_failure");}};
      canonical_lane=std::make_unique<detail::BoundedLane<CanonicalOutput>>(128,
        [this](auto state){if(!inputs_ready.load()||stopping.load()||terminal.load())return;
          std::visit([this](auto value){receive_canonical(std::move(value));},std::move(state));},failure);
      manager::LocalizationDirectSinks manager_sinks;
      manager_sinks.tracking_reset=[this](int64_t barrier){if(adapter)adapter->reset_body_velocity(barrier);};
      if(options.kinematics_tf_mode=="attended_measured_direct")
        manager_sinks.source_time_source_from_base=[this](int64_t stamp,const auto& nominal){return measured_source_from_base(stamp,nominal);};
      manager_sinks.publish_ros_compatibility=options.publish_ros_canonical_compatibility;
      manager_sinks.canonical=[this](auto state){admit_canonical(CanonicalOutput{std::move(state)});};
      manager_sinks.status=[this](auto state){admit_canonical(CanonicalOutput{std::move(state)});};
      manager_owner=manager::make_localization_direct_runtime(options.localization_manager_node,std::move(manager_sinks));
      manager_identity=manager_owner->node()->get_fully_qualified_name();
      if(options.spatial_query.height_limited() || options.kinematics_tf_mode=="attended_measured_direct"){
        const auto manager=manager_owner->node();
        if(manager->get_parameter("map_frame").as_string()!="map" ||
            manager->get_parameter("odom_frame").as_string()!="odom" ||
            manager->get_parameter("base_footprint_frame").as_string()!="base_footprint")
          throw std::invalid_argument("collision_height_requires_fixed_canonical_frames");
        detail::freeze_collision_height_parameters(*manager,{"map_frame","odom_frame","base_footprint_frame"},{});
      }
      // The manager freezes these frames at construction. Its two dynamic
      // edges must never replace the accepted immutable attended geometry.
      const auto& joint_contract=perception_owner->kinematics()->options();
      if(joint_contract.tf_mode=="attended_direct" || joint_contract.tf_mode=="attended_measured_direct"){
        const auto odom_frame=manager_owner->node()->get_parameter("odom_frame").as_string();
        const auto base_frame=manager_owner->node()->get_parameter("base_footprint_frame").as_string();
        for(const auto& edge:joint_contract.attended_static_edges)
          if(edge.child_frame_id==odom_frame || edge.child_frame_id==base_frame)
            throw std::invalid_argument("canonical_frame_overlaps_attended_fixed_geometry");
      }
      const bool preserve_6d=manager_owner->node()->get_parameter("preserve_6d_map_alignment").as_bool();
      original_tracking_maximum_age_ns=rclcpp::Duration::from_seconds(
        manager_owner->node()->get_parameter("tracking_odometry_timeout_sec").as_double()).nanoseconds();
      if(options.kinematics_tf_mode!="attended_measured_direct" && preserve_6d)
        throw std::invalid_argument("full_map_alignment_requires_measured_waist_contract");
      livo_alignment_enabled=manager_owner->node()->get_parameter("livo_ndt_alignment_enabled").as_bool();
      if(options.kinematics_tf_mode=="attended_measured_direct" && (livo_alignment_enabled ||
          manager_owner->node()->get_parameter("active_backend").as_string()!="livo" ||
          !manager_owner->node()->get_parameter("preserve_6d_map_alignment").as_bool()))
        throw std::invalid_argument("measured_waist_requires_livo_tracking_without_fixed_ndt_alignment");
      manager_lane=std::make_unique<detail::BoundedLane<ManagerInput>>(512,
        [this](auto input){if(stopping.load()||terminal.load())return;
          std::visit([this](auto value){receive_manager(std::move(value));},std::move(input));
          if(manager_owner->failed())fault("canonical_localization_owner_failed");},failure);
      ndt::NdtDirectSinks ndt_sinks;
      ndt_sinks.publish_ros_candidates=options.publish_ros_localization_candidates;
      ndt_sinks.publish_ros_status_compatibility=options.publish_ros_canonical_compatibility;
      ndt_sinks.accepted=[this](auto result){
        if(!inputs_ready.load()||stopping.load()||terminal.load())return;
        admit_manager(ManagerInput{NdtDelivery{result,steady_ns(),manager_lane->pending_size()}});
        if(sinks.ndt_accepted)sinks.ndt_accepted(std::move(result));};
      ndt_sinks.status=[this](auto status){
        if(!inputs_ready.load()||stopping.load()||terminal.load())return;
        // Before its first measurement the existing backend emits source=0
        // initialization status. It is not measurement evidence; never invent
        // an epoch merely to admit it into the canonical correction pair.
        if(ns(status->header.stamp)<=0){if(sinks.ndt_status)sinks.ndt_status(std::move(status));return;}
        const manager::LocalizationInputStamp stamp{ns(status->header.stamp),control->now().nanoseconds(),
          steady_ns(),++ndt_status_sequence,ndt_identity+"#status"};
        admit_manager(ManagerInput{BackendStatus{status,stamp}});
        if(sinks.ndt_status)sinks.ndt_status(std::move(status));};
      ndt_owner=ndt::make_ndt_direct_runtime(options.ndt_node,std::move(ndt_sinks));
      const auto ndt_node=ndt_owner->node();
      ndt_accepts_prior=!ndt_node->get_parameter("direct_map_tracking_enabled").as_bool();
      if(options.kinematics_tf_mode=="attended_measured_direct" && (ndt_accepts_prior ||
          ndt_node->get_parameter("backend_name").as_string()!="livo"))
        throw std::invalid_argument("measured_waist_requires_independent_direct_map_tracking_livo_correction");
      ndt_identity=ndt_node->get_fully_qualified_name();
      ndt_backend=ndt_node->get_parameter("backend_name").as_string();
      if(ndt_node->get_parameter("source_to_base_transform_accepted").as_bool()){
        // Capture the same constructed NDT owner's fixed source contract once.
        // These parameters are not reread by registration queries or combined
        // with public body_tilt/canonical planar TF.
        const auto values=ndt_node->get_parameter("source_to_base_xyz_qxyzw").as_double_array();
        geometry_msgs::msg::TransformStamped source_from_base;
        source_from_base.header.frame_id=ndt_node->get_parameter("expected_input_frame").as_string();
        source_from_base.child_frame_id=ndt_node->get_parameter("base_footprint_frame").as_string();
        source_from_base.transform.translation.x=values.at(0);
        source_from_base.transform.translation.y=values.at(1);
        source_from_base.transform.translation.z=values.at(2);
        tf2::Quaternion rotation(values.at(3),values.at(4),values.at(5),values.at(6));rotation.normalize();
        source_from_base.transform.rotation.x=rotation.x();source_from_base.transform.rotation.y=rotation.y();
        source_from_base.transform.rotation.z=rotation.z();source_from_base.transform.rotation.w=rotation.w();
        ndt_nominal_source_from_base=source_from_base.transform;
        manager_owner->configure_pcd_registration_source(source_from_base,
          ndt_node->get_parameter("map_frame").as_string(),ndt_node->get_parameter("map_path").as_string(),
          ndt_identity+"#registration");
      }
      const auto actual_map_path=ndt_node->get_parameter("map_path").as_string();
      perception_owner->configure_registration_ground(ndt_owner->registration_map_sha256());
      const auto ground=perception_owner->ground_reference_owner();
      if(ground && ground->registration_map_asset()){
        auto geometry=std::make_shared<detail::RegistrationGroundGeometryAdapter>(
          manager_owner,transforms,actual_map_path,ground->map_identity());
        perception_owner->set_registration_ground_geometry_provider(
          [geometry](const auto& identity,const auto& times,bool airy_scan){
            return (*geometry)(identity,times,airy_scan);
          });
      }
      if(ndt_backend!="lidar"&&ndt_backend!="livo")throw std::invalid_argument("unsupported configured NDT manager backend");
      vendor::DirectRuntimeOptions vendor_options;vendor_options.node_options=options.livo_vendor_node;
      vendor_options.emit_world_cloud=false; // NDT consumes the original Airy raw scan.
      vendor_owner=vendor::DirectRuntime::create_with_fault_info(std::move(vendor_options),
        [this](auto state){admit_vendor(VendorOutput{std::move(state)});},
        [this](auto state){admit_vendor(VendorOutput{std::move(state)});},
        [this](const auto & failure){vendor_fault(failure);});
      corrected_identity=vendor_owner->node()->get_fully_qualified_name()+std::string("#corrected");
      propagated_identity=vendor_owner->node()->get_fully_qualified_name()+std::string("#propagated");
      // Use the actual constructed estimator, never an independent adapter
      // parameter. LIVMapper has already snapshotted this value at creation.
      const bool livo_visual_inputs_required=
        vendor_owner->node()->get_parameter("common.img_en").as_int()!=0;
      livo::LivoDirectSinks adapter_sinks;
      adapter_sinks.vendor_images_enabled=livo_visual_inputs_required;
      adapter_sinks.body_transform_tf_mode=options.kinematics_tf_mode;
      if(options.kinematics_tf_mode=="attended_measured_direct")
        adapter_sinks.source_time_vendor_child_from_base=[this](int64_t stamp,const auto& nominal){return measured_source_from_base_result(stamp,nominal);};
      adapter_sinks.publish_ros_candidates=options.publish_ros_localization_candidates;
      adapter_sinks.publish_ros_status_compatibility=options.publish_ros_canonical_compatibility;
      adapter_sinks.vendor_points=[this](auto message,const auto & stamp){
        if(!stopping.load()&&!terminal.load())vendor_admission(vendor_owner->admit_points_checked(std::move(message),vendor_stamp(stamp)));};
      adapter_sinks.vendor_imu=[this](auto message,const auto & stamp){
        if(!stopping.load()&&!terminal.load())vendor_admission(vendor_owner->admit_imu_checked(std::move(message),vendor_stamp(stamp)));};
      adapter_sinks.vendor_image=[this](auto message,const auto & stamp){
        if(!stopping.load()&&!terminal.load())vendor_admission(vendor_owner->admit_image_checked(std::move(message),vendor_stamp(stamp)));};
      adapter_sinks.candidate=[this](auto message,const auto & stamp){
        if(stopping.load()||terminal.load())return;
        if(ndt_accepts_prior)ndt_owner->submit_prior(message,ndt_stamp(stamp,candidate_identity));
        admit_manager(ManagerInput{LivoCandidate{message,stamp}});
        if(sinks.livo_candidate)sinks.livo_candidate(std::move(message),stamp);
      };
      adapter_sinks.status=[this](auto status){if(inputs_ready.load()&&!stopping.load()&&!terminal.load()&&sinks.livo_status)sinks.livo_status(std::move(status));};
      adapter=livo::make_livo_direct_adapter(options.livo_adapter_node,std::move(adapter_sinks));
      if(options.kinematics_tf_mode=="attended_measured_direct" &&
          !adapter->node()->get_parameter("body_velocity_feedback_enabled").as_bool())
        throw std::invalid_argument("measured_waist_candidate_requires_body_velocity_feedback");
      candidate_identity=adapter->node()->get_fully_qualified_name()+std::string("#candidate");
      point_lane=std::make_unique<detail::BoundedLane<airy::CloudFrame::ConstSharedPtr>>(4,
        [this](auto input){if(stopping.load()||terminal.load())return;
          normalization::AiryDirectStamp stamp{input->source_ns,input->source_ns,input->first_callback_ros_ns,
            input->first_callback_steady_ns,input->sequence,airy_cloud_identity};
          stamp.ray_origins=input->ray_origins;
          AiryInputTiming record;
          record.sequence=input->sequence;record.sdk_sequence=input->sdk_sequence;
          record.source_ns=input->source_ns;record.receipt_ros_ns=input->first_callback_ros_ns;
          record.receipt_steady_ns=input->first_callback_steady_ns;
          record.conversion_begin_ns=input->conversion_begin_steady_ns;
          record.conversion_end_ns=input->conversion_end_steady_ns;
          record.dispatch=sunhorizon_timing::Sample::now(true,true);
          airy_adapter->receive_points(input->cloud,stamp);
          record.completed=sunhorizon_timing::Sample::now(true,true);
          {std::lock_guard<std::mutex> lock(airy_timing_mutex);
            airy_input_timings[airy_input_next++%airy_input_timings.size()]=record;}
        },failure);
      imu_lane=std::make_unique<detail::BoundedLane<airy::ImuFrame::ConstSharedPtr>>(2048,
        [this](auto input){if(stopping.load()||terminal.load())return;
          airy_adapter->receive_imu(input->imu,{input->source_ns,input->source_ns,input->first_callback_ros_ns,
            input->first_callback_steady_ns,input->sequence,airy_imu_identity});},failure);
      if(livo_visual_inputs_required)image_lane=std::make_unique<detail::BoundedLane<Camera>>(2,
        [this](auto input){if(stopping.load()||terminal.load())return;
          if(!input->color||!input->color_info)throw std::runtime_error("chest_sdk_rgb_or_info_missing");
          const livo::LivoDirectStamp stamp{ns(input->color->header.stamp),input->first_sdk_callback_ros_ns,
            input->first_sdk_callback_steady_ns,input->sequence,camera_identity};
          adapter->receive_camera_info(input->color_info,stamp);adapter->receive_image(input->color,stamp);},failure);
      // The vendor FIFO delivery owner emits both kinds. Preserve their order at
      // the adapter boundary so corrected anchors precede propagated candidates.
      vendor_output_lane=std::make_unique<detail::BoundedLane<VendorDelivery>>(256,
        [this](auto state){if(stopping.load()||terminal.load())return;
          livo::LivoDirectDeliveryTiming timing;
          timing.output_lane_enqueue_steady_ns=state.enqueued_steady_ns;
          timing.output_lane_dequeue_steady_ns=steady_ns();
          std::visit([this,&timing](auto value){receive_vendor(std::move(value),timing);},
            std::move(state.output));},failure);
      joints=perception::make_joint_ingress(perception_owner->kinematics(),kinematics_options(options.joint_node,options.kinematics_tf_mode));
      airy::DirectDriverOptions airy_options;airy_options.node_options=options.airy_node;
      airy_options.config_path=options.airy_config_path;airy_options.raw_ros_publish_enabled=options.debug_publish_raw_sensors;
      airy_owner=airy::create_direct_driver(std::move(airy_options),
        [this](auto input){if(!inputs_ready.load()||terminal.load()||stopping.load())return;
          if(!point_lane->admit(std::move(input)))fault("airy_points_lane_overflow");},
        [this](auto input){if(!inputs_ready.load()||terminal.load()||stopping.load())return;
          if(!imu_lane->admit(std::move(input)))fault("airy_imu_lane_overflow");},
        [this](const auto & failure){fault("airy:"+failure);});
      airy_cloud_identity=airy_owner->node()->get_fully_qualified_name()+std::string("#points");
      airy_imu_identity=airy_owner->node()->get_fully_qualified_name()+std::string("#imu");
      normalization::AiryDirectSinks normalized;
      normalized.points=[this](auto message,const auto & stamp){
        if(stopping.load()||terminal.load())return;
        const livo::LivoDirectStamp delivery{stamp.source_ns,stamp.first_receipt_ros_ns,
          stamp.first_receipt_steady_ns,stamp.sequence,stamp.source_identity};
        AiryNormalizationTiming record;record.stamp=stamp;
        record.stamp.ray_origins.reset(); // Metadata only; never retain geometry for diagnostics.
        record.fanout_begin_ns=steady_ns();
        ndt_owner->submit_points(message,ndt_stamp(delivery,stamp.source_identity));
        if(sinks.airy_frame)sinks.airy_frame(message,stamp);
        record.livo_begin_ns=steady_ns();
        adapter->receive_points(std::move(message),delivery);
        record.livo_end_ns=steady_ns();
        {std::lock_guard<std::mutex> lock(airy_timing_mutex);
          airy_normalization_timings[airy_normalization_next++%airy_normalization_timings.size()]=std::move(record);}
      };
      normalized.imu=[this](auto message,const auto & stamp){
        if(stopping.load()||terminal.load())return;
        adapter->receive_imu(std::move(message),{stamp.source_ns,stamp.first_receipt_ros_ns,
          stamp.first_receipt_steady_ns,stamp.sequence,stamp.source_identity});
      };
      airy_adapter=normalization::make_airy_direct_adapter(options.airy_adapter_node,std::move(normalized),
        airy_cloud_identity,airy_imu_identity);
      control_nodes={coverage_owner->node(),control,perception_owner,manager_owner->node(),ndt_owner->node(),vendor_owner->node(),adapter->node(),airy_owner->node(),airy_adapter->node()};
      detail::require_joint_context_domain(control->get_node_base_interface()->get_context(),
        joints->get_node_base_interface()->get_context());
      std::set<std::string> owner_names;
      for(const auto & owner:control_nodes){
        if(owner->get_node_base_interface()->get_context()!=control->get_node_base_interface()->get_context() ||
          !owner_names.insert(owner->get_fully_qualified_name()).second)
          throw std::invalid_argument("navigation owners require one ROS context and distinct real node identities");
      }
      initialize_identity_observer();
    }catch(...){stop_locked();throw;}
  }
  ~Impl(){stop();}
  static int64_t steady_ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  static void identity_field(diagnostic_msgs::msg::DiagnosticStatus& row,const std::string& name,const std::string& value){
    diagnostic_msgs::msg::KeyValue field;field.key=name;field.value=value;row.values.push_back(std::move(field));
  }
  void initialize_identity_observer(){
    auto& row=identity_static;
    row.name="sunhorizon_navigation_runtime/direct_owner_snapshot_v1";
    row.hardware_id=perception_owner->get_parameter("producer_epoch").as_string();
    identity_field(row,"schema_version","1");
    identity_field(row,"pid",std::to_string(getpid()));
    identity_field(row,"process_startticks",process_startticks());
    identity_field(row,"runtime_node",control->get_fully_qualified_name());
    identity_field(row,"producer_node",perception_owner->get_fully_qualified_name());
    // Perception construction already checked this boot against the real boot
    // file. No filesystem access or parameter dump occurs in any SDK callback.
    for(const auto* key:{"clock_boot_id","producer_epoch","belly_epoch","chest_epoch"})
      identity_field(row,key,perception_owner->get_parameter(key).as_string());
    const auto& fk=joints->kinematics()->options();
    const auto source=joints->contract();
    identity_field(row,"source_receipt_path",source.source_receipt_path);
    identity_field(row,"source_evidence_sha256",source.source_evidence_sha256);
    for(const auto& field:std::vector<std::pair<std::string,std::string>>{
        {"kinematics_tf_mode",fk.tf_mode},{"source_receipt_sha256",fk.source_receipt_sha256},
        {"mapping_receipt_sha256",fk.mapping_receipt_sha256},{"legacy_urdf_sha256",fk.legacy_urdf_sha256},
        {"merged_urdf_sha256",fk.merged_urdf_sha256},{"attended_usage",fk.attended_usage},
        {"attended_tf_asset_sha256",fk.attended_tf_asset_sha256},{"attended_tf_receipt_sha256",fk.attended_tf_receipt_sha256},
        {"production_tf_receipt_sha256",fk.production_tf_receipt_sha256}})
      identity_field(row,field.first,field.second);
    identity_field(row,"body_geometry_contract",fk.tf_mode=="attended_measured_direct"?
      "same_owner_source_time_measured_waist;level_body_origin_full_residual;fixed_epoch_z_datum;prototype_base_link_not_floor_metrology":"original_fixed_or_production_contract");
    identity_field(row,"body_velocity_feedback",adapter->node()->get_parameter("body_velocity_feedback_enabled").as_bool()?"causal_heading_regression_500ms_support;effective_stamp_preserved":"disabled");
    identity_field(row,"livo_visual_input_contract",image_lane?"vendor_img_en_enabled;bounded_rgb_fifo_2":"vendor_img_en_disabled;no_livo_rgb_lane;camera_depth_perception_unchanged");
    identity_field(row,"joint_ingress_node",joints->get_fully_qualified_name());
    identity_field(row,"coverage_node",coverage_owner->node()->get_fully_qualified_name());
    identity_field(row,"boundary","accepted direct source identity; original clocks; no coverage or motion authorization");
    identity_publisher=control->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics",rclcpp::QoS(1).reliable().transient_local());
    const auto own_gid=identity_publisher->get_gid();
    identity_field(row,"snapshot_publisher_gid",gid_hex(own_gid.data,RMW_GID_STORAGE_SIZE));
    identity_observer=std::make_unique<camera::LatestFrameWorker<diagnostic_msgs::msg::DiagnosticArray>>(
      [this](auto& snapshot){identity_publisher->publish(snapshot);},[this]{return !stopping.load();},
      [this](auto){fault("direct_owner_diagnostics_failed");});
    identity_timer=control->create_wall_timer(std::chrono::seconds(1),[this]{
      if(!stopping.load())identity_observer->replace(
        std::make_unique<diagnostic_msgs::msg::DiagnosticArray>(identity_snapshot()));});
  }
  diagnostic_msgs::msg::DiagnosticArray identity_snapshot(){
    diagnostic_msgs::msg::DiagnosticArray result;
    auto row=identity_static;
    const auto gauge=adapter->body_odometry_gauge_origin();
    identity_field(row,"body_odom_z_origin_valid",gauge.valid?"true":"false");
    identity_field(row,"body_odom_z_origin_source_ns",std::to_string(gauge.source_ns));
    identity_field(row,"body_odom_z_origin_epoch",std::to_string(gauge.epoch));
    identity_field(row,"body_odom_z_origin_m",std::to_string(gauge.raw_z_m));
    const auto cameras=camera_evidence.snapshot();const auto joint=joints->accepted_identity();
    bool complete=cameras.failure.empty() && joint.accepted_sequence>0;
    for(std::size_t i=0;i<2;++i){
      const auto& sample=cameras.samples[i];const std::string key=i==0?"belly.":"chest.";
      complete=complete&&bool(sample.owner);
      if(!sample.owner)continue;
      identity_field(row,key+"owner_fqn",sample.owner->owner_fqn);
      identity_field(row,key+"opened_serial",sample.owner->opened_serial);
      identity_field(row,key+"capture_epoch",sample.owner->capture_epoch);
      identity_field(row,key+"sequence",std::to_string(sample.sequence));
      identity_field(row,key+"depth_source_ns",std::to_string(sample.depth_source_ns));
      identity_field(row,key+"color_source_ns",std::to_string(sample.color_source_ns));
      identity_field(row,key+"first_receipt_ros_ns",std::to_string(sample.receipt_ros_ns));
      identity_field(row,key+"first_receipt_steady_ns",std::to_string(sample.receipt_steady_ns));
    }
    identity_field(row,"joint.source_owner_fqn",joint.source_owner_fqn);
    identity_field(row,"joint.source_publisher_gid",joint.source_publisher_gid);
    identity_field(row,"joint.accepted_sequence",std::to_string(joint.accepted_sequence));
    identity_field(row,"joint.source_ns",std::to_string(joint.fk.source_stamp_ns));
    identity_field(row,"joint.first_receipt_ros_ns",std::to_string(joint.fk.receipt_stamp_ns));
    identity_field(row,"joint.first_receipt_steady_ns",std::to_string(joint.fk.receipt_steady_ns));
    identity_field(row,"joint.fk_wall_ns",std::to_string(joint.fk.fk_wall_ns));
    identity_field(row,"joint.fk_thread_cpu_ns",std::to_string(joint.fk.fk_thread_cpu_ns));
    identity_field(row,"camera_identity_failure",cameras.failure);
    VendorTiming timing;
    std::array<vendor::CorrectedState::ConstSharedPtr,16> corrected_rows;
    {std::lock_guard<std::mutex> lock(vendor_timing_mutex);timing=vendor_timing;corrected_rows=corrected_frame_timings;}
    const auto number=[&row](const std::string& key,auto value){
      identity_field(row,key,std::to_string(value));};
    const auto input=[&row,&number](const std::string& prefix,const vendor::InputStamp& value){
      number(prefix+"sequence",value.sequence);number(prefix+"source_ns",value.source_ns);
      number(prefix+"first_receipt_ros_ns",value.receipt_ros_ns);
      number(prefix+"first_receipt_steady_ns",value.receipt_steady_ns);
      identity_field(row,prefix+"identity",value.identity);
    };
    const auto common=[&number](const std::string& prefix,const auto& value){
      number(prefix+"sequence",value.sequence);number(prefix+"source_ns",value.source_ns);
      number(prefix+"output_available_ros_ns",value.receipt_ros_ns);
      number(prefix+"output_available_steady_ns",value.receipt_steady_ns);
      number(prefix+"owner_epoch",value.owner_epoch);
      number(prefix+"anchor_generation",value.anchor_generation);
    };
    if(timing.corrected){
      const auto& value=*timing.corrected;const std::string key="livo.corrected.";
      common(key,value);input(key+"latest_dispatched_imu.",value.latest_dispatched_imu);
      number(key+"epoch_to_corrected_wall_ns",value.epoch_to_corrected_wall_ns);
      number(key+"epoch_to_corrected_cpu_ns",value.epoch_to_corrected_cpu_ns);
      number(key+"adapter_enter_steady_ns",timing.corrected_enter_ns);
      number(key+"adapter_exit_steady_ns",timing.corrected_exit_ns);
    }
    if(timing.propagated){
      const auto& value=*timing.propagated;const std::string key="livo.propagated.";
      common(key,value);input(key+"original_imu.",value.original_imu);
      number(key+"anchor_source_ns",value.anchor_source_ns);
      number(key+"corrected_sequence",value.corrected_sequence);
      number(key+"vendor_admission_steady_ns",value.vendor_admission_steady_ns);
      number(key+"vendor_dispatch_steady_ns",value.vendor_dispatch_steady_ns);
      number(key+"propagation_wall_ns",value.propagation_wall_ns);
      number(key+"propagation_cpu_ns",value.propagation_cpu_ns);
      number(key+"adapter_enter_steady_ns",timing.propagated_enter_ns);
      number(key+"adapter_exit_steady_ns",timing.propagated_exit_ns);
    }
    // This is the already constructed real producer node's sole observed DDS
    // egress endpoint. SDK camera/FK inputs have no invented DDS identities.
    const auto peers=perception_owner->get_publishers_info_by_topic("/sunhorizon/obstacle_coverage/spatial_evidence_v1");
    std::string evidence_gid;
    if(peers.size()==1){
      const auto& peer=peers.front();const auto& space=peer.node_namespace();
      const auto fqn=(space.empty()||space=="/"?"/":space+"/")+peer.node_name();
      if(fqn==perception_owner->get_fully_qualified_name() &&
          peer.topic_type()=="sunhorizon_navigation_interfaces/msg/SpatialCoverageEvidence")
        evidence_gid=gid_hex(peer.endpoint_gid().data(),peer.endpoint_gid().size());
    }
    if(evidence_gid.find_first_not_of('0')==std::string::npos)evidence_gid.clear();
    complete=complete&&!evidence_gid.empty();
    identity_field(row,"spatial_evidence_publisher_gid",evidence_gid);
    identity_field(row,"binding_complete",complete?"true":"false");
    const bool revoked=terminal.load()||perception_owner->failed()||vendor_owner->failed();
    identity_field(row,"terminal",revoked?"true":"false");
    row.level=revoked||!cameras.failure.empty()?diagnostic_msgs::msg::DiagnosticStatus::ERROR:
      complete?diagnostic_msgs::msg::DiagnosticStatus::OK:diagnostic_msgs::msg::DiagnosticStatus::WARN;
    row.message=revoked?"runtime_revoked":complete?"accepted_source_identity_snapshot":"awaiting_accepted_source_identity";
    result.status.push_back(std::move(row));
    std::array<NdtDeliveryTiming,16> ndt_rows;
    {std::lock_guard<std::mutex> lock(ndt_delivery_timing_mutex);ndt_rows=ndt_delivery_timings;}
    for(const auto& t:ndt_rows){
      if(!t.sequence)continue;
      diagnostic_msgs::msg::DiagnosticStatus d;d.name="sunhorizon_navigation_runtime/ndt_delivery_v1";
      d.hardware_id=ndt_identity;d.message="original clocks; map admission is not canonical commit";
      const auto add=[&](const char* key,int64_t value){identity_field(d,key,std::to_string(value));};
      add("sequence",t.sequence);add("source_ns",t.source_ns);add("input_receipt_steady_ns",t.input_receipt_ns);
      add("input_receipt_ros_ns",t.input_receipt_ros_ns);add("result_receipt_ros_ns",t.result_receipt_ros_ns);
      add("result_receipt_steady_ns",t.result_receipt_ns);add("enqueue_steady_ns",t.enqueue_ns);
      add("dequeue_steady_ns",t.dequeue_ns);add("registration_end_steady_ns",t.registration_end_ns);
      add("waist_end_steady_ns",t.waist_end_ns);add("map_pose_end_steady_ns",t.map_pose_end_ns);
      add("pending_before_admit",t.pending_before_admit);add("pending_at_dequeue",t.pending_at_dequeue);
      add("waist_lookup_status",t.waist_lookup_status);add("waist_transform_count",t.waist_transform_count);
      add("waist_status",t.waist_status);add("registration_admitted",t.registration_admitted);
      add("map_pose_admitted",t.map_pose_admitted);result.status.push_back(std::move(d));
    }
    // The original frame sequence joins SDK/normalizer and estimator records.
    // Rings may wrap if observation stalls; sequence gaps remain visible.
    const auto sample_fields=[&](auto& d,const std::string& prefix,const sunhorizon_timing::Sample& v){
      const auto add=[&](const std::string& key,auto value){identity_field(d,prefix+key,std::to_string(value));};
      add("steady_ns",v.wall);add("thread_cpu_ns",v.cpu);add("tid",v.scheduling.tid);
      add("sched_status",static_cast<int>(v.scheduling.status));add("sched_running_ns",v.scheduling.running_ns);
      add("sched_runqueue_ns",v.scheduling.runqueue_ns);add("sched_timeslices",v.scheduling.timeslices);
      add("sched_sampling_ns",v.scheduling.sampling_ns);
    };
    std::array<AiryInputTiming,16> airy_rows;
    std::array<AiryNormalizationTiming,16> normalization_rows;
    {std::lock_guard<std::mutex> lock(airy_timing_mutex);
      airy_rows=airy_input_timings;normalization_rows=airy_normalization_timings;}
    for(const auto& t:airy_rows){
      if(!t.sequence)continue;
      diagnostic_msgs::msg::DiagnosticStatus d;d.name="sunhorizon_navigation_runtime/airy_input_timing_v1";
      d.hardware_id=airy_cloud_identity;d.message="complete SDK cloud receipt, not first packet; original clocks";
      const auto add=[&](const char* key,auto value){identity_field(d,key,std::to_string(value));};
      add("sequence",t.sequence);add("sdk_sequence",t.sdk_sequence);add("source_ns",t.source_ns);
      add("first_receipt_ros_ns",t.receipt_ros_ns);add("first_receipt_steady_ns",t.receipt_steady_ns);
      add("conversion_begin_steady_ns",t.conversion_begin_ns);add("conversion_end_steady_ns",t.conversion_end_ns);
      sample_fields(d,"dispatch.",t.dispatch);sample_fields(d,"completed.",t.completed);
      result.status.push_back(std::move(d));
    }
    for(const auto& t:normalization_rows){
      if(!t.stamp.sequence)continue;
      diagnostic_msgs::msg::DiagnosticStatus d;d.name="sunhorizon_navigation_runtime/airy_normalization_timing_v1";
      d.hardware_id=t.stamp.source_identity;d.message="original normalization and synchronous fanout clocks";
      const auto add=[&](const char* key,auto value){identity_field(d,key,std::to_string(value));};
      add("sequence",t.stamp.sequence);add("raw_source_ns",t.stamp.raw_source_ns);add("source_ns",t.stamp.source_ns);
      add("normalization_start_steady_ns",t.stamp.normalization_start_steady_ns);
      add("normalization_end_steady_ns",t.stamp.normalization_end_steady_ns);
      add("normalization_thread_cpu_ns",t.stamp.normalization_thread_cpu_ns);
      add("fanout_begin_steady_ns",t.fanout_begin_ns);add("livo_begin_steady_ns",t.livo_begin_ns);
      add("livo_end_steady_ns",t.livo_end_ns);result.status.push_back(std::move(d));
    }
    for(const auto& value:corrected_rows){
      if(!value)continue;
      const auto& t=value->lidar_timing;
      diagnostic_msgs::msg::DiagnosticStatus d;d.name="sunhorizon_navigation_runtime/livo_frame_timing_v1";
      d.hardware_id=corrected_identity;d.message="same frame; source is scan header, corrected_source is scan end; no motion authority";
      const auto add=[&](const char* key,auto v){identity_field(d,key,std::to_string(v));};
      add("available",t.available);add("corrected_sequence",value->sequence);add("corrected_source_ns",value->source_ns);
      add("owner_epoch",value->owner_epoch);add("output_available_ros_ns",value->receipt_ros_ns);
      add("output_available_steady_ns",value->receipt_steady_ns);add("sequence",t.original.sequence);
      add("source_ns",t.original.source_ns);add("first_receipt_ros_ns",t.original.receipt_ros_ns);
      add("first_receipt_steady_ns",t.original.receipt_steady_ns);identity_field(d,"input_identity",t.original.identity);
      add("admission_steady_ns",t.admission_steady_ns);add("dispatch_steady_ns",t.dispatch_steady_ns);
      add("history_evictions",t.history_evictions);
      sample_fields(d,"callback.",t.callback);sample_fields(d,"preprocess_start.",t.preprocess_start);
      sample_fields(d,"preprocess_end.",t.preprocess_end);sample_fields(d,"enqueued.",t.enqueued);
      sample_fields(d,"selected.",t.selected);sample_fields(d,"ready.",t.ready);
      sample_fields(d,"epoch_start.",t.epoch_start);sample_fields(d,"corrected.",t.corrected);
      result.status.push_back(std::move(d));
    }
    // Observe time after the snapshots. A concurrent accepted input may be
    // newer than a clock sampled before either owner lock was acquired.
    result.header.stamp=control->now();return result;
  }
  void admit_manager(ManagerInput input){
    if(stopping.load()||terminal.load())return;
    if(!manager_lane->admit(std::move(input)))fault("localization_manager_lane_overflow");
  }
  void admit_canonical(CanonicalOutput output){
    if(stopping.load()||terminal.load())return;
    if(!canonical_lane->admit(std::move(output)))fault("canonical_localization_lane_overflow");
  }
  std::optional<geometry_msgs::msg::Transform> measured_source_from_base(int64_t stamp,
      const geometry_msgs::msg::Transform& nominal)const{
    return measured_source_from_base_result(stamp,nominal).transform;
  }
  livo::SourceTimeBodyTransform measured_source_from_base_result(int64_t stamp,
      const geometry_msgs::msg::Transform& nominal,int* lookup_status=nullptr,
      std::size_t* transform_count=nullptr)const{
    const auto owner=perception_owner->kinematics();
    const auto waist=owner->lookup_measured_waist(stamp,[this]{return perception::JointKinematics::ClockSample{
      control->now().nanoseconds(),steady_ns()};});
    if(lookup_status)*lookup_status=static_cast<int>(waist.status);
    if(transform_count)*transform_count=waist.transforms.size();
    if(waist.status==perception::JointKinematicsStatus::FuturePending)
      return {livo::BodyVelocityGeometryStatus::FuturePending,{}};
    if(waist.status!=perception::JointKinematicsStatus::Ready || waist.transforms.size()!=1)return {};
    return {livo::BodyVelocityGeometryStatus::Ready,
      perception::source_from_base_at_measured_waist(nominal,owner->neutral_base_from_waist(),
        waist.transforms.front().transform)};
  }
  void receive_canonical(std::shared_ptr<const manager::CanonicalLocalizationState> state){
    if(!state||!state->odometry)throw std::runtime_error("canonical_localization_snapshot_missing");
    if(options.spatial_query.height_limited()){
      // Validate the entire pair and odometry before either shared TF write.
      const auto planar=[](const auto& t){perception::detail::require_exact_planar_pose(
        t.translation.x,t.translation.y,t.translation.z,t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w);};
      if(state->map_to_odom.header.frame_id!="map" || state->map_to_odom.child_frame_id!="odom" ||
          state->odom_to_base_footprint.header.frame_id!="odom" ||
          state->odom_to_base_footprint.child_frame_id!="base_footprint" ||
          state->odometry->header.frame_id!="odom" || state->odometry->child_frame_id!="base_footprint")
        throw std::invalid_argument("collision_height_canonical_frame_changed");
      planar(state->map_to_odom.transform);planar(state->odom_to_base_footprint.transform);
      const auto& p=state->odometry->pose.pose;
      perception::detail::require_exact_planar_pose(p.position.x,p.position.y,p.position.z,
        p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w);
    }
    if(!transforms->setTransform(state->map_to_odom,manager_identity,false)||
      !transforms->setTransform(state->odom_to_base_footprint,manager_identity,false))
      throw std::runtime_error("canonical_direct_transform_rejected");
    if(options.kinematics_tf_mode=="attended_measured_direct"){
      if(!state->base_footprint_to_base_link || !state->tracking.body_velocity ||
          !state->tracking.body_velocity->valid || !state->tracking.body_velocity->measured_waist)
        throw std::runtime_error("measured_waist_canonical_residual_or_velocity_missing");
      if(!transforms->setTransform(*state->base_footprint_to_base_link,manager_identity,false))
        throw std::runtime_error("canonical_body_residual_transform_rejected");
    }
    if(sinks.canonical_localization)sinks.canonical_localization(std::move(state));
  }
  void receive_canonical(std::shared_ptr<const manager::CanonicalLocalizationHealth> health){
    if(sinks.localization_health)sinks.localization_health(std::move(health));
  }
  void receive_manager(LivoCandidate input){
    const auto & s=input.stamp;
    manager_owner->receive_odometry("livo",input.message,{s.source_stamp_ns,s.receipt_ros_ns,
      s.receipt_steady_ns,s.sequence,candidate_identity,s.body_velocity});
  }
  void receive_manager(NdtDelivery delivery){
    const auto input=std::move(delivery.result);
    NdtDeliveryTiming timing;
    timing.enqueue_ns=delivery.enqueued_ns;timing.dequeue_ns=steady_ns();
    timing.pending_before_admit=delivery.pending_before_admit;timing.pending_at_dequeue=manager_lane->pending_size();
    // Fixed bounded rows are copied by the existing diagnostics observer; no ROS/logging on this lane.
    const auto record=[this,&timing]{
      std::lock_guard<std::mutex> lock(ndt_delivery_timing_mutex);
      ndt_delivery_timings[ndt_delivery_timing_next++ % ndt_delivery_timings.size()]=timing;
    };
    if(!input||!input->odometry||!input->map_pose)throw std::runtime_error("accepted_ndt_snapshot_missing");
    timing.sequence=input->input.sequence;timing.source_ns=input->input.source_stamp_ns;
    timing.input_receipt_ros_ns=input->input.receipt_ros_ns;timing.result_receipt_ros_ns=input->result_receipt_ros_ns;
    timing.input_receipt_ns=input->input.receipt_steady_ns;timing.result_receipt_ns=input->result_receipt_steady_ns;
    timing.registration_admitted=manager_owner->receive_registration(input,{input->input.source_stamp_ns,input->result_receipt_ros_ns,
      input->result_receipt_steady_ns,input->input.sequence,ndt_identity+"#registration"});
    timing.registration_end_ns=steady_ns();
    const manager::LocalizationInputStamp odometry{ns(input->odometry->header.stamp),
      input->result_receipt_ros_ns,input->result_receipt_steady_ns,input->input.sequence,ndt_identity+"#odometry"};
    if(ndt_backend=="livo"){
      if(livo_alignment_enabled)manager_owner->receive_livo_ndt_alignment(input->odometry,odometry);
    }else{
      manager_owner->receive_odometry(ndt_backend,input->odometry,odometry);
      if(input->independent_tracking_evidence)manager_owner->receive_tracking(ndt_backend,input->odometry,odometry);
    }
    auto map=odometry;map.source_ns=ns(input->map_pose->header.stamp);map.identity=ndt_identity+"#map_pose";
    auto map_pose=input->map_pose;
    if(options.kinematics_tf_mode=="attended_measured_direct"){
      if(!input->map_to_source){record();return;}
      if(!ndt_nominal_source_from_base)throw std::runtime_error("measured_waist_nominal_source_contract_missing");
      const auto lookup=measured_source_from_base_result(map.source_ns,*ndt_nominal_source_from_base,
        &timing.waist_lookup_status,&timing.waist_transform_count);
      timing.waist_status=static_cast<int>(lookup.status);timing.waist_end_ns=steady_ns();
      const auto dynamic=lookup.transform;if(!dynamic){record();return;}
      tf2::Transform map_source,source_base;tf2::fromMsg(input->map_to_source->transform,map_source);tf2::fromMsg(*dynamic,source_base);
      const auto map_body=map_source*source_base;
      auto changed=std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(*map_pose);
      changed->pose.pose.position.x=map_body.getOrigin().x();changed->pose.pose.position.y=map_body.getOrigin().y();changed->pose.pose.position.z=map_body.getOrigin().z();
      changed->pose.pose.orientation=tf2::toMsg(map_body.getRotation());map_pose=std::move(changed);
    }
    timing.map_pose_admitted=manager_owner->receive_map_pose(ndt_backend,map_pose,map);
    timing.map_pose_end_ns=steady_ns();record();
  }
  void receive_manager(BackendStatus input){
    manager_owner->receive_status(ndt_backend,std::move(input.message),input.stamp);
  }
  void receive_vendor(vendor::CorrectedState::ConstSharedPtr value,
      const livo::LivoDirectDeliveryTiming& timing){
    if(!value||!value->odometry)throw std::runtime_error("gated_corrected_snapshot_missing");
    livo::LivoDirectStamp stamp{value->source_ns,value->receipt_ros_ns,
      value->receipt_steady_ns,value->sequence,corrected_identity};
    stamp.delivery_timing=timing;
    const auto enter=steady_ns();
    adapter->receive_corrected_odometry(value->odometry,stamp);
    const auto exit=steady_ns();
    if(value->zupt_ack_epoch&&sinks.zupt_applied)sinks.zupt_applied(value->zupt_ack_epoch,stamp);
    {std::lock_guard<std::mutex> lock(vendor_timing_mutex);
      corrected_frame_timings[corrected_frame_next++%corrected_frame_timings.size()]=value;
      vendor_timing.corrected.swap(value);
      vendor_timing.corrected_enter_ns=enter;vendor_timing.corrected_exit_ns=exit;}
    // The displaced owner is destroyed outside the short snapshot lock.
  }
  void receive_vendor(vendor::PropagatedState::ConstSharedPtr value,
      livo::LivoDirectDeliveryTiming timing){
    if(!value||!value->odometry)throw std::runtime_error("gated_propagated_snapshot_missing");
    timing.geometry_wait_begin_steady_ns=steady_ns();
    if(!wait_for_vendor_source_waist(value->source_ns,value->receipt_ros_ns,value->receipt_steady_ns))return;
    timing.geometry_wait_end_steady_ns=steady_ns();
    const auto enter=steady_ns();
    livo::LivoDirectStamp stamp{value->source_ns,value->receipt_ros_ns,
      value->receipt_steady_ns,value->sequence,propagated_identity};
    stamp.delivery_timing=timing;
    adapter->receive_propagated_odometry(value->odometry,stamp);
    const auto exit=steady_ns();
    {std::lock_guard<std::mutex> lock(vendor_timing_mutex);
      vendor_timing.propagated.swap(value);
      vendor_timing.propagated_enter_ns=enter;vendor_timing.propagated_exit_ns=exit;}
  }
  void admit_vendor(VendorOutput value){
    if(stopping.load()||terminal.load())return;
    if(!vendor_output_lane||!vendor_output_lane->admit(
        VendorDelivery{std::move(value),steady_ns()}))fault("vendor_output_lane_overflow");
  }
  bool wait_for_vendor_source_waist(int64_t source_ns,int64_t receipt_ros_ns,int64_t receipt_steady_ns) {
    if(options.kinematics_tf_mode!="attended_measured_direct")return true;
    const auto owner=perception_owner->kinematics();
    const detail::OriginalSourceGeometryLease lease{source_ns,receipt_ros_ns,receipt_steady_ns,
      std::min(original_tracking_maximum_age_ns,owner->options().maximum_age_ns)};
    // This is the bounded vendor OUTPUT FIFO worker, not an IMU/SDK callback.
    // Keep the original sample while the independent joint ingress catches up,
    // before receive_* can acquire the adapter's writer mutex. Non-pending
    // failures go straight to the existing adapter rejection/reset path.
    const auto result=detail::wait_for_original_source_geometry(lease,
      [this]{return detail::GeometryWaitClocks{control->now().nanoseconds(),steady_ns()};},
      [&]{return owner->lookup_measured_waist(source_ns,[this]{
        return perception::JointKinematics::ClockSample{control->now().nanoseconds(),steady_ns()};});},
      [](const auto& value){return value.status==perception::JointKinematicsStatus::FuturePending;},
      []{std::this_thread::sleep_for(std::chrono::milliseconds(1));},
      [this]{return stopping.load()||terminal.load();});
    return result.has_value();
  }
  std::atomic<bool> vendor_fault_detail_reported{false};
  void vendor_fault(const vendor::FirstFaultInfo& info)noexcept {
    try {
      // Revoke from the bounded cause first; formatting/logging the complete
      // frozen overflow record must never delay the actual safety transition.
      fault(detail::vendor_fault_cause(info));
      if(info.has_overflow&&!vendor_fault_detail_reported.exchange(true)) {
        const auto detail=info.describe();
        RCLCPP_ERROR(control->get_logger(),"LIVO_FIRST_FAULT_DETAIL %s",detail.c_str());
      }
    }catch(...) {}
  }
  bool vendor_admission(const vendor::AdmissionResult& result) {
    return detail::handle_vendor_admission(result,stopping.load(),
      [this](const auto& info){vendor_fault(info);},
      [this](const char* reason){fault(reason);});
  }
  void fault(const std::string & why)noexcept{
    if(stopping.load()||terminal.exchange(true))return;
    try{{std::lock_guard<std::mutex> lock(reason_mutex);reason=why.substr(0,512);}
      if(perception_owner)perception_owner->fault("navigation_runtime:"+why);
      if(coverage_owner)coverage_owner->revoke("navigation_runtime:"+why);
      if(manager_owner)manager_owner->revoke("navigation_runtime:"+why);
      if(sinks.fault)sinks.fault(why);
    }catch(...){}
  }
  void start(){std::lock_guard<std::mutex> lock(lifecycle);
    // Consumer filtering thresholds do not prove low-obstacle safety. The
    // clipped query remains available for offline comparisons only.
    if(options.spatial_query.height_limited()){
      fault("low_obstacle_safety_unvalidated_offline_only");
      throw std::invalid_argument("low_obstacle_safety_unvalidated_offline_only");
    }
    if(started)return;
    if(stopping.load()||terminal.load())throw std::runtime_error("cannot start stopped/faulted navigation runtime");
    if(!sinks.perception_output)throw std::invalid_argument("real perception_output/Nav2 consumer required before start");
    if(!sinks.airy_frame)throw std::invalid_argument("real Airy filtering/Nav2 consumer required before start");
    if(!sinks.canonical_localization||!sinks.localization_health)
      throw std::invalid_argument("actual Nav2 canonical localization and health consumers required before start");
    if(!sinks.fault)throw std::invalid_argument("actual downstream fail-closed fault consumer required before start");
    if(vendor_owner->node()->get_parameter("common.stationary_zupt_enabled").as_bool()&&!sinks.zupt_applied)
      throw std::invalid_argument("enabled stationary ZUPT requires its actual direct ACK consumer");
    try{
      const auto context=control->get_node_base_interface()->get_context();
      executor=std::make_unique<rclcpp::executors::MultiThreadedExecutor>(detail::make_executor_options(context),4);
      const auto joint_context=joints->get_node_base_interface()->get_context();
      detail::require_joint_context_domain(context,joint_context);
      joint_executor=std::make_unique<rclcpp::executors::SingleThreadedExecutor>(detail::make_executor_options(joint_context));
      for(const auto & owner:control_nodes)executor->add_node(owner);
      for(const auto & owner:extra_control_nodes)executor->add_node(owner);
      joint_executor->add_node(joints);
      control_thread=std::thread([this]{try{executor->spin();}catch(const std::exception & e){fault(e.what());}catch(...){fault("control_executor_failure");}});
      joint_thread=std::thread([this,context]{
        detail::spin_joint_executor(*joint_executor,context,[this]{return stopping.load();},
          [this](const std::string& reason){fault(reason);});
      });
      // Camera factory begins capture in its constructor. Everything it can
      // call is already connected; initial construction callbacks are dropped
      // before admission and never count as fresh observations.
      camera_owner=camera::create_camera_runtime(
        observer_option(options.camera_node,"raw_ros_publish_enabled",options.debug_publish_raw_sensors),
        [this](auto frame){if(!inputs_ready.load()||terminal.load()||stopping.load())return;
          if(frame&&frame->source_validated)camera_evidence.accepted(frame->role,{frame->source_identity,
            frame->sequence,ns(frame->source_stamp),frame->color?ns(frame->color->header.stamp):0,
            frame->first_sdk_callback_ros_ns,frame->first_sdk_callback_steady_ns});
          perception_owner->admit(frame);
          if(frame->role=="chest"&&image_lane&&!image_lane->admit(std::move(frame)))fault("camera_rgb_lane_overflow");},
        [this](const auto & error){fault("camera:"+error.reason);});
      camera_identity=camera_owner->get_fully_qualified_name()+std::string("#chest_color");
      if(camera_owner->get_node_base_interface()->get_context()!=control->get_node_base_interface()->get_context())
        throw std::invalid_argument("camera and navigation must share one ROS context");
      for(const auto & owner:control_nodes)if(std::string(owner->get_fully_qualified_name())==camera_owner->get_fully_qualified_name())
        throw std::invalid_argument("camera source owner identity must be unique");
      executor->add_node(camera_owner);
      if(terminal.load())throw std::runtime_error("source fault during navigation startup");
      inputs_ready.store(true);airy_owner->start();
      if(terminal.load())throw std::runtime_error("source fault during Airy acquisition startup");
      started=true;
    }catch(...){stop_locked();throw;}
  }
  void stop(){std::lock_guard<std::mutex> lock(lifecycle);stop_locked();}
  void stop_locked(){
    if(stopping.exchange(true))return;
    inputs_ready.store(false);
    if(coverage_owner)coverage_owner->revoke("navigation_runtime_stopped");
    if(identity_timer)identity_timer->cancel();
    if(airy_owner)airy_owner->stop();
    if(camera_owner)camera::stop_camera_runtime(camera_owner);
    if(point_lane)point_lane->request_stop();
    if(imu_lane)imu_lane->request_stop();
    if(image_lane)image_lane->request_stop();
    if(point_lane)point_lane->join();
    if(imu_lane)imu_lane->join();
    if(image_lane)image_lane->join();
    if(vendor_owner)vendor_owner->stop();
    if(vendor_output_lane){vendor_output_lane->request_stop();vendor_output_lane->join();}
    if(ndt_owner)ndt_owner->stop();
    if(executor)executor->cancel();
    if(control_thread.joinable())control_thread.join();
    if(identity_observer){identity_observer->request_stop();identity_observer->join();}
    if(manager_lane){manager_lane->request_stop();manager_lane->join();}
    if(canonical_lane){canonical_lane->request_stop();canonical_lane->join();}
    if(perception_owner)perception_owner->stop();
    // Evidence/health callbacks have now finished; coverage remains alive until
    // every independent body/Airy/region delivery owner is quiescent.
    if(coverage_belly)coverage_belly->request_stop();
    if(coverage_airy)coverage_airy->request_stop();
    if(coverage_region)coverage_region->request_stop();
    if(coverage_belly)coverage_belly->join();
    if(coverage_airy)coverage_airy->join();
    if(coverage_region)coverage_region->join();
    if(coverage_owner)coverage_owner->stop();
    if(joint_executor)joint_executor->cancel();
    if(joint_thread.joinable())joint_thread.join();
  }
};
std::shared_ptr<NavigationRuntime> make_navigation_runtime(NavigationRuntimeOptions options,NavigationRuntimeSinks sinks){
  return std::shared_ptr<NavigationRuntime>(new NavigationRuntime(std::make_unique<NavigationRuntime::Impl>(std::move(options),std::move(sinks))));}
NavigationRuntime::NavigationRuntime(std::unique_ptr<Impl> impl):impl_(std::move(impl)){}
NavigationRuntime::~NavigationRuntime(){stop();}
void NavigationRuntime::start(){impl_->start();}
void NavigationRuntime::bind_collision_height_responsibility(const perception::CollisionHeightBinding& binding){
  std::lock_guard<std::mutex> lock(impl_->lifecycle);
  if(impl_->started || impl_->stopping.load() || impl_->terminal.load() || impl_->collision_height_bound ||
      !(binding.policy()==impl_->options.spatial_query))
    throw std::logic_error("collision_height_binding_requires_unstarted_matching_runtime");
  impl_->perception_owner->bind_collision_height_responsibility(binding);
  impl_->collision_height_bound=true;
}
void NavigationRuntime::admit_filtered_airy(std::shared_ptr<const sunhorizon_rgbd_adapter::SelfExclusionDirectOutput> output){
  if(impl_->stopping.load()||impl_->terminal.load())return;
  if(!impl_->coverage_airy)throw std::logic_error("airy_obstacle_coverage_disabled");
  if(!output || !output->input || !output->cloud)throw std::invalid_argument("coverage_filtered_airy_owner_missing");
  auto pending=std::make_unique<Impl::CoverageCloud>(Impl::CoverageCloud{output->cloud,output->input->stamp.first_receipt_ros_ns});
  const auto* offered=pending.get();auto retired=impl_->coverage_airy->replace(std::move(pending));
  if(retired.get()==offered)throw std::runtime_error("coverage_airy_delivery_closed");
}
void NavigationRuntime::release_spatial_startup(int64_t deadline){
  if(impl_->stopping.load()||failed())throw std::logic_error("cannot_release_failed_spatial_owner");
  impl_->perception_owner->release_spatial_startup(deadline);
}
perception::SpatialStartupStatus NavigationRuntime::spatial_startup_status()const{
  return impl_->perception_owner->spatial_startup_status();
}
diagnostic_msgs::msg::DiagnosticArray NavigationRuntime::direct_owner_snapshot()const{return impl_->identity_snapshot();}
void NavigationRuntime::stop(){if(impl_)impl_->stop();}
bool NavigationRuntime::submit_stationary_zupt(std_msgs::msg::UInt64::ConstSharedPtr request,const livo::LivoDirectStamp & stamp){
  if(!impl_->inputs_ready.load()||impl_->stopping.load()||impl_->terminal.load())return false;
  return impl_->vendor_admission(impl_->vendor_owner->admit_zupt_checked(std::move(request),vendor_stamp(stamp)));
}
bool NavigationRuntime::failed()const{return impl_->terminal.load()||impl_->perception_owner->failed()||impl_->vendor_owner->failed();}
std::string NavigationRuntime::failure_reason()const{
  {std::lock_guard<std::mutex> lock(impl_->reason_mutex);
    if(!impl_->reason.empty())return impl_->reason;}
  if(impl_->perception_owner->failed()){
    const auto reason=impl_->perception_owner->fault_reason();
    return reason.empty()?"perception_owner_failed":"perception_owner_failed:"+reason;
  }
  if(impl_->vendor_owner->failed())return detail::vendor_owner_failure_reason(*impl_->vendor_owner);
  return {};
}
std::shared_ptr<tf2_ros::Buffer> NavigationRuntime::direct_tf_buffer()const{return impl_->transforms;}
perception::ground_relative::GroundReferenceOwner::ConstSharedPtr NavigationRuntime::ground_reference_owner()const{
  return impl_->perception_owner->ground_reference_owner();
}
perception::RegistrationGroundGeometryProvider NavigationRuntime::registration_ground_geometry_provider()const{
  return impl_->perception_owner->registration_ground_geometry_provider();
}
std::string NavigationRuntime::airy_ground_calibration_id()const{
  // Configuration identity, not physical error qualification. Include the
  // actual pinned rig receipts; the ground owner separately requires bounds.
  std::ifstream stream(impl_->options.airy_config_path,std::ios::binary);
  if(!stream)throw std::runtime_error("airy_ground_configuration_unreadable");
  const std::string config((std::istreambuf_iterator<char>(stream)),{});
  if(config.empty()||config.size()>1024*1024)throw std::runtime_error("airy_ground_configuration_size");
  const auto& k=impl_->perception_owner->kinematics()->options();
  return sunhorizon_rgbd_adapter::dynamic_self_model::sha256(
    config+"\n"+k.merged_urdf_sha256+"\n"+k.attended_tf_asset_sha256+"\n"+
    k.attended_tf_receipt_sha256+"\n"+k.production_tf_receipt_sha256);
}
double NavigationRuntime::self_surface_diagnostic_tolerance_m()const{
  return impl_->perception_owner->self_surface_diagnostic_tolerance_m();
}
sunhorizon_rgbd_adapter::LidarModelProvider NavigationRuntime::lidar_model_provider_for_scan(std::size_t maximum_entries)const{
  return impl_->perception_owner->lidar_model_provider_for_scan(maximum_entries);
}
double NavigationRuntime::lidar_surface_radius_bound(const std::string& source_frame)const{
  return impl_->perception_owner->lidar_surface_radius_bound(source_frame);
}
std::shared_ptr<const sunhorizon_obstacle_coverage::dynamic_self_geometry::PreparedModel>
NavigationRuntime::lidar_self_model(const std::string& source_frame,int64_t source_ns)const{
  return impl_->perception_owner->lidar_model(source_frame,source_ns);
}
rclcpp::Node::SharedPtr NavigationRuntime::node()const{return impl_->control;}
rclcpp::Node::SharedPtr NavigationRuntime::airy_adapter_node()const{return impl_->airy_adapter->node();}
std::shared_ptr<manager::LocalizationDirectRuntime> NavigationRuntime::localization_manager()const{return impl_->manager_owner;}
void NavigationRuntime::add_control_node(rclcpp::Node::SharedPtr node){
  std::lock_guard<std::mutex> lock(impl_->lifecycle);
  if(impl_->started||impl_->stopping.load()||!node)throw std::invalid_argument("attach control nodes before navigation start");
  if(node->get_node_base_interface()->get_context()!=impl_->control->get_node_base_interface()->get_context())
    throw std::invalid_argument("all navigation owners must share one ROS context");
  impl_->control_nodes.push_back(std::move(node));
}
void NavigationRuntime::add_control_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node){
  std::lock_guard<std::mutex> lock(impl_->lifecycle);
  if(impl_->started||impl_->stopping.load()||!node)throw std::invalid_argument("attach lifecycle control nodes before navigation start");
  if(node->get_context()!=impl_->control->get_node_base_interface()->get_context())
    throw std::invalid_argument("all lifecycle owners must share one ROS context");
  impl_->extra_control_nodes.push_back(std::move(node));
}
}  // namespace sunhorizon_navigation_runtime
