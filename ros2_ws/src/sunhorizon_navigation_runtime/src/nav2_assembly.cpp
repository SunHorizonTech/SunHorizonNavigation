#include "sunhorizon_navigation_runtime/detail/near_body_endpoints.hpp"
#include "sunhorizon_navigation_runtime/detail/rgbd_projection_audit.hpp"
#include "sunhorizon_navigation_runtime/detail/rgbd_writer_audit.hpp"
#include "sunhorizon_navigation_runtime/detail/airy_geometry_audit.hpp"
#include "sunhorizon_navigation_runtime/detail/direct_cloud_admission.hpp"
#include "sunhorizon_navigation_runtime/nav2_assembly.hpp"
#include "sunhorizon_navigation_runtime/bounded_lane.hpp"
#include "sunhorizon_navigation_runtime/detail/canonical_health_check.hpp"
#include "sunhorizon_navigation_runtime/detail/raw_clearing_contract.hpp"
#include "sunhorizon_navigation_runtime/detail/collision_height_parameters.hpp"
#include "sunhorizon_navigation_runtime/detail/classified_airy_packet.hpp"
#include "sunhorizon_navigation_runtime/detail/ground_clearance_packet.hpp"
#include "sunhorizon_perception_runtime/runtime_config.hpp"
#include "sunhorizon_perception_runtime/ground_classification_snapshot.hpp"
#include "sunhorizon_perception_runtime/compute_thread_control.hpp"
#include "sunhorizon_camera_runtime/latest_frame_worker.hpp"
#include <nav2_controller/controller_server.hpp>
#include <nav2_planner/planner_server.hpp>
#include <nav2_collision_monitor/collision_monitor_node.hpp>
#include <nav2_collision_monitor/direct_cloud_binding.hpp>
#include <nav2_costmap_2d/direct_cloud_binding.hpp>
#include <nav2_costmap_2d/voxel_layer.hpp>
#include <nav2_costmap_2d/costmap_filters/keepout_filter.hpp>
#include <nav2_util/direct_odometry.hpp>
#include <nav2_util/direct_timing.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sunhorizon_rgbd_adapter/pointcloud_self_exclusion_direct.hpp>
#include <map>
#include <tf2_ros/create_timer_ros.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <variant>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace sunhorizon_navigation_runtime {
namespace {
namespace manager = sunhorizon_localization_manager;
namespace perception = sunhorizon_perception_runtime;
using Body = std::shared_ptr<const perception::PerceptionOutput>;
using State = std::shared_ptr<const manager::CanonicalLocalizationState>;
using Health = std::shared_ptr<const manager::CanonicalLocalizationHealth>;
using CanonicalInput = std::variant<State, Health>;
using AiryInput = std::shared_ptr<const sunhorizon_rgbd_adapter::SelfExclusionDirectInput>;
using AiryOutput = std::shared_ptr<const sunhorizon_rgbd_adapter::SelfExclusionDirectOutput>;
// Original immutable output owner; this extra clock measures queueing only.
struct AiryGroundInput {AiryOutput output; int64_t queued_ns{0};};
struct CloudDelivery {nav2_util::DirectCloud cloud; int64_t queued_ns{0};};
using LifecycleState = lifecycle_msgs::msg::State;
// Read only from the existing diagnostic directory, outside owner locks. Never
// follow a symlink or block on a special file; at most 64 token bytes per tick.
std::string rgbd_audit_request(const std::string& directory,const std::string& filename="rgbd-audit-request") {
  if(directory.empty())return {};
  const auto path=directory+"/"+filename;
  const int fd=::open(path.c_str(),O_RDONLY|O_NOFOLLOW|O_NONBLOCK|O_CLOEXEC);
  if(fd<0)return {};
  struct stat st{};char bytes[65]{};
  const bool valid=::fstat(fd,&st)==0&&S_ISREG(st.st_mode)&&st.st_size>0&&st.st_size<=64;
  const auto n=valid?::read(fd,bytes,sizeof(bytes)):-1;
  ::close(fd);
  return valid&&n==st.st_size?std::string(bytes,std::size_t(n)):std::string{};
}
int64_t stamp_ns(const builtin_interfaces::msg::Time & t) {
  return int64_t(t.sec) * 1000000000LL + t.nanosec;
}
std::string exception_reason(std::exception_ptr error) {
  try {if (error) std::rethrow_exception(error);}
  catch (const std::exception & e) {return e.what();}
  catch (...) {return "unknown_nav2_assembly_failure";}
  return "nav2_assembly_failure";
}
}

struct Nav2Assembly::Impl {
  struct RawSink {
    std::string stage;
    std::unique_ptr<sunhorizon_camera_runtime::LatestFrameWorker<CloudDelivery>> worker;
  };
  struct BodyRoute {
    Nav2BodySourceBinding names;
    nav2_util::DirectCloudResultSink local_sink, global_sink, collision_sink;
    std::unique_ptr<detail::BoundedLane<CloudDelivery>> local, global, collision;
    std::vector<std::unique_ptr<RawSink>> raw_local, raw_global;
    std::mutex admission_mutex;
    std::unique_ptr<nav2_util::DirectObservationGate> admission_gate, raw_admission_gate;
  };
  Nav2AssemblyOptions options;
  rclcpp::Node::SharedPtr host;
  std::shared_ptr<tf2_ros::Buffer> buffer;
  std::shared_ptr<nav2_planner::PlannerServer> planner;
  std::shared_ptr<nav2_controller::ControllerServer> controller;
  std::shared_ptr<nav2_collision_monitor::CollisionMonitor> collision;
  std::vector<std::unique_ptr<BodyRoute>> bodies;
  std::unique_ptr<detail::BoundedLane<CanonicalInput>> canonical;
  std::shared_ptr<sunhorizon_rgbd_adapter::PointcloudSelfExclusionDirect> airy_filter;
  std::unique_ptr<nav2_util::DirectObservationGate> airy_gate;
  std::mutex airy_admission_mutex;
  // Obstacle scans have no estimator continuity requirement: keep the active
  // scan and only the latest validated pending owner, with original clocks.
  std::unique_ptr<sunhorizon_camera_runtime::LatestFrameWorker<AiryInput>> airy_compute;
  // Independent stage: one executing immutable output and one latest pending.
  // Replacement is not classification, delivery or completion credit.
  std::unique_ptr<sunhorizon_camera_runtime::LatestFrameWorker<AiryGroundInput>> airy_ground;
  // Diagnostics only; the executing owners identify/name themselves. No
  // scheduling policy, priority, affinity or evidence eligibility changes.
  std::atomic<int64_t> airy_filter_tid{-1},airy_ground_tid{-1};
  std::unique_ptr<detail::BoundedLane<CloudDelivery>> airy_local, airy_global, airy_collision;
  std::vector<std::pair<std::string,nav2_util::DirectCloudResultSink>> airy_local_sinks, airy_global_sinks;
  nav2_util::DirectCloudResultSink airy_collision_sink;
  mutable std::mutex timing_mutex;
  std::map<std::string,nav2_util::DirectStageTiming> timings;
  struct AdmissionSnapshot {nav2_util::DirectObservationMetadata metadata; nav2_util::DirectAdmissionResult result; std::string detail;};
  std::map<std::string,AdmissionSnapshot> admissions;
  struct AiryPacketSnapshot {
    nav2_util::DirectObservationMetadata metadata;
    std::size_t input_points{0},finite_points{0},invalid_points{0};
    bool deliverable{false},ground_unavailable{false};
    sunhorizon_perception_runtime::ground_relative::GroundQualification ground;
  } airy_packet;
  const std::string ground_snapshot_directory=perception::detail::ground_snapshot_directory_from_environment();
  // One immutable diagnostic handoff only. The existing timing worker owns
  // serialization and disk I/O; the Airy compute lane only retains a pointer.
  bool airy_ground_snapshot_selected{false}; // timing_mutex
  std::shared_ptr<std::atomic<bool>> airy_audit_requested=ground_snapshot_directory.empty()?nullptr:
    std::make_shared<std::atomic<bool>>(false);
  struct AiryAuditFrame {
    nav2_util::DirectObservationMetadata metadata;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr source_cloud;
    perception::ground_relative::ClassifiedObstacleFrame::ConstSharedPtr classification;
    AiryOutput filtered;
    std::shared_ptr<const nav2_costmap_2d::DirectProjectionSnapshot> projection;
    int64_t selected_steady_ns;
  };
  bool airy_audit_armed{false};
  std::atomic<bool> airy_audit_failed{false};
  std::unique_ptr<AiryAuditFrame> airy_audit_frame; // timing_mutex; at most one selected frame
  perception::ground_relative::ClassifiedObstacleFrame::ConstSharedPtr airy_ground_snapshot_pending;
  // Global belly/chest, then local belly/chest: each consumer independently
  // retains only its own actual accepted source and original transformation.
  std::array<detail::RgbdProjectionCapture,4> rgbd_projection_capture{{
    detail::RgbdProjectionCapture(!ground_snapshot_directory.empty()),
    detail::RgbdProjectionCapture(!ground_snapshot_directory.empty()),
    detail::RgbdProjectionCapture(!ground_snapshot_directory.empty(),detail::RgbdProjectionConsumer::Local),
    detail::RgbdProjectionCapture(!ground_snapshot_directory.empty(),detail::RgbdProjectionConsumer::Local)}}; // timing_mutex
  std::shared_ptr<detail::RgbdWriterQueue> writer_audit_queue=std::make_shared<detail::RgbdWriterQueue>();
  std::shared_ptr<detail::WriterCaptureRequest> writer_audit_request;
  std::array<std::shared_ptr<detail::WriterCaptureRequest>,4> writer_audit_history;
  std::string writer_audit_rejected_token;
  std::size_t writer_audit_request_count{0}; // only timing worker mutates history
  std::array<std::array<bool,4>,4> writer_audit_failure_reported{};
  std::array<std::array<bool,4>,4> writer_audit_status_attempted{};
  std::array<std::array<std::string,4>,4> writer_audit_paths;
  std::array<bool,4> rgbd_audit_exhaustion_reported{{false,false,false,false}}; // timing_mutex

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr timing_publisher;
  rclcpp::TimerBase::SharedPtr timing_timer;
  std::atomic<bool> accepting{false}, stopping{false}, terminal{false}, activated{false};
  mutable std::mutex state_mutex, reason_mutex;
  std::mutex lifecycle_mutex;
  std::condition_variable changed;
  State latest_state;
  Health latest_health;
  uint64_t reset_epoch{0}, state_sequence{0}, injected_sequence{0};
  bool epoch_pinned{false}, started{false};
  std::atomic<bool> collision_height_verified{false};
  // Initialization keeps sensor lanes alive; motion permission is separately
  // latched off after the first post-activation localization outage.
  std::atomic<bool> canonical_motion_hold{false}, controller_suspended{false};
  std::string canonical_hold_reason; // state_mutex; preserve the first outage.
  std::string reason;
  std::thread activation_worker;

  Impl(Nav2AssemblyOptions settings, std::shared_ptr<tf2_ros::Buffer> transforms)
    : options(std::move(settings)), buffer(std::move(transforms)) {
    if (!buffer || !options.fault || !options.canonical_owner || options.localization_health_maximum_age_ns <= 0 ||
      options.localization_health_maximum_age_ns > 10000000000LL || options.body_sources.empty())
      throw std::invalid_argument("nav2_assembly_requires_buffer_original_limits_sources_and_fault_sink");
    sunhorizon_obstacle_coverage::validate_camera_age(options.camera_maximum_age_ns);
    if (options.canonical_odometry.source_id != options.canonical_owner->node()->get_fully_qualified_name())
      throw std::invalid_argument("canonical_contract_must_pin_actual_manager_owner");
    // Constructing the gate verifies identity/epoch/frame and bounded age now.
    nav2_util::DirectObservationGate validate(options.canonical_odometry);
    if (options.canonical_odometry.maximum_age_ns > options.canonical_age_limit_ns())
      throw std::invalid_argument("nav2_canonical_odometry_age_exceeds_retained_direct_limit");
    std::set<std::string> roles, local_names, global_names, collision_names;
    std::map<std::string,std::string> local_owners, global_owners;
    for (const auto & source : options.body_sources) {
      if (options.classified_obstacles_enabled &&
          (!source.local_clearing_sources.empty() || !source.global_clearing_sources.empty()))
        throw std::invalid_argument("classified_obstacles_require_atomic_source_without_raw_routes");
      if ((source.camera_role != "belly" && source.camera_role != "chest") ||
        source.local_layer.empty() || source.local_source.empty() ||
        (source.global_layer.empty() && (!source.global_source.empty() || !source.global_clearing_sources.empty())) ||
        (!source.global_layer.empty() && source.global_source.empty() && source.global_clearing_sources.empty()) || source.collision_source.empty() ||
        !roles.insert(source.camera_role).second ||
        !local_names.insert(source.local_layer + "/" + source.local_source).second ||
        (!source.global_source.empty() && !global_names.insert(source.global_layer + "/" + source.global_source).second) ||
        !collision_names.insert(source.collision_source).second)
        throw std::invalid_argument("invalid_or_duplicate_nav2_body_source_binding");
      for (const auto & sources : {source.local_clearing_sources, source.global_clearing_sources}) {
        std::set<std::string> unique;
        for (const auto & name : sources) if (name.empty() || !unique.insert(name).second)
          throw std::invalid_argument("invalid_or_duplicate_nav2_raw_clearing_binding");
      }
      const auto claim = [&](auto & owners, const auto & layer, const auto & name) {
        if (name.empty()) return;
        const auto inserted = owners.emplace(layer+"/"+name,source.camera_role);
        if (!inserted.second && inserted.first->second != source.camera_role)
          throw std::invalid_argument("nav2_source_cannot_mix_actual_camera_owners");
      };
      claim(local_owners,source.local_layer,source.local_source);
      claim(global_owners,source.global_layer,source.global_source);
      for (const auto & name : source.local_clearing_sources) claim(local_owners,source.local_layer,name);
      for (const auto & name : source.global_clearing_sources) claim(global_owners,source.global_layer,name);
    }
    if (roles != std::set<std::string>{"belly", "chest"})
      throw std::invalid_argument("nav2_body_routes_require_both_actual_cameras");
    const bool airy_obstacles=obstacle_uses_airy(options.obstacle_source_mode);
    if(!airy_obstacles && (!options.classified_obstacles_enabled || options.spatial_query.height_limited() ||
        !options.airy_source.local_layer.empty() || !options.airy_source.global_layer.empty() ||
        !options.airy_source.local_sources.empty() || !options.airy_source.global_sources.empty() ||
        !options.airy_source.collision_source.empty()))
      throw std::invalid_argument("rgbd_only_requires_classified_full_region_and_no_airy_obstacle_routes");
    if (airy_obstacles && (options.airy_source.local_layer.empty() || options.airy_source.local_sources.empty() ||
      options.airy_source.collision_source.empty() ||
      (options.airy_source.global_layer.empty() != options.airy_source.global_sources.empty())))
      throw std::invalid_argument("nav2_airy_requires_actual_retained_local_and_collision_sources");
    if (airy_obstacles && options.classified_obstacles_enabled && (options.airy_source.local_sources.size()!=1 ||
        options.airy_source.global_sources.size()>1))
      throw std::invalid_argument("classified_airy_requires_single_atomic_source_per_costmap");
    for (const auto & name : options.airy_source.local_sources)
      if (name.empty() || !local_owners.emplace(options.airy_source.local_layer+"/"+name,"airy").second)
        throw std::invalid_argument("duplicate_or_mixed_nav2_airy_local_source");
    for (const auto & name : options.airy_source.global_sources)
      if (name.empty() || !global_owners.emplace(options.airy_source.global_layer+"/"+name,"airy").second)
        throw std::invalid_argument("duplicate_or_mixed_nav2_airy_global_source");
    if (airy_obstacles && !collision_names.insert(options.airy_source.collision_source).second)
      throw std::invalid_argument("duplicate_nav2_airy_collision_source");
    airy_gate = std::make_unique<nav2_util::DirectObservationGate>(options.airy_input_contract);
    host = std::make_shared<rclcpp::Node>("sunhorizon_nav2_assembly", options.host_node);
    if (!(perception::load_spatial_query_policy(*host)==options.spatial_query))
      throw std::invalid_argument("entry_nav2_spatial_query_policy_mismatch");
    if (host->get_parameter("use_sim_time").as_bool())
      throw std::invalid_argument("nav2_assembly_requires_original_realtime_clocks");
    buffer->setUsingDedicatedThread(true);
    buffer->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
      host->get_node_base_interface(), host->get_node_timers_interface()));
    const bool immutable_costmaps=options.spatial_query.height_limited() || options.classified_obstacles_enabled;
    planner = std::make_shared<nav2_planner::PlannerServer>(options.planner_node,immutable_costmaps);
    controller = std::make_shared<nav2_controller::ControllerServer>(options.controller_node,immutable_costmaps);
    collision = std::make_shared<nav2_collision_monitor::CollisionMonitor>(options.collision_node);
    if (!controller->get_parameter("sunhorizon_command_generation_evidence_enabled").as_bool() ||
      !collision->get_parameter("sunhorizon_command_generation_evidence_enabled").as_bool() ||
      controller->get_parameter("direct_velocity_output").as_bool())
      throw std::invalid_argument("nav2_assembly_requires_retained_typed_dds_command_evidence");
    planner->getCostmapROS()->setDirectTransformBuffer(buffer);
    controller->getCostmapROS()->setDirectTransformBuffer(buffer);
    collision->setDirectTransformBuffer(buffer);
    controller->setDirectOdometryContract(options.canonical_odometry);
    if(airy_obstacles) {
    sunhorizon_rgbd_adapter::SelfExclusionDirectOptions filter;
    filter.node_options = options.airy_filter_node; filter.transforms = buffer;
    filter.source_identity = options.airy_input_contract.source_id;
    filter.source_epoch = options.airy_input_contract.epoch;
    filter.input_frame = options.airy_input_contract.frame_id;
    filter.publish_ros_compatibility = options.publish_filtered_airy_compatibility;
    filter.lidar_model=options.airy_lidar_model;
    filter.lidar_model_for_scan=options.airy_lidar_model_for_scan;
    filter.lidar_audit_requested=airy_audit_requested;
    filter.lidar_surface_radius_bound_m=options.airy_lidar_surface_radius_bound_m;
    filter.lidar_surface_tolerance_m=options.airy_lidar_surface_tolerance_m;
    filter.lidar_remove_self_surfaces=options.airy_lidar_remove_self_surfaces;
    airy_filter = sunhorizon_rgbd_adapter::make_pointcloud_self_exclusion_direct(
      std::move(filter), [this](AiryOutput output) {receive_airy_filtered(output);});
    }
    timing_publisher = host->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1);
    timing_timer = host->create_wall_timer(std::chrono::milliseconds(200), [this] {publish_timings();});
  }

  void fault(const std::string & message) noexcept {
    {
      std::lock_guard<std::mutex> lock(reason_mutex);
      if (terminal.exchange(true)) return;
      reason = message;
    }
    accepting.store(false);
    controller->getCostmapROS()->requestDirectStop();
    planner->getCostmapROS()->requestDirectStop();
    changed.notify_all();
    try {options.fault(message);} catch (...) {}
    RCLCPP_ERROR(host->get_logger(), "nav2_first_fault: %s", message.c_str());
  }
  bool running() const {return accepting.load() && !stopping.load() && !terminal.load();}
  void validate_identity(const Body & output) const {
    if (!output || !output->input || !output->input->source_validated)
      throw std::invalid_argument("nav2_requires_validated_original_camera_frame");
    const auto & id = output->frame.decoded.identity();
    if (output->input->role != id.source_id || output->input->sequence != id.sequence ||
      stamp_ns(output->input->source_stamp) != id.source_stamp_ros_ns ||
      output->input->first_sdk_callback_ros_ns != id.receipt_ros_ns ||
      output->input->first_sdk_callback_steady_ns != id.receipt_steady_ns)
      throw std::invalid_argument("nav2_body_original_identity_or_receipt_mismatch");
  }
  nav2_util::DirectCloud cloud(const Body & output) const {
    validate_identity(output);
    if (!output->frame.body_filter.complete || !output->frame.body_filter.retained_count)
      throw std::invalid_argument("nav2_body_requires_completed_retained_original_frame");
    const auto & id = output->frame.decoded.identity();
    nav2_util::DirectCloud result{std::shared_ptr<const sensor_msgs::msg::PointCloud2>(output, &output->frame.body_cloud),
      {id.source_id, id.epoch, output->frame.body_cloud.header.frame_id, id.sequence,
        id.source_stamp_ros_ns, id.receipt_ros_ns, id.receipt_steady_ns}};
    const auto& classified=output->frame.classified_obstacles;
    if (options.classified_obstacles_enabled != bool(classified))
      throw std::invalid_argument("classified_obstacle_producer_consumer_mode_mismatch");
    if (classified) {
      const auto& source=classified->source().identity;
      if (!classified->classification_complete() || source.source_id!=id.source_id ||
          source.epoch!=id.epoch || source.calibration_id!=id.calibration_id ||
          source.sequence!=id.sequence || source.source_ns!=id.source_stamp_ros_ns ||
          source.first_receipt_ros_ns!=id.receipt_ros_ns ||
          source.first_receipt_steady_ns!=id.receipt_steady_ns ||
          source.expiry_budget_ns!=id.expiry_budget_ns ||
          source.source_frame!=output->frame.body_cloud.header.frame_id ||
          classified->source().points.size()!=classified->classes().size() ||
          classified->classes().size()!=std::size_t(result.cloud->width)*result.cloud->height)
        throw std::invalid_argument("classified_obstacle_original_identity_or_layout_mismatch");
      auto classes=std::make_shared<std::vector<uint8_t>>();classes->reserve(classified->classes().size());
      for (std::size_t point_index=0;point_index<classified->classes().size();++point_index) {
        classes->push_back(detail::rgbd_audit_direct_class(classified->classes()[point_index],
          classified->source(),point_index,classified->decisions().at(point_index)));
      }
      result.kind=nav2_util::DirectCloudKind::ClassifiedObstacleFrame;
      result.classes=std::move(classes);result.complete=true;
      result.ground_qualified=classified->ground().safety_qualified;
      result.ground_basis=detail::direct_ground_basis(classified->ground());
      detail::attach_ground_validity(result,classified);
      detail::attach_ground_clearance(result,classified,options.ground_reference_owner,output->frame.near_ground_clearance);
      detail::attach_empirical_ground_support(result,classified,options.ground_reference_owner);
      if(!ground_snapshot_directory.empty())try {
        const auto request=std::atomic_load(&writer_audit_request);
        const auto role=id.source_id=="belly"?0u:1u;
        if(request && (id.source_id=="belly"||id.source_id=="chest") &&
          (request->states[role].load()==detail::WriterCaptureState::Armed ||
           request->states[role+2].load()==detail::WriterCaptureState::Armed))
          result.writer_audit_owner=std::make_shared<const detail::RgbdWriterOwner>(output,result.metadata,request,writer_audit_queue);
      }catch(...){} // Diagnostic allocation cannot change the product result.
      if (!nav2_util::directCloudMatchesMetadata(result))
        throw std::invalid_argument("classified_obstacle_packet_invalid");
    }
    return result;
  }
  nav2_util::DirectCloud raw_cloud(const Body & output) const {
    validate_identity(output);
    const auto & frame = output->frame;
    if (!frame.raw_projection_complete || !frame.raw_projection_cloud ||
      frame.raw_projection_source_ns <= 0 || frame.raw_projection_raw_source_ns <= 0 ||
      frame.raw_projection_expiry_budget_ns <= 0)
      throw std::invalid_argument("nav2_raw_clearing_requires_original_projection_and_budget");
    const auto & id = frame.decoded.identity();
    if (frame.raw_projection_raw_source_ns != id.source_stamp_ros_ns)
      throw std::invalid_argument("nav2_raw_projection_original_depth_clock_mismatch");
    nav2_util::DirectCloud raw{frame.raw_projection_cloud,
      {id.source_id,id.epoch,frame.raw_projection_cloud->header.frame_id,id.sequence,
        frame.raw_projection_source_ns,id.receipt_ros_ns,id.receipt_steady_ns,
        frame.raw_projection_raw_source_ns},nav2_util::DirectCloudKind::RetainedIndependentSource};
    if (!nav2_util::directCloudMatchesMetadata(raw))
      throw std::invalid_argument("nav2_raw_clearing_layout_or_original_stamp_mismatch");
    return raw;
  }
  void record(const std::string & prefix, nav2_util::DirectStageTiming timing) {
    timing.stage = prefix + timing.stage;
    const auto key = timing.stage + "/" + timing.metadata.source_id;
    std::lock_guard<std::mutex> lock(timing_mutex);
    const auto previous = timings.find(key);
    // Keep the first completed insertion/consumption for each original frame.
    if (previous != timings.end() && previous->second.metadata.epoch == timing.metadata.epoch &&
      previous->second.metadata.sequence >= timing.metadata.sequence) return;
    if (previous == timings.end() && timings.size() >= 128) return;
    timings[key] = std::move(timing);
  }
  void publish_timings() {
    const auto rgbd_request=rgbd_audit_request(ground_snapshot_directory);
    const auto writer_token=rgbd_audit_request(ground_snapshot_directory,"rgbd-writer-audit-request");
    if(detail::WriterCaptureRequest::valid(writer_token) && writer_audit_request_count<writer_audit_history.size() &&
       std::none_of(writer_audit_history.begin(),writer_audit_history.end(),[&](const auto& r){return r&&r->token==writer_token;})) {
      try {
        auto request=std::make_shared<detail::WriterCaptureRequest>(writer_token,nav2_util::directSteadyNowNs());
        if(const auto old=std::atomic_load(&writer_audit_request))for(auto& state:old->states) {
          auto expected=detail::WriterCaptureState::Armed;
          state.compare_exchange_strong(expected,detail::WriterCaptureState::SupersededUnwritten);
        }
        writer_audit_history[writer_audit_request_count++]=request;
        std::atomic_store(&writer_audit_request,std::move(request));
        RCLCPP_INFO(host->get_logger(),"RGBD actual-writer audit armed request=%s",writer_token.c_str());
      }catch(...){RCLCPP_WARN(host->get_logger(),"RGBD actual-writer audit FAILED request=%s reason=diagnostic_arm_allocation",writer_token.c_str());}
    }
    if(detail::WriterCaptureRequest::valid(writer_token)&&writer_audit_request_count==writer_audit_history.size()&&
       writer_token!=writer_audit_rejected_token&&std::none_of(writer_audit_history.begin(),writer_audit_history.end(),
         [&](const auto& request){return request&&request->token==writer_token;})) {
      writer_audit_rejected_token=writer_token;
      RCLCPP_WARN(host->get_logger(),"RGBD actual-writer audit FAILED request=%s reason=request_budget_exhausted",writer_token.c_str());
    }
    for(auto& item:writer_audit_queue->take())if(item) {
      auto& event=*item;
      try {
        auto bytes=nlohmann::json::parse(detail::serialize_rgbd_projection_audit(
          detail::writer_projection_frame(event),options.ground_reference_owner.get(),false));
        bytes["actual_writer"]=detail::writer_hit_json(event);
        bytes["capture_basis"]="actual_254_writer_no_ground_promotion";
        bytes["raw_sdk"]=nlohmann::json{{"depth_scale_m",event.body->input->depth_scale_m},
          {"sdk_depth_scale_m",event.body->input->sdk_depth_scale_m},
          {"device_us",event.body->input->depth_time.device_us},{"global_us",event.body->input->depth_time.global_us},
          {"sdk_system_us",event.body->input->depth_time.sdk_system_us},
          {"first_sdk_callback_ros_ns",event.body->input->first_sdk_callback_ros_ns},
          {"first_sdk_callback_steady_ns",event.body->input->first_sdk_callback_steady_ns}};
        bytes["raw_sdk"]["source_identity"]=nullptr;
        if(const auto identity=event.body->input->source_identity)
          bytes["raw_sdk"]["source_identity"]={{"owner_fqn",identity->owner_fqn},
            {"opened_serial",identity->opened_serial},{"capture_epoch",identity->capture_epoch}};
        const auto role=event.projection->metadata.source_id;
        if(!event.body->input||!event.body->input->depth||!event.body->input->depth_info)
          throw std::invalid_argument("writer_audit_original_primary_owner_missing");
        bytes["primary_raw"]=perception::detail::write_rgbd_writer_primary_snapshot(ground_snapshot_directory,
          role,event.projection->metadata.sequence,event.request->token,event.slot<2?"global":"local",
          *event.body->input->depth,*event.body->input->depth_info);
        const auto path=perception::detail::write_rgbd_writer_classification_snapshot(ground_snapshot_directory,
          role,event.projection->metadata.sequence,event.request->token,event.slot<2?"global":"local",bytes.dump());
        for(std::size_t r=0;r<writer_audit_request_count;++r)
          if(writer_audit_history[r]==event.request)writer_audit_paths[r][event.slot]=path;
        event.request->states[event.slot].store(detail::WriterCaptureState::Written);
        RCLCPP_INFO(host->get_logger(),"RGBD actual-writer audit written request=%s slot=%zu path=%s",event.request->token.c_str(),event.slot,path.c_str());
        RCLCPP_INFO(host->get_logger(),"VOXEL_ACTUAL_WRITER seal=%s payload=%s",path.c_str(),bytes["actual_writer"].dump().c_str());
      }catch(const std::exception& error){
        event.request->states[event.slot].store(detail::WriterCaptureState::FailedWrite);
        RCLCPP_WARN(host->get_logger(),"RGBD actual-writer audit FAILED request=%s slot=%zu reason=%s",event.request->token.c_str(),event.slot,error.what());
      }catch(...){
        event.request->states[event.slot].store(detail::WriterCaptureState::FailedWrite);
        RCLCPP_WARN(host->get_logger(),"RGBD actual-writer audit FAILED request=%s slot=%zu reason=unknown_worker_exception",event.request->token.c_str(),event.slot);
      }
    }
    for(std::size_t r=0;r<writer_audit_request_count;++r)for(std::size_t slot=0;slot<4;++slot) {
      const auto request=writer_audit_history[r];
      if(nav2_util::directSteadyNowNs()-request->armed_steady_ns>=35000000000LL) {
        auto expected=detail::WriterCaptureState::Armed;
        request->states[slot].compare_exchange_strong(expected,detail::WriterCaptureState::ExpiredUnwritten);
      }
      const auto state=request->states[slot].load();
      if(state!=detail::WriterCaptureState::Armed&&state!=detail::WriterCaptureState::Claimed&&
          state!=detail::WriterCaptureState::Queued&&!writer_audit_status_attempted[r][slot]) {
        writer_audit_status_attempted[r][slot]=true;
        const char* name=state==detail::WriterCaptureState::Written?"Written":
          state==detail::WriterCaptureState::FailedQueue?"FailedQueue":
          state==detail::WriterCaptureState::FailedWrite?"FailedWrite":
          state==detail::WriterCaptureState::SupersededUnwritten?"SupersededUnwritten":"ExpiredUnwritten";
        try {
          nlohmann::json status={{"schema","rgbd-actual-writer-status-v1"},{"request",request->token},
            {"slot",slot},{"consumer",slot<2?"global":"local"},{"role",slot%2==0?"belly":"chest"},
            {"state",name},{"written",state==detail::WriterCaptureState::Written},
            {"seal_path",writer_audit_paths[r][slot]},
            {"source_identity_scope","Read exact source from seal only when Written; no source invented for an unqueued/missing writer"}};
          (void)perception::detail::write_rgbd_writer_status_snapshot(ground_snapshot_directory,request->token,slot,status.dump());
        }catch(const std::exception& error){RCLCPP_WARN(host->get_logger(),"RGBD actual-writer status FAILED request=%s slot=%zu reason=%s",request->token.c_str(),slot,error.what());}
        catch(...){RCLCPP_WARN(host->get_logger(),"RGBD actual-writer status FAILED request=%s slot=%zu",request->token.c_str(),slot);}
      }
      if((state==detail::WriterCaptureState::FailedQueue||state==detail::WriterCaptureState::SupersededUnwritten||
          state==detail::WriterCaptureState::ExpiredUnwritten)&&!writer_audit_failure_reported[r][slot]) {
        writer_audit_failure_reported[r][slot]=true;
        const char* reason=state==detail::WriterCaptureState::FailedQueue?"bounded_queue_full_or_contended":
          state==detail::WriterCaptureState::SupersededUnwritten?"superseded_without_actual_writer":"no_actual_254_writer_or_diagnostic_projection_unavailable";
        RCLCPP_WARN(host->get_logger(),"RGBD actual-writer audit FAILED request=%s slot=%zu reason=%s",request->token.c_str(),slot,reason);
      }
    }

    diagnostic_msgs::msg::DiagnosticArray report; report.header.stamp = host->now();
    std::vector<nav2_util::DirectStageTiming> snapshot;
    AiryPacketSnapshot airy;
    perception::ground_relative::ClassifiedObstacleFrame::ConstSharedPtr ground_snapshot;
    std::map<std::string,AdmissionSnapshot> admission_snapshot;
    std::unique_ptr<AiryAuditFrame> geometry_audit;
    std::array<std::unique_ptr<detail::RgbdProjectionAuditFrame>,4> rgbd_audits;
    std::array<bool,4> rgbd_audit_exhausted{{false,false,false,false}};
    std::array<bool,4> rgbd_audit_rearmed{{false,false,false,false}};
    {std::lock_guard<std::mutex> lock(timing_mutex);
      for (const auto & value : timings) snapshot.push_back(value.second);
      airy=airy_packet; admission_snapshot=admissions;
      ground_snapshot=std::move(airy_ground_snapshot_pending);
      if(!ground_snapshot_directory.empty())for(std::size_t role=0;role<rgbd_audits.size();++role) {
        rgbd_audits[role]=rgbd_projection_capture[role].take(nav2_util::directSteadyNowNs());
        if(rgbd_projection_capture[role].exhausted()&&!rgbd_audit_exhaustion_reported[role]) {
          rgbd_audit_exhaustion_reported[role]=true;rgbd_audit_exhausted[role]=true;
        }
        if(!rgbd_request.empty()&&rgbd_projection_capture[role].rearm(rgbd_request)) {
          rgbd_audit_rearmed[role]=true;rgbd_audit_exhaustion_reported[role]=false;
        }
      }
      if(airy_audit_frame && (airy_audit_frame->projection ||
          nav2_util::directSteadyNowNs()-airy_audit_frame->selected_steady_ns>=2000000000LL))
        geometry_audit=std::move(airy_audit_frame);
    }
    {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name="sunhorizon_nav2_direct/canonical_motion_permission";
      status.hardware_id=options.canonical_odometry.source_id;
      std::lock_guard<std::mutex> lock(state_mutex);
      status.level=canonical_motion_hold.load()?status.ERROR:status.OK;
      status.message=canonical_motion_hold.load()?canonical_hold_reason:"localization motion permission retained";
      const auto add=[&](const std::string& key,const std::string& value) {
        diagnostic_msgs::msg::KeyValue row;row.key=key;row.value=value;status.values.push_back(std::move(row));
      };
      add("initialized",activated.load()?"true":"false");
      add("motion_held",canonical_motion_hold.load()?"true":"false");
      add("controller_suspended",controller_suspended.load()?"true":"false");
      add("latest_state_sequence",std::to_string(state_sequence));
      // This existing 5 Hz snapshot may skip intermediate canonical states.
      // Copy only their original receipts; never renew localization evidence.
      add("canonical_lineage_available",latest_state?"true":"false");
      if(latest_state){
        const auto& state=*latest_state;
        add("canonical_reset_epoch",std::to_string(state.reset_epoch));
        add("correction_source_ns",std::to_string(state.map_pose.source_ns));
        add("correction_sequence",std::to_string(state.map_pose.sequence));
        add("correction_pose_receipt_ros_ns",std::to_string(state.map_pose.first_receipt_ros_ns));
        add("correction_pose_receipt_steady_ns",std::to_string(state.map_pose.first_receipt_steady_ns));
        add("correction_status_receipt_ros_ns",std::to_string(state.backend_status.first_receipt_ros_ns));
        add("correction_status_receipt_steady_ns",std::to_string(state.backend_status.first_receipt_steady_ns));
        // alignment_metadata() supplies an exact sample or its two brackets.
        // Report an unexpected larger lineage explicitly without changing gates.
        const auto count=state.alignment_inputs.size();
        add("alignment_count",std::to_string(count));
        add("alignment_complete",count>0&&count<=2?"true":"false");
        for(std::size_t i=0;i<std::min<std::size_t>(count,2);++i){
          const auto& input=state.alignment_inputs[i];
          const auto prefix="alignment_"+std::to_string(i)+"_";
          add(prefix+"source_ns",std::to_string(input.source_ns));
          add(prefix+"sequence",std::to_string(input.sequence));
          add(prefix+"identity",input.identity);
          add(prefix+"receipt_ros_ns",std::to_string(input.first_receipt_ros_ns));
          add(prefix+"receipt_steady_ns",std::to_string(input.first_receipt_steady_ns));
        }
      }
      add("recovery","explicit_new_runtime_session_required; fresh localization does not rearm motion");
      report.status.push_back(std::move(status));
    }
    for (const auto & item : std::array<std::pair<const char *, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>>, 2>{{
      {"local", controller->getCostmapROS()}, {"global", planner->getCostmapROS()}}})
    {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = std::string("sunhorizon_nav2_direct/") + item.first + ".costmap_current";
      const bool current = item.second->getLayeredCostmap()->isCurrent();
      status.level = current ? status.OK : status.WARN;
      status.message = current ? "original layered current check passed" : "original layered current check unavailable";
      // Observe the same check used by PlannerServer, without renewing a lease.
      diagnostic_msgs::msg::KeyValue value; value.key = "current";
      value.value = current ? "true" : "false"; status.values.push_back(std::move(value));
      report.status.push_back(std::move(status));
    }
    for(const auto& item:admission_snapshot) {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name="sunhorizon_nav2_direct/admission/"+item.first;
      status.hardware_id=item.second.metadata.source_id;
      const auto& result=item.second.result;
      status.level=result.accepted()?status.OK:result.state==nav2_util::DirectAdmissionState::InternalFault?status.ERROR:status.WARN;
      status.message=item.second.detail.empty()?nav2_util::directRejectionReasonName(result.reason):item.second.detail;
      const auto add=[&](const std::string& key,int64_t value) {
        diagnostic_msgs::msg::KeyValue row;row.key=key;row.value=std::to_string(value);status.values.push_back(std::move(row));
      };
      add("admission_state",static_cast<int>(result.state));
      add("gate_snapshot_ros_ns",result.snapshot_ros_ns);add("gate_snapshot_steady_ns",result.snapshot_steady_ns);
      add("sequence",item.second.metadata.sequence);add("source_ns",item.second.metadata.source_ros_ns);
      add("first_receipt_ros_ns",item.second.metadata.first_receipt_ros_ns);
      add("first_receipt_steady_ns",item.second.metadata.first_receipt_steady_ns);
      report.status.push_back(std::move(status));
    }
    if(airy.metadata.sequence) {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name="sunhorizon_nav2_direct/airy.classified_packet";status.hardware_id=airy.metadata.source_id;
      status.level=airy.deliverable?status.OK:status.WARN;
      status.message=airy.deliverable?"qualified classified hits":airy.ground_unavailable?
        "ground source unavailable; no mark/clear or consumer evidence refresh":
        "no finite returns; no consumer evidence refresh";
      const auto add=[&](const std::string& key,const auto& value) {
        diagnostic_msgs::msg::KeyValue item;item.key=key;item.value=std::to_string(value);
        status.values.push_back(std::move(item));
      };
      diagnostic_msgs::msg::KeyValue epoch;epoch.key="source_epoch";epoch.value=airy.metadata.epoch;
      status.values.push_back(std::move(epoch));
      add("sequence",airy.metadata.sequence);add("source_ns",airy.metadata.source_ros_ns);
      add("input_points",airy.input_points);add("finite_points",airy.finite_points);
      add("invalid_points",airy.invalid_points);add("delivered",airy.deliverable);
      add("ground_source_unavailable",airy.ground_unavailable);
      add("geometry_audit_failed",airy_audit_failed.load());
      // A published Airy packet originates from this already constructed
      // worker. Reuse its queue counters; none is measurement/completion credit.
      const auto filter_stats=airy_compute->stats();
      add("filter_received",filter_stats.admitted);add("filter_processed",filter_stats.completed);
      add("filter_replaced",filter_stats.replaced);add("filter_in_flight",filter_stats.in_flight);
      add("filter_pending",filter_stats.pending);add("filter_closed",filter_stats.closed);
      add("filter_errors",filter_stats.errors);
      add("filter_tid",airy_filter_tid.load());
      const auto ground_stats=airy_ground->stats();
      add("ground_received",ground_stats.admitted);add("ground_processed",ground_stats.completed);
      add("ground_replaced",ground_stats.replaced);add("ground_in_flight",ground_stats.in_flight);
      add("ground_pending",ground_stats.pending);add("ground_closed",ground_stats.closed);
      add("ground_errors",ground_stats.errors);
      add("ground_tid",airy_ground_tid.load());
      add("ground_qualification",static_cast<unsigned>(airy.ground.result));
      add("ground_qualified",airy.ground.safety_qualified);
      add("ground_transform_provider_state",airy.ground.transform_diagnostic.provider_state);
      diagnostic_msgs::msg::KeyValue transform_reason;
      transform_reason.key="ground_transform_reason";
      transform_reason.value=airy.ground.transform_diagnostic.reason;
      status.values.push_back(std::move(transform_reason));
      add("candidate_ground_points",airy.ground.candidate_ground_points);
      add("candidate_obstacle_points",airy.ground.candidate_obstacle_points);
      add("candidate_uncertain_points",airy.ground.candidate_uncertain_points);
      report.status.push_back(std::move(status));
    }
    for (const auto & value : snapshot) {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "sunhorizon_nav2_direct/" + value.stage; status.hardware_id = value.metadata.source_id;
      status.message = "stage completion timing; control use unobserved";
      diagnostic_msgs::msg::KeyValue epoch; epoch.key = "source_epoch"; epoch.value = value.metadata.epoch;
      status.values.push_back(std::move(epoch));
      const auto add = [&](const std::string & key, const auto & data) {
        diagnostic_msgs::msg::KeyValue item; item.key = key; item.value = std::to_string(data); status.values.push_back(item);
      };
      add("sequence", value.metadata.sequence); add("source_ns", value.metadata.source_ros_ns);
      add("raw_source_ns", value.metadata.raw_source_ns);
      add("normalization_start_steady_ns", value.metadata.normalization_start_steady_ns);
      add("normalization_end_steady_ns", value.metadata.normalization_end_steady_ns);
      add("normalization_thread_cpu_ns", value.metadata.normalization_thread_cpu_ns);
      add("first_sdk_receipt_ros_ns", value.metadata.first_receipt_ros_ns);
      add("first_sdk_receipt_steady_ns", value.metadata.first_receipt_steady_ns);
      add("queue_admitted_steady_ns", value.queue_admitted_steady_ns); add("dequeued_steady_ns", value.dequeued_steady_ns);
      add("sink_begin_steady_ns", value.begin_steady_ns); add("sink_end_steady_ns", value.end_steady_ns);
      add("thread_cpu_ns", value.thread_cpu_ns);
      report.status.push_back(std::move(status));
    }
    for (const auto & route : bodies) {
      const auto append = [&](const auto & sinks) {
        for (const auto & sink : sinks) {
          const auto stats = sink->worker->stats();
          diagnostic_msgs::msg::DiagnosticStatus status;
          status.name = "sunhorizon_nav2_direct/" + sink->stage + ".latest_queue";
          status.hardware_id = route->names.camera_role;
          status.message = "instantaneous raw clearing; one in-flight and one latest pending";
          status.level = stats.errors ? diagnostic_msgs::msg::DiagnosticStatus::ERROR : diagnostic_msgs::msg::DiagnosticStatus::OK;
          const auto add = [&](const std::string & key, const auto & data) {
            diagnostic_msgs::msg::KeyValue item; item.key = key; item.value = std::to_string(data);
            status.values.push_back(std::move(item));
          };
          add("admitted",stats.admitted); add("replacements",stats.replaced);
          add("taken",stats.taken); add("completed",stats.completed); add("skipped",stats.skipped);
          add("errors",stats.errors); add("closed",stats.closed);
          add("pending",stats.pending); add("in_flight",stats.in_flight);
          report.status.push_back(std::move(status));
        }
      };
      append(route->raw_local); append(route->raw_global);
    }
    if (!report.status.empty()) timing_publisher->publish(std::move(report));
    for(std::size_t role=0;role<rgbd_audit_rearmed.size();++role)if(rgbd_audit_rearmed[role])
      RCLCPP_INFO(host->get_logger(),"RGBD projection audit rearmed: consumer=%s role=%s request=%s",role<2?"global":"local",role%2==0?"belly":"chest",rgbd_request.c_str());
    for(std::size_t role=0;role<rgbd_audit_exhausted.size();++role)if(rgbd_audit_exhausted[role])
      RCLCPP_WARN(host->get_logger(),"RGBD projection audit unavailable: consumer=%s role=%s attempts=8; no same-frame accepted projection",role<2?"global":"local",role%2==0?"belly":"chest");
    for(std::size_t role=0;role<rgbd_audits.size();++role)if(rgbd_audits[role])try {
      const auto& frame=*rgbd_audits[role];
      const auto bytes=detail::serialize_rgbd_projection_audit(frame,options.ground_reference_owner.get());
      const auto path=perception::detail::write_rgbd_projection_audit_snapshot(ground_snapshot_directory,
        role%2==0?"belly":"chest",frame.metadata.sequence,bytes,role<2?"global":"local");
      RCLCPP_INFO(host->get_logger(),"RGBD accepted %s projection audit: %s",role<2?"global":"local",path.c_str());
    }catch(const std::exception& error){
      RCLCPP_WARN(host->get_logger(),"RGBD projection audit failed: %s",error.what());
    }catch(...){RCLCPP_WARN(host->get_logger(),"RGBD projection audit failed");}
    if(ground_snapshot)try {
      const auto path=perception::detail::write_ground_classification_snapshot(
        ground_snapshot_directory,"airy",*ground_snapshot,options.ground_reference_owner.get());
      RCLCPP_INFO(host->get_logger(),"Airy classification snapshot: %s",path.c_str());
    }catch(const std::exception& error) {
      RCLCPP_WARN(host->get_logger(),"Airy classification snapshot failed: %s",error.what());
    }
    if(geometry_audit)try {
      const auto bytes=detail::serialize_airy_geometry_audit(*geometry_audit->classification,
        *geometry_audit->filtered,geometry_audit->projection.get());
      const auto path=perception::detail::write_airy_geometry_audit_snapshot(ground_snapshot_directory,
        geometry_audit->metadata.sequence,bytes);
      RCLCPP_INFO(host->get_logger(),"Airy geometry audit snapshot: %s",path.c_str());
    }catch(const std::exception& error){
      RCLCPP_WARN(host->get_logger(),"Airy geometry audit snapshot failed: %s",error.what());
    }
  }
  void bind_costmap_timing(const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & map, const std::string & prefix) {
    for (const auto & layer : *map->getLayeredCostmap()->getPlugins()) {
      if (auto obstacle = std::dynamic_pointer_cast<nav2_costmap_2d::ObstacleLayer>(layer)) {
        obstacle->setDirectTimingSink([this,prefix](const auto & timing) {record(prefix, timing);});
        if(!ground_snapshot_directory.empty() && (prefix=="global."||prefix=="local."))obstacle->setDirectProjectionSink(
          [this,prefix](const auto& metadata){std::lock_guard<std::mutex> lock(timing_mutex);
            return (prefix=="global." && airy_audit_frame && !airy_audit_frame->projection &&
              nav2_util::directMetadataEqual(airy_audit_frame->metadata,metadata)) ||
              std::any_of(rgbd_projection_capture.begin(),rgbd_projection_capture.end(),
                [&](const auto& capture){return capture.wants(metadata,prefix=="local."?
                  detail::RgbdProjectionConsumer::Local:detail::RgbdProjectionConsumer::Global);});},
          [this,prefix](const nav2_costmap_2d::DirectProjectionSnapshot& snapshot){
            std::lock_guard<std::mutex> lock(timing_mutex);
            for(auto& capture:rgbd_projection_capture)(void)capture.receive(snapshot,prefix=="local."?
              detail::RgbdProjectionConsumer::Local:detail::RgbdProjectionConsumer::Global);
            if(prefix=="global." && airy_audit_frame && !airy_audit_frame->projection &&
                nav2_util::directMetadataEqual(airy_audit_frame->metadata,snapshot.metadata) &&
                airy_audit_frame->source_cloud==snapshot.source_cloud)
              airy_audit_frame->projection=std::make_shared<const nav2_costmap_2d::DirectProjectionSnapshot>(snapshot);
          });
      }
    }
  }
  void verify_classified_source_inventory() const {
    if(options.camera_maximum_age_ns>sunhorizon_obstacle_coverage::kDefaultCameraAgeNs) {
      sunhorizon_obstacle_coverage::validate_low_speed_camera_limits(
        controller->get_parameter("FollowPath.vx_max").as_double(),
        controller->get_parameter("FollowPath.vy_max").as_double(),
        controller->get_parameter("FollowPath.wz_max").as_double());
      if(!options.classified_obstacles_enabled ||
          !controller->get_parameter("FollowPath.stopping_prediction.enabled").as_bool() ||
          controller->get_parameter("FollowPath.vx_min").as_double()!=0.)
        throw std::invalid_argument("extended_camera_age_requires_forward_measured_stopping_controller");
    }
    if (!options.classified_obstacles_enabled) return;
    const auto require_source_lease=[](const auto& node,const std::string& source,int64_t producer_budget) {
      const double age=node->get_parameter(source+".direct_maximum_age_sec").as_double();
      // Classification's reference lease covers the complete original source
      // lease. No consumer may retain Ground longer than that source lease.
      if(!std::isfinite(age) || age<=0. || age>10. || producer_budget<=0 ||
          rclcpp::Duration::from_seconds(age).nanoseconds()>producer_budget)
        throw std::invalid_argument("classified_consumer_age_exceeds_original_source_lease:"+source);
    };
    const auto check_map=[&](const auto& map,bool local) {
      std::set<std::string> expected,actual,rgbd_layers;
      for (const auto& body:options.body_sources) {
        const auto& layer=local?body.local_layer:body.global_layer;
        const auto& source=local?body.local_source:body.global_source;
        if (!source.empty()) {
          expected.insert(layer+"."+source);rgbd_layers.insert(layer);
          require_source_lease(map,layer+"."+source,options.camera_maximum_age_ns);
        }
      }
      const auto& airy=options.airy_source;
      for(const auto& source:local?airy.local_sources:airy.global_sources) {
        expected.insert((local?airy.local_layer:airy.global_layer)+"."+source);
        require_source_lease(map,(local?airy.local_layer:airy.global_layer)+"."+source,
          options.airy_input_contract.maximum_age_ns);
      }
      for(const auto& plugin:*map->getLayeredCostmap()->getPlugins()) {
        if(map->get_parameter(plugin->getName()+".plugin").as_string()=="nav2_costmap_2d::InflationLayer") {
          const auto key=plugin->getName()+".inflate_unknown";
          if(map->get_parameter(key).as_bool() || !map->describe_parameter(key).read_only)
            throw std::invalid_argument("classified_unknown_must_survive_inflation");
        }
        if (rgbd_layers.count(plugin->getName()) &&
            (map->get_parameter(plugin->getName()+".plugin").as_string()!="nav2_costmap_2d::VoxelLayer" ||
             !std::dynamic_pointer_cast<nav2_costmap_2d::VoxelLayer>(plugin)))
          throw std::invalid_argument("classified_rgbd_requires_measurement_support_voxel_layer");
        if (!std::dynamic_pointer_cast<nav2_costmap_2d::ObstacleLayer>(plugin)) continue;
        std::istringstream names(map->get_parameter(plugin->getName()+".observation_sources").as_string());
        std::string source;
        while(names>>source) {
          const auto prefix=plugin->getName()+"."+source;
          if(!actual.insert(prefix).second ||
             map->get_parameter(prefix+".input_mode").as_string()!="direct-classified-v1" ||
             !map->get_parameter(prefix+".marking").as_bool() ||
             !map->get_parameter(prefix+".clearing").as_bool() ||
             map->get_parameter(prefix+".observation_persistence").as_double()!=0.)
            throw std::invalid_argument("classified_costmap_requires_atomic_nonpersistent_sources");
        }
      }
      if(actual!=expected)throw std::invalid_argument("classified_costmap_has_unbound_or_missing_source");
    };
    check_map(controller->getCostmapROS(),true);check_map(planner->getCostmapROS(),false);
    std::set<std::string> expected;
    if(obstacle_uses_airy(options.obstacle_source_mode)) {
      expected.insert(options.airy_source.collision_source);
      require_source_lease(collision,options.airy_source.collision_source,options.airy_input_contract.maximum_age_ns);
    }
    for(const auto& body:options.body_sources) {
      expected.insert(body.collision_source);
      require_source_lease(collision,body.collision_source,options.camera_maximum_age_ns);
    }
    const auto actual=collision->get_parameter("observation_sources").as_string_array();
    if(std::set<std::string>(actual.begin(),actual.end())!=expected || actual.size()!=expected.size())
      throw std::invalid_argument("classified_collision_has_unbound_or_missing_source");
    for(const auto& source:actual)
      if(collision->get_parameter(source+".input_mode").as_string()!="direct-classified-v1")
        throw std::invalid_argument("classified_collision_requires_same_classification");
  }
  void observe_admission(const nav2_util::DirectObservationMetadata& metadata,
    const nav2_util::DirectAdmissionResult& result, const std::string& stage, const std::string& reason_detail={}) {
    {
      std::lock_guard<std::mutex> lock(timing_mutex);
      const auto key=stage+"/"+metadata.source_id;
      if(admissions.count(key) || admissions.size()<128)admissions[key]={metadata,result,reason_detail};
      // Keep the last rejected original snapshot through a later accepted frame;
      // the periodic observer may otherwise miss the transient failure entirely.
      if(result.state==nav2_util::DirectAdmissionState::MotionUnavailable ||
          result.state==nav2_util::DirectAdmissionState::InternalFault) {
        const auto rejected=key+"/last_rejected";
        if(admissions.count(rejected) || admissions.size()<128)admissions[rejected]={metadata,result,reason_detail};
      }
    }
    // Only a structural, identity, or clock contradiction is an owner fault.
    // Expiry/TF/drop leaves the original consumer's validity checks in charge.
    detail::require_nonfatal_cloud_admission(result,stage+":"+reason_detail+":frame{role="+metadata.source_id+
      ",epoch="+metadata.epoch+",seq="+std::to_string(metadata.sequence)+
      ",source="+std::to_string(metadata.source_ros_ns)+
      ",receipt="+std::to_string(metadata.first_receipt_ros_ns)+
      ",receipt_s="+std::to_string(metadata.first_receipt_steady_ns)+"}");
  }
  void forward_cloud(CloudDelivery delivery, const nav2_util::DirectCloudResultSink & sink, const std::string & stage) {
    if (!running() || !activated.load()) return;
    const auto dequeued = nav2_util::directSteadyNowNs(), cpu = nav2_util::directThreadCpuNs();
    const auto result = sink(delivery.cloud);
    const auto end = nav2_util::directSteadyNowNs(), end_cpu = nav2_util::directThreadCpuNs();
    record("", {delivery.cloud.metadata, stage, delivery.queued_ns, dequeued, dequeued, end,
      cpu >= 0 && end_cpu >= 0 ? end_cpu-cpu : -1});
    observe_admission(delivery.cloud.metadata,result,stage);
  }
  std::unique_ptr<RawSink> raw_sink(const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>& map,
    const std::string& role, const std::string& layer, const std::string& source, const std::string& prefix) {
    const auto binding=role+":"+prefix+layer+"/"+source;
    // Read the configured CHILD costmap parameter used by the real original
    // ObservationBuffer, not a host/default or an independent runtime overlay.
    detail::require_instantaneous_raw_clearing(
      map->get_parameter(layer+"."+source+".observation_persistence").as_double(),binding);
    const auto sink=nav2_costmap_2d::bindDirectCostmapResultSource(map,layer,source,
      nav2_util::DirectCloudKind::RetainedIndependentSource);
    auto output=std::make_unique<RawSink>();
    output->stage=prefix+"raw_clearing_buffer_returned."+source;
    const auto stage=output->stage;
    output->worker=std::make_unique<sunhorizon_camera_runtime::LatestFrameWorker<CloudDelivery>>(
      [this,sink,stage](auto& delivery){forward_cloud(delivery,sink,stage);},
      [this]{return running() && activated.load();},
      [this,binding](std::exception_ptr error){fault("nav2_raw_clearing_worker_failed:"+binding+":"+exception_reason(error));});
    return output;
  }
  void replace_raw(RawSink& sink, const CloudDelivery& delivery,
    std::vector<std::unique_ptr<CloudDelivery>>& retired) {
    auto value=std::make_unique<CloudDelivery>(delivery);
    const auto* offered=value.get();
    auto previous=sink.worker->replace(std::move(value));
    const bool rejected=previous.get()==offered;
    // The caller declares retired before its admission lock, so destroying an
    // evicted immutable cloud owner never runs under that gate's mutex.
    retired.push_back(std::move(previous));
    const auto stats=sink.worker->stats();
    if(rejected || stats.closed || stats.errors)
      throw std::runtime_error("nav2_raw_clearing_worker_closed:"+delivery.cloud.metadata.source_id+":"+sink.stage);
  }
  void receive_airy_filtered(const AiryOutput & output) {
    if (!running()) return;
    if (!output || !output->input || !output->cloud) throw std::invalid_argument("missing_filtered_airy_owner");
    if(options.filtered_airy)options.filtered_airy(output);
    if (!running()) return;
    auto pending=std::make_unique<AiryGroundInput>(AiryGroundInput{output,nav2_util::directSteadyNowNs()});
    const auto* offered=pending.get();
    auto retired=airy_ground->replace(std::move(pending));
    const auto stats=airy_ground->stats();
    if(retired.get()==offered || stats.closed || stats.errors) {
      if (!running()) return;
      throw std::runtime_error("nav2_airy_ground_worker_closed");
    }
    // Any replaced owner is destroyed here, outside the worker queue mutex.
  }
  void process_airy_ground(const AiryOutput & output) {
    if (!running() || !activated.load()) return;
    const auto & stamp = output->input->stamp;
    nav2_util::DirectObservationMetadata metadata{stamp.source_identity, stamp.epoch, output->cloud->header.frame_id,
      stamp.sequence, stamp.source_ns, stamp.first_receipt_ros_ns, stamp.first_receipt_steady_ns, stamp.raw_source_ns,
      stamp.normalization_start_steady_ns,stamp.normalization_end_steady_ns,stamp.normalization_thread_cpu_ns};
    CloudDelivery delivery{{output->cloud, metadata, nav2_util::DirectCloudKind::RetainedIndependentSource}, nav2_util::directSteadyNowNs()};
    delivery.cloud.ray_origins=output->ray_origins;
    if (options.classified_obstacles_enabled) {
      detail::AiryGroundContext ground;
      if(options.ground_reference_owner) {
        ground.owner=options.ground_reference_owner;
        ground.calibration_id=options.airy_ground_calibration_id;
        ground.expiry_budget_ns=options.airy_input_contract.maximum_age_ns;
        ground.registration_geometry=options.registration_ground_geometry;
        auto& transform=ground.source_to_reference;
        transform.source_frame=metadata.frame_id;transform.common_frame=ground.owner->reference_frame();
        transform.source_ns=transform.common_ns=metadata.source_ros_ns;
        if(!ground.owner->registration_map_asset())try {
          // This compute owner never waits for latest TF or extends the scan's
          // freshness deadline. Missing source-time TF makes this source unavailable.
          const auto tf=buffer->lookupTransform(transform.common_frame,transform.source_frame,
            rclcpp::Time(metadata.source_ros_ns,host->get_clock()->get_clock_type()),rclcpp::Duration::from_seconds(0.));
          const auto rigid=sunhorizon_rgbd_adapter::dynamic_self_model::rigid(tf.transform);
          transform.rotation=rigid.rotation;
          transform.translation={rigid.translation.x,rigid.translation.y,rigid.translation.z};
          transform.qualified=true;
        } catch(const tf2::TransformException&) {transform.qualified=false;}
      }
      auto classified=detail::make_classified_airy_packet(delivery.cloud,ground,output->ground_blocked,output->sample_ids);
      if(classified.deliverable)
        detail::attach_ground_clearance(classified.frame,classified.classification,ground.owner);
      {std::lock_guard<std::mutex> lock(timing_mutex);
        airy_packet={metadata,classified.input_points,classified.finite_points,
          classified.invalid_points,classified.deliverable,classified.ground_unavailable};
        if(classified.classification)airy_packet.ground=classified.classification->ground();
        if(!ground_snapshot_directory.empty()&&!airy_ground_snapshot_selected&&classified.classification&&
            classified.classification->ground().transform_diagnostic.provider_state==0&&
            classified.classification->ground().result==perception::ground_relative::Qualification::Qualified&&
            classified.classification->ground().safety_qualified) {
          if(!airy_audit_armed){
            // Arm only after actual ground qualification. At most eight later
            // filter attempts can capture; startup cannot allocate indefinitely.
            airy_audit_armed=true;airy_audit_requested->store(true);
          }else if(output->lidar_audit || !airy_audit_requested->load()){
            airy_ground_snapshot_pending=classified.classification;
            airy_ground_snapshot_selected=true;
            try {airy_audit_frame=std::make_unique<AiryAuditFrame>(AiryAuditFrame{metadata,classified.frame.cloud,
              classified.classification,output,{},nav2_util::directSteadyNowNs()});}
            catch(...){airy_audit_failed.store(true);}
            airy_audit_requested->store(false);
          }
        }
      }
      if(!classified.deliverable)return;
      delivery.cloud=std::move(classified.frame);
    }
    if (!running() || !activated.load()) return;
    const bool local = airy_local->admit(delivery), global = !airy_global || airy_global->admit(delivery),
      collision_ok = airy_collision->admit(std::move(delivery));
    if (!local || !global || !collision_ok) throw std::runtime_error("nav2_filtered_airy_lane_overflow_or_closed");
  }
  std::string health_failure_locked() const {
    detail::CanonicalHealthSnapshot s;
    s.check_ros_ns = host->now().nanoseconds();
    s.check_steady_ns = nav2_util::directSteadyNowNs();
    s.reset_epoch = reset_epoch;
    s.tracking_budget_ns = options.canonical_odometry.maximum_age_ns;
    s.health_budget_ns = options.localization_health_maximum_age_ns;
    if (latest_state) {
      s.has_state = true; s.state_epoch = latest_state->reset_epoch; s.state_sequence = latest_state->sequence;
      const auto & t = latest_state->tracking;
      s.source_ns = t.source_ns; s.receipt_ros_ns = t.first_receipt_ros_ns;
      s.receipt_steady_ns = t.first_receipt_steady_ns;
      s.tracking_sequence = t.sequence; s.tracking_identity = t.identity;
    }
    if (latest_health) {
      s.has_health = true; s.health_epoch = latest_health->reset_epoch;
      s.health_ros_ns = latest_health->evaluated_ros_ns;
      s.health_steady_ns = latest_health->evaluated_steady_ns;
      if (latest_health->status) {
        s.has_status = true;
        s.status_healthy = latest_health->status->state == latest_health->status->STATE_HEALTHY;
        s.measurement_accepted = latest_health->status->measurement_accepted;
        s.status_stamp_ns = stamp_ns(latest_health->status->header.stamp);
      }
      if (latest_health->last_state) {
        s.has_health_state = true; s.health_state_epoch = latest_health->last_state->reset_epoch;
        s.health_state_sequence = latest_health->last_state->sequence;
      }
    }
    const auto * failure = detail::canonical_health_failure(s);
    std::string diagnostic = *failure ? detail::canonical_health_diagnostic(s, failure) : std::string{};
    if (!diagnostic.empty() && latest_health && latest_health->status) {
      // Freeze the original status under this same state_mutex observation.
      // Its asynchronous ROS observer may skip a short-lived rejection.
      const auto & status = *latest_health->status;
      std::ostringstream original;
      original.precision(17);
      original << " canonical_status_state=" << static_cast<unsigned>(status.state)
          << " canonical_status_backend=" << status.backend
          << " canonical_status_measurement_age_sec=" << status.measurement_age_sec
          << " canonical_status_reason=" << status.reason;
      diagnostic += original.str();
    }
    if (*failure && !detail::canonical_health_motion_unavailable(failure))
      throw std::runtime_error("canonical_localization_internal_fault:" + diagnostic);
    return diagnostic;
  }
  void hold_localization_locked(const std::string & failure) {
    if (failure.empty() || canonical_motion_hold.load()) return;
    canonical_hold_reason=failure;
    canonical_motion_hold.store(true);
    changed.notify_all();
    RCLCPP_ERROR(host->get_logger(), "canonical_localization_motion_held: %s; explicit new runtime session required", failure.c_str());
  }
  void forward_odometry_locked() {
    // Deactivation closes the original direct odometry ingress. Keep caching
    // new canonical evidence, but never turn its expected rejection into fault.
    if (!activated.load() || canonical_motion_hold.load() || !latest_state || latest_state->sequence <= injected_sequence) return;
    const auto & state = *latest_state;
    const auto & stamp = state.tracking;
    const auto & contract = options.canonical_odometry;
    nav2_util::DirectOdometry direct{state.odometry,
      {contract.source_id,contract.epoch,contract.frame_id,state.sequence,
        stamp.source_ns,stamp.first_receipt_ros_ns,stamp.first_receipt_steady_ns}};
    if(stamp.body_velocity){
      const auto& b=*stamp.body_velocity;
      nav2_util::DirectBodyVelocityEvidence evidence;
      evidence.window_begin_ns=b.window_begin_ns;evidence.window_end_ns=b.window_end_ns;
      evidence.effective_source_ns=b.effective_source_ns;evidence.reset_epoch=b.reset_epoch;
      evidence.sample_count=b.sample_count;evidence.variance_vx=b.variance_vx;
      evidence.variance_vy=b.variance_vy;evidence.variance_wz=b.variance_wz;
      evidence.valid=b.valid;evidence.measured_waist=b.measured_waist;direct.body_velocity=evidence;
    }
    if (!controller->submitDirectOdometry(std::move(direct))) {
      // Admission takes its own clocks. An original lease can expire after our
      // health check; that race is still motion unavailability, not owner loss.
      const auto failure=health_failure_locked();
      if (failure.empty()) throw std::runtime_error("nav2_original_canonical_odometry_rejected");
      hold_localization_locked(failure);
      return;
    }
    injected_sequence = state.sequence;
  }
  void receive(const State & state) {
    if (!running()) return;
    if (!state || !state->odometry || !state->sequence ||
      !state->tracking.sequence || state->tracking.identity.empty() ||
      state->tracking.source_ns != stamp_ns(state->odometry->header.stamp))
      throw std::invalid_argument("invalid_original_canonical_state");
    std::lock_guard<std::mutex> lock(state_mutex);
    if (!epoch_pinned) {reset_epoch = state->reset_epoch; epoch_pinned = true;}
    if (state->reset_epoch != reset_epoch || state->sequence <= state_sequence)
      throw std::invalid_argument("canonical_reset_or_sequence_requires_new_nav2_owner");
    latest_state = state; state_sequence = state->sequence;
    if (activated.load()) {
      const auto failure = health_failure_locked();
      hold_localization_locked(failure);
    }
    forward_odometry_locked();
    changed.notify_all();
  }
  void receive(const Health & health) {
    if (!running()) return;
    if (!health || !health->status) throw std::invalid_argument("missing_original_canonical_health");
    std::lock_guard<std::mutex> lock(state_mutex);
    if (epoch_pinned && health->reset_epoch != reset_epoch)
      throw std::runtime_error("canonical_health_reset_requires_new_nav2_owner");
    latest_health = health;
    if (activated.load()) {
      const auto failure = health_failure_locked();
      hold_localization_locked(failure);
    }
    changed.notify_all();
  }
  bool transforms_ready() const {
    const auto local = controller->getCostmapROS(), global = planner->getCostmapROS();
    return buffer->canTransform(local->getGlobalFrameID(), local->getBaseFrameID(), tf2::TimePointZero) &&
      buffer->canTransform(global->getGlobalFrameID(), global->getBaseFrameID(), tf2::TimePointZero);
  }
  void activation_loop() noexcept {
    try {
      while (running()) {
        if (options.canonical_owner->failed()) throw std::runtime_error("canonical_manager_owner_failed");
        std::string health_failure;
        {std::lock_guard<std::mutex> lock(state_mutex);
          health_failure = health_failure_locked();
          if (activated.load()) hold_localization_locked(health_failure);
        }
        if (activated.load() && canonical_motion_hold.load() && !controller_suspended.load()) {
          // The original lifecycle transition aborts current/pending FollowPath
          // goals, revokes odometry ingress and emits its typed explicit zero.
          // Collision Monitor and the independent final supervisor stay active;
          // acquisition and localization continue. No automatic goal resumption.
          if (controller->deactivate().id() != LifecycleState::PRIMARY_STATE_INACTIVE)
            throw std::runtime_error("nav2_localization_hold_controller_deactivation_failed");
          controller_suspended.store(true);
        } else if (!activated.load() && health_failure.empty() && transforms_ready()) {
          if (collision->activate().id() != LifecycleState::PRIMARY_STATE_ACTIVE ||
            planner->activate().id() != LifecycleState::PRIMARY_STATE_ACTIVE ||
            controller->activate().id() != LifecycleState::PRIMARY_STATE_ACTIVE)
            throw std::runtime_error("nav2_original_component_activation_failed");
          if (options.spatial_query.height_limited()) {
            // Immutable descriptors already reject before every callback.
            // Recheck actual configured members before startup qualification.
            bind_collision_height_responsibility();
          }
          std::lock_guard<std::mutex> lock(state_mutex);
          const auto failure = health_failure_locked();
          activated.store(true);
          hold_localization_locked(failure);
          forward_odometry_locked();
        }
        std::unique_lock<std::mutex> wait_lock(reason_mutex);
        changed.wait_for(wait_lock, std::chrono::milliseconds(20));
      }
    } catch (...) {fault(exception_reason(std::current_exception()));}
    activated.store(false);
    // Keep Collision Monitor active until external stop so the real DDS zero
    // and typed origin produced by controller deactivation can still be read.
    if (terminal.load()) {
      try {if (controller->get_current_state().id() == LifecycleState::PRIMARY_STATE_ACTIVE)
        controller->deactivate();} catch (...) {}
    }
  }
  void freeze_collision_height_configuration() {
    if (!options.spatial_query.height_limited() && !options.classified_obstacles_enabled) return;
    const auto require_overrides=[](const auto& overrides,const std::vector<std::string>& keys) {
      for (const auto& key:keys) if (!overrides.count(key))
        throw std::invalid_argument("collision_height_explicit_parameter_required:"+key);
    };
    detail::freeze_costmap_collision_height_configuration(*controller->getCostmapROS(),options.classified_obstacles_enabled);
    detail::freeze_costmap_collision_height_configuration(*planner->getCostmapROS(),options.classified_obstacles_enabled);
    const auto overrides=collision->get_node_parameters_interface()->get_parameter_overrides();
    require_overrides(overrides,{"observation_sources","base_frame_id","odom_frame_id"});
    std::set<std::string> prefixes;
    std::map<std::string,rclcpp::ParameterValue> defaults;
    for (const auto& source:overrides.at("observation_sources").get<std::vector<std::string>>()) {
      prefixes.insert(source+".");
      require_overrides(overrides,{source+".type",source+".min_height",source+".max_height"});
      defaults.emplace(source+".enabled",rclcpp::ParameterValue(true));
    }
    detail::freeze_collision_height_parameters(*collision,
      {"observation_sources","base_frame_id","odom_frame_id"},prefixes,defaults);
  }
  void bind_collision_height_responsibility() {
    if (!options.spatial_query.height_limited()) return;
    using Source=perception::detail::CollisionHeightSource;
    std::vector<Source> sources;
    std::map<std::string,std::set<std::string>> expected{{"local",{}},{"global",{}},{"collision",{}}};
    for (const auto& body:options.body_sources) {
      expected["local"].insert(body.local_layer+"/"+body.local_source);
      if (!body.global_source.empty()) expected["global"].insert(body.global_layer+"/"+body.global_source);
      expected["collision"].insert(body.collision_source);
    }
    const auto expect_airy=[&](const auto& costmap,const auto& names,const std::string& layer,const std::string& consumer){
      for (const auto& source:names) {
        // The full route includes floor clearing. Only actual marking routes
        // contribute collision-height responsibility; every route is still
        // required by the original bindDirectCostmapResultSource below.
        if (costmap->get_parameter(layer+"."+source+".marking").as_bool())
          expected[consumer].insert(layer+"/"+source);
      }
    };
    expect_airy(controller->getCostmapROS(),options.airy_source.local_sources,options.airy_source.local_layer,"local");
    expect_airy(planner->getCostmapROS(),options.airy_source.global_sources,options.airy_source.global_layer,"global");
    expected["collision"].insert(options.airy_source.collision_source);
    const auto costmap_sources=[&](const auto& costmap,const std::string& consumer,const std::string& frame) {
      if (!costmap || !costmap->getLayeredCostmap() || costmap->getGlobalFrameID()!=frame ||
          costmap->getGlobalFrameID()!=costmap->get_parameter("global_frame").as_string() ||
          costmap->getBaseFrameID()!=costmap->get_parameter("robot_base_frame").as_string() ||
          (costmap->getBaseFrameID()!="base_footprint" && costmap->getBaseFrameID()!="base_link"))
        throw std::invalid_argument("collision_height_costmap_frame_mismatch");
      const auto declared=costmap->get_parameter("plugins").as_string_array();
      std::set<std::string> plugins;
      for (const auto& plugin:*costmap->getLayeredCostmap()->getPlugins()) {
        const auto layer=plugin->getName();
        if (!plugins.insert(layer).second) throw std::invalid_argument("collision_height_duplicate_actual_plugin");
        const auto obstacle=std::dynamic_pointer_cast<nav2_costmap_2d::ObstacleLayer>(plugin);
        if (!obstacle) {
          const auto type=costmap->get_parameter(layer+".plugin").as_string();
          if (type!="nav2_costmap_2d::StaticLayer" && type!="nav2_costmap_2d::InflationLayer")
            throw std::invalid_argument("collision_height_unreviewed_costmap_plugin");
          continue;
        }
        if (!costmap->get_parameter(layer+".enabled").as_bool())
          throw std::invalid_argument("collision_height_marking_layer_disabled");
        std::istringstream names(costmap->get_parameter(layer+".observation_sources").as_string());
        std::set<std::string> unique;
        for (std::string name;names>>name;) {
          if (!unique.insert(name).second) throw std::invalid_argument("collision_height_duplicate_observation_source");
          const auto prefix=layer+"."+name+".";
          const bool marking=costmap->get_parameter(prefix+"marking").as_bool();
          if (!marking) continue;
          if (costmap->get_parameter(prefix+"data_type").as_string()!="PointCloud2")
            throw std::invalid_argument("collision_height_requires_original_cloud_marking");
          sources.push_back({consumer,layer+"/"+name,frame,true,
            costmap->get_parameter(prefix+"min_obstacle_height").as_double(),
            costmap->get_parameter(prefix+"max_obstacle_height").as_double()});
        }
      }
      if (plugins!=std::set<std::string>(declared.begin(),declared.end()) || plugins.size()!=declared.size())
        throw std::invalid_argument("collision_height_configured_plugin_inventory_mismatch");
      const auto declared_filters=costmap->get_parameter("filters").as_string_array();
      const auto& filters=*costmap->getLayeredCostmap()->getFilters();
      if (declared_filters!=std::vector<std::string>{"keepout_filter"} || filters.size()!=1 ||
          !filters[0] || filters[0]->getName()!="keepout_filter" ||
          !std::dynamic_pointer_cast<nav2_costmap_2d::KeepoutFilter>(filters[0]) ||
          costmap->get_parameter("keepout_filter.plugin").as_string()!="nav2_costmap_2d::KeepoutFilter" ||
          !costmap->get_parameter("keepout_filter.enabled").as_bool() || !filters[0]->isEnabled() ||
          costmap->get_parameter("keepout_filter.filter_info_topic").as_string()!="/costmap_filter_info")
        throw std::invalid_argument("collision_height_requires_actual_original_keepout_filter");
    };
    costmap_sources(controller->getCostmapROS(),"local","odom");
    costmap_sources(planner->getCostmapROS(),"global","map");
    const auto collision_frame=collision->get_parameter("base_frame_id").as_string();
    if ((collision_frame!="base_footprint" && collision_frame!="base_link") ||
        collision->get_parameter("odom_frame_id").as_string()!="odom")
      throw std::invalid_argument("collision_height_collision_frame_mismatch");
    const auto collision_sources=collision->get_parameter("observation_sources").as_string_array();
    if (collision_sources.size()!=3) throw std::invalid_argument("collision_height_requires_three_collision_sources");
    for (const auto& name:collision_sources) {
      if (collision->get_parameter(name+".type").as_string()!="pointcloud" ||
          !collision->get_parameter(name+".enabled").as_bool())
        throw std::invalid_argument("collision_height_collision_source_type_or_enabled");
      sources.push_back({"collision",name,collision_frame,true,
        collision->get_parameter(name+".min_height").as_double(),
        collision->get_parameter(name+".max_height").as_double()});
    }
    perception::detail::require_collision_height_sources(options.spatial_query,sources,expected);
    collision_height_verified.store(true);
  }
  void start() {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex);
    if (started || stopping.load() || terminal.load()) throw std::logic_error("nav2_assembly_start_once");
    started = true;
    try {
      // No component executor/NodeThread exists yet. Seal original values before
      // configure reads them or installs any mutating dynamic callbacks.
      freeze_collision_height_configuration();
      if (planner->configure().id() != LifecycleState::PRIMARY_STATE_INACTIVE ||
        controller->configure().id() != LifecycleState::PRIMARY_STATE_INACTIVE ||
        collision->configure().id() != LifecycleState::PRIMARY_STATE_INACTIVE)
        throw std::runtime_error("nav2_original_component_configuration_failed");
      if (collision->get_parameter("cmd_vel_input_mode").as_string() != "dds" ||
        collision->get_parameter("candidate_output_mode").as_string() != "dds" ||
        collision->get_node_topics_interface()->resolve_topic_name(
          collision->get_parameter("cmd_vel_out_topic").as_string()) != "/sunhorizon/navigation/cmd_vel_candidate" ||
        controller->get_node_topics_interface()->resolve_topic_name("cmd_vel") != "/sunhorizon/navigation/cmd_vel_nav2_raw" ||
        collision->get_node_topics_interface()->resolve_topic_name(collision->get_parameter("cmd_vel_in_topic").as_string()) !=
          "/sunhorizon/navigation/cmd_vel_nav2_raw")
        throw std::invalid_argument("nav2_requires_original_candidate_route_and_independent_final_supervisor");
      bind_collision_height_responsibility();
      verify_classified_source_inventory();
      auto failure = [this](std::exception_ptr error) {fault(exception_reason(error));};
      for (const auto & names : options.body_sources) {
        auto route = std::make_unique<BodyRoute>(); route->names = names;
        const auto kind=options.classified_obstacles_enabled ? nav2_util::DirectCloudKind::ClassifiedObstacleFrame :
          nav2_util::DirectCloudKind::SelfFilteredMarking;
        route->local_sink = nav2_costmap_2d::bindDirectCostmapResultSource(controller->getCostmapROS(), names.local_layer, names.local_source,kind);
        if (!names.global_source.empty()) route->global_sink = nav2_costmap_2d::bindDirectCostmapResultSource(planner->getCostmapROS(), names.global_layer, names.global_source,kind);
        route->collision_sink = nav2_collision_monitor::bindDirectCollisionResultSource(collision, names.collision_source,kind);
        for (const auto & source : names.local_clearing_sources)
          route->raw_local.push_back(raw_sink(controller->getCostmapROS(),names.camera_role,names.local_layer,source,"local."));
        for (const auto & source : names.global_clearing_sources)
          route->raw_global.push_back(raw_sink(planner->getCostmapROS(),names.camera_role,names.global_layer,source,"global."));
        auto * owner = route.get();
        route->local = std::make_unique<detail::BoundedLane<CloudDelivery>>(4, [this,owner](auto value) {forward_cloud(std::move(value),owner->local_sink,"local.observation_buffer_returned");}, failure);
        if (route->global_sink) route->global = std::make_unique<detail::BoundedLane<CloudDelivery>>(4, [this,owner](auto value) {forward_cloud(std::move(value),owner->global_sink,"global.observation_buffer_returned");}, failure);
        route->collision = std::make_unique<detail::BoundedLane<CloudDelivery>>(4, [this,owner](auto value) {forward_cloud(std::move(value),owner->collision_sink,"collision.cloud_retained");}, failure);
        bodies.push_back(std::move(route));
      }
      bind_costmap_timing(controller->getCostmapROS(), "local.");
      bind_costmap_timing(planner->getCostmapROS(), "global.");
      collision->setDirectCloudTimingSink([this](const auto & timing) {record("collision.", timing);});
      if(obstacle_uses_airy(options.obstacle_source_mode)) {
      const auto & airy = options.airy_source;
      const auto airy_kind=options.classified_obstacles_enabled ? nav2_util::DirectCloudKind::ClassifiedObstacleFrame :
        nav2_util::DirectCloudKind::RetainedIndependentSource;
      for (const auto & source : airy.local_sources)
        airy_local_sinks.push_back({source,nav2_costmap_2d::bindDirectCostmapResultSource(controller->getCostmapROS(),airy.local_layer,source,airy_kind)});
      for (const auto & source : airy.global_sources)
        airy_global_sinks.push_back({source,nav2_costmap_2d::bindDirectCostmapResultSource(planner->getCostmapROS(),airy.global_layer,source,airy_kind)});
      airy_collision_sink = nav2_collision_monitor::bindDirectCollisionResultSource(collision,airy.collision_source,airy_kind);
      airy_local = std::make_unique<detail::BoundedLane<CloudDelivery>>(4, [this](auto delivery) {
        for (const auto & sink : airy_local_sinks) forward_cloud(delivery,sink.second,"local.observation_buffer_returned."+sink.first);
      },failure);
      if (!airy_global_sinks.empty()) airy_global = std::make_unique<detail::BoundedLane<CloudDelivery>>(4, [this](auto delivery) {
        for (const auto & sink : airy_global_sinks) forward_cloud(delivery,sink.second,"global.observation_buffer_returned."+sink.first);
      },failure);
      airy_collision = std::make_unique<detail::BoundedLane<CloudDelivery>>(4, [this](auto delivery) {
        forward_cloud(std::move(delivery),airy_collision_sink,"collision.cloud_retained");
      },failure);
      // Construct the receiving stage before the self-filter can hand off.
      airy_ground = std::make_unique<sunhorizon_camera_runtime::LatestFrameWorker<AiryGroundInput>>([this](auto& input) {
        if (!running() || !activated.load()) return;
        if(airy_ground_tid.load()<0){perception::detail::ComputeThreadSyscalls calls;
          calls.name_self("shn_airy_ground");airy_ground_tid.store(calls.self_tid());}
        const auto begin=nav2_util::directSteadyNowNs(),cpu=nav2_util::directThreadCpuNs();
        process_airy_ground(input.output);
        const auto end=nav2_util::directSteadyNowNs(),end_cpu=nav2_util::directThreadCpuNs();
        const auto& output=input.output;const auto& stamp=output->input->stamp;
        record("", {{stamp.source_identity,stamp.epoch,output->cloud->header.frame_id,stamp.sequence,
          stamp.source_ns,stamp.first_receipt_ros_ns,stamp.first_receipt_steady_ns,stamp.raw_source_ns,
          stamp.normalization_start_steady_ns,stamp.normalization_end_steady_ns,stamp.normalization_thread_cpu_ns},
          "airy.ground_fanout_returned",input.queued_ns,begin,begin,end,
          cpu>=0 && end_cpu>=0 ? end_cpu-cpu : -1});
      },[this]{return running() && activated.load();},failure);
      airy_compute = std::make_unique<sunhorizon_camera_runtime::LatestFrameWorker<AiryInput>>([this](auto& input) {
        if (!running() || !activated.load()) return;
        if(airy_filter_tid.load()<0){perception::detail::ComputeThreadSyscalls calls;
          calls.name_self("shn_airy_filter");airy_filter_tid.store(calls.self_tid());}
        const auto begin = nav2_util::directSteadyNowNs(), cpu = nav2_util::directThreadCpuNs();
        // The sink now only preserves filtered_airy and enqueues its immutable
        // output. Ground classification/fanout has its own timing below.
        const auto result = airy_filter->receive_result(input);
        const auto end = nav2_util::directSteadyNowNs(), end_cpu = nav2_util::directThreadCpuNs();
        const auto & stamp = input->stamp;
        record("", {{stamp.source_identity,stamp.epoch,input->cloud->header.frame_id,stamp.sequence,
          stamp.source_ns,stamp.first_receipt_ros_ns,stamp.first_receipt_steady_ns,stamp.raw_source_ns,
          stamp.normalization_start_steady_ns,stamp.normalization_end_steady_ns,stamp.normalization_thread_cpu_ns},
          "airy.rig_filter_returned",input->queue_admitted_steady_ns,begin,begin,end,
          cpu >= 0 && end_cpu >= 0 ? end_cpu-cpu : -1});
        using FilterState=sunhorizon_rgbd_adapter::SelfExclusionDirectState;
        using State=nav2_util::DirectAdmissionState;
        using Reason=nav2_util::DirectRejectionReason;
        nav2_util::DirectAdmissionResult admission{State::InternalFault,Reason::InvalidCloud,
          result.snapshot_ros_ns,result.snapshot_steady_ns};
        switch(result.state) {
          case FilterState::Accepted: admission.state=State::Accepted; admission.reason=Reason::None; break;
          case FilterState::Duplicate: admission.state=State::Duplicate; admission.reason=Reason::Duplicate; break;
          case FilterState::MotionUnavailable: admission.state=State::MotionUnavailable; admission.reason=Reason::Invalidated; break;
          case FilterState::InternalFault: break;
        }
        const nav2_util::DirectObservationMetadata metadata{stamp.source_identity,stamp.epoch,
          input->cloud->header.frame_id,stamp.sequence,stamp.source_ns,stamp.first_receipt_ros_ns,
          stamp.first_receipt_steady_ns,stamp.raw_source_ns,stamp.normalization_start_steady_ns,
          stamp.normalization_end_steady_ns,stamp.normalization_thread_cpu_ns};
        observe_admission(metadata,admission,"airy.rig_filter",result.reason);
        if (airy_filter->failed()) throw std::runtime_error("original_airy_rig_filter_failed");
      },[this]{return running() && activated.load();},failure);
      }
      canonical = std::make_unique<detail::BoundedLane<CanonicalInput>>(256,
        [this](const auto & value) {std::visit([this](const auto & input) {receive(input);}, value);}, failure);
      accepting.store(true);
      activation_worker = std::thread([this] {activation_loop();});
    } catch (...) {fault(exception_reason(std::current_exception())); throw;}
  }
  void stop() {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex);
    if (stopping.exchange(true)) return;
    accepting.store(false);
    controller->getCostmapROS()->requestDirectStop();
    planner->getCostmapROS()->requestDirectStop();
    changed.notify_all();
    if (canonical) canonical->request_stop();
    if (airy_compute) airy_compute->request_stop();
    if (airy_ground) airy_ground->request_stop();
    for (auto & route : bodies) {
      route->local->request_stop(); if (route->global) route->global->request_stop(); route->collision->request_stop();
      for(auto& sink:route->raw_local)sink->worker->request_stop();
      for(auto& sink:route->raw_global)sink->worker->request_stop();
    }
    if (airy_local) airy_local->request_stop();
    if (airy_global) airy_global->request_stop();
    if (airy_collision) airy_collision->request_stop();
    if (canonical) canonical->join();
    if (airy_compute) airy_compute->join();
    if (airy_ground) airy_ground->join();
    if (airy_local) airy_local->join();
    if (airy_global) airy_global->join();
    if (airy_collision) airy_collision->join();
    if(airy_filter)airy_filter->stop();
    for (auto & route : bodies) {
      route->local->join(); if (route->global) route->global->join(); route->collision->join();
      for(auto& sink:route->raw_local)sink->worker->join();
      for(auto& sink:route->raw_global)sink->worker->join();
    }
    if (activation_worker.joinable()) activation_worker.join();
    // No direct worker can reference a plugin during lifecycle cleanup. A
    // partially failed parent transition must not strand its child's map thread.
    const std::vector<std::shared_ptr<nav2_util::LifecycleNode>> parents{controller,planner,collision};
    const std::vector<std::shared_ptr<nav2_costmap_2d::Costmap2DROS>> maps{
      controller->getCostmapROS(),planner->getCostmapROS()};
    for (const auto & node : parents) try {
      if (node->get_current_state().id() == LifecycleState::PRIMARY_STATE_ACTIVE) node->deactivate();
    } catch (...) {fault("nav2_parent_deactivation_failed");}
    for (const auto & map : maps) try {
      if (map->get_current_state().id() == LifecycleState::PRIMARY_STATE_ACTIVE) map->deactivate();
    } catch (...) {fault("nav2_child_costmap_deactivation_failed");}
    for (const auto & node : parents) try {
      if (node->get_current_state().id() == LifecycleState::PRIMARY_STATE_INACTIVE) node->cleanup();
    } catch (...) {fault("nav2_parent_cleanup_failed");}
    for (const auto & map : maps) try {
      if (map->get_current_state().id() == LifecycleState::PRIMARY_STATE_INACTIVE) map->cleanup();
    } catch (...) {fault("nav2_child_costmap_cleanup_failed");}
    activated.store(false);
  }
};
Nav2Assembly::Nav2Assembly(std::unique_ptr<Impl> implementation) : impl_(std::move(implementation)) {}
Nav2Assembly::~Nav2Assembly() {try {impl_->stop();} catch (...) {}}
std::vector<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr> Nav2Assembly::nodes() const {
  std::vector<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr> result{impl_->host->get_node_base_interface(),
    impl_->planner->get_node_base_interface(),impl_->controller->get_node_base_interface(),impl_->collision->get_node_base_interface()};
  if(impl_->airy_filter)result.push_back(impl_->airy_filter->node()->get_node_base_interface());
  return result;
}
void Nav2Assembly::start() {impl_->start();}
bool Nav2Assembly::consume_body(Body body) {
  if (!impl_->running() || !body || !body->input) return false;
  for (auto & route : impl_->bodies) if (route->names.camera_role == body->input->role) {
    // Raw projection completes independently of FK/body computation. Each
    // stream owns its evidence high-water mark; neither can renew the other.
    std::vector<std::unique_ptr<CloudDelivery>> retired_raw;
    retired_raw.reserve(route->raw_local.size()+route->raw_global.size());
    std::lock_guard<std::mutex> lock(route->admission_mutex);
    try {
      impl_->validate_identity(body);
      const auto ros = impl_->host->now().nanoseconds(), steady = nav2_util::directSteadyNowNs();
      if ((!route->raw_local.empty() || !route->raw_global.empty()) && body->frame.raw_projection_complete) {
        const auto raw = impl_->raw_cloud(body);
        if (!route->raw_admission_gate) route->raw_admission_gate = std::make_unique<nav2_util::DirectObservationGate>(
          nav2_util::DirectSourceContract{raw.metadata.source_id,raw.metadata.epoch,raw.metadata.frame_id,
            body->frame.raw_projection_expiry_budget_ns});
        const auto & pinned = route->raw_admission_gate->contract();
        if (body->frame.raw_projection_expiry_budget_ns != pinned.maximum_age_ns)
          throw std::invalid_argument("raw_projection_original_budget_changed");
        const auto result = route->raw_admission_gate->admitResult(raw.metadata,ros,steady);
        impl_->observe_admission(raw.metadata,result,"raw_projection.ingress");
        if (result.accepted() && impl_->activated.load()) {
          CloudDelivery delivery{raw,steady};
          for(auto& sink:route->raw_local)impl_->replace_raw(*sink,delivery,retired_raw);
          for(auto& sink:route->raw_global)impl_->replace_raw(*sink,delivery,retired_raw);
        }
      }
      // Incomplete/raw-only and empty filtered frames never enter marking or
      // Collision Monitor and cannot refresh a previously retained obstacle.
      if (!body->frame.body_filter.complete || !body->frame.body_filter.retained_count) return true;
      const auto frame = impl_->cloud(body);
      if (!route->admission_gate) {
        const auto & id = body->frame.decoded.identity();
        route->admission_gate = std::make_unique<nav2_util::DirectObservationGate>(
          nav2_util::DirectSourceContract{id.source_id,id.epoch,frame.metadata.frame_id,id.expiry_budget_ns});
      }
      const auto source_budget=body->frame.decoded.identity().expiry_budget_ns;
      if(source_budget!=route->admission_gate->contract().maximum_age_ns ||
          (impl_->options.classified_obstacles_enabled && source_budget!=impl_->options.camera_maximum_age_ns))
        throw std::invalid_argument("classified_body_original_source_budget_changed");
      const auto result = route->admission_gate->admitResult(frame.metadata,ros,steady);
      impl_->observe_admission(frame.metadata,result,"body.ingress");
      if (!result.accepted()) return true;
      if (!impl_->activated.load()) return true;
      // Diagnostic retention only: use the very classified owner/alias cloud
      // handed to the normal consumers. No classification, TF or query repeats.
      if(!impl_->ground_snapshot_directory.empty())try {
        const auto role=body->input->role=="belly"?0u:1u;
        std::lock_guard<std::mutex> timing(impl_->timing_mutex);
        for(const auto slot:{role,role+2u})
          (void)impl_->rgbd_projection_capture[slot].offer(frame.metadata,frame.cloud,
            body->frame.classified_obstacles,steady,body->frame.body_model_diagnostic,
            std::shared_ptr<const std::vector<std::size_t>>(body,&body->frame.near_body_indices));
      }catch(...){} // Never turn optional evidence allocation into a source fault.
      const auto filtered = detail::exclude_near_body_endpoints(frame, body->frame.near_body_indices);
      const CloudDelivery map_delivery{filtered,steady};
      const CloudDelivery collision_delivery{body->frame.apply_near_body_mask_to_collision ? filtered : frame,steady};
      const bool local = route->local->admit(map_delivery);
      const bool global = !route->global || route->global->admit(map_delivery);
      const bool collision = route->collision->admit(collision_delivery);
      if (!local || !global || !collision) throw std::runtime_error("nav2_body_lane_overflow_or_closed");
      return true;
    } catch (...) {impl_->fault(exception_reason(std::current_exception())); return false;}
  }
  impl_->fault("nav2_unbound_actual_camera_body"); return false;
}
bool Nav2Assembly::consume_airy(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud,
  const nav2_util::DirectObservationMetadata & metadata,
  std::shared_ptr<const std::vector<std::array<float,3>>> ray_origins) {
  if (!impl_->running()) return false;
  // Retire a replaced immutable owner after releasing the ingress/gate lock.
  std::unique_ptr<AiryInput> retired;
  const nav2_util::DirectCloud frame{cloud,metadata,nav2_util::DirectCloudKind::RetainedIndependentSource};
  std::lock_guard<std::mutex> lock(impl_->airy_admission_mutex);
  if (metadata.raw_source_ns <= 0 || !nav2_util::directCloudMatchesMetadata(frame)) {
    impl_->fault("invalid_normalized_airy_layout_or_raw_clock"); return false;
  }
  const auto result = impl_->airy_gate->admitResult(metadata,impl_->host->now().nanoseconds(),nav2_util::directSteadyNowNs());
  try {impl_->observe_admission(metadata,result,"airy.ingress");}
  catch (...) {impl_->fault(exception_reason(std::current_exception())); return false;}
  if (!result.accepted()) return true;
  // The sole driver still feeds LIO/NDT in NavigationRuntime. In this mode
  // no obstacle filter/classifier/consumer may receive or renew Airy evidence.
  if (!obstacle_uses_airy(impl_->options.obstacle_source_mode) || !impl_->activated.load()) return true;
  auto input = std::make_shared<sunhorizon_rgbd_adapter::SelfExclusionDirectInput>();
  input->cloud = std::move(cloud);
  input->ray_origins=std::move(ray_origins);
  input->stamp = {metadata.raw_source_ns,metadata.source_ros_ns,metadata.first_receipt_ros_ns,
    metadata.first_receipt_steady_ns,metadata.sequence,metadata.source_id,metadata.epoch,
    metadata.normalization_start_steady_ns,metadata.normalization_end_steady_ns,metadata.normalization_thread_cpu_ns};
  input->queue_admitted_steady_ns = nav2_util::directSteadyNowNs();
  auto pending=std::make_unique<AiryInput>(std::move(input));
  const auto* offered=pending.get();
  retired=impl_->airy_compute->replace(std::move(pending));
  const auto stats=impl_->airy_compute->stats();
  if(retired.get()==offered || stats.closed || stats.errors){
    impl_->fault("airy_filter_lane_closed");return false;
  }
  return true;
}
bool Nav2Assembly::consume_localization(State state) {
  if (!impl_->running()) return false;
  if (!impl_->canonical->admit(CanonicalInput{std::move(state)})) {impl_->fault("nav2_canonical_lane_overflow_or_closed"); return false;}
  return true;
}
bool Nav2Assembly::consume_health(Health health) {
  if (!impl_->running()) return false;
  if (!impl_->canonical->admit(CanonicalInput{std::move(health)})) {impl_->fault("nav2_health_lane_overflow_or_closed"); return false;}
  return true;
}
void Nav2Assembly::revoke(const std::string & reason) noexcept {impl_->fault(reason);}
void Nav2Assembly::stop() {impl_->stop();}
bool Nav2Assembly::initialized() const {return impl_->activated.load();}
bool Nav2Assembly::motion_held() const {return impl_->canonical_motion_hold.load();}
bool Nav2Assembly::active() const {return initialized() && !motion_held();}
bool Nav2Assembly::failed() const {return impl_->terminal.load();}
std::shared_ptr<const perception::CollisionHeightBinding> Nav2Assembly::collision_height_binding()const{
  if(!impl_->options.spatial_query.height_limited())return {};
  if(!impl_->collision_height_verified.load() || !impl_->accepting.load() || impl_->terminal.load() || impl_->stopping.load())
    throw std::logic_error("collision_height_actual_nav2_configuration_not_ready");
  return std::shared_ptr<const perception::CollisionHeightBinding>(new perception::CollisionHeightBinding(impl_->options.spatial_query));
}
std::string Nav2Assembly::failure_reason() const {std::lock_guard<std::mutex> lock(impl_->reason_mutex); return impl_->reason;}
std::shared_ptr<Nav2Assembly> make_nav2_assembly(Nav2AssemblyOptions options, std::shared_ptr<tf2_ros::Buffer> buffer) {
  return std::shared_ptr<Nav2Assembly>(new Nav2Assembly(std::make_unique<Nav2Assembly::Impl>(std::move(options),std::move(buffer))));
}
}  // namespace sunhorizon_navigation_runtime
