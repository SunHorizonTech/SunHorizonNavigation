#include "sunhorizon_navigation_runtime/navigation_runtime.hpp"
#include "sunhorizon_navigation_runtime/nav2_assembly.hpp"
#include "sunhorizon_perception_runtime/runtime_config.hpp"
#include "sunhorizon_navigation_runtime/bounded_lane.hpp"
#include "sunhorizon_navigation_runtime/detail/peripheral_startup.hpp"
#include "sunhorizon_navigation_runtime/detail/joint_context.hpp"
#include <rclcpp/contexts/default_context.hpp>
#include "sunhorizon_camera_runtime/latest_frame_worker.hpp"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/compute_path_through_poses.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <future>
#include <mutex>
#include <stdexcept>
#include <string>

namespace runtime = sunhorizon_navigation_runtime;
namespace {
// Only the external main thread stops and joins owners. Component fault
// callbacks immediately revoke Nav2 admission and wake this lifecycle owner.
struct FaultLatch {
  std::atomic<bool> failed{false};
  std::mutex mutex;
  std::condition_variable changed;
  std::string reason;
  void latch(const std::string & message) noexcept {
    try {
      {std::lock_guard<std::mutex> lock(mutex);
        if (failed.exchange(true)) return;
        reason = message.substr(0, 512);}
      changed.notify_all();
    } catch (...) {failed.store(true); changed.notify_all();}
  }
  std::string message() {
    std::lock_guard<std::mutex> lock(mutex); return reason;
  }
};
// Independent BT/outer consumers still use global TF. This observer
// publishes the exact validated cache writes; it never calculates another FK
// model. Static seeds cannot be replaced by dynamic updates or another seed.
class TransformCompatibilityObserver {
  using Edges = std::vector<geometry_msgs::msg::TransformStamped>;
 public:
  TransformCompatibilityObserver(rclcpp::Node::SharedPtr host,
    std::function<bool()> permitted, std::function<void(const std::string &)> fault)
    : fixed_publisher_(host), dynamic_publisher_(host), fault_(std::move(fault)) {
    auto failure = [this](std::exception_ptr error) {
      try {if (error) std::rethrow_exception(error);}
      catch (const std::exception & e) {fault_(std::string("tf_compatibility:") + e.what());}
      catch (...) {fault_("unknown_tf_compatibility_failure");}
    };
    fixed_ = std::make_unique<runtime::detail::BoundedLane<Edges>>(4,
      [this, permitted](Edges edges) {if (!stopping_.load() && permitted()) fixed_publisher_.sendTransform(edges);}, failure);
    dynamic_ = std::make_unique<sunhorizon_camera_runtime::LatestFrameWorker<Edges>>(
      [this](Edges & edges) {dynamic_publisher_.sendTransform(edges);},
      [this, permitted] {return !stopping_.load() && permitted();}, failure);
  }
  ~TransformCompatibilityObserver() {stop();}
  void admit(const Edges & edges, bool fixed) {
    if (stopping_.load() || edges.empty()) return;
    if (edges.size() > 256) {fault_("tf_compatibility_edge_batch_exceeds_bound"); return;}
    if (fixed) {
      if (!fixed_->admit(edges)) fault_("tf_compatibility_static_queue_overflow");
    } else {
      dynamic_->replace(std::make_unique<Edges>(edges));
    }
  }
  void stop() {
    if (stopping_.exchange(true)) return;
    if (fixed_) fixed_->request_stop();
    if (dynamic_) dynamic_->request_stop();
    if (fixed_) fixed_->join();
    if (dynamic_) dynamic_->join();
  }
 private:
  tf2_ros::StaticTransformBroadcaster fixed_publisher_;
  tf2_ros::TransformBroadcaster dynamic_publisher_;
  std::function<void(const std::string &)> fault_;
  std::atomic<bool> stopping_{false};
  std::unique_ptr<runtime::detail::BoundedLane<Edges>> fixed_;
  std::unique_ptr<sunhorizon_camera_runtime::LatestFrameWorker<Edges>> dynamic_;
};
rcl_interfaces::msg::ParameterDescriptor read_only() {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  return descriptor;
}
std::string required_text(rclcpp::Node & node, const std::string & name) {
  auto text = node.declare_parameter<std::string>(name, "", read_only());
  if (text.empty()) throw std::invalid_argument("missing unified runtime parameter: " + name);
  return text;
}
int64_t required_age(rclcpp::Node & node, const std::string & name, double maximum) {
  const auto seconds = node.declare_parameter<double>(name, 0., read_only());
  if (!std::isfinite(seconds) || seconds <= 0. || seconds > maximum)
    throw std::invalid_argument(name + " must retain an explicit original age within (0," +
      std::to_string(maximum) + "] seconds");
  return rclcpp::Duration::from_seconds(seconds).nanoseconds();
}
runtime::Nav2BodySourceBinding body_binding(rclcpp::Node & node, const std::string & role) {
  runtime::Nav2BodySourceBinding result;
  result.camera_role = role;
  // Existing plugin/source names identify which already configured source to
  // bind. The binding API rejects missing, DDS, clearing or unaccepted sources.
  const auto key = "nav2." + role + ".";
  result.local_layer = required_text(node, key + "local_layer");
  result.local_source = required_text(node, key + "local_source");
  result.global_layer = node.declare_parameter<std::string>(key + "global_layer", "", read_only());
  result.global_source = node.declare_parameter<std::string>(key + "global_source", "", read_only());
  result.collision_source = required_text(node, key + "collision_source");
  result.local_clearing_sources = node.declare_parameter<std::vector<std::string>>(
    key + "local_clearing_sources", {}, read_only());
  result.global_clearing_sources = node.declare_parameter<std::vector<std::string>>(
    key + "global_clearing_sources", {}, read_only());
  return result;
}
}  // namespace

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  FaultLatch fault;
  std::shared_ptr<runtime::NavigationRuntime> navigation;
  std::shared_ptr<runtime::Nav2Assembly> assembly;
  // Shared-pointer atomic operations let early constructor fault callbacks see
  // either no consumer or the fully constructed owner, never a partial object.
  std::shared_ptr<runtime::Nav2Assembly> downstream;
  std::shared_ptr<sunhorizon_localization_manager::LocalizationDirectRuntime> canonical_owner;
  rclcpp::Node::SharedPtr entry;
  rclcpp::Context::SharedPtr joint_context;
  std::unique_ptr<TransformCompatibilityObserver> tf_observer;
  int result = 0;
  try {
    entry = std::make_shared<rclcpp::Node>("sunhorizon_navigation_runtime_entry");
    if (entry->get_parameter("use_sim_time").as_bool())
      throw std::invalid_argument("unified navigation requires accepted realtime clocks");
    runtime::NavigationRuntimeOptions options;
    const auto default_context=rclcpp::contexts::get_global_default_context();
    joint_context=runtime::detail::make_joint_context(default_context,argc,argv);
    if(joint_context==default_context)
      throw std::logic_error("production joint context must be distinct");
    options.joint_node.context(joint_context);
    options.defer_spatial_evidence_until_startup = true;
    options.airy_config_path = required_text(*entry, "airy_config_path");
    options.kinematics_tf_mode = required_text(*entry, "kinematics_tf_mode");
    if (options.airy_config_path.front() != '/')
      throw std::invalid_argument("airy_config_path must identify an absolute accepted SDK YAML");
    // Coverage inputs share the actual owners. Its unchanged typed DDS outputs
    // remain the independent final supervisor qualification boundary.
    options.debug_publish_body_clouds = false;
    options.publish_ros_canonical_compatibility = true;
    options.debug_publish_raw_sensors = entry->declare_parameter<bool>("debug_publish_raw_sensors", false, read_only());
    options.direct_tf_buffer = std::make_shared<tf2_ros::Buffer>(entry->get_clock(), tf2::durationFromSec(2.));

    options.spatial_query=sunhorizon_perception_runtime::load_spatial_query_policy(*entry);
    runtime::Nav2AssemblyOptions nav2_options;
    options.obstacle_source_mode=entry->declare_parameter<std::string>(
      "obstacle_source_mode","lidar-rgbd-v1",read_only());
    nav2_options.obstacle_source_mode=options.obstacle_source_mode;
    const bool airy_obstacles=runtime::obstacle_uses_airy(options.obstacle_source_mode);
    nav2_options.classified_obstacles_enabled=entry->declare_parameter<bool>(
      "classified_obstacles_enabled",false,read_only());
    nav2_options.spatial_query=options.spatial_query;
    nav2_options.camera_maximum_age_ns=sunhorizon_obstacle_coverage::camera_freshness_age_ns(
      entry->declare_parameter<std::string>("camera_freshness_profile",
        sunhorizon_obstacle_coverage::kStrictCameraFreshness,read_only()));
    nav2_options.canonical_odometry.epoch = required_text(*entry, "nav2.canonical_epoch");
    nav2_options.canonical_odometry.maximum_age_ns = required_age(
      *entry, "nav2.canonical_odometry_maximum_age_sec", nav2_options.canonical_age_limit_ns() / 1e9);
    nav2_options.localization_health_maximum_age_ns = required_age(*entry, "nav2.localization_health_maximum_age_sec", 10.);
    nav2_options.body_sources = {body_binding(*entry, "belly"), body_binding(*entry, "chest")};
    nav2_options.airy_input_contract.epoch = required_text(*entry, "nav2.airy_epoch");
    nav2_options.airy_input_contract.maximum_age_ns = required_age(*entry, "nav2.airy_maximum_age_sec", 10.);
    nav2_options.airy_source.local_layer = entry->declare_parameter<std::string>("nav2.airy.local_layer", "", read_only());
    nav2_options.airy_source.local_sources = entry->declare_parameter<std::vector<std::string>>(
      "nav2.airy.local_sources", {}, read_only());
    if (airy_obstacles && nav2_options.airy_source.local_sources.empty())
      throw std::invalid_argument("nav2.airy.local_sources must name the retained original Airy sources");
    nav2_options.airy_source.global_layer = entry->declare_parameter<std::string>("nav2.airy.global_layer", "", read_only());
    nav2_options.airy_source.global_sources = entry->declare_parameter<std::vector<std::string>>(
      "nav2.airy.global_sources", {}, read_only());
    nav2_options.airy_source.collision_source = entry->declare_parameter<std::string>("nav2.airy.collision_source", "", read_only());
    nav2_options.publish_filtered_airy_compatibility = false;
    const auto airy_epoch = nav2_options.airy_input_contract.epoch;
    nav2_options.controller_node.arguments({"--ros-args", "-r",
      "cmd_vel:=/sunhorizon/navigation/cmd_vel_nav2_raw"});
    nav2_options.airy_filter_node.arguments({"--ros-args", "-r",
      "__node:=sunhorizon_attended_rig_filter"});
    nav2_options.fault = [&fault, &canonical_owner](const auto & reason) {
      if (auto owner = std::atomic_load(&canonical_owner)) owner->revoke("nav2:" + reason);
      fault.latch("nav2:" + reason);
    };
    const double startup_timeout = entry->declare_parameter<double>("startup_timeout_sec", 30., read_only());
    if (!std::isfinite(startup_timeout) || startup_timeout <= 0. || startup_timeout > 120.)
      throw std::invalid_argument("startup_timeout_sec must be within (0,120]");
    const auto peripheral_manager = required_text(*entry, "nav2.peripheral_lifecycle_manager");
    if (peripheral_manager != "/lifecycle_manager_navigation")
      throw std::invalid_argument("unified_runtime_requires_original_peripheral_lifecycle_manager");
    auto peripherals = entry->create_client<nav2_msgs::srv::ManageLifecycleNodes>(peripheral_manager + "/manage_nodes");
    auto maps = entry->create_client<std_srvs::srv::Trigger>("/lifecycle_manager_map_prerequisites/is_active");
    auto plan_pose = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(entry,"compute_path_to_pose");
    auto plan_poses = rclcpp_action::create_client<nav2_msgs::action::ComputePathThroughPoses>(entry,"compute_path_through_poses");
    auto follow = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(entry,"follow_path");

    auto revoke = [&fault, &downstream, &canonical_owner](const std::string & reason) {
      if (auto owner = std::atomic_load(&canonical_owner)) owner->revoke(reason);
      if (auto owner = std::atomic_load(&downstream)) owner->revoke(reason);
      fault.latch(reason);
    };
    runtime::NavigationRuntimeSinks sinks;
    sinks.fault = revoke;
    tf_observer = std::make_unique<TransformCompatibilityObserver>(entry,
      [&fault] {return !fault.failed.load();}, revoke);
    sinks.tf_compatibility = [&tf_observer](const auto & edges, bool fixed) {tf_observer->admit(edges, fixed);};
    sinks.perception_output = [&downstream, revoke](auto output) {
      auto owner = std::atomic_load(&downstream);
      if (!owner || !owner->consume_body(std::move(output))) revoke("body_consumer_admission_rejected");
    };
    sinks.airy_frame = [&downstream, revoke, airy_epoch](auto output, const auto & stamp) {
      auto owner = std::atomic_load(&downstream);
      if (!output) {revoke("missing_normalized_airy_owner"); return;}
      const nav2_util::DirectObservationMetadata metadata{stamp.source_identity, airy_epoch,
        output->header.frame_id, stamp.sequence, stamp.source_ns, stamp.first_receipt_ros_ns,
        stamp.first_receipt_steady_ns, stamp.raw_source_ns, stamp.normalization_start_steady_ns,
        stamp.normalization_end_steady_ns, stamp.normalization_thread_cpu_ns};
      if (!owner || !owner->consume_airy(std::move(output), metadata,stamp.ray_origins)) revoke("airy_consumer_admission_rejected");
    };
    sinks.canonical_localization = [&downstream, revoke](auto state) {
      auto owner = std::atomic_load(&downstream);
      if (!owner || !owner->consume_localization(std::move(state))) revoke("canonical_consumer_admission_rejected");
    };
    sinks.localization_health = [&downstream, revoke](auto health) {
      auto owner = std::atomic_load(&downstream);
      if (!owner || !owner->consume_health(std::move(health))) revoke("localization_health_admission_rejected");
    };
    // No synthetic region consumer or ZUPT ACK: perception's independent
    // actual coverage owner receives region/RGB-D/spatial objects directly.
    // An explicitly enabled vendor ZUPT requires a real qualified direct
    // request/ACK owner; NavigationRuntime::start rejects its absence.
    navigation = runtime::make_navigation_runtime(std::move(options), std::move(sinks));
    nav2_options.ground_reference_owner = navigation->ground_reference_owner();
    nav2_options.registration_ground_geometry = navigation->registration_ground_geometry_provider();
    if (nav2_options.ground_reference_owner)
      nav2_options.airy_ground_calibration_id = navigation->airy_ground_calibration_id();
    nav2_options.filtered_airy = [owner=navigation](auto output){owner->admit_filtered_airy(std::move(output));};
    nav2_options.canonical_owner = navigation->localization_manager();
    std::atomic_store(&canonical_owner, nav2_options.canonical_owner);
    nav2_options.canonical_odometry.source_id = nav2_options.canonical_owner->node()->get_fully_qualified_name();
    nav2_options.canonical_odometry.frame_id = nav2_options.canonical_owner->node()->get_parameter("odom_frame").as_string();
    const auto normalizer = navigation->airy_adapter_node();
    nav2_options.airy_input_contract.source_id = normalizer->get_fully_qualified_name() + std::string("#points");
    nav2_options.airy_input_contract.frame_id = normalizer->get_parameter("output_points_frame").as_string();
    if(normalizer->get_parameter("timestamp_mode").as_string()!="scan_relative" ||
       normalizer->get_parameter("timestamp_field").as_string()!="timestamp" ||
       normalizer->get_parameter("timestamp_field_datatype").as_string()!="float32")
      throw std::invalid_argument("shared_lidar_cad_requires_actual_compact_relative_time_contract");
    nav2_options.airy_lidar_surface_radius_bound_m=navigation->lidar_surface_radius_bound(
      nav2_options.airy_input_contract.frame_id);
    nav2_options.airy_lidar_model_for_scan=[owner=navigation](std::size_t maximum_entries){
      return owner->lidar_model_provider_for_scan(maximum_entries);
    };
    nav2_options.airy_lidar_model=[owner=navigation](const auto& frame,int64_t source_ns){
      return owner->lidar_self_model(frame,source_ns);
    };
    nav2_options.airy_lidar_surface_tolerance_m=entry->declare_parameter<double>(
      "nav2.airy.cad_surface_tolerance_m",navigation->self_surface_diagnostic_tolerance_m(),read_only());
    if(nav2_options.airy_lidar_surface_tolerance_m!=navigation->self_surface_diagnostic_tolerance_m())
      throw std::invalid_argument("lidar_and_rgbd_self_surface_tolerance_mismatch");
    nav2_options.airy_lidar_remove_self_surfaces=entry->declare_parameter<bool>(
      "nav2.airy.cad_remove_self_surfaces",false,read_only());
    assembly = runtime::make_nav2_assembly(std::move(nav2_options), navigation->direct_tf_buffer());
    std::atomic_store(&downstream, assembly);
    if (fault.failed.load()) throw std::runtime_error(fault.message());
    navigation->add_control_node(entry);
    for (const auto & node : assembly->nodes()) navigation->add_control_node(node);
    assembly->start(); // Configure original plugins and bind every source first.
    if(auto binding=assembly->collision_height_binding())
      navigation->bind_collision_height_responsibility(*binding);
    navigation->start(); // Start executors, then the sole camera/Airy acquisition.
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(startup_timeout));
    const auto deadline_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(deadline.time_since_epoch()).count();
    runtime::detail::PeripheralStartup startup(deadline);
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedFuture startup_response;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture map_response;
    bool maps_active = false, peripherals_complete = false, healthy_once = false;
    bool held_startup_expired_reported = false;
    while (rclcpp::ok() && !fault.failed.load()) {
      if (navigation->failed()) revoke("navigation:" + navigation->failure_reason());
      if (assembly->failed()) fault.latch("nav2:" + assembly->failure_reason());
      if (fault.failed.load()) break;
      // A localization outage after component initialization suspends control,
      // but must not terminate the sensor/localization owners needed to recover.
      // The held assembly cannot accept a goal until an explicit new session.
      const auto startup_poll_now = std::chrono::steady_clock::now();
      if (!healthy_once && startup.should_poll(assembly->motion_held(),startup_poll_now)) {
        try {
          // Maps/filter owners start independently, so waiting here cannot
          // prevent the core costmaps from receiving their prerequisite data.
          if (!maps_active && map_response.valid() && map_response.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            maps_active = map_response.get()->success; map_response = {};
          }
          if (!maps_active && !map_response.valid() && maps->service_is_ready())
            map_response = maps->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>()).share();
          std::optional<bool> response;
          if (startup_response.valid() && startup_response.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            response = startup_response.get()->success;
            startup_response = {};
          }
          const bool ready = maps_active && peripherals->service_is_ready() &&
            plan_pose->action_server_is_ready() && plan_poses->action_server_is_ready() && follow->action_server_is_ready();
          const auto step = startup.advance(assembly->initialized(),ready,response,startup_poll_now);
          if (step == runtime::detail::PeripheralStartup::Step::Request) {
            if (fault.failed.load()) continue;
            auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
            request->command = nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP;
            startup_response = peripherals->async_send_request(request).share();
          }
          if (step == runtime::detail::PeripheralStartup::Step::Complete && !peripherals_complete) {
            // Actual one-shot service success precedes ORIGINAL current RGB-D
            // qualification. No cached/in-flight source may become first proof.
            navigation->release_spatial_startup(deadline_ns);
            peripherals_complete = true;
          }
          if (peripherals_complete && !assembly->initialized())
            throw std::runtime_error("nav2_core_lost_waiting_for_first_spatial_evidence");
        } catch (const std::exception & error) {revoke(error.what());}
      }
      // Observe actual publication even if this poll arrives after the held
      // startup deadline. This grants no new publication or motion authority.
      const auto spatial = navigation->spatial_startup_status();
      healthy_once = peripherals_complete && spatial.first_published_steady_ns > 0 &&
        spatial.first_published_steady_ns < deadline_ns;
      if (!healthy_once && !assembly->motion_held() && std::chrono::steady_clock::now() >= deadline)
        revoke("startup_deadline_waiting_for_original_localization_tf_map_or_sensor_gates");
      if (!healthy_once && assembly->motion_held() && !held_startup_expired_reported &&
          std::chrono::steady_clock::now() >= deadline) {
        held_startup_expired_reported = true;
        RCLCPP_WARN(entry->get_logger(),
          "spatial_startup_original_deadline_expired_while_motion_held; acquisition retained; explicit new runtime session required");
      }
      std::unique_lock<std::mutex> lock(fault.mutex);
      fault.changed.wait_for(lock, std::chrono::milliseconds(20), [&fault] {return fault.failed.load();});
    }
    if (fault.failed.load()) result = 2;
  } catch (const std::exception & error) {
    fault.latch(error.what()); result = 2;
  } catch (...) {
    fault.latch("unknown_navigation_startup_failure"); result = 2;
  }
  const auto reason = fault.failed.load() ? fault.message() : "navigation_shutdown";
  // Revoke before joining sources. This preserves the original controller ->
  // Collision Monitor typed zero path and the independent final supervisor.
  if (assembly) assembly->revoke(reason);
  if (navigation) {
    try {navigation->localization_manager()->revoke(reason);} catch (...) {}
    try {navigation->stop();} catch (const std::exception & error) {
      std::fprintf(stderr, "navigation stop failed: %s\n", error.what()); result = 2;
    }
  }
  if (assembly) {
    try {assembly->stop();} catch (const std::exception & error) {
      std::fprintf(stderr, "Nav2 stop failed: %s\n", error.what()); result = 2;
    }
  }
  if (tf_observer) tf_observer->stop();
  std::atomic_store(&downstream, std::shared_ptr<runtime::Nav2Assembly>{});
  navigation.reset(); assembly.reset(); tf_observer.reset(); entry.reset();
  std::atomic_store(&canonical_owner, std::shared_ptr<sunhorizon_localization_manager::LocalizationDirectRuntime>{});
  if (fault.failed.load()) std::fprintf(stderr, "navigation runtime refused/stopped: %s\n", reason.c_str());
  // JointIngress and its bounded proof egress have been destroyed above. The
  // secondary context must outlive those publishers, including startup failures.
  if(joint_context && joint_context->is_valid()) {
    try {joint_context->shutdown(reason);}
    catch(const std::exception& error) {
      std::fprintf(stderr,"joint context shutdown failed: %s\n",error.what());result=2;
    }
  }
  joint_context.reset();
  rclcpp::shutdown();
  return result;
}
