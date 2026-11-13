#pragma once
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_ros/buffer.h>
#include "sunhorizon_camera_runtime/direct_frame.hpp"
#include "sunhorizon_rgbd_adapter/pointcloud_self_exclusion_direct.hpp"
#include "sunhorizon_perception_runtime/joint_kinematics.hpp"
#include "sunhorizon_perception_runtime/perception_engine.hpp"
#include "sunhorizon_perception_runtime/spatial_startup_gate.hpp"
#include "sunhorizon_perception_runtime/thread_scheduling.hpp"
#include "sunhorizon_perception_runtime/compute_thread_control.hpp"
#include "sunhorizon_perception_runtime/collision_height_query.hpp"
#include "sunhorizon_perception_runtime/registration_ground_geometry.hpp"
#include "sunhorizon_navigation_interfaces/msg/spatial_coverage_evidence.hpp"
#include "sunhorizon_navigation_interfaces/msg/rgbd_status.hpp"
#include "sunhorizon_obstacle_coverage/spatial_delivery_result.hpp"

namespace sunhorizon_perception_runtime {
// Immutable result ownership for downstream in-process consumers. The cloud
// egress is an observer of this object, never its authoritative storage.
struct RegionDiagnosticOutput {
  sunhorizon_camera_runtime::CameraFrame::ConstSharedPtr input;
  diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr diagnostics;
};
using SpatialOutputSink=std::function<sunhorizon_obstacle_coverage::SpatialDeliveryResult(
  sunhorizon_navigation_interfaces::msg::SpatialCoverageEvidence::ConstSharedPtr)>;
using HealthOutputSink=std::function<void(sunhorizon_navigation_interfaces::msg::RgbdStatus::ConstSharedPtr)>;
using RegionOutputSink=std::function<void(std::shared_ptr<const RegionDiagnosticOutput>)>;
struct PerceptionOutput {
  sunhorizon_camera_runtime::CameraFrame::ConstSharedPtr input;
  FrameResult frame;
  std::shared_ptr<const RegionDiagnosticOutput> region;
  int64_t compute_start_steady_ns{0}, compute_end_steady_ns{0};
  int64_t fk_wait_ns{0};
  JointKinematicsEvidence joints;
  int64_t preparation_start_steady_ns{0}, preparation_end_steady_ns{0};
  detail::ThreadSchedulingDelta preparation_scheduling;
  detail::ComputeThreadState preparation_thread_state;
  detail::ThreadSchedulingDelta engine_scheduling;
  detail::ComputeThreadState engine_thread_state,geometry_helper_thread_state;
};
using PerceptionOutputSink=std::function<void(std::shared_ptr<const PerceptionOutput>)>;
class PerceptionRuntime final : public rclcpp::Node {
 public:
  explicit PerceptionRuntime(const rclcpp::NodeOptions& = rclcpp::NodeOptions(),
      bool defer_ground_reference=false);
  // Unified ownership selects direct TF before constructing any listener
  // thread. The existing two-argument ABI keeps standalone external TF enabled.
  PerceptionRuntime(const rclcpp::NodeOptions&,bool defer_ground_reference,
      bool external_tf_listener);
  ~PerceptionRuntime() override;
  std::shared_ptr<JointKinematics> kinematics() const;
  // Shared immutable CAD storage and original point-time FK; no second model,
  // joint owner or private TF bridge. Unavailable history returns no snapshot.
  std::shared_ptr<const dg::PreparedModel> lidar_model(const std::string& source_frame,int64_t source_ns) const;
  sunhorizon_rgbd_adapter::LidarModelProvider lidar_model_provider_for_scan(std::size_t maximum_entries) const;
  double lidar_surface_radius_bound(const std::string& source_frame) const;
  double self_surface_diagnostic_tolerance_m() const;
  ground_relative::GroundReferenceOwner::ConstSharedPtr ground_reference_owner() const;
  // Unified construction only, before acquisition. Identity comes from the
  // actual NDT map loader; standalone timed references keep their old path.
  void configure_registration_ground(const std::string& actual_ndt_map_sha256);
  void set_registration_ground_geometry_provider(RegistrationGroundGeometryProvider);
  RegistrationGroundGeometryProvider registration_ground_geometry_provider() const;
  // Bounded admission only, safe on independent camera SDK conversion workers.
  // Must be called before JointIngress/camera starts. Disables external TF
  // listener and mirrors validated joint/static edges into this direct cache.
  // attended_direct preserves its original fixed waist and only mirrors legs.
  // Optional compatibility sink only enqueues immutable validated edges into
  // independent bounded static/dynamic egress observers; never publish here.
  void configure_direct_transforms(std::shared_ptr<tf2_ros::Buffer>,
      JointKinematics::TransformSink compatibility = {});
  void admit(sunhorizon_camera_runtime::CameraFrame::ConstSharedPtr);
  void fault(const std::string&);
  bool failed() const;
  // Snapshot the latched first failure without waiting for health publication.
  // Remains available after stop; empty until the first fault.
  std::string fault_reason() const;
  // Unified owner only: defer before the first SDK admission, release once after
  // actual peripheral STARTUP success. Current RGB-D health then opens the gate;
  // only subsequently acquired source/first-receipt clocks can produce evidence.
  void defer_spatial_startup();
  void release_spatial_startup(int64_t original_deadline_steady_ns);
  SpatialStartupStatus spatial_startup_status() const;
  SpatialQueryPolicy spatial_query_policy() const;
  // External owner only, outside worker/sink callbacks. Stop camera first;
  // quiesce/remove control-executor callbacks before releasing this node.
  void stop();
  // Configure before acquisition; sinks must be thread-safe bounded handoffs.
  // Compute synchronously delivers joined spatial evidence after original gates;
  // the control timer may deliver terminal revocation, serialized with compute.
  // DDS observation runs separately and cannot deliver or renew direct evidence.
  // Spatial returns typed acceptance/unavailability/internal failure; a motion
  // stop does not revoke this producer. Health keeps its own worker.
  void set_spatial_output_sink(SpatialOutputSink);
  void set_health_output_sink(HealthOutputSink);
  rclcpp::Publisher<sunhorizon_navigation_interfaces::msg::SpatialCoverageEvidence>::SharedPtr spatial_publisher() const;
  void set_region_output_sink(RegionOutputSink);
  // Configure before acquisition. Calls can overlap across both terminal raw
  // owners and both body role owners; sink must be a thread-safe bounded enqueue.
  // Each immutable body result is handed off once as soon as its own gates pass,
  // before coverage joins the roles. Raw-only results have no body health credit.
  void set_output_sink(PerceptionOutputSink);
 private:
  void bind_collision_height_responsibility(const CollisionHeightBinding&);
  friend class sunhorizon_navigation_runtime::NavigationRuntime;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
