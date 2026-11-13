#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sunhorizon_navigation_interfaces/msg/localization_status.hpp>
#include <sunhorizon_3d_localizer/ndt_direct_runtime.hpp>
#include <sunhorizon_navigation_core/body_velocity_evidence.hpp>

namespace sunhorizon_localization_manager {
struct LocalizationInputStamp {
  int64_t source_ns{0}, first_receipt_ros_ns{0}, first_receipt_steady_ns{0};
  uint64_t sequence{0};
  std::string identity;
  std::optional<sunhorizon_navigation_core::BodyVelocityEvidence> body_velocity{};
};
struct CanonicalLocalizationState {
  nav_msgs::msg::Odometry::ConstSharedPtr odometry;
  geometry_msgs::msg::TransformStamped map_to_odom, odom_to_base_footprint;
  std::optional<geometry_msgs::msg::TransformStamped> base_footprint_to_base_link;
  LocalizationInputStamp tracking, map_pose, backend_status;
  std::vector<LocalizationInputStamp> alignment_inputs;
  uint64_t sequence{0}, reset_epoch{0};
};
struct CanonicalLocalizationHealth {
  sunhorizon_navigation_interfaces::msg::LocalizationStatus::ConstSharedPtr status;
  // Status heartbeat/egress never renews these source/receipt clocks.
  std::shared_ptr<const CanonicalLocalizationState> last_state;
  int64_t evaluated_ros_ns{0}, evaluated_steady_ns{0};
  uint64_t reset_epoch{0};
};
// Internal registration datum geometry only. A Ready result is not physical
// error qualification and does not publish or change canonical planar TF.
enum class PcdGeometryState {Ready, NotConfigured, Unavailable, WaitingForHistory,
  WaitingForManager, HistoryExpired, AnchorExpired, IdentityMismatch, Fault};
struct PcdFromAirySnapshot {
  using ConstSharedPtr=std::shared_ptr<const PcdFromAirySnapshot>;
  geometry_msgs::msg::TransformStamped pcd_from_airy;
  geometry_msgs::msg::TransformStamped source_from_nominal_base;
  std::shared_ptr<const sunhorizon_3d_localizer::NdtDirectResult> anchor;
  LocalizationInputStamp registration_delivery;
  std::optional<sunhorizon_3d_localizer::NdtDirectStamp> query_scan;
  std::vector<LocalizationInputStamp> anchor_tracking_inputs,query_tracking_inputs;
  uint64_t reset_epoch{0};
  // Absolute original-evidence deadlines under the existing manager limits;
  // downstream frame/binding construction must not renew either clock.
  int64_t geometry_valid_until_ros_ns{0},geometry_valid_until_steady_ns{0};
  // Same object for an entire manager epoch. reset()/revoke() permanently set
  // the old flag, including snapshots/bindings still waiting in other lanes.
  std::shared_ptr<const std::atomic<bool>> reset_revoked;
  bool same_scan{false},anchor_interpolated{false},query_interpolated{false};
  std::string registration_map_path;
  // No inferred zero error, covariance substitution, or qualification bit.
  std::optional<double> translation_error_bound_m,rotation_error_bound_rad;
};
struct PcdGeometryLookup {
  PcdGeometryState state{PcdGeometryState::Unavailable};
  std::string reason;
  PcdFromAirySnapshot::ConstSharedPtr snapshot;
};

struct PcdGeometryBatchLookup {
  PcdGeometryState state{PcdGeometryState::Unavailable};
  std::string reason;
  // Input order; every element shares one original anchor and reset token.
  std::vector<PcdFromAirySnapshot::ConstSharedPtr> snapshots;
  // Captured with the original anchor under the manager lock, including a
  // pending history bracket. Waiting cannot silently cross this reset epoch.
  std::shared_ptr<const std::atomic<bool>> reset_revoked{};
  int64_t reset_barrier_steady_ns{0};
};

// Compact projection of the same batch query. The immutable common evidence
// is sealed once; each original query keeps its exact time and 6D geometry.
// Receipt lineage is still checked per query by the shared implementation, but
// consumers that do not inspect it need not allocate a rich snapshot per point.
struct PcdGeometryBatchContext {
  using ConstSharedPtr=std::shared_ptr<const PcdGeometryBatchContext>;
  std::shared_ptr<const sunhorizon_3d_localizer::NdtDirectResult> anchor;
  geometry_msgs::msg::TransformStamped source_from_nominal_base;
  LocalizationInputStamp registration_delivery;
  std::optional<sunhorizon_3d_localizer::NdtDirectStamp> query_scan;
  std::vector<LocalizationInputStamp> anchor_tracking_inputs;
  uint64_t reset_epoch{0};
  int64_t geometry_valid_until_ros_ns{0},geometry_valid_until_steady_ns{0};
  std::shared_ptr<const std::atomic<bool>> reset_revoked;
  std::string registration_map_path;
};
struct PcdFromAiryGeometry {
  int64_t source_ns{0};
  geometry_msgs::msg::Transform pcd_from_airy;
  bool same_scan{false},anchor_interpolated{false},query_interpolated{false};
};
struct PcdGeometryCompactBatchLookup {
  PcdGeometryState state{PcdGeometryState::Unavailable};
  std::string reason;
  PcdGeometryBatchContext::ConstSharedPtr context;
  std::vector<PcdFromAiryGeometry> geometries;
  // Preserve the original failure/reset lineage, including waiting batches
  // for which no successful common context or geometry can be issued.
  std::shared_ptr<const std::atomic<bool>> reset_revoked{};
  int64_t reset_barrier_steady_ns{0};
};

struct LocalizationDirectSinks {
  // Explicit canonical reset flushes causal velocity history at this barrier.
  std::function<void(int64_t)> tracking_reset;
  // Same JointKinematics owner as canonical adapter; absent in fixed rollback.
  std::function<std::optional<geometry_msgs::msg::Transform>(int64_t,
    const geometry_msgs::msg::Transform&)> source_time_source_from_base;
  // Bounded-mailbox callbacks only; no reentry, recording, DDS or controller
  // work while the manager serializes its existing acceptance/history gates.
  std::function<void(std::shared_ptr<const CanonicalLocalizationState>)> canonical;
  std::function<void(std::shared_ptr<const CanonicalLocalizationHealth>)> status;
  bool publish_ros_compatibility{true};
};
class LocalizationDirectRuntime {
 public:
  virtual ~LocalizationDirectRuntime() = default;
  virtual std::shared_ptr<rclcpp::Node> node() = 0;
  virtual bool receive_odometry(const std::string& backend, nav_msgs::msg::Odometry::ConstSharedPtr,
                               const LocalizationInputStamp&) = 0;
  virtual bool receive_map_pose(const std::string& backend,
      geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr, const LocalizationInputStamp&) = 0;
  virtual bool receive_status(const std::string& backend,
      sunhorizon_navigation_interfaces::msg::LocalizationStatus::ConstSharedPtr,
      const LocalizationInputStamp&) = 0;
  virtual bool receive_tracking(const std::string& backend, nav_msgs::msg::Odometry::ConstSharedPtr,
                               const LocalizationInputStamp&) = 0;
  virtual bool receive_livo_ndt_alignment(nav_msgs::msg::Odometry::ConstSharedPtr,
                                        const LocalizationInputStamp&) = 0;
  // Configure once before ingress from the SAME constructed NDT owner's
  // accepted source_to_base parameters. This is E=T_Airy<-nominalBase, not
  // body_tilt or a lookup through canonical map/odom/base_footprint.
  virtual void configure_pcd_registration_source(
      const geometry_msgs::msg::TransformStamped& source_from_nominal_base,
      const std::string& map_frame,const std::string& registration_map_path,
      const std::string& registration_owner) = 0;
  // Existing manager lane only. Source and result receipts stay frozen.
  virtual bool receive_registration(
      std::shared_ptr<const sunhorizon_3d_localizer::NdtDirectResult>,
      const LocalizationInputStamp& registration_delivery) = 0;
  // Explicit time-only queries serve other sensors. An Airy scan query must
  // retain its full source identity/sequence/time, never silently lose identity.
  virtual PcdGeometryLookup lookup_pcd_from_airy(int64_t source_ns,
      const std::optional<sunhorizon_3d_localizer::NdtDirectStamp>& query_scan = {}) const = 0;
  // The original scan stamp remains its header time while individual group
  // times vary. One bounded history/receipt snapshot is copied under the owner
  // lock; interpolation and transport run outside it with a fixed anchor.
  virtual PcdGeometryBatchLookup lookup_pcd_from_airy_times(const std::vector<int64_t>& source_ns,
      const std::optional<sunhorizon_3d_localizer::NdtDirectStamp>& original_scan = {}) const = 0;
  // Same owner, gates, original times, interpolation and per-query lineage
  // checks as the complete API above; only the returned allocation/layout is
  // compact. No physical error bound or ground qualification is inferred.
  virtual PcdGeometryCompactBatchLookup lookup_pcd_from_airy_geometry_times(
      const std::vector<int64_t>& source_ns,
      const std::optional<sunhorizon_3d_localizer::NdtDirectStamp>& original_scan = {}) const = 0;
  // Same explicit reinitialization as the retained reset service. Input source
  // identities stay pinned across reset; reset does not authorize owner changes.
  virtual void reset() = 0;
  // Fatal owner/source failure: bounded gate update and health-mailbox egress.
  // Safe from canonical/health sink callbacks; this latch survives reset().
  // Only reconstructing the runtime can recover.
  virtual void revoke(const std::string& reason) = 0;
  virtual bool failed() const = 0;
};

// Replaces the canonical manager owner; no candidate/tracking DDS subscriptions.
// The returned node keeps its original status timer and explicit reset service.
std::shared_ptr<LocalizationDirectRuntime> make_localization_direct_runtime(
    const rclcpp::NodeOptions& options, LocalizationDirectSinks sinks);
// Legacy executable and direct factory share the exact same implementation.
std::shared_ptr<rclcpp::Node> make_localization_ros_node(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
}  // namespace sunhorizon_localization_manager
