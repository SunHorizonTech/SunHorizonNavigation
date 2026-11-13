#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <functional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include "sunhorizon_platform_adapter_casbot02/live_joint_state_bridge.hpp"

namespace tf2 { class BufferCore; }

namespace sunhorizon_perception_runtime {

struct JointKinematicsOptions {
  std::string urdf_xml;
  // Runtime loader pins the signed source/mapping and production sensor TF
  // acceptance used to build this in-memory model. Pure FK tests need no owner.
  std::string source_receipt_sha256, mapping_receipt_sha256;
  std::string production_tf_receipt_sha256, legacy_urdf_sha256;
  std::string merged_urdf_sha256;
  std::string tf_mode;
  std::string attended_usage, attended_tf_asset_sha256, attended_tf_receipt_sha256;
  std::vector<geometry_msgs::msg::TransformStamped> attended_static_edges;
  std::vector<std::string> expected_joint_names;
  std::string base_frame{"base_link"};
  // Must match the parsed URDF root; it is never inferred from incoming TF.
  std::string root_frame{"base_link"};
  std::vector<std::string> target_frames;
  int64_t cache_time_ns{2000000000LL};
  std::size_t max_history_samples{512};
  int64_t maximum_age_ns{250000000LL};
  int64_t future_tolerance_ns{20000000LL};
};

enum class JointKinematicsStatus {Ready, FuturePending, Stale, Missing, Fault, QueryExpired};

enum class JointFrameDisposition {Ready, Wait, Unavailable, InternalFault};

// A failed camera-time query does not revoke the independently owned joint
// history. No unavailable result may produce transforms or completion credit.
inline JointFrameDisposition joint_frame_disposition(
    JointKinematicsStatus status, bool source_fresh, bool was_qualified) {
  if (status == JointKinematicsStatus::Fault ||
      (status == JointKinematicsStatus::Missing && was_qualified))
    return JointFrameDisposition::InternalFault;
  if (!source_fresh || status == JointKinematicsStatus::Stale ||
      status == JointKinematicsStatus::QueryExpired)
    return JointFrameDisposition::Unavailable;
  if (status == JointKinematicsStatus::Ready) return JointFrameDisposition::Ready;
  if (status == JointKinematicsStatus::FuturePending || status == JointKinematicsStatus::Missing)
    return JointFrameDisposition::Wait;
  return JointFrameDisposition::InternalFault;
}

struct JointIngressClockPoint {
  int64_t steady_ns{-1}, thread_cpu_ns{-1};
};

// Diagnostic provenance of one accepted raw callback, never a freshness input.
// RMW timestamps are the middleware's original values, not sensor/ROS clock
// substitutes. Nonpositive RMW values and negative clock points are unavailable.
struct JointIngressTrace {
  bool available{false};
  int64_t rmw_source_timestamp_ns{0}, rmw_received_timestamp_ns{0};
  JointIngressClockPoint callback_entry, owner_end, gate_end;
  JointIngressClockPoint ingest_begin, ingest_commit;
};

struct JointKinematicsEvidence {
  int64_t source_stamp_ns{0};
  int64_t receipt_stamp_ns{0};
  int64_t receipt_steady_ns{0};
  std::size_t history_samples{0};
  int64_t accepted_source_interval_ns{0}, accepted_receipt_interval_ns{0};
  int64_t fk_wall_ns{0}, fk_thread_cpu_ns{0};
  JointIngressTrace ingress;
  // Failure-path formatting of this frozen accepted sample only. No lookup or clocks.
  std::string ingress_detail() const;
};

struct JointKinematicsIngestResult {
  bool accepted{false};
  bool fault_latched{false};
  std::string reason;
  JointKinematicsEvidence evidence;
};

struct JointKinematicsAgeDiagnostics {
  // Captured with evidence under the SAME lookup shared lock. Ages are not
  // evaluated until the original clock/input checks permit their arithmetic.
  int64_t now_ros_ns{0}, now_steady_ns{0}, maximum_age_ns{0};
  int64_t joint_source_age_ns{-1}, joint_receipt_age_ns{-1}, query_source_age_ns{-1};
  bool evaluated{false};
  enum : uint8_t {JointSource=1, JointReceipt=2};
  uint8_t stale_mask{0};
};

struct JointNumericGeometry {
  // Immutable target order from this owner/model; no clocks or eligibility.
  std::vector<geometry_msgs::msg::Transform> transforms;
};
struct JointKinematicsLookup {
  JointKinematicsStatus status{JointKinematicsStatus::Missing};
  std::string reason;
  int64_t query_source_stamp_ns{0};
  JointKinematicsEvidence evidence;
  JointKinematicsAgeDiagnostics age_diagnostics;
  // Fault-path formatting only; immutable snapshot, no new clocks or lookup.
  std::string failure_detail(const std::string& role, uint64_t sequence,
      int64_t sdk_receipt_ros_ns, int64_t sdk_receipt_steady_ns) const;
  // Complete, first-failure log only; the existing compact failure_detail and
  // 512-byte fault propagation remain unchanged.
  std::string ingress_failure_detail(const std::string& role, uint64_t sequence) const;
  // All targets, in options order, or empty. Never latest-time substitutes.
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  std::shared_ptr<const JointNumericGeometry> numeric_geometry;
  bool numeric_geometry_reused{false};
};

// No ROS subscriptions or TF listener. The independently scheduled joint input
// owner first applies the unchanged LiveJointStateBridgeGate (49 -> 13, owner,
// publisher GID and source timing), then passes its decision here. The proof
// JointState must still be published by that owner for platform/global RSP.
//
// Each KDL joint edge is stored at the original measurement stamp and tf2
// interpolates each edge before composition, exactly as RSP + BufferCore. We do
// not interpolate joint angles or a composed base-to-foot pose. Only paths to
// targets and the expected joints are retained; unobserved arms are excluded.
// Mimic on those paths is explicitly unsupported and rejected at construction.
class JointKinematics {
 public:
  explicit JointKinematics(JointKinematicsOptions options);
  ~JointKinematics();
  JointKinematics(const JointKinematics&) = delete;
  JointKinematics& operator=(const JointKinematics&) = delete;

  JointKinematicsIngestResult ingest(
      const sunhorizon_platform_adapter_casbot02::LiveJointStateDecision& decision,
      int64_t receipt_stamp_ns, int64_t receipt_steady_ns,
      JointIngressTrace ingress = {});

  struct ClockSample {int64_t ros_ns{0}, steady_ns{0};};
  using ClockSupplier = std::function<ClockSample()>;
  // The supplier runs AFTER acquiring the same read lock as the FK snapshot.
  // Production must sample current clocks here, never capture pre-lock values:
  // a concurrent ingest can otherwise make a real current time appear regressed.
  // Keep it bounded and non-reentrant. Tests/replay can supply fixed clocks.
  // Live joint ROS source and monotonic receipt ages retain their gates. Query
  // time only selects retained history; the camera owner owns its frame deadline.
  JointKinematicsLookup lookup(
      int64_t source_stamp_ns, const ClockSupplier&) const;
  // Same source history, freshness and edge interpolation as lookup(). The
  // immutable target topology shares each ancestor evaluation within this one
  // query; no joint result is cached across source times.
  JointKinematicsLookup lookup_shared_edges(int64_t source_stamp_ns,const ClockSupplier&) const;
  // Airy numerical path. All original per-query gates/evidence remain, while
  // geometry contains no repeated frame strings or restamped per-link messages.
  JointKinematicsLookup lookup_numeric(int64_t source_stamp_ns,const ClockSupplier&) const;
  // Narrow source-time waist query from this SAME owner and freshness gates.
  JointKinematicsLookup lookup_measured_waist(int64_t source_stamp_ns,const ClockSupplier&) const;
  geometry_msgs::msg::Transform neutral_base_from_waist() const;
  // Configure once before ingress. Sink only inserts bounded edge batches in
  // the process-local TF cache; it must not publish, wait or run algorithms.
  using TransformSink = std::function<void(
      const std::vector<geometry_msgs::msg::TransformStamped>&, bool)>;
  void set_transform_sink(TransformSink);
  // Installs the validated public sensor chain once. The twelve dynamic leg
  // targets stay private; production waist stays dynamic and attended waist fixed.
  void mirror_into(std::shared_ptr<tf2::BufferCore>, TransformSink compatibility = {});
  JointKinematicsEvidence evidence() const;
  const JointKinematicsOptions& options() const noexcept;
  // Sum of parsed rigid edge translation lengths along the model path. Valid
  // for arbitrary revolute positions; unsupported/unbounded paths return inf.
  // Attended accepted fixed sensor edges replace their corresponding URDF edges.
  double origin_radius_bound_from_base(const std::string& frame) const;

 private:
  JointKinematicsLookup lookup_impl(int64_t,const ClockSupplier&,bool shared_edges,bool numeric_only=false,bool waist_only=false) const;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace sunhorizon_perception_runtime
