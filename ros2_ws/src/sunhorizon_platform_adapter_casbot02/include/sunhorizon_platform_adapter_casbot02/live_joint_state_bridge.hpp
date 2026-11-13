#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sunhorizon_platform_adapter_casbot02 {

struct LiveJointStatePublisherIdentity {
  std::string node_name;
  std::string node_namespace;
  std::string topic_type;
  std::string endpoint_gid;
};

enum class LiveJointStateOwnerStatus {
  kMatched,
  kDiscoveryPending,
  kInvalid,
};

LiveJointStateOwnerStatus live_joint_source_owner_status(
    std::size_t endpoint_count,
    const LiveJointStatePublisherIdentity& expected,
    const LiveJointStatePublisherIdentity& observed);

bool live_joint_source_owner_matches_contract(
    std::size_t endpoint_count,
    const LiveJointStatePublisherIdentity& expected,
    const LiveJointStatePublisherIdentity& observed);

struct LiveJointStateDiscoveryDecision {
  bool accepted{false};
  bool fault_latched{false};
  std::string reason;
};

// Allows only the startup interval needed for Fast DDS to resolve an endpoint's
// ROS node identity.  After the first accepted identity, a publisher restart
// may temporarily remove or rotate the endpoint; that interval produces no
// output and the consuming supervisor remains responsible for latching zero.
class LiveJointStateDiscoveryGate {
 public:
  explicit LiveJointStateDiscoveryGate(int64_t grace_period_ns);

  LiveJointStateDiscoveryDecision evaluate(
      LiveJointStateOwnerStatus status, int64_t elapsed_ns);
  bool fault_latched() const { return fault_latched_; }
  const std::string& fault_reason() const { return fault_reason_; }

 private:
  LiveJointStateDiscoveryDecision latch(const std::string& reason);

  int64_t grace_period_ns_{0};
  bool identity_resolved_{false};
  bool fault_latched_{false};
  std::string fault_reason_;
};

struct LiveJointStateInput {
  int64_t source_stamp_ns{0};
  int64_t receipt_stamp_ns{0};
  std::vector<std::string> names;
  std::vector<double> positions;
  bool source_owner_unique{false};
  bool message_publisher_gid_matches{false};
};

struct LiveJointStateDecision {
  bool accepted{false};
  bool fault_latched{false};
  std::string reason;
  int64_t source_stamp_ns{0};
  std::vector<std::string> names;
  std::vector<double> positions;
};

// Strictly narrows the accepted 49-joint vendor stream to the reviewed twelve
// leg mappings and waist. Structural/content violations latch output closed and
// require a node restart with the same launch-time receipt. Transient transport
// timing violations (a stale or non-increasing frame) reject only that frame so
// a later fresh, strictly increasing frame from the same verified owner can
// restore evidence. The downstream velocity supervisor independently latches
// zero on the resulting evidence gap and still requires an operator reset.
// Runtime graph/GID gaps likewise reject individual frames without trusting the
// recording-time endpoint GID.
class LiveJointStateBridgeGate {
 public:
  LiveJointStateBridgeGate(
      std::vector<std::string> expected_source_names,
      std::vector<std::string> selected_source_names,
      std::vector<std::size_t> selected_source_indices,
      std::vector<std::string> output_names,
      std::string position_unit,
      std::string value_transform,
      int64_t maximum_age_ns,
      int64_t future_tolerance_ns);

  LiveJointStateDecision evaluate(const LiveJointStateInput& input);
  bool fault_latched() const { return fault_latched_; }
  const std::string& fault_reason() const { return fault_reason_; }

 private:
  LiveJointStateDecision latch(const std::string& reason);

  std::vector<std::string> expected_source_names_;
  std::vector<std::size_t> selected_indices_;
  std::vector<std::string> output_names_;
  int64_t maximum_age_ns_{0};
  int64_t future_tolerance_ns_{0};
  int64_t last_source_stamp_ns_{0};
  bool fault_latched_{false};
  std::string fault_reason_;
};

}  // namespace sunhorizon_platform_adapter_casbot02
