#include "sunhorizon_platform_adapter_casbot02/live_joint_state_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>

namespace sunhorizon_platform_adapter_casbot02 {

namespace {

bool endpoint_gid_valid(const std::string& value) {
  if (value.size() < 16U || value.size() > 128U || value.size() % 2U != 0U) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](const unsigned char value) {
    return (value >= '0' && value <= '9') || (value >= 'a' && value <= 'f');
  });
}

}  // namespace

LiveJointStateOwnerStatus live_joint_source_owner_status(
    std::size_t endpoint_count,
    const LiveJointStatePublisherIdentity& expected,
    const LiveJointStatePublisherIdentity& observed) {
  if (endpoint_count == 0U) {
    return LiveJointStateOwnerStatus::kDiscoveryPending;
  }
  if (endpoint_count != 1U || observed.topic_type != expected.topic_type ||
      !endpoint_gid_valid(observed.endpoint_gid)) {
    return LiveJointStateOwnerStatus::kInvalid;
  }
  if (observed.node_name == expected.node_name &&
      observed.node_namespace == expected.node_namespace) {
    return LiveJointStateOwnerStatus::kMatched;
  }
  const bool name_pending = observed.node_name == "_NODE_NAME_UNKNOWN_";
  const bool namespace_pending =
      observed.node_namespace == "_NODE_NAMESPACE_UNKNOWN_";
  if ((observed.node_name == expected.node_name || name_pending) &&
      (observed.node_namespace == expected.node_namespace || namespace_pending) &&
      (name_pending || namespace_pending)) {
    return LiveJointStateOwnerStatus::kDiscoveryPending;
  }
  return LiveJointStateOwnerStatus::kInvalid;
}

bool live_joint_source_owner_matches_contract(
    std::size_t endpoint_count,
    const LiveJointStatePublisherIdentity& expected,
    const LiveJointStatePublisherIdentity& observed) {
  return live_joint_source_owner_status(endpoint_count, expected, observed) ==
      LiveJointStateOwnerStatus::kMatched;
}

LiveJointStateDiscoveryGate::LiveJointStateDiscoveryGate(
    int64_t grace_period_ns)
    : grace_period_ns_(grace_period_ns) {
  if (grace_period_ns_ <= 0) {
    throw std::invalid_argument("live joint discovery grace must be positive");
  }
}

LiveJointStateDiscoveryDecision LiveJointStateDiscoveryGate::latch(
    const std::string& reason) {
  fault_latched_ = true;
  if (fault_reason_.empty()) fault_reason_ = reason;
  return {false, true, fault_reason_};
}

LiveJointStateDiscoveryDecision LiveJointStateDiscoveryGate::evaluate(
    LiveJointStateOwnerStatus status, int64_t elapsed_ns) {
  if (fault_latched_) return {false, true, fault_reason_};
  if (elapsed_ns < 0) return latch("live_joint_source_discovery_clock_invalid");
  if (status == LiveJointStateOwnerStatus::kMatched) {
    identity_resolved_ = true;
    return {true, false, "ok"};
  }
  if (status == LiveJointStateOwnerStatus::kInvalid) {
    if (identity_resolved_) {
      return {false, false, "live_joint_source_owner_invalid"};
    }
    return latch("live_joint_source_owner_invalid");
  }
  if (identity_resolved_) {
    return {false, false, "live_joint_source_owner_unavailable"};
  }
  if (elapsed_ns >= grace_period_ns_) {
    // The vendor process may become active before its ROS publisher exists.
    // Keep the bridge fail-closed, but continue discovery so a late, uniquely
    // identified publisher can recover without restarting the evidence stack.
    return {false, false, "live_joint_source_owner_discovery_timeout"};
  }
  return {false, false, "live_joint_source_owner_discovery_pending"};
}

LiveJointStateBridgeGate::LiveJointStateBridgeGate(
    std::vector<std::string> expected_source_names,
    std::vector<std::string> selected_source_names,
    std::vector<std::size_t> selected_source_indices,
    std::vector<std::string> output_names,
    std::string position_unit,
    std::string value_transform,
    int64_t maximum_age_ns,
    int64_t future_tolerance_ns)
    : expected_source_names_(std::move(expected_source_names)),
      output_names_(std::move(output_names)),
      maximum_age_ns_(maximum_age_ns),
      future_tolerance_ns_(future_tolerance_ns) {
  const std::set<std::string> expected_unique(
      expected_source_names_.begin(), expected_source_names_.end());
  const std::set<std::string> selected_unique(
      selected_source_names.begin(), selected_source_names.end());
  const std::set<std::string> output_unique(output_names_.begin(), output_names_.end());
  if (expected_source_names_.size() != 49U || expected_unique.size() != 49U ||
      selected_source_names.size() != 13U || selected_unique.size() != 13U ||
      selected_source_indices.size() != 13U ||
      output_names_.size() != 13U || output_unique.size() != 13U ||
      position_unit != "rad" || value_transform != "identity" ||
      maximum_age_ns_ <= 0 || future_tolerance_ns_ < 0) {
    throw std::invalid_argument("live joint state bridge contract shape invalid");
  }
  selected_indices_.reserve(selected_source_names.size());
  for (const auto& selected : selected_source_names) {
    const auto iterator = std::find(
        expected_source_names_.begin(), expected_source_names_.end(), selected);
    if (selected.empty() || iterator == expected_source_names_.end()) {
      throw std::invalid_argument("selected live joint absent from source contract");
    }
    const auto derived_index = static_cast<std::size_t>(
        std::distance(expected_source_names_.begin(), iterator));
    if (selected_source_indices[selected_indices_.size()] != derived_index) {
      throw std::invalid_argument("selected live joint index disagrees with name contract");
    }
    selected_indices_.push_back(derived_index);
  }
  if (output_names_.back() != "waist_yaw" ||
      selected_source_names.back() != "waist_yaw") {
    throw std::invalid_argument("waist joint must terminate live bridge contract");
  }
}

LiveJointStateDecision LiveJointStateBridgeGate::latch(const std::string& reason) {
  fault_latched_ = true;
  if (fault_reason_.empty()) fault_reason_ = reason;
  return {false, true, fault_reason_, 0, {}, {}};
}

LiveJointStateDecision LiveJointStateBridgeGate::evaluate(
    const LiveJointStateInput& input) {
  if (fault_latched_) return {false, true, fault_reason_, 0, {}, {}};
  if (!input.source_owner_unique) {
    return {false, false, "live_joint_source_owner_invalid", 0, {}, {}};
  }
  if (!input.message_publisher_gid_matches) {
    return {
        false, false, "live_joint_message_publisher_gid_mismatch", 0, {}, {}};
  }
  if (input.names != expected_source_names_) {
    return latch("live_joint_source_names_mismatch");
  }
  if (input.positions.size() != input.names.size()) {
    return latch("live_joint_source_position_length_mismatch");
  }
  if (!std::all_of(input.positions.begin(), input.positions.end(),
                   [](double value) { return std::isfinite(value); })) {
    return latch("live_joint_source_position_non_finite");
  }
  if (input.source_stamp_ns <= 0 || input.receipt_stamp_ns <= 0) {
    return latch("live_joint_source_stamp_invalid");
  }
  if (input.source_stamp_ns > input.receipt_stamp_ns + future_tolerance_ns_) {
    return latch("live_joint_source_stamp_future");
  }
  if (last_source_stamp_ns_ > 0 && input.source_stamp_ns <= last_source_stamp_ns_) {
    // A delayed/replayed transport frame must never be published, but it does
    // not change the reviewed source contract. Keep the last accepted stamp so
    // only a later, strictly increasing sample can recover the bridge.
    return {
        false, false, "live_joint_source_stamp_not_increasing", 0, {}, {}};
  }
  if (input.receipt_stamp_ns > input.source_stamp_ns + maximum_age_ns_) {
    // Vendor /joint_states can pause and Fast DDS can deliver one queued frame
    // after the stream resumes. Reject that stale frame fail-closed, but allow
    // the next fresh and strictly increasing frame to restore evidence. The
    // downstream supervisor remains responsible for latching zero across this
    // gap and therefore cannot resume motion without an operator reset.
    return {false, false, "live_joint_source_stale_at_receipt", 0, {}, {}};
  }

  LiveJointStateDecision output;
  output.accepted = true;
  output.reason = "ok";
  output.source_stamp_ns = input.source_stamp_ns;
  output.names = output_names_;
  output.positions.reserve(selected_indices_.size());
  for (const auto index : selected_indices_) output.positions.push_back(input.positions[index]);
  last_source_stamp_ns_ = input.source_stamp_ns;
  return output;
}

}  // namespace sunhorizon_platform_adapter_casbot02
