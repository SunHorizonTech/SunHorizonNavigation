#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "sunhorizon_platform_adapter_casbot02/live_joint_state_bridge.hpp"

using sunhorizon_platform_adapter_casbot02::LiveJointStateBridgeGate;
using sunhorizon_platform_adapter_casbot02::LiveJointStateDiscoveryGate;
using sunhorizon_platform_adapter_casbot02::LiveJointStateInput;
using sunhorizon_platform_adapter_casbot02::LiveJointStateOwnerStatus;
using sunhorizon_platform_adapter_casbot02::LiveJointStatePublisherIdentity;
using sunhorizon_platform_adapter_casbot02::live_joint_source_owner_matches_contract;
using sunhorizon_platform_adapter_casbot02::live_joint_source_owner_status;

namespace {

std::vector<std::string> selected_names() {
  return {
      "LJ0", "LJ1", "LJ2", "LJ3", "LJ4", "LJ5",
      "RJ6", "RJ7", "RJ8", "RJ9", "RJ10", "RJ11", "waist_yaw"};
}

std::vector<std::string> output_names() {
  return {
      "left_leg_pelvic_pitch_joint", "left_leg_pelvic_roll_joint",
      "left_leg_pelvic_yaw_joint", "left_leg_knee_pitch_joint",
      "left_leg_ankle_pitch_joint", "left_leg_ankle_roll_joint",
      "right_leg_pelvic_pitch_joint", "right_leg_pelvic_roll_joint",
      "right_leg_pelvic_yaw_joint", "right_leg_knee_pitch_joint",
      "right_leg_ankle_pitch_joint", "right_leg_ankle_roll_joint", "waist_yaw"};
}

std::vector<std::string> source_names() {
  std::vector<std::string> output;
  output.reserve(49U);
  for (int index = 0; index < 49; ++index) {
    output.push_back("vendor_joint_" + std::to_string(index));
  }
  const auto selected = selected_names();
  const std::vector<std::size_t> indices = {8, 2, 31, 4, 45, 12, 6, 29, 10, 38, 15, 47, 20};
  for (std::size_t index = 0; index < selected.size(); ++index) {
    output[indices[index]] = selected[index];
  }
  return output;
}

LiveJointStateBridgeGate gate() {
  const auto names = source_names();
  std::vector<std::size_t> indices;
  for (const auto& selected : selected_names()) {
    indices.push_back(static_cast<std::size_t>(
        std::find(names.begin(), names.end(), selected) - names.begin()));
  }
  return LiveJointStateBridgeGate(
      names, selected_names(), indices, output_names(), "rad", "identity",
      250000000LL, 50000000LL);
}

LiveJointStateInput valid_input() {
  const auto names = source_names();
  std::vector<double> positions;
  positions.reserve(names.size());
  for (std::size_t index = 0; index < names.size(); ++index) {
    positions.push_back(static_cast<double>(index) / 100.0);
  }
  return {1000000000LL, 1100000000LL, names, positions, true, true};
}

void test_selects_by_reviewed_name_not_source_position() {
  auto bridge = gate();
  const auto input = valid_input();
  const auto result = bridge.evaluate(input);
  assert(result.accepted);
  assert(!result.fault_latched);
  assert(result.reason == "ok");
  assert(result.names == output_names());
  assert(result.positions.size() == 13U);
  assert(std::abs(result.positions[0] - 0.08) < 1e-12);
  assert(std::abs(result.positions[1] - 0.02) < 1e-12);
  assert(std::abs(result.positions[12] - 0.20) < 1e-12);
}

void test_full_49_name_sequence_is_exact_and_fault_latched() {
  auto bridge = gate();
  auto input = valid_input();
  std::swap(input.names[0], input.names[1]);
  auto result = bridge.evaluate(input);
  assert(!result.accepted);
  assert(result.fault_latched);
  assert(result.reason == "live_joint_source_names_mismatch");
  result = bridge.evaluate(valid_input());
  assert(!result.accepted);
  assert(result.reason == "live_joint_source_names_mismatch");
}

void test_position_shape_and_finiteness_fail_closed() {
  auto length_bridge = gate();
  auto input = valid_input();
  input.positions.pop_back();
  auto result = length_bridge.evaluate(input);
  assert(result.fault_latched);
  assert(result.reason == "live_joint_source_position_length_mismatch");
  assert(!length_bridge.evaluate(valid_input()).accepted);

  auto finite_bridge = gate();
  input = valid_input();
  input.positions[48] = std::numeric_limits<double>::infinity();
  result = finite_bridge.evaluate(input);
  assert(result.fault_latched);
  assert(result.reason == "live_joint_source_position_non_finite");
  assert(!finite_bridge.evaluate(valid_input()).accepted);
}

void test_owner_and_gid_gaps_reject_frames_then_recover() {
  auto owner_bridge = gate();
  auto input = valid_input();
  input.source_owner_unique = false;
  auto owner_result = owner_bridge.evaluate(input);
  assert(owner_result.reason == "live_joint_source_owner_invalid");
  assert(!owner_result.fault_latched);
  assert(owner_bridge.evaluate(valid_input()).accepted);

  auto message_gid_bridge = gate();
  input = valid_input();
  input.message_publisher_gid_matches = false;
  auto message_gid_result = message_gid_bridge.evaluate(input);
  assert(message_gid_result.reason == "live_joint_message_publisher_gid_mismatch");
  assert(!message_gid_result.fault_latched);
  assert(message_gid_bridge.evaluate(valid_input()).accepted);
}

void test_stale_and_nonincreasing_frames_reject_then_fresh_frame_recovers() {
  auto stale_bridge = gate();
  assert(stale_bridge.evaluate(valid_input()).accepted);
  auto input = valid_input();
  input.source_stamp_ns = 1100000000LL;
  input.receipt_stamp_ns = 1400000000LL;
  auto stale_result = stale_bridge.evaluate(input);
  assert(!stale_result.accepted);
  assert(!stale_result.fault_latched);
  assert(stale_result.reason == "live_joint_source_stale_at_receipt");
  input = valid_input();
  input.source_stamp_ns = 1500000000LL;
  input.receipt_stamp_ns = 1600000000LL;
  assert(stale_bridge.evaluate(input).accepted);

  auto monotonic_bridge = gate();
  assert(monotonic_bridge.evaluate(valid_input()).accepted);
  input = valid_input();
  input.receipt_stamp_ns = 1101000000LL;
  auto monotonic_result = monotonic_bridge.evaluate(input);
  assert(!monotonic_result.accepted);
  assert(!monotonic_result.fault_latched);
  assert(monotonic_result.reason == "live_joint_source_stamp_not_increasing");
  input.source_stamp_ns = 1200000000LL;
  input.receipt_stamp_ns = 1300000000LL;
  assert(monotonic_bridge.evaluate(input).accepted);
}

void test_invalid_or_future_timestamps_remain_fault_latched() {
  auto invalid_bridge = gate();
  auto input = valid_input();
  input.source_stamp_ns = 0;
  auto invalid_result = invalid_bridge.evaluate(input);
  assert(!invalid_result.accepted);
  assert(invalid_result.fault_latched);
  assert(invalid_result.reason == "live_joint_source_stamp_invalid");
  assert(!invalid_bridge.evaluate(valid_input()).accepted);

  auto future_bridge = gate();
  input = valid_input();
  input.source_stamp_ns = 1200000000LL;
  input.receipt_stamp_ns = 1100000000LL;
  auto future_result = future_bridge.evaluate(input);
  assert(!future_result.accepted);
  assert(future_result.fault_latched);
  assert(future_result.reason == "live_joint_source_stamp_future");
  assert(!future_bridge.evaluate(valid_input()).accepted);

  auto accepted_then_future_bridge = gate();
  assert(accepted_then_future_bridge.evaluate(valid_input()).accepted);
  input = valid_input();
  input.source_stamp_ns = 1000000000LL;
  input.receipt_stamp_ns = 900000000LL;
  future_result = accepted_then_future_bridge.evaluate(input);
  assert(future_result.fault_latched);
  assert(future_result.reason == "live_joint_source_stamp_future");
}

void test_contract_rejects_missing_selected_name() {
  auto names = source_names();
  for (auto& name : names) {
    if (name == "RJ11") name = "different_joint";
  }
  bool threw = false;
  try {
    LiveJointStateBridgeGate invalid(
        names, selected_names(), std::vector<std::size_t>(13U, 0U),
        output_names(), "rad", "identity", 250000000LL, 50000000LL);
    (void)invalid;
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  assert(threw);
}

void test_contract_rejects_wrong_index_or_value_transform() {
  auto names = source_names();
  std::vector<std::size_t> indices;
  for (const auto& selected : selected_names()) {
    indices.push_back(static_cast<std::size_t>(
        std::find(names.begin(), names.end(), selected) - names.begin()));
  }
  auto wrong_indices = indices;
  std::swap(wrong_indices[0], wrong_indices[1]);
  bool wrong_index_rejected = false;
  try {
    LiveJointStateBridgeGate invalid(
        names, selected_names(), wrong_indices, output_names(), "rad", "identity",
        250000000LL, 50000000LL);
    (void)invalid;
  } catch (const std::invalid_argument&) {
    wrong_index_rejected = true;
  }
  assert(wrong_index_rejected);

  bool transform_rejected = false;
  try {
    LiveJointStateBridgeGate invalid(
        names, selected_names(), indices, output_names(), "rad", "scale_offset",
        250000000LL, 50000000LL);
    (void)invalid;
  } catch (const std::invalid_argument&) {
    transform_rejected = true;
  }
  assert(transform_rejected);
}

void test_runtime_owner_uses_current_endpoint_and_allows_gid_rotation() {
  const LiveJointStatePublisherIdentity expected{
      "hl_orin_core_msg", "/", "sensor_msgs/msg/JointState", ""};
  auto observed = expected;
  observed.endpoint_gid = std::string(48, '1');
  assert(live_joint_source_owner_matches_contract(1U, expected, observed));
  observed.endpoint_gid = std::string(48, '2');
  assert(live_joint_source_owner_matches_contract(1U, expected, observed));
  assert(!live_joint_source_owner_matches_contract(2U, expected, observed));
}

void test_runtime_owner_distinguishes_bounded_identity_discovery() {
  const LiveJointStatePublisherIdentity expected{
      "hl_orin_core_msg", "/", "sensor_msgs/msg/JointState", ""};
  auto observed = expected;
  observed.endpoint_gid = std::string(48, '1');
  observed.node_name = "_NODE_NAME_UNKNOWN_";
  observed.node_namespace = "_NODE_NAMESPACE_UNKNOWN_";
  assert(live_joint_source_owner_status(1U, expected, observed) ==
      LiveJointStateOwnerStatus::kDiscoveryPending);
  assert(live_joint_source_owner_status(0U, expected, {}) ==
      LiveJointStateOwnerStatus::kDiscoveryPending);

  observed.endpoint_gid = std::string(48, '2');
  observed.node_name = expected.node_name;
  observed.node_namespace = expected.node_namespace;
  assert(live_joint_source_owner_status(1U, expected, observed) ==
      LiveJointStateOwnerStatus::kMatched);
  observed.node_name = "wrong_owner";
  assert(live_joint_source_owner_status(1U, expected, observed) ==
      LiveJointStateOwnerStatus::kInvalid);
  assert(live_joint_source_owner_status(2U, expected, observed) ==
      LiveJointStateOwnerStatus::kInvalid);
}

void test_discovery_gate_waits_without_output_then_accepts_resolved_owner() {
  LiveJointStateDiscoveryGate discovery(12000000000LL);
  auto decision = discovery.evaluate(
      LiveJointStateOwnerStatus::kDiscoveryPending, 0LL);
  assert(!decision.accepted);
  assert(!decision.fault_latched);
  assert(decision.reason == "live_joint_source_owner_discovery_pending");
  decision = discovery.evaluate(
      LiveJointStateOwnerStatus::kDiscoveryPending, 11999999999LL);
  assert(!decision.accepted);
  assert(!decision.fault_latched);
  decision = discovery.evaluate(
      LiveJointStateOwnerStatus::kMatched, 11999999999LL);
  assert(decision.accepted);
  assert(!decision.fault_latched);
}

void test_discovery_gate_timeout_stays_fail_closed_and_recovers_late_owner() {
  LiveJointStateDiscoveryGate timeout(12000000000LL);
  auto decision = timeout.evaluate(
      LiveJointStateOwnerStatus::kDiscoveryPending, 12000000000LL);
  assert(!decision.accepted);
  assert(!decision.fault_latched);
  assert(decision.reason == "live_joint_source_owner_discovery_timeout");
  decision = timeout.evaluate(LiveJointStateOwnerStatus::kMatched, 12000000001LL);
  assert(decision.accepted);
  assert(!decision.fault_latched);
  assert(decision.reason == "ok");
}

void test_discovery_gate_rejects_invalid_owner_fail_closed() {
  LiveJointStateDiscoveryGate invalid(12000000000LL);
  auto decision = invalid.evaluate(LiveJointStateOwnerStatus::kInvalid, 1LL);
  assert(!decision.accepted);
  assert(decision.fault_latched);
  assert(decision.reason == "live_joint_source_owner_invalid");

  LiveJointStateDiscoveryGate restarted(12000000000LL);
  assert(restarted.evaluate(LiveJointStateOwnerStatus::kMatched, 1LL).accepted);
  decision = restarted.evaluate(
      LiveJointStateOwnerStatus::kDiscoveryPending, 2LL);
  assert(!decision.accepted);
  assert(!decision.fault_latched);
  assert(decision.reason == "live_joint_source_owner_unavailable");
  decision = restarted.evaluate(LiveJointStateOwnerStatus::kInvalid, 3LL);
  assert(!decision.accepted);
  assert(!decision.fault_latched);
  assert(decision.reason == "live_joint_source_owner_invalid");
  decision = restarted.evaluate(LiveJointStateOwnerStatus::kMatched, 4LL);
  assert(decision.accepted);
  assert(!decision.fault_latched);
}

void test_runtime_gid_rotation_recovers_without_accepting_the_gap() {
  const LiveJointStatePublisherIdentity expected{
      "hl_orin_core_msg", "/", "sensor_msgs/msg/JointState", ""};
  LiveJointStatePublisherIdentity original = expected;
  original.endpoint_gid = std::string(48, '1');
  LiveJointStatePublisherIdentity replacement = expected;
  replacement.endpoint_gid = std::string(48, '2');
  LiveJointStateDiscoveryGate discovery(12000000000LL);
  auto bridge = gate();

  assert(live_joint_source_owner_status(1U, expected, original) ==
      LiveJointStateOwnerStatus::kMatched);
  assert(discovery.evaluate(LiveJointStateOwnerStatus::kMatched, 1LL).accepted);
  auto original_frame = valid_input();
  assert(bridge.evaluate(original_frame).accepted);

  auto gap = discovery.evaluate(
      LiveJointStateOwnerStatus::kDiscoveryPending, 2LL);
  assert(!gap.accepted);
  assert(!gap.fault_latched);
  auto duplicate = discovery.evaluate(LiveJointStateOwnerStatus::kInvalid, 3LL);
  assert(!duplicate.accepted);
  assert(!duplicate.fault_latched);

  assert(live_joint_source_owner_status(1U, expected, replacement) ==
      LiveJointStateOwnerStatus::kMatched);
  assert(discovery.evaluate(LiveJointStateOwnerStatus::kMatched, 4LL).accepted);
  auto replacement_frame = valid_input();
  replacement_frame.source_stamp_ns += 10000000LL;
  replacement_frame.receipt_stamp_ns += 10000000LL;
  assert(bridge.evaluate(replacement_frame).accepted);
}

}  // namespace

int main() {
  test_selects_by_reviewed_name_not_source_position();
  test_full_49_name_sequence_is_exact_and_fault_latched();
  test_position_shape_and_finiteness_fail_closed();
  test_owner_and_gid_gaps_reject_frames_then_recover();
  test_stale_and_nonincreasing_frames_reject_then_fresh_frame_recovers();
  test_invalid_or_future_timestamps_remain_fault_latched();
  test_contract_rejects_missing_selected_name();
  test_contract_rejects_wrong_index_or_value_transform();
  test_runtime_owner_uses_current_endpoint_and_allows_gid_rotation();
  test_runtime_owner_distinguishes_bounded_identity_discovery();
  test_discovery_gate_waits_without_output_then_accepts_resolved_owner();
  test_discovery_gate_timeout_stays_fail_closed_and_recovers_late_owner();
  test_discovery_gate_rejects_invalid_owner_fail_closed();
  test_runtime_gid_rotation_recovers_without_accepting_the_gap();
  return 0;
}
