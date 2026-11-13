#include <cassert>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "sunhorizon_platform_adapter_casbot02/joint_log_evidence.hpp"

using sunhorizon_platform_adapter_casbot02::parse_latest_ethercat_joint_block;
using sunhorizon_platform_adapter_casbot02::build_bound_joint_info_json;
using sunhorizon_platform_adapter_casbot02::validate_joint_state_sample;

namespace {

std::string block(uint16_t statusword = 0x0237, uint32_t error_code = 0) {
  std::string output = "状态信息(ROH)\n";
  char line[256];
  for (int id = 0; id < 12; ++id) {
    std::snprintf(line, sizeof(line),
        "  %d: 状态字=0x%04x | 模式=0xffffffff | 位置=%.6f | 错误=0x%04x\n",
        id, statusword, static_cast<double>(id) * 0.01, error_code);
    output += line;
  }
  output += "控制命令(ROH)\n";
  return output;
}

void test_nominal_latest_block() {
  const auto parsed = parse_latest_ethercat_joint_block(
      block(0x0040, 1) + block(), {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11});
  assert(parsed.evidence.parsed);
  assert(parsed.evidence.complete);
  assert(parsed.evidence.error_free);
  assert(parsed.evidence.operation_enabled);
  assert(parsed.evidence.reason == "ok");
  assert(parsed.json.find("\"index\":11") != std::string::npos);
}

void test_missing_joint_fails_closed() {
  auto text = block();
  const auto position = text.find("  7:");
  text.erase(position, text.find('\n', position) - position + 1U);
  const auto parsed = parse_latest_ethercat_joint_block(
      text, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11});
  assert(parsed.evidence.parsed);
  assert(!parsed.evidence.complete);
  assert(parsed.evidence.reason == "required_joint_missing");
}

void test_partial_newest_block_falls_back_to_latest_complete_block() {
  auto partial = block();
  const auto position = partial.find("  7:");
  partial.erase(position, partial.find('\n', position) - position + 1U);
  const auto parsed = parse_latest_ethercat_joint_block(
      block() + partial, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11});
  assert(parsed.evidence.complete);
  assert(parsed.evidence.error_free);
  assert(parsed.evidence.reason == "ok");
}

void test_partial_newest_block_never_hides_new_fault() {
  auto partial = block();
  const auto missing = partial.find("  7:");
  partial.erase(missing, partial.find('\n', missing) - missing + 1U);
  const auto error = partial.find("错误=0x0000", partial.find("  4:"));
  partial.replace(error, std::string("错误=0x0000").size(), "错误=0x3804");
  const auto parsed = parse_latest_ethercat_joint_block(
      block() + partial, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11});
  assert(!parsed.evidence.complete);
  assert(!parsed.evidence.error_free);
  assert(parsed.evidence.reason == "leg_joint_error");
}

void test_error_and_non_operation_enabled_fail_closed() {
  auto parsed = parse_latest_ethercat_joint_block(block(0x0237, 2), {0, 1});
  assert(!parsed.evidence.error_free);
  assert(parsed.evidence.reason == "leg_joint_error");
  parsed = parse_latest_ethercat_joint_block(block(0x0040, 0), {0, 1});
  assert(!parsed.evidence.operation_enabled);
  assert(parsed.evidence.reason == "leg_joint_not_operation_enabled");
}

void test_explicit_3804_policy_is_non_blocking_and_other_errors_still_block() {
  const auto accepted = parse_latest_ethercat_joint_block(
      block(0x0237, 0x3804), {0, 1}, {0x3804U});
  assert(accepted.evidence.complete);
  assert(accepted.evidence.error_free);
  assert(accepted.evidence.operation_enabled);
  assert(accepted.evidence.reason == "ok_non_blocking_leg_error_code");
  assert(accepted.evidence.joints[0].error_code == 0x3804U);
  assert(accepted.json.find("\"error_code\":14340") != std::string::npos);

  const auto rejected = parse_latest_ethercat_joint_block(
      block(0x0237, 0x3805), {0, 1}, {0x3804U});
  assert(!rejected.evidence.error_free);
  assert(rejected.evidence.reason == "leg_joint_error");
}

void test_ansi_prefixed_error_line_is_not_reported_missing() {
  auto text = block();
  const auto line_start = text.find("  4:");
  assert(line_start != std::string::npos);
  const auto error_start = text.find("错误=0x0000", line_start);
  assert(error_start != std::string::npos);
  text.replace(error_start, std::string("错误=0x0000").size(), "错误=0x3804");
  text.insert(line_start, "\x1b[31m");
  const auto line_end = text.find('\n', line_start);
  assert(line_end != std::string::npos);
  text.insert(line_end, "\x1b[0m");

  const auto parsed = parse_latest_ethercat_joint_block(
      text, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11});
  assert(parsed.evidence.parsed);
  assert(parsed.evidence.complete);
  assert(!parsed.evidence.error_free);
  assert(parsed.evidence.reason == "leg_joint_error");
  assert(parsed.evidence.joints.size() == 12U);
  assert(parsed.evidence.joints[4].id == 4);
  assert(parsed.evidence.joints[4].error_code == 0x3804U);
}

std::vector<std::string> leg_names() {
  return {
      "left_leg_pelvic_pitch_joint", "left_leg_pelvic_roll_joint",
      "left_leg_pelvic_yaw_joint", "left_leg_knee_pitch_joint",
      "left_leg_ankle_pitch_joint", "left_leg_ankle_roll_joint",
      "right_leg_pelvic_pitch_joint", "right_leg_pelvic_roll_joint",
      "right_leg_pelvic_yaw_joint", "right_leg_knee_pitch_joint",
      "right_leg_ankle_pitch_joint", "right_leg_ankle_roll_joint"};
}

std::vector<int> leg_ids() {
  return {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
}

void test_duplicate_raw_fault_record_fails_closed() {
  auto duplicate = block();
  const auto line_start = duplicate.find("  4:");
  const auto line_end = duplicate.find('\n', line_start);
  duplicate.insert(line_end + 1U,
      duplicate.substr(line_start, line_end - line_start + 1U));
  const auto parsed_duplicate = parse_latest_ethercat_joint_block(duplicate, leg_ids());
  assert(parsed_duplicate.evidence.reason == "duplicate_joint_record");

}

void test_raw_position_text_is_ignored_for_fault_attribution() {
  auto text = block();
  const auto position = text.find("位置=0.030000");
  assert(position != std::string::npos);
  text.replace(position, std::string("位置=0.030000").size(), "位置=not-a-number");
  const auto parsed = parse_latest_ethercat_joint_block(text, leg_ids());
  assert(parsed.evidence.complete);
  assert(parsed.evidence.error_free);
  assert(parsed.evidence.operation_enabled);
  assert(parsed.evidence.reason == "ok");
}

void test_consumer_validates_exact_names_lengths_finite_stamp_and_owner() {
  auto names = leg_names();
  names.push_back("waist_yaw");
  std::vector<double> positions(13U, 0.01);
  auto result = validate_joint_state_sample(
      names, positions, leg_names(), "waist_yaw", 1000, 1100, 200, true);
  assert(result.valid);
  assert(result.waist_position_rad == 0.01);
  auto wrong_names = names;
  std::swap(wrong_names[0], wrong_names[1]);
  assert(validate_joint_state_sample(
      wrong_names, positions, leg_names(), "waist_yaw", 1000, 1100, 200,
      true).reason == "joint_state_names_or_array_length_invalid");
  positions.pop_back();
  assert(validate_joint_state_sample(
      names, positions, leg_names(), "waist_yaw", 1000, 1100, 200,
      true).reason == "joint_state_names_or_array_length_invalid");
  positions.assign(13U, 0.01);
  positions[4] = std::numeric_limits<double>::infinity();
  assert(validate_joint_state_sample(
      names, positions, leg_names(), "waist_yaw", 1000, 1100, 200,
      true).reason == "joint_state_position_not_finite");
  assert(validate_joint_state_sample(
      names, std::vector<double>(13U, 0.01), leg_names(), "waist_yaw",
      1000, 1100, 200, false).reason == "joint_state_proof_owner_not_unique");
}

void test_joint_info_exposes_active_mapping_binding() {
  auto names = leg_names();
  const auto value = build_bound_joint_info_json(
      "[{\"index\":0,\"error_code\":0,\"statusword\":567}]", names,
      123456789, "/data/navigation/calibration/mapping.receipt.json",
      std::string(64U, 'a'), std::string(64U, 'b'));
  assert(value.find("robot_a_platform_joint_fault_attribution") != std::string::npos);
  assert(value.find("\"ignored_fields\":[\"position\"]") != std::string::npos);
  assert(value.find("\"source_stamp_ns\":123456789") != std::string::npos);
  assert(value.find("\"receipt_sha256\":\"" + std::string(64U, 'a')) !=
      std::string::npos);
  assert(value.find("left_leg_pelvic_pitch_joint") != std::string::npos);
  assert(value.find("\"motion_authorized\":false") != std::string::npos);
}

}  // namespace

int main() {
  test_nominal_latest_block();
  test_missing_joint_fails_closed();
  test_partial_newest_block_falls_back_to_latest_complete_block();
  test_partial_newest_block_never_hides_new_fault();
  test_error_and_non_operation_enabled_fail_closed();
  test_explicit_3804_policy_is_non_blocking_and_other_errors_still_block();
  test_ansi_prefixed_error_line_is_not_reported_missing();
  test_duplicate_raw_fault_record_fails_closed();
  test_raw_position_text_is_ignored_for_fault_attribution();
  test_consumer_validates_exact_names_lengths_finite_stamp_and_owner();
  test_joint_info_exposes_active_mapping_binding();
  return 0;
}
