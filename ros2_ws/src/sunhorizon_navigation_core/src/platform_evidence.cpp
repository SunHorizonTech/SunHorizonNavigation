#include "sunhorizon_navigation_core/platform_evidence.hpp"

#include <algorithm>
#include <cctype>
#include <map>
#include <regex>
#include <set>
#include <string_view>

namespace sunhorizon_navigation_core {
namespace {

bool balanced_json(const std::string& text) {
  int braces = 0;
  int brackets = 0;
  bool quoted = false;
  bool escaped = false;
  for (const char character : text) {
    if (quoted) {
      if (escaped) escaped = false;
      else if (character == '\\') escaped = true;
      else if (character == '"') quoted = false;
      continue;
    }
    if (character == '"') quoted = true;
    else if (character == '{') ++braces;
    else if (character == '}') --braces;
    else if (character == '[') ++brackets;
    else if (character == ']') --brackets;
    if (braces < 0 || brackets < 0) return false;
  }
  return !quoted && braces == 0 && brackets == 0 && text.find('{') != std::string::npos;
}

bool extract_integer(const std::string& text, const std::string& key, int64_t* output) {
  const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*(-?[0-9]+)");
  std::smatch match;
  if (!std::regex_search(text, match, pattern)) return false;
  try {
    *output = std::stoll(match[1].str());
  } catch (...) {
    return false;
  }
  return true;
}

bool extract_any_integer(
    const std::string& text, const std::vector<std::string>& keys, int64_t* output) {
  for (const auto& key : keys) {
    if (extract_integer(text, key, output)) return true;
  }
  return false;
}

std::vector<std::string> json_objects(const std::string& text) {
  std::vector<std::string> objects;
  std::vector<std::size_t> starts;
  bool quoted = false;
  bool escaped = false;
  for (std::size_t index = 0; index < text.size(); ++index) {
    const char character = text[index];
    if (quoted) {
      if (escaped) escaped = false;
      else if (character == '\\') escaped = true;
      else if (character == '"') quoted = false;
      continue;
    }
    if (character == '"') quoted = true;
    else if (character == '{') starts.push_back(index);
    else if (character == '}' && !starts.empty()) {
      const auto start = starts.back();
      starts.pop_back();
      objects.push_back(text.substr(start, index - start + 1U));
    }
  }
  return objects;
}

std::string trim_copy(std::string_view input) {
  const auto first = input.find_first_not_of(" \t\r\n");
  if (first == std::string_view::npos) return {};
  const auto last = input.find_last_not_of(" \t\r\n");
  return std::string(input.substr(first, last - first + 1U));
}

bool split_contract_token(
    const std::string& token, const std::string& field, std::string* value) {
  const auto normalized = trim_copy(token);
  const auto prefix = field + ":";
  if (normalized.rfind(prefix, 0U) != 0U) return false;
  *value = trim_copy(std::string_view(normalized).substr(prefix.size()));
  return !value->empty() && value->find_first_of(",;\r\n") == std::string::npos;
}

bool extract_exact_field(
    const std::string& text, const std::string& field, std::string* value) {
  const auto prefix = field + ":";
  bool seen = false;
  std::size_t begin = 0U;
  while (begin <= text.size()) {
    const auto end = text.find_first_of(",;\r\n", begin);
    const auto length = end == std::string::npos ? text.size() - begin : end - begin;
    const auto segment = trim_copy(std::string_view(text).substr(begin, length));
    if (segment.rfind(prefix, 0U) == 0U) {
      if (seen) return false;
      *value = trim_copy(std::string_view(segment).substr(prefix.size()));
      if (value->empty()) return false;
      seen = true;
    }
    if (end == std::string::npos) break;
    begin = end + 1U;
  }
  return seen;
}

}  // namespace

bool cia402_operation_enabled(uint16_t statusword) {
  return (statusword & 0x006fU) == 0x0027U;
}

bool joint_error_code_allows_motion(
    uint32_t error_code,
    const std::vector<uint32_t>& non_blocking_error_codes) {
  return error_code == 0U ||
      std::find(
          non_blocking_error_codes.begin(), non_blocking_error_codes.end(),
          error_code) != non_blocking_error_codes.end();
}

bool robot_mode_allowed(const std::string& actual_mode, const std::string& required_mode) {
  return !required_mode.empty() && actual_mode == required_mode;
}

JointSetEvidence parse_joint_info_json(
    const std::string& json, const std::vector<int>& required_joint_ids,
    const std::vector<uint32_t>& non_blocking_error_codes) {
  JointSetEvidence output;
  if (!balanced_json(json) || required_joint_ids.empty()) {
    output.reason = required_joint_ids.empty() ? "required_joint_ids_empty" : "malformed_joint_json";
    return output;
  }
  std::map<int, JointEvidence> by_id;
  // Bound joint-log evidence carries the original file mtime. Keep it
  // independent from callback receipt time so a republished old log cannot
  // refresh the slow fault/statusword channel.
  extract_integer(json, "source_stamp_ns", &output.source_stamp_ns);
  for (const auto& object : json_objects(json)) {
    int64_t id = -1;
    int64_t error = -1;
    int64_t statusword = -1;
    if (!extract_any_integer(object, {"id", "joint_id", "index"}, &id) ||
        !extract_any_integer(object, {"error_code", "error"}, &error) ||
        !extract_any_integer(object, {"statusword", "status_word"}, &statusword)) {
      continue;
    }
    if (id < 0 || id > 65535 || error < 0 || error > 0xffffffffLL ||
        statusword < 0 || statusword > 0xffff) {
      output.reason = "joint_field_out_of_range";
      return output;
    }
    by_id[static_cast<int>(id)] = {
        static_cast<int>(id), static_cast<uint32_t>(error), static_cast<uint16_t>(statusword)};
  }
  output.parsed = !by_id.empty();
  output.complete = output.parsed;
  output.error_free = output.parsed;
  output.operation_enabled = output.parsed;
  std::set<int> required(required_joint_ids.begin(), required_joint_ids.end());
  if (required.size() != required_joint_ids.size()) {
    output.parsed = false;
    output.complete = false;
    output.error_free = false;
    output.operation_enabled = false;
    output.reason = "duplicate_required_joint_id";
    return output;
  }
  for (const int id : required) {
    const auto iterator = by_id.find(id);
    if (iterator == by_id.end()) {
      output.complete = false;
      output.error_free = false;
      output.operation_enabled = false;
      continue;
    }
    output.joints.push_back(iterator->second);
    output.error_free = output.error_free && joint_error_code_allows_motion(
        iterator->second.error_code, non_blocking_error_codes);
    output.operation_enabled = output.operation_enabled &&
                               cia402_operation_enabled(iterator->second.statusword);
  }
  if (!output.parsed) output.reason = "no_joint_records";
  else if (!output.complete) output.reason = "required_joint_missing";
  else if (!output.error_free) output.reason = "leg_joint_error";
  else if (!output.operation_enabled) output.reason = "leg_joint_not_operation_enabled";
  else {
    const bool nonzero_code_accepted = std::any_of(
        output.joints.begin(), output.joints.end(),
        [](const JointEvidence& joint) { return joint.error_code != 0U; });
    output.reason = nonzero_code_accepted ?
        "ok_non_blocking_leg_error_code" : "ok";
  }
  return output;
}

MotionStateEvidence parse_motion_state(
    const std::string& text,
    const std::string& required_state_token,
    const std::string& navigation_control_token,
    const std::string& remote_control_exit_token,
    const std::string& allowed_running_state_token) {
  MotionStateEvidence output;
  std::string stand_state;
  std::string walking_state;
  std::string navigation_mode;
  std::string remote_control_state;
  if (text.empty() ||
      !split_contract_token(required_state_token, "State", &stand_state) ||
      !split_contract_token(allowed_running_state_token, "State", &walking_state) ||
      !split_contract_token(navigation_control_token, "Mode", &navigation_mode) ||
      !split_contract_token(
          remote_control_exit_token, "Remote Control State", &remote_control_state) ||
      stand_state == walking_state) {
    output.reason = "invalid_motion_state_contract";
    return output;
  }
  std::string actual_mode;
  std::string actual_remote_control_state;
  output.parsed = extract_exact_field(text, "State", &output.state_name) &&
      extract_exact_field(text, "Mode", &actual_mode) &&
      extract_exact_field(text, "Remote Control State", &actual_remote_control_state);
  output.stand_state_active = output.parsed && output.state_name == stand_state;
  output.walking_state_active = output.parsed && output.state_name == walking_state;
  output.state_ready = output.stand_state_active || output.walking_state_active;
  output.navigation_control_active = output.parsed && actual_mode == navigation_mode;
  output.remote_control_exited =
      output.parsed && actual_remote_control_state == remote_control_state;
  if (!output.parsed) output.reason = "motion_state_malformed";
  else if (!output.state_ready) output.reason = "unsupported_motion_state:" + output.state_name;
  else if (!output.navigation_control_active) output.reason = "navigation_control_not_active";
  else if (!output.remote_control_exited) output.reason = "remote_control_not_exited";
  else output.reason = "ok";
  return output;
}

MotionStateDecision MotionStateGate::evaluate(
    const MotionStateEvidence& evidence, bool evidence_fresh,
    bool platform_prerequisites_safe) {
  const auto reject = [this](const std::string& reason) {
    reset();
    return MotionStateDecision{false, false, false, reason};
  };
  if (!evidence_fresh) return reject("motion_state_stale");
  if (!platform_prerequisites_safe) {
    return reject("platform_prerequisite_not_ready");
  }
  if (!evidence.parsed) return reject(evidence.reason);
  if (!evidence.state_ready) return reject(evidence.reason);
  if (!evidence.navigation_control_active) {
    return reject("navigation_control_not_active");
  }
  if (!evidence.remote_control_exited) return reject("remote_control_not_exited");

  if (evidence.stand_state_active) {
    phase_ = Phase::kArmedStand;
    return {true, true, false, "stand_ready"};
  }
  if (evidence.walking_state_active) {
    if (phase_ == Phase::kWaitingForStand) {
      return reject("walking_without_prestart_stand");
    }
    phase_ = Phase::kWalking;
    return {true, true, true, "walking_after_stand"};
  }
  return reject("unsupported_motion_state:" + evidence.state_name);
}

void MotionStateGate::reset() {
  phase_ = Phase::kWaitingForStand;
}

EthercatEvidence parse_ethercat_text(const std::string& text, int expected_wkc) {
  EthercatEvidence output;
  if (expected_wkc <= 0 || text.empty()) {
    output.reason = "invalid_ethercat_input";
    return output;
  }
  output.master_active = std::regex_search(
      text, std::regex("Active\\s*:\\s*yes", std::regex::icase));
  output.slave_op = std::regex_search(
      text, std::regex("(^|[\\s:+])OP([\\s,;]|$)", std::regex::icase));
  std::smatch match;
  const std::regex wkc_pattern("(?:WKC|WorkingCounter)[^0-9]*([0-9]+)\\s*/\\s*([0-9]+)",
                               std::regex::icase);
  if (std::regex_search(text, match, wkc_pattern)) {
    output.working_counter = std::stoi(match[1].str());
    output.expected_working_counter = std::stoi(match[2].str());
  }
  output.parsed = output.working_counter >= 0 && output.expected_working_counter > 0;
  output.wkc_normal = output.parsed && output.working_counter == expected_wkc &&
                      output.expected_working_counter == expected_wkc;
  if (!output.master_active) output.reason = "ethercat_master_not_active";
  else if (!output.slave_op) output.reason = "ethercat_slave_not_op";
  else if (!output.parsed) output.reason = "ethercat_wkc_parse_failed";
  else if (!output.wkc_normal) output.reason = "ethercat_wkc_mismatch";
  else output.reason = "ok";
  return output;
}

}  // namespace sunhorizon_navigation_core
