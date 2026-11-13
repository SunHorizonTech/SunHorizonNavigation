#include "sunhorizon_platform_adapter_casbot02/joint_log_evidence.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>

namespace sunhorizon_platform_adapter_casbot02 {

namespace {

int64_t source_stamp_ns(const struct stat& metadata) {
#if defined(__APPLE__)
  return static_cast<int64_t>(metadata.st_mtimespec.tv_sec) * 1000000000LL +
      static_cast<int64_t>(metadata.st_mtimespec.tv_nsec);
#else
  return static_cast<int64_t>(metadata.st_mtim.tv_sec) * 1000000000LL +
      static_cast<int64_t>(metadata.st_mtim.tv_nsec);
#endif
}

bool same_file_snapshot(const struct stat& left, const struct stat& right) {
  return left.st_dev == right.st_dev && left.st_ino == right.st_ino &&
      left.st_mode == right.st_mode && left.st_nlink == right.st_nlink &&
      left.st_size == right.st_size && source_stamp_ns(left) == source_stamp_ns(right);
}

std::string json_string(const std::string& input) {
  std::ostringstream output;
  output << '"';
  for (const unsigned char character : input) {
    switch (character) {
      case '"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\b': output << "\\b"; break;
      case '\f': output << "\\f"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (character < 0x20U) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                 << static_cast<unsigned int>(character) << std::dec;
        } else {
          output << static_cast<char>(character);
        }
    }
  }
  output << '"';
  return output.str();
}

std::string strip_ansi_csi_sequences(const std::string& input) {
  std::string output;
  output.reserve(input.size());
  for (std::size_t index = 0; index < input.size();) {
    if (input[index] != '\x1b' || index + 1U >= input.size() ||
        input[index + 1U] != '[') {
      output.push_back(input[index++]);
      continue;
    }

    std::size_t end = index + 2U;
    while (end < input.size() &&
           static_cast<unsigned char>(input[end]) >= 0x20U &&
           static_cast<unsigned char>(input[end]) <= 0x3fU) {
      ++end;
    }
    if (end < input.size() &&
        static_cast<unsigned char>(input[end]) >= 0x40U &&
        static_cast<unsigned char>(input[end]) <= 0x7eU) {
      index = end + 1U;
      continue;
    }

    output.push_back(input[index++]);
  }
  return output;
}

}  // namespace

JointLogParseResult parse_latest_ethercat_joint_block(
    const std::string& text, const std::vector<int>& required_joint_ids,
    const std::vector<uint32_t>& non_blocking_error_codes) {
  if (required_joint_ids.empty()) {
    JointLogParseResult output;
    output.evidence.reason = "required_joint_ids_empty";
    return output;
  }
  const std::set<int> required(required_joint_ids.begin(), required_joint_ids.end());
  if (required.size() != required_joint_ids.size()) {
    JointLogParseResult output;
    output.evidence.reason = "duplicate_required_joint_id";
    return output;
  }

  const std::regex line_pattern(
      R"(^\s*([0-9]+):\s*状态字=0x([0-9A-Fa-f]+).*错误=0x([0-9A-Fa-f]+))",
      std::regex::ECMAScript);
  const auto parse_block = [&](const std::string& block) {
    JointLogParseResult output;
    std::map<int, sunhorizon_navigation_core::JointEvidence> by_id;
    std::set<int> seen_relevant_ids;
    std::istringstream lines(block);
    std::string line;
    while (std::getline(lines, line)) {
      line = strip_ansi_csi_sequences(line);
      std::smatch match;
      if (!std::regex_search(line, match, line_pattern)) continue;
      try {
        const auto id = std::stoi(match[1].str());
        if (required.count(id) == 0U) continue;
        if (!seen_relevant_ids.insert(id).second) {
          output.evidence.reason = "duplicate_joint_record";
          return output;
        }
        const auto statusword = std::stoul(match[2].str(), nullptr, 16);
        const auto error_code = std::stoul(match[3].str(), nullptr, 16);
        if (statusword > 0xffffU || error_code > 0xffffffffU) {
          output.evidence.reason = "joint_field_out_of_range";
          return output;
        }
        by_id[id] = {id, static_cast<uint32_t>(error_code),
                     static_cast<uint16_t>(statusword)};
      } catch (...) {
        output.evidence.reason = "joint_field_parse_failed";
        return output;
      }
    }

    output.evidence.parsed = !by_id.empty();
    output.evidence.complete = by_id.size() == required.size();
    output.evidence.error_free = output.evidence.complete;
    output.evidence.operation_enabled = output.evidence.complete;
    std::ostringstream json;
    json << '[';
    bool first = true;
    for (const int id : required) {
      const auto iterator = by_id.find(id);
      if (iterator == by_id.end()) continue;
      const auto& joint = iterator->second;
      output.evidence.joints.push_back(joint);
      output.evidence.error_free = output.evidence.error_free &&
          sunhorizon_navigation_core::joint_error_code_allows_motion(
              joint.error_code, non_blocking_error_codes);
      output.evidence.operation_enabled = output.evidence.operation_enabled &&
          sunhorizon_navigation_core::cia402_operation_enabled(joint.statusword);
      if (!first) json << ',';
      first = false;
      json << "{\"index\":" << joint.id << ",\"error_code\":"
           << joint.error_code << ",\"statusword\":" << joint.statusword << '}';
    }
    json << ']';
    output.json = json.str();

    if (!output.evidence.parsed) output.evidence.reason = "no_joint_records";
    else if (!output.evidence.complete) output.evidence.reason = "required_joint_missing";
    else if (!output.evidence.error_free) output.evidence.reason = "leg_joint_error";
    else if (!output.evidence.operation_enabled) {
      output.evidence.reason = "leg_joint_not_operation_enabled";
    } else {
      const bool nonzero_code_accepted = std::any_of(
          output.evidence.joints.begin(), output.evidence.joints.end(),
          [](const sunhorizon_navigation_core::JointEvidence& joint) {
            return joint.error_code != 0U;
          });
      output.evidence.reason = nonzero_code_accepted ?
          "ok_non_blocking_leg_error_code" : "ok";
    }
    return output;
  };

  // The vendor writes one large status block incrementally. At 10 Hz the reader
  // often observes the newest block before all 12 leg records have
  // arrived. Prefer that block once complete; otherwise fall back to the previous
  // complete block instead of producing a periodic false stale/fault pulse.
  const auto newest_block_start = text.rfind("状态信息(ROH)");
  auto block_start = newest_block_start;
  JointLogParseResult newest_failure;
  bool inspected = false;
  while (block_start != std::string::npos) {
    auto block_end = text.find("控制命令(ROH)", block_start);
    if (block_end == std::string::npos) block_end = text.size();
    auto parsed = parse_block(text.substr(block_start, block_end - block_start));
    if (!inspected) {
      newest_failure = parsed;
      inspected = true;
      // Never let fallback hide a newly observed leg fault or disabled drive.
      // A partial block is tolerated only while every record already written
      // in that block is itself safe.
      for (const auto& joint : parsed.evidence.joints) {
        if (!sunhorizon_navigation_core::joint_error_code_allows_motion(
                joint.error_code, non_blocking_error_codes)) {
          parsed.evidence.reason = "leg_joint_error";
          return parsed;
        }
        if (!sunhorizon_navigation_core::cia402_operation_enabled(
                joint.statusword)) {
          parsed.evidence.reason = "leg_joint_not_operation_enabled";
          return parsed;
        }
      }
    }
    if (parsed.evidence.complete) return parsed;
    if (block_start == 0U) break;
    block_start = text.rfind("状态信息(ROH)", block_start - 1U);
  }
  if (inspected) return newest_failure;
  newest_failure.evidence.reason = "joint_status_block_missing";
  return newest_failure;
}

std::string read_file_tail(const std::string& path, std::size_t maximum_bytes) {
  return read_file_tail_snapshot(path, maximum_bytes).text;
}

JointLogSnapshot read_file_tail_snapshot(
    const std::string& path, std::size_t maximum_bytes) {
  if (path.empty() || maximum_bytes == 0U) {
    throw std::invalid_argument("joint log path and tail size must be nonempty");
  }
  struct stat before {};
  if (::lstat(path.c_str(), &before) != 0 || !S_ISREG(before.st_mode) ||
      before.st_nlink != 1 || source_stamp_ns(before) <= 0) {
    throw std::runtime_error("vendor joint log must be one regular non-symlink file");
  }
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("cannot open vendor joint log");
  stream.seekg(0, std::ios::end);
  const auto end = stream.tellg();
  if (end < 0) throw std::runtime_error("cannot size vendor joint log");
  const auto size = static_cast<std::size_t>(end);
  const auto begin = size > maximum_bytes ? size - maximum_bytes : 0U;
  stream.seekg(static_cast<std::streamoff>(begin), std::ios::beg);
  std::string output(size - begin, '\0');
  stream.read(output.data(), static_cast<std::streamsize>(output.size()));
  output.resize(static_cast<std::size_t>(stream.gcount()));
  struct stat after {};
  if (::lstat(path.c_str(), &after) != 0 || !same_file_snapshot(before, after)) {
    throw std::runtime_error("vendor joint log changed while reading");
  }
  return {output, source_stamp_ns(after)};
}

JointStateSampleValidation validate_joint_state_sample(
    const std::vector<std::string>& names,
    const std::vector<double>& positions_rad,
    const std::vector<std::string>& expected_leg_joint_names,
    const std::string& waist_joint_name,
    int64_t source_stamp_ns_value,
    int64_t now_ns,
    int64_t maximum_age_ns,
    bool source_owner_unique) {
  JointStateSampleValidation output;
  if (!source_owner_unique) {
    output.reason = "joint_state_proof_owner_not_unique";
    return output;
  }
  if (expected_leg_joint_names.size() != 12U || waist_joint_name.empty()) {
    output.reason = "joint_state_expected_mapping_invalid";
    return output;
  }
  std::vector<std::string> expected = expected_leg_joint_names;
  expected.push_back(waist_joint_name);
  if (names != expected || positions_rad.size() != expected.size() ||
      std::set<std::string>(names.begin(), names.end()).size() != names.size()) {
    output.reason = "joint_state_names_or_array_length_invalid";
    return output;
  }
  if (source_stamp_ns_value <= 0 || maximum_age_ns <= 0 ||
      now_ns < source_stamp_ns_value || now_ns - source_stamp_ns_value > maximum_age_ns) {
    output.reason = "joint_state_source_stamp_stale_or_future";
    return output;
  }
  for (const auto position : positions_rad) {
    if (!std::isfinite(position)) {
      output.reason = "joint_state_position_not_finite";
      return output;
    }
  }
  output.waist_position_rad = positions_rad.back();
  output.valid = true;
  output.reason = "ok";
  return output;
}

std::string build_bound_joint_info_json(
    const std::string& joints_json,
    const std::vector<std::string>& ordered_joint_names,
    int64_t source_stamp_ns_value,
    const std::string& mapping_receipt_path,
    const std::string& mapping_receipt_sha256,
    const std::string& mapping_evidence_sha256) {
  if (joints_json.size() < 2U || joints_json.front() != '[' ||
      joints_json.back() != ']' || ordered_joint_names.size() != 12U ||
      source_stamp_ns_value <= 0 || mapping_receipt_path.empty() ||
      mapping_receipt_sha256.size() != 64U || mapping_evidence_sha256.size() != 64U) {
    throw std::invalid_argument("bound joint info inputs invalid");
  }
  std::ostringstream output;
  output << "{\"schema_version\":2,"
         << "\"kind\":\"robot_a_platform_joint_fault_attribution\","
         << "\"joints\":" << joints_json << ','
         << "\"fault_source\":{"
         << "\"format\":\"roh_ethercat_station_log\","
         << "\"source_stamp\":\"filesystem_mtime_ns\","
         << "\"source_stamp_ns\":" << source_stamp_ns_value << ','
         << "\"used_fields\":[\"statusword\",\"error_code\"],"
         << "\"ignored_fields\":[\"position\"]},"
         << "\"attribution\":{\"receipt_path\":"
         << json_string(mapping_receipt_path)
         << ",\"receipt_sha256\":" << json_string(mapping_receipt_sha256)
         << ",\"evidence_sha256\":" << json_string(mapping_evidence_sha256)
         << ",\"ordered_joint_names\":[";
  for (std::size_t index = 0; index < ordered_joint_names.size(); ++index) {
    if (index != 0U) output << ',';
    output << json_string(ordered_joint_names[index]);
  }
  output << "]},\"motion_authorized\":false}";
  return output.str();
}

}  // namespace sunhorizon_platform_adapter_casbot02
