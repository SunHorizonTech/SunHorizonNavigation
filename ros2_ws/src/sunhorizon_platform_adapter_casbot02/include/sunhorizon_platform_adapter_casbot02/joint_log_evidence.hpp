#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <sunhorizon_navigation_core/platform_evidence.hpp>

namespace sunhorizon_platform_adapter_casbot02 {

struct JointLogParseResult {
  sunhorizon_navigation_core::JointSetEvidence evidence;
  std::string json;
};

struct JointLogSnapshot {
  std::string text;
  int64_t source_stamp_ns{0};
};

struct JointStateSampleValidation {
  bool valid{false};
  std::string reason;
  double waist_position_rad{0.0};
};

JointLogParseResult parse_latest_ethercat_joint_block(
    const std::string& text, const std::vector<int>& required_joint_ids,
    const std::vector<uint32_t>& non_blocking_error_codes = {});

std::string read_file_tail(const std::string& path, std::size_t maximum_bytes);

JointLogSnapshot read_file_tail_snapshot(
    const std::string& path, std::size_t maximum_bytes);

JointStateSampleValidation validate_joint_state_sample(
    const std::vector<std::string>& names,
    const std::vector<double>& positions_rad,
    const std::vector<std::string>& expected_leg_joint_names,
    const std::string& waist_joint_name,
    int64_t source_stamp_ns,
    int64_t now_ns,
    int64_t maximum_age_ns,
    bool source_owner_unique);

std::string build_bound_joint_info_json(
    const std::string& joints_json,
    const std::vector<std::string>& ordered_joint_names,
    int64_t source_stamp_ns,
    const std::string& mapping_receipt_path,
    const std::string& mapping_receipt_sha256,
    const std::string& mapping_evidence_sha256);

}  // namespace sunhorizon_platform_adapter_casbot02
