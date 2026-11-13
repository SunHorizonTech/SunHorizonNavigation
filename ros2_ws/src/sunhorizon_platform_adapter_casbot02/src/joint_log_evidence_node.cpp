#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <regex>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "sunhorizon_platform_adapter_casbot02/joint_log_evidence.hpp"

namespace {

class JointLogEvidenceNode final : public rclcpp::Node {
 public:
  JointLogEvidenceNode() : Node("sunhorizon_joint_log_evidence") {
    log_path_ = declare_parameter("vendor_joint_log_path", "");
    const auto poll_rate_hz = declare_parameter("poll_rate_hz", 2.0);
    maximum_age_sec_ = declare_parameter("maximum_log_age_sec", 1.5);
    const auto maximum_tail_bytes = declare_parameter<int64_t>(
        "maximum_tail_bytes", 262144);
    const auto topic = declare_parameter(
        "joint_evidence_topic", "/sunhorizon/platform/proof/joint_info");
    const auto reason_topic = declare_parameter(
        "joint_evidence_reason_topic", "/sunhorizon/platform/proof/joint_reason");
    const auto ids = declare_parameter<std::vector<int64_t>>(
        "leg_joint_ids", std::vector<int64_t>{
            0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11});
    const auto non_blocking_codes = declare_parameter<std::vector<int64_t>>(
        "non_blocking_leg_error_codes", std::vector<int64_t>{});
    leg_joint_names_ = declare_parameter<std::vector<std::string>>(
        "leg_joint_names", std::vector<std::string>{});
    joint_name_mapping_verified_ = declare_parameter(
        "leg_joint_name_mapping_verified", false);
    joint_name_mapping_receipt_path_ = declare_parameter(
        "leg_joint_name_mapping_receipt_path", "");
    joint_name_mapping_receipt_sha256_ = declare_parameter(
        "leg_joint_name_mapping_receipt_sha256", "");
    joint_name_mapping_evidence_sha256_ = declare_parameter(
        "leg_joint_name_mapping_evidence_sha256", "");
    if (log_path_.empty() || !std::filesystem::path(log_path_).is_absolute() ||
        !std::isfinite(poll_rate_hz) || poll_rate_hz <= 0.0 || poll_rate_hz > 10.0 ||
        !std::isfinite(maximum_age_sec_) || maximum_age_sec_ <= 0.0 ||
        maximum_tail_bytes < 65536 || maximum_tail_bytes > 1048576 || topic.empty()) {
      throw std::runtime_error("invalid vendor joint log evidence parameters");
    }
    maximum_tail_bytes_ = static_cast<std::size_t>(maximum_tail_bytes);
    leg_joint_ids_.reserve(ids.size());
    for (const auto id : ids) {
      if (id < 0 || id > 65535) {
        throw std::runtime_error("leg joint id outside supported range");
      }
      leg_joint_ids_.push_back(static_cast<int>(id));
    }
    non_blocking_leg_error_codes_.reserve(non_blocking_codes.size());
    for (const auto code : non_blocking_codes) {
      if (code <= 0 || static_cast<uint64_t>(code) > 0xffffffffULL) {
        throw std::runtime_error("non-blocking leg error code outside supported range");
      }
      non_blocking_leg_error_codes_.push_back(static_cast<uint32_t>(code));
    }
    if (std::set<uint32_t>(
            non_blocking_leg_error_codes_.begin(),
            non_blocking_leg_error_codes_.end()).size() !=
        non_blocking_leg_error_codes_.size()) {
      throw std::runtime_error("non-blocking leg error codes must be unique");
    }
    const std::set<int> unique_ids(leg_joint_ids_.begin(), leg_joint_ids_.end());
    const std::set<std::string> unique_names(
        leg_joint_names_.begin(), leg_joint_names_.end());
    const std::regex sha256_pattern("[0-9a-f]{64}");
    if (leg_joint_ids_.size() != 12U || unique_ids.size() != 12U) {
      throw std::runtime_error("exactly 12 unique leg joint ids are required");
    }
    for (std::size_t index = 0; index < leg_joint_ids_.size(); ++index) {
      if (leg_joint_ids_[index] != static_cast<int>(index)) {
        throw std::runtime_error("leg joint ids must be ordered exactly 0 through 11");
      }
    }
    if (joint_name_mapping_verified_) {
      if (leg_joint_names_.size() != 12U || unique_names.size() != 12U ||
          unique_names.count("") != 0U ||
          joint_name_mapping_receipt_path_.empty() ||
          !std::filesystem::path(joint_name_mapping_receipt_path_).is_absolute() ||
          std::filesystem::is_symlink(joint_name_mapping_receipt_path_) ||
          !std::filesystem::is_regular_file(joint_name_mapping_receipt_path_) ||
          !std::regex_match(joint_name_mapping_receipt_sha256_, sha256_pattern) ||
          !std::regex_match(joint_name_mapping_evidence_sha256_, sha256_pattern)) {
        throw std::runtime_error("verified leg joint mapping parameters are invalid");
      }
    } else if (!leg_joint_names_.empty() ||
               !joint_name_mapping_receipt_path_.empty() ||
               !joint_name_mapping_receipt_sha256_.empty() ||
               !joint_name_mapping_evidence_sha256_.empty()) {
      throw std::runtime_error("unverified leg joint mapping must remain empty");
    }
    evidence_publisher_ =
        create_publisher<diagnostic_msgs::msg::KeyValue>(topic, 10);
    reason_publisher_ = create_publisher<std_msgs::msg::String>(reason_topic, 10);
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / poll_rate_hz),
        std::bind(&JointLogEvidenceNode::poll, this));
  }

 private:
  void publish_failure(const std::string& reason) {
    diagnostic_msgs::msg::KeyValue evidence;
    evidence.key = "joint_info";
    evidence.value = "";
    evidence_publisher_->publish(evidence);
    std_msgs::msg::String detail;
    detail.data = reason;
    reason_publisher_->publish(detail);
  }

  void poll() {
    try {
      const auto snapshot =
          sunhorizon_platform_adapter_casbot02::read_file_tail_snapshot(
              log_path_, maximum_tail_bytes_);
      const auto parsed =
          sunhorizon_platform_adapter_casbot02::parse_latest_ethercat_joint_block(
              snapshot.text, leg_joint_ids_, non_blocking_leg_error_codes_);
      if (!parsed.evidence.parsed || !parsed.evidence.complete) {
        publish_failure(parsed.evidence.reason);
        return;
      }
      const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
      const auto maximum_age_ns = static_cast<int64_t>(maximum_age_sec_ * 1e9);
      if (!joint_name_mapping_verified_ || snapshot.source_stamp_ns <= 0 ||
          now_ns < snapshot.source_stamp_ns ||
          now_ns - snapshot.source_stamp_ns > maximum_age_ns) {
        publish_failure(!joint_name_mapping_verified_ ?
            "leg_joint_name_mapping_unverified" : "joint_fault_source_stale_or_future");
        return;
      }
      diagnostic_msgs::msg::KeyValue evidence;
      evidence.key = "joint_info";
      evidence.value = sunhorizon_platform_adapter_casbot02::build_bound_joint_info_json(
          parsed.json, leg_joint_names_, snapshot.source_stamp_ns,
          joint_name_mapping_receipt_path_, joint_name_mapping_receipt_sha256_,
          joint_name_mapping_evidence_sha256_);
      evidence_publisher_->publish(evidence);
      std_msgs::msg::String detail;
      detail.data = parsed.evidence.reason;
      reason_publisher_->publish(detail);
    } catch (const std::exception& error) {
      publish_failure(std::string("vendor_joint_log_read_failed:") + error.what());
    }
  }

  std::string log_path_;
  double maximum_age_sec_{1.5};
  std::size_t maximum_tail_bytes_{262144U};
  std::vector<int> leg_joint_ids_;
  std::vector<uint32_t> non_blocking_leg_error_codes_;
  std::vector<std::string> leg_joint_names_;
  bool joint_name_mapping_verified_{false};
  std::string joint_name_mapping_receipt_path_;
  std::string joint_name_mapping_receipt_sha256_;
  std::string joint_name_mapping_evidence_sha256_;
  rclcpp::Publisher<diagnostic_msgs::msg::KeyValue>::SharedPtr evidence_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reason_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointLogEvidenceNode>());
  rclcpp::shutdown();
  return 0;
}
