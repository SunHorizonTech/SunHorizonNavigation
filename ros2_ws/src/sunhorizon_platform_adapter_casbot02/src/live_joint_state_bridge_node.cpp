#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <memory>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include "sunhorizon_platform_adapter_casbot02/live_joint_state_bridge.hpp"

namespace {

std::string gid_bytes_hex(const uint8_t* data, const std::size_t size) {
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (std::size_t index = 0; index < size; ++index) {
    output << std::setw(2) << static_cast<unsigned int>(data[index]);
  }
  return output.str();
}

std::string endpoint_gid_hex(const rmw_gid_t& gid) {
  return gid_bytes_hex(gid.data, RMW_GID_STORAGE_SIZE);
}

template <std::size_t Size>
std::string endpoint_gid_hex(const std::array<uint8_t, Size>& gid) {
  return gid_bytes_hex(gid.data(), gid.size());
}

class LiveJointStateBridgeNode final : public rclcpp::Node {
 public:
  LiveJointStateBridgeNode() : Node("sunhorizon_live_joint_state_bridge") {
    const bool contract_accepted = declare_parameter(
        "source_contract_accepted", false);
    source_topic_ = declare_parameter("source_topic", "/joint_states");
    const auto source_type = declare_parameter(
        "source_type", "sensor_msgs/msg/JointState");
    source_owner_name_ = declare_parameter("source_owner_name", "");
    source_owner_namespace_ = declare_parameter("source_owner_namespace", "");
    const auto runtime_binding = declare_parameter("runtime_binding", "");
    const auto output_topic = declare_parameter(
        "output_topic", "/sunhorizon/platform/proof/live_joint_states");
    const auto reason_topic = declare_parameter(
        "reason_topic", "/sunhorizon/platform/proof/live_joint_state_reason");
    const auto expected_source_names = declare_parameter<std::vector<std::string>>(
        "expected_source_joint_names", std::vector<std::string>{});
    const auto selected_source_names = declare_parameter<std::vector<std::string>>(
        "selected_source_joint_names", std::vector<std::string>{});
    const auto selected_source_indices_parameter = declare_parameter<std::vector<int64_t>>(
        "selected_source_joint_indices", std::vector<int64_t>{});
    const auto output_names = declare_parameter<std::vector<std::string>>(
        "output_joint_names", std::vector<std::string>{});
    const auto position_unit = declare_parameter("position_unit", "");
    const auto value_transform = declare_parameter("value_transform", "");
    const double maximum_age_sec = declare_parameter(
        "maximum_source_age_sec", 0.25);
    const double future_tolerance_sec = declare_parameter(
        "source_future_tolerance_sec", 0.05);
    const double owner_discovery_grace_sec = declare_parameter(
        "source_owner_discovery_grace_sec", 12.0);
    const auto receipt_path = declare_parameter("source_receipt_path", "");
    const auto receipt_sha256 = declare_parameter("source_receipt_sha256", "");
    const auto evidence_sha256 = declare_parameter("source_evidence_sha256", "");
    const std::regex sha256_pattern("[0-9a-f]{64}");
    if (!contract_accepted || source_topic_ != "/joint_states" ||
        source_type != "sensor_msgs/msg/JointState" ||
        source_owner_name_ != "hl_orin_core_msg" || source_owner_namespace_ != "/" ||
        runtime_binding != "unique_current_endpoint_name_type_and_message_gid" ||
        output_topic != "/sunhorizon/platform/proof/live_joint_states" ||
        position_unit != "rad" || value_transform != "identity" ||
        !std::isfinite(maximum_age_sec) || maximum_age_sec <= 0.0 ||
        maximum_age_sec > 0.25 || !std::isfinite(future_tolerance_sec) ||
        future_tolerance_sec < 0.0 || future_tolerance_sec > 0.05 ||
        !std::isfinite(owner_discovery_grace_sec) ||
        owner_discovery_grace_sec <= 0.0 || owner_discovery_grace_sec > 12.0 ||
        receipt_path.empty() || !std::filesystem::path(receipt_path).is_absolute() ||
        !std::regex_match(receipt_sha256, sha256_pattern) ||
        !std::regex_match(evidence_sha256, sha256_pattern)) {
      throw std::runtime_error("live joint state source contract invalid");
    }
    std::vector<std::size_t> selected_source_indices;
    selected_source_indices.reserve(selected_source_indices_parameter.size());
    for (const auto index : selected_source_indices_parameter) {
      if (index < 0 || index >= 49) {
        throw std::runtime_error("live joint state selected index invalid");
      }
      selected_source_indices.push_back(static_cast<std::size_t>(index));
    }
    gate_ = std::make_unique<
        sunhorizon_platform_adapter_casbot02::LiveJointStateBridgeGate>(
        expected_source_names, selected_source_names, selected_source_indices,
        output_names, position_unit, value_transform,
        static_cast<int64_t>(maximum_age_sec * 1e9),
        static_cast<int64_t>(future_tolerance_sec * 1e9));
    discovery_gate_ = std::make_unique<
        sunhorizon_platform_adapter_casbot02::LiveJointStateDiscoveryGate>(
        static_cast<int64_t>(owner_discovery_grace_sec * 1e9));
    discovery_started_at_ = std::chrono::steady_clock::now();

    rclcpp::SensorDataQoS qos;
    qos.keep_last(1);
    publisher_ = create_publisher<sensor_msgs::msg::JointState>(output_topic, qos);
    reason_publisher_ = create_publisher<std_msgs::msg::String>(reason_topic, 10);
    subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
        source_topic_, qos,
        std::bind(
            &LiveJointStateBridgeNode::on_joint_state, this,
            std::placeholders::_1, std::placeholders::_2));
    discovery_timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&LiveJointStateBridgeNode::on_discovery_timer, this));
  }

 private:
  struct CurrentSourceOwner {
    sunhorizon_platform_adapter_casbot02::LiveJointStateOwnerStatus status;
    std::string endpoint_gid;
  };

  CurrentSourceOwner source_owner_status() const {
    const auto endpoints = get_publishers_info_by_topic(source_topic_);
    const sunhorizon_platform_adapter_casbot02::LiveJointStatePublisherIdentity expected{
        source_owner_name_, source_owner_namespace_,
        "sensor_msgs/msg/JointState", ""};
    sunhorizon_platform_adapter_casbot02::LiveJointStatePublisherIdentity observed;
    if (endpoints.size() == 1U) {
      const auto& endpoint = endpoints.front();
      observed = {
          endpoint.node_name(), endpoint.node_namespace(), endpoint.topic_type(),
          endpoint_gid_hex(endpoint.endpoint_gid())};
    }
    return {
        sunhorizon_platform_adapter_casbot02::live_joint_source_owner_status(
            endpoints.size(), expected, observed),
        observed.endpoint_gid};
  }

  int64_t discovery_elapsed_ns() const {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - discovery_started_at_).count();
  }

  void report_fault_once(const std::string& fault_reason) {
    if (fault_reported_) return;
    fault_reported_ = true;
    RCLCPP_ERROR(get_logger(), "live joint state bridge latched: %s",
                 fault_reason.c_str());
    std_msgs::msg::String reason;
    reason.data = fault_reason;
    reason_publisher_->publish(reason);
  }

  bool source_discovery_ready(const CurrentSourceOwner& owner) {
    const auto decision = discovery_gate_->evaluate(
        owner.status, discovery_elapsed_ns());
    if (decision.accepted) {
      if (discovery_timeout_reported_) {
        RCLCPP_INFO(
            get_logger(),
            "live joint state owner recovered after discovery timeout");
        // on_joint_state() calls this path at the source rate. Clear the edge
        // marker after reporting recovery so a 200 Hz stream logs it once.
        discovery_timeout_reported_ = false;
      }
      if (discovery_timer_) discovery_timer_->cancel();
      return true;
    }
    if (decision.fault_latched) {
      report_fault_once(decision.reason);
    } else if (
        decision.reason == "live_joint_source_owner_discovery_timeout" &&
        !discovery_timeout_reported_) {
      discovery_timeout_reported_ = true;
      RCLCPP_WARN(
          get_logger(),
          "live joint state owner discovery timed out; continuing fail-closed discovery");
      std_msgs::msg::String reason;
      reason.data = decision.reason;
      reason_publisher_->publish(reason);
    }
    return false;
  }

  void on_discovery_timer() {
    (void)source_discovery_ready(source_owner_status());
  }

  void on_joint_state(
      const sensor_msgs::msg::JointState::ConstSharedPtr input,
      const rclcpp::MessageInfo& message_info) {
    const auto owner = source_owner_status();
    if (!source_discovery_ready(owner) ||
        owner.status != sunhorizon_platform_adapter_casbot02::
            LiveJointStateOwnerStatus::kMatched) {
      return;
    }
    const rclcpp::Time source_stamp(input->header.stamp, get_clock()->get_clock_type());
    const auto message_publisher_gid = endpoint_gid_hex(
        message_info.get_rmw_message_info().publisher_gid);
    const auto decision = gate_->evaluate({
        source_stamp.nanoseconds(), get_clock()->now().nanoseconds(),
        input->name, input->position, true,
        message_publisher_gid == owner.endpoint_gid});
    if (!decision.accepted) {
      if (decision.fault_latched) report_fault_once(decision.reason);
      return;
    }
    sensor_msgs::msg::JointState output;
    output.header = input->header;
    output.name = decision.names;
    output.position = decision.positions;
    publisher_->publish(output);
  }

  std::string source_topic_;
  std::string source_owner_name_;
  std::string source_owner_namespace_;
  std::chrono::steady_clock::time_point discovery_started_at_;
  bool fault_reported_{false};
  bool discovery_timeout_reported_{false};
  std::unique_ptr<
      sunhorizon_platform_adapter_casbot02::LiveJointStateDiscoveryGate>
      discovery_gate_;
  std::unique_ptr<sunhorizon_platform_adapter_casbot02::LiveJointStateBridgeGate> gate_;
  rclcpp::TimerBase::SharedPtr discovery_timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reason_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LiveJointStateBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
