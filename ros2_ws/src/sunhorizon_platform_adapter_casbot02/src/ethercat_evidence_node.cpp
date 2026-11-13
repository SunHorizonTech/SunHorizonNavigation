#include <array>
#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sunhorizon_navigation_core/platform_evidence.hpp>

namespace {

struct CommandResult {
  bool success{false};
  std::string output;
};

CommandResult run_read_only_command(const char* command) {
  CommandResult result;
  std::array<char, 512> buffer{};
  FILE* stream = popen(command, "r");
  if (stream == nullptr) return result;
  while (fgets(buffer.data(), static_cast<int>(buffer.size()), stream) != nullptr) {
    result.output += buffer.data();
    if (result.output.size() > 65536U) break;
  }
  result.success = pclose(stream) == 0;
  return result;
}

class EthercatEvidenceNode final : public rclcpp::Node {
 public:
  EthercatEvidenceNode() : Node("sunhorizon_ethercat_evidence") {
    const double poll_rate_hz = declare_parameter("poll_rate_hz", 2.0);
    expected_wkc_ = declare_parameter("expected_wkc", 2);
    if (poll_rate_hz <= 0.0 || poll_rate_hz > 5.0 || expected_wkc_ <= 0) {
      throw std::runtime_error("invalid EtherCAT evidence monitor parameters");
    }
    op_pub_ = create_publisher<std_msgs::msg::Bool>(
        declare_parameter(
            "ethercat_op_proof_topic", "/sunhorizon/platform/proof/ethercat_op"), 10);
    wkc_pub_ = create_publisher<std_msgs::msg::Bool>(
        declare_parameter(
            "wkc_normal_proof_topic", "/sunhorizon/platform/proof/wkc_normal"), 10);
    reason_pub_ = create_publisher<std_msgs::msg::String>(
        declare_parameter(
            "evidence_reason_topic", "/sunhorizon/platform/proof/ethercat_reason"), 10);
    const auto period = std::chrono::duration<double>(1.0 / poll_rate_hz);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&EthercatEvidenceNode::poll, this));
  }

 private:
  void poll() {
    // Fixed absolute commands are intentionally not configurable: this node is
    // evidence-only and must never expose an arbitrary privileged shell hook.
    // Robot A grants the service account read/write access to /dev/EtherCAT0
    // through the device's `sudo` group.  Invoke the read-only CLI directly so
    // this monitor remains compatible with systemd NoNewPrivileges=true.
    const auto master = run_read_only_command(
        "/usr/bin/timeout 0.3s /usr/bin/ethercat master 2>&1");
    const auto slaves = run_read_only_command(
        "/usr/bin/timeout 0.3s /usr/bin/ethercat slaves 2>&1");
    const auto domains = run_read_only_command(
        "/usr/bin/timeout 0.3s /usr/bin/ethercat domains 2>&1");
    const auto evidence = sunhorizon_navigation_core::parse_ethercat_text(
        master.output + '\n' + slaves.output + '\n' + domains.output, expected_wkc_);
    const bool commands_ok = master.success && slaves.success && domains.success;
    std_msgs::msg::Bool op;
    op.data = commands_ok && evidence.master_active && evidence.slave_op;
    op_pub_->publish(op);
    std_msgs::msg::Bool wkc;
    wkc.data = commands_ok && evidence.wkc_normal;
    wkc_pub_->publish(wkc);
    std_msgs::msg::String reason;
    reason.data = commands_ok ? evidence.reason : "ethercat_read_only_command_failed";
    reason_pub_->publish(reason);
  }

  int expected_wkc_{2};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr op_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wkc_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reason_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EthercatEvidenceNode>());
  rclcpp::shutdown();
  return 0;
}
