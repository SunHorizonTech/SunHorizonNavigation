#include <chrono>
#include <cctype>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sunhorizon_navigation_interfaces/msg/platform_safety_state.hpp>

#include "sunhorizon_platform_adapter_casbot02/vendor_velocity_contract.hpp"

namespace {

using PlatformSafetyState = sunhorizon_navigation_interfaces::msg::PlatformSafetyState;
using sunhorizon_platform_adapter_casbot02::VendorVelocityCommand;
using sunhorizon_platform_adapter_casbot02::VendorVelocityGate;
using sunhorizon_platform_adapter_casbot02::VendorVelocityLimits;
using sunhorizon_platform_adapter_casbot02::kProjectFinalVelocityTopic;
using sunhorizon_platform_adapter_casbot02::resolved_vendor_topics_distinct;
using sunhorizon_platform_adapter_casbot02::vendor_output_topic_allowed;

constexpr char kSupportedMessageType[] = "geometry_msgs/msg/Twist";
constexpr char kSupportedSemantics[] = "ros_twist_base_link_flu";

bool is_lower_hex_sha256(const std::string& value) {
  if (value.size() != 64U) return false;
  for (const char character : value) {
    if (!std::isdigit(static_cast<unsigned char>(character)) &&
        !(character >= 'a' && character <= 'f')) {
      return false;
    }
  }
  return true;
}

VendorVelocityCommand from_message(const geometry_msgs::msg::Twist& input) {
  return {
      input.linear.x, input.linear.y, input.linear.z,
      input.angular.x, input.angular.y, input.angular.z};
}

geometry_msgs::msg::Twist to_message(const VendorVelocityCommand& input) {
  geometry_msgs::msg::Twist output;
  output.linear.x = input.linear_x;
  output.linear.y = input.linear_y;
  output.linear.z = input.linear_z;
  output.angular.x = input.angular_x;
  output.angular.y = input.angular_y;
  output.angular.z = input.angular_z;
  return output;
}

class VendorVelocityAdapter final : public rclcpp::Node {
 public:
  VendorVelocityAdapter()
      : Node("sunhorizon_vendor_velocity_adapter"), gate_(declare_limits()) {
    contract_accepted_ = declare_parameter("vendor_contract_accepted", false);
    input_topic_ = declare_parameter(
        "supervised_input_topic", kProjectFinalVelocityTopic);
    output_topic_ = declare_parameter("vendor_velocity_topic", "");
    const auto message_type = declare_parameter("vendor_velocity_message_type", "");
    const auto semantics = declare_parameter("vendor_velocity_semantics", "");
    const auto reliability = declare_parameter("vendor_qos_reliability", "");
    const auto durability = declare_parameter("vendor_qos_durability", "");
    const auto receipt_sha256 = declare_parameter("vendor_contract_receipt_sha256", "");
    platform_topic_ = declare_parameter(
        "platform_state_topic", "/sunhorizon/platform/safety_state");
    control_rate_hz_ = declare_parameter("control_rate_hz", 50.0);
    publisher_discovery_grace_sec_ = declare_parameter(
        "publisher_discovery_grace_sec", 2.0);
    vendor_watchdog_timeout_sec_ = declare_parameter(
        "vendor_watchdog_timeout_sec", -1.0);
    qos_depth_ = declare_parameter("vendor_qos_depth", 0);

    const auto resolved_input_topic = input_topic_.empty()
        ? input_topic_
        : get_node_topics_interface()->resolve_topic_name(input_topic_);
    const auto resolved_output_topic = output_topic_.empty()
        ? output_topic_
        : get_node_topics_interface()->resolve_topic_name(output_topic_);
    if (!vendor_output_topic_allowed(resolved_output_topic)) {
      throw std::runtime_error(
          "vendor velocity adapter must never publish the project final velocity topic");
    }
    if (!contract_accepted_) {
      RCLCPP_WARN(
          get_logger(),
          "Vendor velocity contract is disconnected: no command subscription or vendor "
          "publisher was created");
      return;
    }

    if (!resolved_vendor_topics_distinct(resolved_input_topic, resolved_output_topic)) {
      throw std::runtime_error(
          "accepted vendor contract requires distinct resolved input and output topics");
    }
    if (message_type != kSupportedMessageType || semantics != kSupportedSemantics) {
      throw std::runtime_error("unsupported or unproven vendor velocity message semantics");
    }
    if (reliability != "reliable" && reliability != "best_effort") {
      throw std::runtime_error("vendor_qos_reliability must be explicitly accepted");
    }
    if (durability != "volatile") {
      throw std::runtime_error("only an explicitly accepted volatile vendor contract is supported");
    }
    if (!is_lower_hex_sha256(receipt_sha256)) {
      throw std::runtime_error("accepted vendor contract requires a lowercase receipt SHA-256");
    }
    if (qos_depth_ <= 0 || control_rate_hz_ <= 0.0 ||
        publisher_discovery_grace_sec_ < 0.0 ||
        vendor_watchdog_timeout_sec_ <= 0.0 || vendor_watchdog_timeout_sec_ > 0.50 ||
        gate_.input_timeout_sec() > vendor_watchdog_timeout_sec_ ||
        gate_.platform_timeout_sec() > vendor_watchdog_timeout_sec_ ||
        (1.0 / control_rate_hz_) > vendor_watchdog_timeout_sec_ / 2.0) {
      throw std::runtime_error("invalid accepted vendor timing or QoS contract");
    }

    auto vendor_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(qos_depth_)));
    if (reliability == "best_effort") {
      vendor_qos.best_effort();
    } else {
      vendor_qos.reliable();
    }
    vendor_qos.durability_volatile();
    vendor_qos_ = vendor_qos;

    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        input_topic_, rclcpp::QoS(10).reliable().durability_volatile(),
        std::bind(&VendorVelocityAdapter::on_command, this, std::placeholders::_1));
    platform_sub_ = create_subscription<PlatformSafetyState>(
        platform_topic_, rclcpp::QoS(10).reliable().durability_volatile(),
        std::bind(&VendorVelocityAdapter::on_platform, this, std::placeholders::_1));
    reset_service_ = create_service<std_srvs::srv::Trigger>(
        "/sunhorizon/platform/reset_vendor_velocity_latch",
        std::bind(&VendorVelocityAdapter::on_reset, this,
          std::placeholders::_1, std::placeholders::_2));
    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&VendorVelocityAdapter::on_timer, this));
    startup_time_ = get_clock()->now();
  }

  ~VendorVelocityAdapter() override {
    if (output_connected_ && output_pub_ != nullptr && rclcpp::ok()) {
      output_pub_->publish(geometry_msgs::msg::Twist{});
    }
  }

 private:
  VendorVelocityLimits declare_limits() {
    VendorVelocityLimits limits;
    limits.input_timeout_sec = declare_parameter("supervised_input_timeout_sec", 0.25);
    limits.platform_timeout_sec = declare_parameter("platform_timeout_sec", 0.25);
    limits.max_forward_mps = declare_parameter("max_forward_mps", 0.15);
    limits.max_yaw_rps = declare_parameter("max_yaw_rps", 0.25);
    limits.zero_epsilon = declare_parameter("zero_epsilon", 1e-6);
    if (limits.input_timeout_sec <= 0.0 || limits.platform_timeout_sec <= 0.0 ||
        limits.max_forward_mps <= 0.0 || limits.max_yaw_rps <= 0.0 ||
        limits.zero_epsilon < 0.0) {
      throw std::runtime_error("invalid vendor velocity safety limits");
    }
    return limits;
  }

  void on_command(const geometry_msgs::msg::Twist::SharedPtr input) {
    command_ = from_message(*input);
    command_seen_ = true;
    command_stamp_sec_ = get_clock()->now().seconds();
  }

  void on_platform(const PlatformSafetyState::SharedPtr input) {
    platform_safe_ = input->mode_allowed && input->ethercat_op && input->wkc_normal &&
        input->joint_feedback_fresh && input->leg_joints_error_free &&
        input->leg_joints_operation_enabled &&
        input->motion_ready && input->navigation_mode_discovered && !input->platform_fault;
    platform_seen_ = true;
    platform_stamp_sec_ = get_clock()->now().seconds();
  }

  void try_connect_output() {
    if (output_connected_ || output_refused_) return;
    if ((get_clock()->now() - startup_time_).seconds() < publisher_discovery_grace_sec_) return;
    const auto input_publishers = count_publishers(input_topic_);
    if (input_publishers == 0U) return;
    if (input_publishers != 1U) {
      refuse_output("supervised_input_publisher_count:" + std::to_string(input_publishers));
      return;
    }
    const auto existing_output_publishers = count_publishers(output_topic_);
    if (existing_output_publishers != 0U) {
      refuse_output(
          "existing_vendor_output_publisher_count:" +
          std::to_string(existing_output_publishers));
      return;
    }
    output_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, vendor_qos_);
    output_connected_ = true;
    RCLCPP_WARN(
        get_logger(), "Accepted vendor velocity output connected to %s", output_topic_.c_str());
  }

  void refuse_output(const std::string& reason) {
    output_refused_ = true;
    gate_.force_latch(reason);
    RCLCPP_FATAL(get_logger(), "%s", reason.c_str());
  }

  void on_timer() {
    try_connect_output();
    if (!output_connected_) return;
    const auto decision = gate_.evaluate(
        get_clock()->now().seconds(), command_seen_, command_stamp_sec_, command_,
        platform_seen_, platform_stamp_sec_, platform_safe_);
    output_pub_->publish(to_message(decision.output));
    if (decision.latched && decision.reason != last_latch_reason_) {
      last_latch_reason_ = decision.reason;
      RCLCPP_ERROR(get_logger(), "Vendor velocity latched zero: %s", decision.reason.c_str());
    }
  }

  void on_reset(
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
    response->success = gate_.reset(
        get_clock()->now().seconds(), command_seen_, command_stamp_sec_, command_,
        platform_seen_, platform_stamp_sec_, platform_safe_, &response->message);
    if (response->success) last_latch_reason_.clear();
  }

  VendorVelocityGate gate_;
  bool contract_accepted_{false};
  bool command_seen_{false};
  bool platform_seen_{false};
  bool platform_safe_{false};
  bool output_connected_{false};
  bool output_refused_{false};
  int qos_depth_{0};
  double command_stamp_sec_{0.0};
  double platform_stamp_sec_{0.0};
  double control_rate_hz_{50.0};
  double publisher_discovery_grace_sec_{2.0};
  double vendor_watchdog_timeout_sec_{-1.0};
  std::string input_topic_;
  std::string output_topic_;
  std::string platform_topic_;
  std::string last_latch_reason_;
  VendorVelocityCommand command_;
  rclcpp::QoS vendor_qos_{rclcpp::KeepLast(1)};
  rclcpp::Time startup_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<PlatformSafetyState>::SharedPtr platform_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VendorVelocityAdapter>());
  rclcpp::shutdown();
  return 0;
}
