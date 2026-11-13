#pragma once

#include <algorithm>
#include <cmath>
#include <string>

namespace sunhorizon_platform_adapter_casbot02 {

inline constexpr char kProjectFinalVelocityTopic[] = "/navigation/cmd_vel";

inline bool vendor_output_topic_allowed(const std::string& topic) {
  return topic != kProjectFinalVelocityTopic;
}

inline bool resolved_vendor_topics_distinct(
    const std::string& input_topic,
    const std::string& output_topic) {
  return !input_topic.empty() && !output_topic.empty() && input_topic != output_topic;
}

struct VendorVelocityCommand {
  double linear_x{0.0};
  double linear_y{0.0};
  double linear_z{0.0};
  double angular_x{0.0};
  double angular_y{0.0};
  double angular_z{0.0};
};

struct VendorVelocityLimits {
  double input_timeout_sec{0.25};
  double platform_timeout_sec{0.25};
  double max_forward_mps{0.15};
  double max_yaw_rps{0.25};
  double zero_epsilon{1e-6};
};

struct VendorVelocityDecision {
  bool allowed{false};
  bool latched{false};
  VendorVelocityCommand output;
  std::string reason;
};

class VendorVelocityGate {
 public:
  explicit VendorVelocityGate(VendorVelocityLimits limits) : limits_(limits) {}

  VendorVelocityDecision evaluate(
      double now_sec,
      bool command_seen,
      double command_stamp_sec,
      const VendorVelocityCommand& command,
      bool platform_seen,
      double platform_stamp_sec,
      bool platform_safe) {
    if (latched_) return zero_decision(false, latch_reason_);
    if (!std::isfinite(now_sec)) return latch("invalid_clock");

    if (!platform_seen) return zero_decision(false, "waiting_for_platform");
    if (!fresh(now_sec, platform_stamp_sec, limits_.platform_timeout_sec)) {
      return latch("platform_state_stale_or_future");
    }
    if (!platform_safe) return latch("platform_state_unsafe");

    if (!command_seen) return zero_decision(false, "waiting_for_supervised_command");
    if (!fresh(now_sec, command_stamp_sec, limits_.input_timeout_sec)) {
      return latch("supervised_command_stale_or_future");
    }
    if (!finite(command)) return latch("supervised_command_non_finite");
    if (command.linear_x < -limits_.zero_epsilon) {
      return latch("reverse_command_forbidden");
    }
    if (std::abs(command.linear_y) > limits_.zero_epsilon) {
      return latch("lateral_command_forbidden");
    }
    if (std::abs(command.linear_z) > limits_.zero_epsilon ||
        std::abs(command.angular_x) > limits_.zero_epsilon ||
        std::abs(command.angular_y) > limits_.zero_epsilon) {
      return latch("unsupported_twist_component");
    }

    VendorVelocityDecision decision;
    decision.allowed = true;
    decision.output.linear_x = std::clamp(command.linear_x, 0.0, limits_.max_forward_mps);
    decision.output.angular_z = std::clamp(
        command.angular_z, -limits_.max_yaw_rps, limits_.max_yaw_rps);
    decision.reason =
        decision.output.linear_x == command.linear_x &&
            decision.output.angular_z == command.angular_z
        ? "ok"
        : "limited";
    return decision;
  }

  bool reset(
      double now_sec,
      bool command_seen,
      double command_stamp_sec,
      const VendorVelocityCommand& command,
      bool platform_seen,
      double platform_stamp_sec,
      bool platform_safe,
      std::string* reason) {
    if (!latched_) {
      if (reason != nullptr) *reason = "not_latched";
      return true;
    }
    if (!std::isfinite(now_sec) || !platform_seen || !platform_safe ||
        !fresh(now_sec, platform_stamp_sec, limits_.platform_timeout_sec)) {
      if (reason != nullptr) *reason = "platform_not_safe_for_reset";
      return false;
    }
    if (!command_seen || !fresh(now_sec, command_stamp_sec, limits_.input_timeout_sec) ||
        !finite(command) || !zero(command)) {
      if (reason != nullptr) *reason = "fresh_zero_command_required_for_reset";
      return false;
    }
    latched_ = false;
    latch_reason_.clear();
    if (reason != nullptr) *reason = "reset";
    return true;
  }

  void force_latch(const std::string& reason) {
    latched_ = true;
    latch_reason_ = reason;
  }

  bool latched() const { return latched_; }
  const std::string& latch_reason() const { return latch_reason_; }
  double input_timeout_sec() const { return limits_.input_timeout_sec; }
  double platform_timeout_sec() const { return limits_.platform_timeout_sec; }

 private:
  bool fresh(double now_sec, double stamp_sec, double timeout_sec) const {
    if (!std::isfinite(stamp_sec) || stamp_sec <= 0.0) return false;
    const double age = now_sec - stamp_sec;
    return age >= 0.0 && age <= timeout_sec;
  }

  bool finite(const VendorVelocityCommand& command) const {
    return std::isfinite(command.linear_x) && std::isfinite(command.linear_y) &&
        std::isfinite(command.linear_z) && std::isfinite(command.angular_x) &&
        std::isfinite(command.angular_y) && std::isfinite(command.angular_z);
  }

  bool zero(const VendorVelocityCommand& command) const {
    return std::abs(command.linear_x) <= limits_.zero_epsilon &&
        std::abs(command.linear_y) <= limits_.zero_epsilon &&
        std::abs(command.linear_z) <= limits_.zero_epsilon &&
        std::abs(command.angular_x) <= limits_.zero_epsilon &&
        std::abs(command.angular_y) <= limits_.zero_epsilon &&
        std::abs(command.angular_z) <= limits_.zero_epsilon;
  }

  VendorVelocityDecision zero_decision(bool allowed, const std::string& reason) const {
    VendorVelocityDecision decision;
    decision.allowed = allowed;
    decision.latched = latched_;
    decision.reason = reason;
    return decision;
  }

  VendorVelocityDecision latch(const std::string& reason) {
    force_latch(reason);
    return zero_decision(false, reason);
  }

  VendorVelocityLimits limits_;
  bool latched_{false};
  std::string latch_reason_;
};

}  // namespace sunhorizon_platform_adapter_casbot02
