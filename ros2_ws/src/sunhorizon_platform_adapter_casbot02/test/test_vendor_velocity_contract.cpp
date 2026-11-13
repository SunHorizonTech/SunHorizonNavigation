#include <cassert>
#include <cmath>
#include <string>

#include "sunhorizon_platform_adapter_casbot02/vendor_velocity_contract.hpp"

using sunhorizon_platform_adapter_casbot02::VendorVelocityCommand;
using sunhorizon_platform_adapter_casbot02::VendorVelocityGate;
using sunhorizon_platform_adapter_casbot02::VendorVelocityLimits;
using sunhorizon_platform_adapter_casbot02::resolved_vendor_topics_distinct;
using sunhorizon_platform_adapter_casbot02::vendor_output_topic_allowed;

namespace {

VendorVelocityGate make_gate() {
  VendorVelocityLimits limits;
  limits.input_timeout_sec = 0.25;
  limits.platform_timeout_sec = 0.25;
  limits.max_forward_mps = 0.15;
  limits.max_yaw_rps = 0.25;
  return VendorVelocityGate(limits);
}

void test_project_final_topic_is_never_a_vendor_output() {
  assert(!vendor_output_topic_allowed("/navigation/cmd_vel"));
  assert(vendor_output_topic_allowed("/vendor/navigation/cmd_vel"));
  assert(vendor_output_topic_allowed(""));
}

void test_vendor_topics_must_be_distinct_after_ros_name_resolution() {
  assert(resolved_vendor_topics_distinct(
      "/navigation/cmd_vel", "/vendor/navigation/cmd_vel"));
  assert(!resolved_vendor_topics_distinct(
      "/robot/velocity", "/robot/velocity"));
  assert(!resolved_vendor_topics_distinct("", "/robot/velocity"));
  assert(!resolved_vendor_topics_distinct("/navigation/cmd_vel", ""));
}

void test_waiting_is_zero_without_latching() {
  auto gate = make_gate();
  const auto decision = gate.evaluate(1.0, false, 0.0, {}, false, 0.0, false);
  assert(!decision.allowed);
  assert(!decision.latched);
  assert(decision.output.linear_x == 0.0);
}

void test_limits_forward_and_yaw() {
  auto gate = make_gate();
  VendorVelocityCommand command;
  command.linear_x = 0.30;
  command.angular_z = -0.50;
  const auto decision = gate.evaluate(1.0, true, 0.9, command, true, 0.9, true);
  assert(decision.allowed);
  assert(!decision.latched);
  assert(decision.reason == "limited");
  assert(decision.output.linear_x == 0.15);
  assert(decision.output.angular_z == -0.25);
}

void test_reverse_lateral_and_non_finite_latch_zero() {
  for (int variant = 0; variant < 3; ++variant) {
    auto gate = make_gate();
    VendorVelocityCommand command;
    if (variant == 0) command.linear_x = -0.01;
    if (variant == 1) command.linear_y = 0.01;
    if (variant == 2) command.angular_z = std::nan("");
    const auto decision = gate.evaluate(1.0, true, 0.9, command, true, 0.9, true);
    assert(!decision.allowed);
    assert(decision.latched);
    assert(decision.output.linear_x == 0.0);
    assert(decision.output.angular_z == 0.0);
  }
}

void test_stale_or_unsafe_evidence_latches_zero() {
  {
    auto gate = make_gate();
    const auto decision = gate.evaluate(1.0, true, 0.70, {}, true, 0.9, true);
    assert(decision.latched);
    assert(decision.reason == "supervised_command_stale_or_future");
  }
  {
    auto gate = make_gate();
    const auto decision = gate.evaluate(1.0, true, 0.9, {}, true, 0.9, false);
    assert(decision.latched);
    assert(decision.reason == "platform_state_unsafe");
  }
}

void test_reset_requires_fresh_safe_platform_and_fresh_zero() {
  auto gate = make_gate();
  VendorVelocityCommand reverse;
  reverse.linear_x = -0.01;
  assert(gate.evaluate(1.0, true, 0.9, reverse, true, 0.9, true).latched);

  std::string reason;
  assert(!gate.reset(1.1, true, 1.0, reverse, true, 1.0, true, &reason));
  assert(reason == "fresh_zero_command_required_for_reset");
  assert(gate.reset(1.1, true, 1.0, {}, true, 1.0, true, &reason));
  assert(reason == "reset");
  assert(!gate.latched());
}

}  // namespace

int main() {
  test_project_final_topic_is_never_a_vendor_output();
  test_vendor_topics_must_be_distinct_after_ros_name_resolution();
  test_waiting_is_zero_without_latching();
  test_limits_forward_and_yaw();
  test_reverse_lateral_and_non_finite_latch_zero();
  test_stale_or_unsafe_evidence_latches_zero();
  test_reset_requires_fresh_safe_platform_and_fresh_zero();
  return 0;
}
