#include <cassert>
#include <string>

#include "sunhorizon_navigation_core/navigation_state_machine.hpp"
#include "sunhorizon_navigation_server/terminal_zero_guard.hpp"

using sunhorizon_navigation_core::NavigationState;
using sunhorizon_navigation_core::NavigationStateMachine;
using sunhorizon_navigation_server::TerminalZeroGuard;

int main(int argc, char** argv) {
  assert(argc == 2);
  const std::string test = argv[1];
  TerminalZeroGuard guard;
  constexpr std::int64_t request = 1789013694943000000LL;
  // Actual bag17 supervisor status stamp; it arrived while finish_outer was
  // awaiting the zero ACK after MPPI aborted. Request time is a test boundary,
  // not a recorded ROS service request timestamp.
  constexpr std::int64_t report = 1789013694950573839LL;
  constexpr std::int64_t now = 1789013694969346000LL;
  constexpr std::int64_t budget = 300000000;
  constexpr std::int64_t start = 1000000000;
  if (test == "normal_stop") {
    assert(!guard.expected_stop("task", true, "operator_stop", report, now, start));
    assert(!guard.acknowledged("task", true, "zero_velocity_latched"));
    return 0;
  }
  assert(guard.begin("task", request, start, budget));
  if (test == "failure" || test == "success") {
    NavigationStateMachine machine;
    assert(machine.configure().accepted);
    assert(machine.load_map("map").accepted);
    assert(machine.localization_result(true, "").accepted);
    assert(machine.start_navigation("task").accepted);
    auto old_machine = machine;
    old_machine.fault("velocity_supervisor:operator_stop");
    assert(!old_machine.finish_navigation("task", false, "nav2_navigation_failed").accepted);
    assert(guard.expected_stop("task", true, "operator_stop", report, now, start + 27000000));
    assert(guard.acknowledged("task", true, "zero_velocity_latched"));
    const bool success = test == "success";
    assert(machine.finish_navigation("task", success,
        success ? "navigation_complete" : "nav2_navigation_failed").accepted);
    assert(machine.state() == (success ? NavigationState::kReady : NavigationState::kStopped));
    assert(machine.reason() == (success ? "navigation_complete" : "nav2_navigation_failed"));
  } else if (test == "external_fault") {
    assert(!guard.expected_stop("task", true, "collision_stop", report, now, start));
    guard.note_fault("velocity_supervisor:collision_stop");
    // A later own stop must not replace the first independent safety fault.
    guard.note_fault("velocity_supervisor:operator_stop");
    assert(!guard.expected_stop("task", true, "operator_stop", report, now, start));
    assert(!guard.acknowledged("task", true, "zero_velocity_latched"));
    assert(guard.fault() == "velocity_supervisor:collision_stop");
  } else if (test == "cancel_and_localization") {
    for (const auto& reason : {"operator_cancel_pending_zero_ack", "localization_lost"}) {
      guard.note_fault(reason);
      assert(!guard.expected_stop("task", true, "operator_stop", report, now, start));
      assert(!guard.acknowledged("task", true, "zero_velocity_latched"));
      assert(guard.fault() == reason);
      guard.clear("task");
      assert(guard.begin("task", request, start, budget));
    }
  } else if (test == "ack_failure") {
    assert(guard.expected_stop("task", true, "operator_stop", report, now, start));
    assert(!guard.acknowledged("task", false, "zero_velocity_latch_timeout"));
    assert(!guard.acknowledged("task", true, "operator_stop"));
    assert(!guard.acknowledged("different", true, "zero_velocity_latched"));
  } else if (test == "task_and_clock_scope") {
    assert(!guard.expected_stop("different", true, "operator_stop", report, now, start));
    assert(!guard.expected_stop("task", false, "operator_stop", report, now, start));
    assert(!guard.expected_stop("task", true, "operator_stop", request - 1, now, start));
    assert(!guard.expected_stop("task", true, "operator_stop", 0, now, start));
    assert(!guard.expected_stop("task", true, "operator_stop", now + 1, now, start));
    assert(!guard.expected_stop("task", true, "operator_stop", report, now, start - 1));
    assert(!guard.expected_stop("task", true, "operator_stop", report, now, start + budget + 1));
    assert(guard.expected_stop("task", true, "operator_stop", report, now, start + budget));
  } else if (test == "clear_and_reuse") {
    assert(!guard.begin("next", request, start, budget));
    guard.clear("other");
    assert(guard.matches("task"));
    guard.note_fault("localization_lost");
    guard.clear("task");
    assert(!guard.expected_stop("task", true, "operator_stop", report, now, start));
    assert(guard.begin("next", report + 1, start + 1, budget));
    assert(guard.fault().empty());
    assert(!guard.expected_stop("next", true, "operator_stop", report, now, start + 1));
  } else {
    assert(false);
  }
}
