#pragma once

#include <rclcpp_action/exceptions.hpp>

namespace sunhorizon_navigation_server {

// A result callback may remove a goal from the action client's registry while
// the caller is waiting for the supervisor's zero acknowledgement. Cancellation
// is then already obsolete. The client's locked lookup, rather than a separate
// goal-status check, arbitrates that race. Other transport errors still propagate.
template <typename Client, typename GoalHandle>
bool cancel_goal_if_known(Client& client, const GoalHandle& handle) {
  try {
    client.async_cancel_goal(handle);
    return true;
  } catch (const rclcpp_action::exceptions::UnknownGoalHandleError&) {
    return false;
  }
}

}  // namespace sunhorizon_navigation_server
