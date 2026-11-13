#include "sunhorizon_navigation_core/navigation_state_machine.hpp"

namespace sunhorizon_navigation_core {

TransitionResult NavigationStateMachine::reject(const std::string& operation) const {
  return {false, operation + " invalid in state " + std::to_string(static_cast<int>(state_))};
}

TransitionResult NavigationStateMachine::configure() {
  if (state_ != NavigationState::kUnconfigured) return reject("configure");
  state_ = NavigationState::kIdle;
  reason_ = "configured";
  return {true, reason_};
}

TransitionResult NavigationStateMachine::start_mapping(const std::string& session_id) {
  if (state_ != NavigationState::kIdle || session_id.empty()) return reject("start_mapping");
  session_id_ = session_id;
  state_ = NavigationState::kMapping;
  reason_ = "mapping";
  return {true, reason_};
}

TransitionResult NavigationStateMachine::stop_mapping(const std::string& session_id) {
  if (state_ != NavigationState::kMapping || session_id != session_id_) return reject("stop_mapping");
  state_ = NavigationState::kMapReady;
  reason_ = "mapping_stopped";
  return {true, reason_};
}

TransitionResult NavigationStateMachine::save_map(const std::string& map_id) {
  if (state_ != NavigationState::kMapReady || map_id.empty()) return reject("save_map");
  map_id_ = map_id;
  session_id_.clear();
  state_ = NavigationState::kIdle;
  reason_ = "map_saved";
  return {true, reason_};
}

TransitionResult NavigationStateMachine::load_map(const std::string& map_id) {
  if ((state_ != NavigationState::kIdle && state_ != NavigationState::kReady) || map_id.empty()) {
    return reject("load_map");
  }
  map_id_ = map_id;
  state_ = NavigationState::kLocalizing;
  reason_ = "map_loaded_waiting_for_localization";
  return {true, reason_};
}

TransitionResult NavigationStateMachine::localization_result(
    bool healthy, const std::string& reason) {
  if (state_ != NavigationState::kLocalizing && state_ != NavigationState::kReady) {
    return reject("localization_result");
  }
  if (!healthy) {
    reason_ = reason.empty() ? "localization_failed" : reason;
    // A rejected startup seed is not a terminal navigation fault.  Staying in
    // kLocalizing keeps every goal gate closed while allowing an operator to
    // correct the pose from RViz.  Losing localization after the robot was
    // already ready is different: that remains a latched fault.
    if (state_ == NavigationState::kLocalizing) {
      return {false, reason_};
    }
    state_ = NavigationState::kFault;
    return {false, reason_};
  }
  state_ = NavigationState::kReady;
  reason_ = "localized";
  return {true, reason_};
}

TransitionResult NavigationStateMachine::start_navigation(const std::string& task_id) {
  if (state_ != NavigationState::kReady || task_id.empty()) return reject("start_navigation");
  task_id_ = task_id;
  state_ = NavigationState::kNavigating;
  reason_ = "navigating";
  return {true, reason_};
}

TransitionResult NavigationStateMachine::finish_navigation(
    const std::string& task_id, bool success, const std::string& reason) {
  if (state_ != NavigationState::kNavigating || task_id.empty() || task_id != task_id_) {
    return reject("finish_navigation");
  }
  task_id_.clear();
  if (success) {
    state_ = NavigationState::kReady;
    reason_ = "navigation_complete";
    return {true, reason_};
  }
  state_ = NavigationState::kStopped;
  reason_ = reason.empty() ? "navigation_failed" : reason;
  return {true, reason_};
}

TransitionResult NavigationStateMachine::cancel(const std::string& task_id) {
  if (state_ == NavigationState::kStopped && task_id_.empty()) return {true, "already_stopped"};
  if (state_ != NavigationState::kNavigating || (!task_id.empty() && task_id != task_id_)) {
    return reject("cancel");
  }
  task_id_.clear();
  state_ = NavigationState::kStopped;
  reason_ = "cancelled";
  return {true, reason_};
}

TransitionResult NavigationStateMachine::acknowledge_stop() {
  if (state_ != NavigationState::kStopped) return reject("acknowledge_stop");
  state_ = map_id_.empty() ? NavigationState::kIdle : NavigationState::kReady;
  reason_ = "stop_acknowledged";
  return {true, reason_};
}

void NavigationStateMachine::fault(const std::string& reason) {
  state_ = NavigationState::kFault;
  task_id_.clear();
  reason_ = reason.empty() ? "fault" : reason;
}

TransitionResult NavigationStateMachine::clear_fault() {
  if (state_ != NavigationState::kFault) return reject("clear_fault");
  state_ = NavigationState::kIdle;
  map_id_.clear();
  reason_ = "fault_cleared";
  return {true, reason_};
}

}  // namespace sunhorizon_navigation_core
