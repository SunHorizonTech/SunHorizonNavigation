#pragma once

#include <string>

namespace sunhorizon_navigation_core {

enum class NavigationState : unsigned char {
  kUnconfigured = 0,
  kIdle = 1,
  kMapping = 2,
  kMapReady = 3,
  kLocalizing = 4,
  kReady = 5,
  kNavigating = 6,
  kStopped = 7,
  kFault = 8,
};

struct TransitionResult {
  bool accepted{false};
  std::string reason;
};

class NavigationStateMachine {
 public:
  TransitionResult configure();
  TransitionResult start_mapping(const std::string& session_id);
  TransitionResult stop_mapping(const std::string& session_id);
  TransitionResult save_map(const std::string& map_id);
  TransitionResult load_map(const std::string& map_id);
  TransitionResult localization_result(bool healthy, const std::string& reason);
  TransitionResult start_navigation(const std::string& task_id);
  TransitionResult finish_navigation(
      const std::string& task_id, bool success, const std::string& reason);
  TransitionResult cancel(const std::string& task_id);
  TransitionResult acknowledge_stop();
  void fault(const std::string& reason);
  TransitionResult clear_fault();

  NavigationState state() const { return state_; }
  const std::string& map_id() const { return map_id_; }
  const std::string& task_id() const { return task_id_; }
  const std::string& reason() const { return reason_; }

 private:
  TransitionResult reject(const std::string& operation) const;
  NavigationState state_{NavigationState::kUnconfigured};
  std::string session_id_;
  std::string map_id_;
  std::string task_id_;
  std::string reason_;
};

}  // namespace sunhorizon_navigation_core
