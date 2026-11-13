#pragma once

#include <cstdint>
#include <string>

namespace sunhorizon_navigation_server {

// Serialised by NavigationServer::mutex_. This recognises only the stop issued
// while collecting an already-terminal Nav2 result. It never grants motion or
// substitutes for the zero service ACK. Trigger has no request ID in its status:
// a simultaneous external operator_stop cannot be distinguished from this one.
// Both require the same latched zero; other faults and explicit cancels win.
class TerminalZeroGuard {
 public:
  bool begin(const std::string& task, std::int64_t request_stamp_ns,
      std::int64_t steady_ns, std::int64_t ack_budget_ns) {
    if (!task_.empty() || task.empty() || request_stamp_ns <= 0 ||
        ack_budget_ns <= 0) return false;
    task_ = task;
    request_stamp_ns_ = request_stamp_ns;
    steady_start_ns_ = steady_ns;
    ack_budget_ns_ = ack_budget_ns;
    fault_.clear();
    return true;
  }

  bool matches(const std::string& task) const {
    return !task_.empty() && task_ == task;
  }

  bool expected_stop(const std::string& task, bool latched,
      const std::string& reason, std::int64_t stamp_ns,
      std::int64_t now_ns, std::int64_t steady_ns) const {
    return matches(task) && fault_.empty() && latched &&
           reason == "operator_stop" && stamp_ns >= request_stamp_ns_ &&
           stamp_ns <= now_ns && steady_ns >= steady_start_ns_ &&
           steady_ns - steady_start_ns_ <= ack_budget_ns_;
  }

  void note_fault(const std::string& reason) {
    if (!task_.empty() && fault_.empty()) fault_ = reason;
  }

  const std::string& fault() const { return fault_; }

  bool acknowledged(const std::string& task, bool latched,
      const std::string& reason) const {
    return matches(task) && fault_.empty() && latched &&
           reason == "zero_velocity_latched";
  }

  void clear(const std::string& task) {
    if (!matches(task)) return;
    task_.clear();
    fault_.clear();
  }

 private:
  std::string task_, fault_;
  std::int64_t request_stamp_ns_{0}, steady_start_ns_{0}, ack_budget_ns_{0};
};

}  // namespace sunhorizon_navigation_server
