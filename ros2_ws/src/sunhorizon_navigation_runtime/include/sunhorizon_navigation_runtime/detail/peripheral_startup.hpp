#pragma once
#include <chrono>
#include <optional>
#include <stdexcept>

namespace sunhorizon_navigation_runtime::detail {
// One owner issues STARTUP once. Discovery and its actual service response
// share the original monotonic startup deadline; no retry grants another grace.
class PeripheralStartup {
 public:
  using Clock = std::chrono::steady_clock;
  enum class Step {Waiting, Request, Complete};
  explicit PeripheralStartup(Clock::time_point deadline) : deadline_(deadline) {}
  // A motion hold cannot abandon an already-issued initialization request.
  // It grants neither a new request nor extra time, and owns no motion state.
  bool should_poll(bool motion_held, Clock::time_point now = Clock::now()) const {
    return !motion_held || (requested_ && now < deadline_);
  }
  Step advance(bool core_active, bool dependencies_ready, std::optional<bool> response,
    Clock::time_point now = Clock::now()) {
    if (complete_) return Step::Complete;
    if (now >= deadline_) throw std::runtime_error("peripheral_startup_original_deadline_expired");
    if (requested_ && !core_active) throw std::runtime_error("nav2_core_lost_during_peripheral_startup");
    if (response) {
      if (!requested_) throw std::logic_error("unsolicited_peripheral_startup_response");
      if (!*response) throw std::runtime_error("peripheral_lifecycle_startup_rejected");
      complete_ = true; return Step::Complete;
    }
    if (!requested_ && core_active && dependencies_ready) {
      requested_ = true; return Step::Request;
    }
    return Step::Waiting;
  }
 private:
  Clock::time_point deadline_;
  bool requested_{false}, complete_{false};
};
}  // namespace sunhorizon_navigation_runtime::detail
