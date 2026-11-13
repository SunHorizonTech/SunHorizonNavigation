#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <stdexcept>

namespace sunhorizon_platform_adapter_casbot02 {

class ModeRequestLifecycle {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  using Duration = Clock::duration;

  struct ExpiredRequest {
    std::optional<int64_t> request_id;
  };

  ModeRequestLifecycle(Duration timeout, Duration request_period)
      : timeout_(timeout), request_period_(request_period) {
    if (timeout_ <= Duration::zero()) {
      throw std::invalid_argument("mode request timeout must be positive");
    }
    if (request_period_ <= Duration::zero()) {
      throw std::invalid_argument("mode request period must be positive");
    }
  }

  bool pending() const { return active_.has_value(); }

  bool ready(TimePoint now) const {
    return !active_ &&
        (!last_started_at_ || now - *last_started_at_ >= request_period_);
  }

  uint64_t begin(TimePoint now) {
    if (!ready(now)) {
      throw std::logic_error("mode request not ready");
    }
    const uint64_t token = ++last_token_;
    active_ = PendingRequest{token, std::nullopt, now};
    last_started_at_ = now;
    return token;
  }

  bool bind_request_id(uint64_t token, int64_t request_id) {
    if (!matches(token) || active_->request_id) return false;
    active_->request_id = request_id;
    return true;
  }

  bool complete(uint64_t token) {
    if (!matches(token)) return false;
    active_.reset();
    return true;
  }

  bool abandon(uint64_t token) { return complete(token); }

  std::optional<ExpiredRequest> expire(TimePoint now) {
    if (!active_ || now - active_->started_at < timeout_) {
      return std::nullopt;
    }
    ExpiredRequest expired{active_->request_id};
    active_.reset();
    return expired;
  }

 private:
  struct PendingRequest {
    uint64_t token;
    std::optional<int64_t> request_id;
    TimePoint started_at;
  };

  bool matches(uint64_t token) const {
    return active_ && active_->token == token;
  }

  Duration timeout_;
  Duration request_period_;
  uint64_t last_token_{0};
  std::optional<TimePoint> last_started_at_;
  std::optional<PendingRequest> active_;
};

}  // namespace sunhorizon_platform_adapter_casbot02
