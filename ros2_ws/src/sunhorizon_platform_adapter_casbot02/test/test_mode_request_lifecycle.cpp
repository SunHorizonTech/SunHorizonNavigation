#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <chrono>
#include <cstdint>
#include <stdexcept>

#include "sunhorizon_platform_adapter_casbot02/mode_request_lifecycle.hpp"

using Lifecycle = sunhorizon_platform_adapter_casbot02::ModeRequestLifecycle;

namespace {

void test_rejects_nonpositive_timeout() {
  bool rejected = false;
  try {
    Lifecycle invalid(Lifecycle::Duration::zero(), std::chrono::milliseconds(100));
    (void)invalid;
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  assert(rejected);

  rejected = false;
  try {
    Lifecycle invalid(std::chrono::seconds(1), Lifecycle::Duration::zero());
    (void)invalid;
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  assert(rejected);
}

void test_pending_request_expires_at_deadline_and_can_retry() {
  const auto start = Lifecycle::TimePoint{};
  Lifecycle lifecycle(
      std::chrono::milliseconds(500), std::chrono::milliseconds(200));
  const auto first_token = lifecycle.begin(start);
  assert(lifecycle.pending());
  assert(lifecycle.bind_request_id(first_token, 41));
  assert(!lifecycle.bind_request_id(first_token, 99));
  assert(!lifecycle.expire(start + std::chrono::milliseconds(499)));

  const auto expired = lifecycle.expire(start + std::chrono::milliseconds(500));
  assert(expired);
  assert(expired->request_id);
  assert(*expired->request_id == 41);
  assert(!lifecycle.pending());

  const auto retry_token = lifecycle.begin(start + std::chrono::milliseconds(500));
  assert(retry_token != first_token);
  assert(lifecycle.bind_request_id(retry_token, 42));
  assert(lifecycle.complete(retry_token));
  assert(!lifecycle.pending());
}

void test_late_response_cannot_clear_retry() {
  const auto start = Lifecycle::TimePoint{};
  Lifecycle lifecycle(
      std::chrono::milliseconds(100), std::chrono::milliseconds(50));
  const auto stale_token = lifecycle.begin(start);
  assert(lifecycle.bind_request_id(stale_token, 7));
  assert(lifecycle.expire(start + std::chrono::milliseconds(100)));

  const auto retry_token = lifecycle.begin(start + std::chrono::milliseconds(101));
  assert(lifecycle.bind_request_id(retry_token, 8));
  assert(!lifecycle.complete(stale_token));
  assert(lifecycle.pending());
  assert(lifecycle.complete(retry_token));
  assert(!lifecycle.pending());
}

void test_send_failure_abandons_only_active_attempt() {
  const auto start = Lifecycle::TimePoint{};
  Lifecycle lifecycle(std::chrono::seconds(1), std::chrono::milliseconds(200));
  const auto token = lifecycle.begin(start);
  assert(lifecycle.abandon(token));
  assert(!lifecycle.pending());
  assert(!lifecycle.abandon(token));
  assert(!lifecycle.ready(start + std::chrono::milliseconds(199)));
  assert(lifecycle.ready(start + std::chrono::milliseconds(200)));
}

void test_fast_response_cannot_exceed_request_period() {
  const auto start = Lifecycle::TimePoint{};
  Lifecycle lifecycle(std::chrono::seconds(1), std::chrono::milliseconds(200));
  const auto token = lifecycle.begin(start);
  assert(lifecycle.bind_request_id(token, 10));
  assert(lifecycle.complete(token));
  assert(!lifecycle.ready(start + std::chrono::milliseconds(50)));

  bool rejected = false;
  try {
    lifecycle.begin(start + std::chrono::milliseconds(199));
  } catch (const std::logic_error&) {
    rejected = true;
  }
  assert(rejected);
  assert(lifecycle.ready(start + std::chrono::milliseconds(200)));
  assert(lifecycle.begin(start + std::chrono::milliseconds(200)) != token);
}

}  // namespace

int main() {
  test_rejects_nonpositive_timeout();
  test_pending_request_expires_at_deadline_and_can_retry();
  test_late_response_cannot_clear_retry();
  test_send_failure_abandons_only_active_attempt();
  test_fast_response_cannot_exceed_request_period();
  return 0;
}
