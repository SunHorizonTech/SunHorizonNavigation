#include "sunhorizon_navigation_runtime/detail/canonical_health_check.hpp"
#include <array>
#include <iostream>
#include <stdexcept>
#include <string>

using sunhorizon_navigation_runtime::detail::CanonicalHealthSnapshot;
using sunhorizon_navigation_runtime::detail::canonical_health_diagnostic;
using sunhorizon_navigation_runtime::detail::canonical_health_failure;
using sunhorizon_navigation_runtime::detail::canonical_health_motion_unavailable;

void check(bool value) {if (!value) throw std::runtime_error("canonical health contract");}
void expect(const CanonicalHealthSnapshot & s, const char * reason) {
  if (std::string(canonical_health_failure(s)) != reason)
    throw std::runtime_error(std::string("expected ") + reason + ", got " + canonical_health_failure(s));
}
CanonicalHealthSnapshot healthy() {
  CanonicalHealthSnapshot s;
  s.has_state = s.has_health = s.has_status = s.has_health_state = true;
  s.status_healthy = s.measurement_accepted = true;
  s.reset_epoch = s.state_epoch = s.health_epoch = s.health_state_epoch = 7;
  s.state_sequence = 91; s.tracking_sequence = 103; s.health_state_sequence = 90;
  s.tracking_identity = "/sunhorizon_livo_adapter#candidate";
  s.check_ros_ns = 2000000000; s.check_steady_ns = 3000000000;
  s.source_ns = 1800000000; s.receipt_ros_ns = 1990000000; s.receipt_steady_ns = 2990000000;
  s.status_stamp_ns = s.health_ros_ns = 1999000000; s.health_steady_ns = 2999000000;
  s.tracking_budget_ns = 250000000; s.health_budget_ns = 800000000;
  return s;
}
// Stream-liveness specification: source transport delay is diagnostic only.
bool original_healthy(const CanonicalHealthSnapshot & s) {
  if (!s.has_state || !s.has_health || !s.has_status || !s.has_health_state ||
    s.state_epoch != s.reset_epoch || s.health_epoch != s.reset_epoch ||
    s.health_state_epoch != s.reset_epoch || !s.status_healthy || !s.measurement_accepted ||
    s.status_stamp_ns != s.health_ros_ns) return false;
  const auto within_age = [](int64_t source, int64_t received_ros, int64_t received_steady,
      int64_t ros, int64_t steady, int64_t budget) {
    return source > 0 && received_ros >= source && received_steady > 0 &&
      ros >= received_ros && steady >= received_steady &&
      ros - received_ros <= budget && steady - received_steady <= budget;
  };
  return within_age(s.source_ns, s.receipt_ros_ns, s.receipt_steady_ns,
    s.check_ros_ns, s.check_steady_ns, s.tracking_budget_ns) &&
    within_age(s.health_ros_ns, s.health_ros_ns, s.health_steady_ns,
      s.check_ros_ns, s.check_steady_ns, s.health_budget_ns);
}

int main() {
  auto s = healthy(); expect(s, "");
  // Delayed source with a live receipt is not a localization failure.
  s.source_ns = s.check_ros_ns - s.tracking_budget_ns; expect(s, "");
  --s.source_ns; expect(s, "");
  s.source_ns = 1; expect(s, "");
  s = healthy(); s.receipt_steady_ns = s.check_steady_ns - s.tracking_budget_ns; expect(s, "");
  --s.receipt_steady_ns; expect(s, "tracking_receipt_steady_age_exceeded");
  s = healthy(); s.status_stamp_ns = s.health_ros_ns = s.check_ros_ns - s.health_budget_ns; expect(s, "");
  --s.status_stamp_ns; --s.health_ros_ns; expect(s, "health_ros_age_exceeded");
  s = healthy(); s.health_steady_ns = s.check_steady_ns - s.health_budget_ns; expect(s, "");
  --s.health_steady_ns; expect(s, "health_steady_age_exceeded");
  s = healthy(); s.receipt_ros_ns = s.source_ns - 1; expect(s, "tracking_source_after_receipt");
  s = healthy(); s.check_ros_ns = s.receipt_ros_ns - 1; expect(s, "tracking_ros_clock_regression");
  s = healthy(); s.check_steady_ns = s.receipt_steady_ns - 1; expect(s, "tracking_steady_clock_regression");
  s = healthy(); s.status_stamp_ns = s.health_ros_ns = s.check_ros_ns + 1; expect(s, "health_ros_clock_regression");
  s = healthy(); s.health_steady_ns = s.check_steady_ns + 1; expect(s, "health_steady_clock_regression");
  s = healthy(); s.has_state = false; expect(s, "canonical_state_missing");
  s = healthy(); s.has_health = false; expect(s, "canonical_health_missing");
  s = healthy(); s.has_status = false; expect(s, "canonical_health_status_missing");
  s = healthy(); s.has_health_state = false; expect(s, "canonical_health_last_state_missing");
  s = healthy(); ++s.state_epoch; expect(s, "canonical_state_epoch_mismatch");
  s = healthy(); ++s.health_epoch; expect(s, "canonical_health_epoch_mismatch");
  s = healthy(); ++s.health_state_epoch; expect(s, "canonical_health_last_state_epoch_mismatch");
  s = healthy(); s.status_healthy = false; expect(s, "canonical_status_not_healthy");
  s = healthy(); s.measurement_accepted = false; expect(s, "canonical_measurement_not_accepted");
  s = healthy(); ++s.status_stamp_ns; expect(s, "canonical_health_evaluation_stamp_mismatch");
  s = healthy(); s.source_ns = 0; expect(s, "tracking_source_invalid");
  s = healthy(); s.receipt_steady_ns = 0; expect(s, "tracking_receipt_steady_invalid");
  s = healthy(); s.status_stamp_ns = s.health_ros_ns = 0; expect(s, "health_evaluation_ros_invalid");
  s = healthy(); s.health_steady_ns = 0; expect(s, "health_evaluation_steady_invalid");

  // Only an explicit allowlist of unavailable data/status can preserve the
  // acquisition owner. Unknown, identity, malformed and clock faults stay fatal.
  for (const auto * gate : {"canonical_state_missing", "canonical_health_missing",
      "canonical_health_last_state_missing", "canonical_status_not_healthy",
      "canonical_measurement_not_accepted",
      "tracking_receipt_ros_age_exceeded", "tracking_receipt_steady_age_exceeded",
      "health_ros_age_exceeded", "health_steady_age_exceeded"})
    check(canonical_health_motion_unavailable(gate));
  for (const auto * gate : {"", "unknown_future_gate", "canonical_health_status_missing",
      "canonical_state_epoch_mismatch", "canonical_health_epoch_mismatch",
      "canonical_health_last_state_epoch_mismatch", "canonical_health_evaluation_stamp_mismatch",
      "tracking_source_invalid", "tracking_source_after_receipt", "tracking_receipt_steady_invalid",
      "tracking_ros_clock_regression", "tracking_steady_clock_regression",
      "health_evaluation_ros_invalid", "health_evaluation_steady_invalid",
      "health_ros_clock_regression", "health_steady_clock_regression"})
    check(!canonical_health_motion_unavailable(gate));
  // Real run73 and boot896a source delays no longer suspend a healthy stream.
  s=healthy(); s.source_ns=s.check_ros_ns-252727000;
  expect(s,"");
  s.source_ns=s.check_ros_ns-264355048; expect(s,"");
  s.source_ns=s.check_ros_ns-600000000; expect(s,"");
  s.receipt_steady_ns=s.check_steady_ns-s.tracking_budget_ns-1;
  expect(s,"tracking_receipt_steady_age_exceeded");
  s=healthy(); ++s.state_sequence; ++s.tracking_sequence;
  expect(s,"");

  // Cross-product around exact bounds, invalid clocks and separate clock domains.
  const std::array<int64_t, 7> ages{-1, 0, 1, 250000000, 250000001, 800000000, 800000001};
  for (auto source_age : ages) for (auto receipt_age : ages) for (auto steady_age : ages)
    for (auto health_age : ages) for (auto health_steady_age : ages) {
      s = healthy();
      s.source_ns = s.check_ros_ns - source_age;
      s.receipt_ros_ns = s.check_ros_ns - receipt_age;
      s.receipt_steady_ns = s.check_steady_ns - steady_age;
      s.status_stamp_ns = s.health_ros_ns = s.check_ros_ns - health_age;
      s.health_steady_ns = s.check_steady_ns - health_steady_age;
      check(original_healthy(s) == (canonical_health_failure(s)[0] == '\0'));
    }
  s = healthy(); s.source_ns = s.check_ros_ns - s.tracking_budget_ns - 1;
  s.receipt_steady_ns=s.check_steady_ns-s.tracking_budget_ns-1;
  const auto message = canonical_health_diagnostic(s, canonical_health_failure(s));
  for (const auto * field : {"gate=tracking_receipt_steady_age_exceeded", "check_ros_ns=2000000000",
      "check_steady_ns=3000000000", "tracking_source_ns=1749999999", "tracking_receipt_ros_ns=1990000000",
      "tracking_receipt_steady_ns=2749999999", "tracking_sequence=103", "tracking_identity=/sunhorizon_livo_adapter#candidate",
      "state_sequence=91", "reset_epoch=7", "health_last_state_sequence=90", "health_evaluated_ros_ns=1999000000",
      "health_evaluated_steady_ns=2999000000", "tracking_budget_ns=250000000", "health_budget_ns=800000000",
      "canonical_queue_statistics=unavailable"}) check(message.find(field) != std::string::npos);
  std::cout << "canonical health boundaries, diagnostics and 16807 liveness-predicate cases passed\n";
}
