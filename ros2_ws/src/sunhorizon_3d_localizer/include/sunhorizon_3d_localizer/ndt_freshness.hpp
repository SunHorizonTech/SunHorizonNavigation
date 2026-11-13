#pragma once

#include <cstdint>
#include <string>

namespace sunhorizon_3d_localizer {

struct NdtFreshnessDecision {
  bool accepted{false};
  double age_sec{0.0};
  std::string reason{"timestamp_invalid"};
};

// Validate the absolute age of a sensor-stamped result exactly once, when the
// completed NDT result is about to be accepted. Timestamps must be positive;
// a bounded clock-lead tolerance is clamped to zero age, and larger future
// measurements fail closed.
NdtFreshnessDecision evaluate_ndt_input_age(
    std::int64_t sensor_stamp_ns, std::int64_t receipt_at_accept_ns,
    double maximum_input_age_sec, double future_tolerance_sec);

// Validate liveness of the accepted-result stream using the local receipt time
// captured at acceptance. This deliberately does not reinterpret sensor time.
NdtFreshnessDecision evaluate_ndt_result_stream_age(
    std::int64_t accepted_result_receipt_ns, std::int64_t now_ns,
    double accepted_result_timeout_sec);

}  // namespace sunhorizon_3d_localizer
