#include "sunhorizon_3d_localizer/ndt_freshness.hpp"

#include <cmath>

namespace sunhorizon_3d_localizer {
namespace {

NdtFreshnessDecision evaluate_age(
    std::int64_t origin_ns, std::int64_t observation_ns, double limit_sec,
    double future_tolerance_sec,
    const char* invalid_reason, const char* future_reason,
    const char* stale_reason) {
  if (origin_ns <= 0 || observation_ns <= 0 || !std::isfinite(limit_sec) ||
      limit_sec <= 0.0 || !std::isfinite(future_tolerance_sec) ||
      future_tolerance_sec < 0.0) {
    return {false, 0.0, invalid_reason};
  }
  if (observation_ns < origin_ns) {
    const double future_sec =
        static_cast<double>(origin_ns - observation_ns) * 1e-9;
    if (future_sec <= future_tolerance_sec) {
      return {true, 0.0, "ok"};
    }
    return {
        false,
        -future_sec,
        future_reason};
  }
  const double age_sec =
      static_cast<double>(observation_ns - origin_ns) * 1e-9;
  if (!std::isfinite(age_sec)) {
    return {false, 0.0, invalid_reason};
  }
  if (age_sec > limit_sec) {
    return {false, age_sec, stale_reason};
  }
  return {true, age_sec, "ok"};
}

}  // namespace

NdtFreshnessDecision evaluate_ndt_input_age(
    std::int64_t sensor_stamp_ns, std::int64_t receipt_at_accept_ns,
    double maximum_input_age_sec, double future_tolerance_sec) {
  return evaluate_age(
      sensor_stamp_ns, receipt_at_accept_ns, maximum_input_age_sec,
      future_tolerance_sec,
      "ndt_input_timestamp_invalid", "ndt_input_timestamp_future",
      "ndt_input_measurement_stale");
}

NdtFreshnessDecision evaluate_ndt_result_stream_age(
    std::int64_t accepted_result_receipt_ns, std::int64_t now_ns,
    double accepted_result_timeout_sec) {
  return evaluate_age(
      accepted_result_receipt_ns, now_ns, accepted_result_timeout_sec, 0.0,
      "ndt_accepted_result_receipt_invalid",
      "ndt_accepted_result_receipt_future", "ndt_result_stream_stale");
}

}  // namespace sunhorizon_3d_localizer
