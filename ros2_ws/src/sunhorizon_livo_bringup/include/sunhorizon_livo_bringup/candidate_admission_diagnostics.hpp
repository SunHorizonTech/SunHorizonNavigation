#pragma once

#include <algorithm>
#include <cstdint>
#include <optional>
#include <string>

#include "sunhorizon_livo_bringup/measurement_freshness.hpp"

namespace sunhorizon_livo_bringup
{

struct CandidateAdmissionDiagnosticsSnapshot
{
  std::uint64_t freshness_stale_rejections{0U};
  std::uint64_t freshness_future_rejections{0U};
  std::uint64_t freshness_invalid_rejections{0U};
  std::uint64_t current_consecutive_freshness_rejections{0U};
  std::uint64_t maximum_consecutive_freshness_rejections{0U};
  std::string current_freshness_reason{"waiting_for_vendor_odometry"};
  std::uint64_t aggregate_suppressions{0U};
  std::string current_aggregate_suppression_reason{"none"};
  std::uint64_t candidate_published{0U};
  double maximum_candidate_sensor_stamp_gap_sec{0.0};
  double maximum_candidate_receipt_gap_sec{0.0};
};

class CandidateAdmissionDiagnostics
{
public:
  void observe_freshness(const MeasurementFreshnessDecision & decision)
  {
    current_freshness_reason_ = decision.reason;
    if (decision.accepted) {
      current_consecutive_freshness_rejections_ = 0U;
      return;
    }

    ++current_consecutive_freshness_rejections_;
    maximum_consecutive_freshness_rejections_ = std::max(
      maximum_consecutive_freshness_rejections_,
      current_consecutive_freshness_rejections_);
    if (decision.reason == "measurement_stale") {
      ++freshness_stale_rejections_;
    } else if (decision.reason == "measurement_stamp_in_future") {
      ++freshness_future_rejections_;
    } else {
      ++freshness_invalid_rejections_;
    }
  }

  void record_aggregate_suppression(const std::string & reason)
  {
    ++aggregate_suppressions_;
    current_aggregate_suppression_reason_ =
      reason.empty() ? "all_inputs_unhealthy_unspecified" : reason;
  }

  void clear_aggregate_suppression()
  {
    current_aggregate_suppression_reason_ = "none";
  }

  void record_candidate_published(
    std::int64_t sensor_stamp_nanoseconds,
    std::int64_t receipt_nanoseconds)
  {
    ++candidate_published_;
    update_gap(
      sensor_stamp_nanoseconds, &last_candidate_sensor_stamp_nanoseconds_,
      &maximum_candidate_sensor_stamp_gap_sec_);
    update_gap(
      receipt_nanoseconds, &last_candidate_receipt_nanoseconds_,
      &maximum_candidate_receipt_gap_sec_);
  }

  CandidateAdmissionDiagnosticsSnapshot snapshot() const
  {
    return {
      freshness_stale_rejections_,
      freshness_future_rejections_,
      freshness_invalid_rejections_,
      current_consecutive_freshness_rejections_,
      maximum_consecutive_freshness_rejections_,
      current_freshness_reason_,
      aggregate_suppressions_,
      current_aggregate_suppression_reason_,
      candidate_published_,
      maximum_candidate_sensor_stamp_gap_sec_,
      maximum_candidate_receipt_gap_sec_};
  }

private:
  static void update_gap(
    std::int64_t value_nanoseconds,
    std::optional<std::int64_t> * last_nanoseconds,
    double * maximum_gap_sec)
  {
    if (value_nanoseconds <= 0) {
      return;
    }
    if (*last_nanoseconds && value_nanoseconds > **last_nanoseconds) {
      *maximum_gap_sec = std::max(
        *maximum_gap_sec,
        static_cast<double>(value_nanoseconds - **last_nanoseconds) * 1.0e-9);
    }
    if (!*last_nanoseconds || value_nanoseconds > **last_nanoseconds) {
      *last_nanoseconds = value_nanoseconds;
    }
  }

  std::uint64_t freshness_stale_rejections_{0U};
  std::uint64_t freshness_future_rejections_{0U};
  std::uint64_t freshness_invalid_rejections_{0U};
  std::uint64_t current_consecutive_freshness_rejections_{0U};
  std::uint64_t maximum_consecutive_freshness_rejections_{0U};
  std::string current_freshness_reason_{"waiting_for_vendor_odometry"};
  std::uint64_t aggregate_suppressions_{0U};
  std::string current_aggregate_suppression_reason_{"none"};
  std::uint64_t candidate_published_{0U};
  std::optional<std::int64_t> last_candidate_sensor_stamp_nanoseconds_;
  std::optional<std::int64_t> last_candidate_receipt_nanoseconds_;
  double maximum_candidate_sensor_stamp_gap_sec_{0.0};
  double maximum_candidate_receipt_gap_sec_{0.0};
};

}  // namespace sunhorizon_livo_bringup
