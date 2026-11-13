#pragma once

#include <array>
#include <cstdio>
#include <cstdint>
#include <string>
#include <type_traits>

#include "sunhorizon_obstacle_coverage/rgbd_upstream_diagnostics.hpp"

namespace sunhorizon_obstacle_coverage {

// Diagnostic identity only. Copied with the original SourceObservations under
// their mutex; never consulted by the evaluator or optional trace exporter.
struct RgbdFaultReference {
  int64_t header_ns{0}, receipt_ns{0};
  uint64_t callback_sequence{0};
  std::array<uint64_t, 4> ages{};  // chest pair/points, belly depth/points IEEE bits.
  uint16_t health{0};  // Same nine input flags/order as upstream diagnostics.
  bool input_seen{false}, header_valid{false}, header_ambiguous{false};
};
static_assert(std::is_trivially_copyable<RgbdFaultReference>::value);
static_assert(sizeof(RgbdFaultReference) <= 64);

class RgbdFaultReferenceState {
 public:
  template<class Status>
  RgbdFaultReference capture(const Status& input, int64_t receipt_ns) noexcept {
    RgbdFaultReference r;
    r.header_ns = static_cast<int64_t>(input.header.stamp.sec) * 1000000000LL +
        input.header.stamp.nanosec;
    r.receipt_ns = receipt_ns;
    r.callback_sequence = ++sequence_;
    r.ages = upstream_diagnostics::age_bits(input);
    r.health = upstream_diagnostics::health_bits(input);
    r.input_seen = true;
    r.header_valid = input.header.stamp.sec >= 0 &&
        input.header.stamp.nanosec < 1000000000U && r.header_ns > 0;
    // Malformed headers cannot advance the valid-header high water mark.
    r.header_ambiguous = !r.header_valid || header_.observe(r.header_ns);
    return r;
  }
 private:
  uint64_t sequence_{0};
  upstream_diagnostics::HeaderIdentity header_;
};

// Only called as an argument of the existing throttled unhealthy log. All
// integer fields are bounded in length; no clock sampling or source inference.
inline std::string rgbd_fault_reference_diagnostics(
    const RgbdFaultReference& r, int64_t decision_ns) {
  char text[512];
  std::snprintf(text, sizeof(text),
      "rgbd_fault_ref_v1(D_ns=%lld,H_ns=%lld,C_ns=%lld,callback_seq=%llu,"
      "input_seen=%u,header_valid=%u,header_ambiguous=%u,"
      "age_bits=[%llu,%llu,%llu,%llu],health_bits=%u)",
      static_cast<long long>(decision_ns), static_cast<long long>(r.header_ns),
      static_cast<long long>(r.receipt_ns),
      static_cast<unsigned long long>(r.callback_sequence),
      unsigned(r.input_seen), unsigned(r.header_valid), unsigned(r.header_ambiguous),
      static_cast<unsigned long long>(r.ages[0]),
      static_cast<unsigned long long>(r.ages[1]),
      static_cast<unsigned long long>(r.ages[2]),
      static_cast<unsigned long long>(r.ages[3]), unsigned(r.health));
  return text;
}

}  // namespace sunhorizon_obstacle_coverage
