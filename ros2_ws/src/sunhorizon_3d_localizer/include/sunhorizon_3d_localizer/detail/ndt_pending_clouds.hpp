#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>

#include "sunhorizon_3d_localizer/raw_scan_admission_gate.hpp"

namespace sunhorizon_3d_localizer {

struct NdtPendingCloudCounters {
  uint64_t complete_submitted{0};
  uint64_t incomplete_submitted{0};
  uint64_t complete_overwritten{0};
  uint64_t incomplete_overwritten{0};
  uint64_t complete_preserved{0};
};

namespace detail {

// Caller holds the direct queue mutex and validates strictly increasing source
// stamps/sequence/identity before push(). Completeness is only a queue policy:
// both original inputs still go through the worker's unchanged on_points gates.
// One incomplete input is retained for those gates and their diagnostics, but
// can never evict the complete pending input. No point data are copied.
template<class Cloud>
class NdtPendingClouds {
 public:
  bool push(Cloud cloud, int64_t source_ns, std::size_t raw_points,
      std::size_t minimum_raw_points) {
    const bool complete =
        evaluate_raw_scan_admission(raw_points, minimum_raw_points).accepted;
    auto& slot = complete ? complete_ : incomplete_;
    const bool overwritten = slot.has_value();
    if (complete) {
      ++counters_.complete_submitted;
      if (overwritten) ++counters_.complete_overwritten;
    } else {
      ++counters_.incomplete_submitted;
      if (overwritten) ++counters_.incomplete_overwritten;
      if (complete_) ++counters_.complete_preserved;
    }
    slot = Entry{std::move(cloud), source_ns};
    return overwritten;
  }

  std::optional<Cloud> take() {
    if (empty()) return std::nullopt;
    // Preserve the worker's strict source ordering, including the raw-skip
    // stamp watermark. The other slot remains replaceable until its dequeue.
    auto& slot = !incomplete_ ||
        (complete_ && complete_->source_ns < incomplete_->source_ns) ?
        complete_ : incomplete_;
    std::optional<Cloud> result(std::move(slot->cloud));
    slot.reset();
    return result;
  }

  bool empty() const {return !complete_ && !incomplete_;}
  void clear() {complete_.reset(); incomplete_.reset();}
  const NdtPendingCloudCounters& counters() const {return counters_;}

 private:
  struct Entry {Cloud cloud; int64_t source_ns;};
  std::optional<Entry> complete_, incomplete_;
  NdtPendingCloudCounters counters_;
};

}  // namespace detail
}  // namespace sunhorizon_3d_localizer
