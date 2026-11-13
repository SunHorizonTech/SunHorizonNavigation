#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <utility>
#include "sunhorizon_obstacle_coverage/spatial_delivery_result.hpp"

namespace sunhorizon_perception_runtime::detail {

template<class Evidence>
bool expire_spatial_evidence(Evidence& evidence, int64_t ros, int64_t steady) {
  if (!evidence.evaluation_complete) return false; // Do not hide an existing internal error.
  for (const auto& source : evidence.sources) {
    if ((source.source_id == 0 || source.contribution) &&
        (ros > source.valid_until_ros_ns || steady > source.valid_until_steady_ns)) {
      evidence.evaluation_complete = false;
      evidence.reason = "source_expired_before_egress";
      return true;
    }
  }
  return false;
}

template<class Evidence, class Fault>
void apply_spatial_delivery_result(Evidence& evidence,
    const sunhorizon_obstacle_coverage::SpatialDeliveryResult& result, Fault fault) {
  sunhorizon_obstacle_coverage::propagate_spatial_internal_fault(result,fault);
  if (result.accepted() && !evidence.evaluation_complete) {
    fault("spatial_consumer_accepted_incomplete_evidence");
    return;
  }
  if (!result.accepted()) {
    // Observers must see this stop, not a newer positive frame which the
    // authoritative consumer rejected. Original measurement clocks stay intact.
    evidence.evaluation_complete=false;evidence.reason=result.reason;
  }
}

// Serialize only bounded direct delivery and observer queue replacement. DDS
// never calls into this owner or holds its mutex. A callback may latch a fault,
// but must not recursively submit/revoke/close this owner or join its threads.
template<class Evidence>
class SpatialEvidenceDelivery {
 public:
  template<class Prepare, class Deliver, class Observe>
  bool submit(Evidence evidence, Prepare prepare, Deliver deliver, Observe observe) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_ || revoked_) return false;
    submit_locked(std::move(evidence), prepare, deliver, observe);
    return true;
  }

  template<class Prepare, class Deliver, class Observe>
  bool revoke(Prepare prepare, Deliver deliver, Observe observe) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_ || revoked_) return false;
    revoked_ = true;
    if (!last_ || !last_->evaluation_complete) return false;
    auto evidence = *last_;
    evidence.evaluation_complete = false;
    submit_locked(std::move(evidence), prepare, deliver, observe);
    return true;
  }

  void close() {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true;
  }

 private:
  template<class Prepare, class Deliver, class Observe>
  void submit_locked(Evidence evidence, Prepare prepare, Deliver deliver, Observe observe) {
    prepare(evidence);
    evidence.evaluation_sequence = ++sequence_;
    last_ = evidence;  // Retain revocation identity even if a sink throws.
    deliver(evidence);
    last_ = evidence;
    observe(std::move(evidence));  // Only a bounded queue handoff, never DDS.
  }

  std::mutex mutex_;
  std::optional<Evidence> last_;
  uint64_t sequence_{0};
  bool closed_{false}, revoked_{false};
};

}  // namespace sunhorizon_perception_runtime::detail
