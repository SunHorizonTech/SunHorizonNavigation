#pragma once

#include "sunhorizon_velocity_supervisor/coverage_decision_tracker.hpp"
#include "sunhorizon_obstacle_coverage/spatial_evidence_tracker.hpp"

namespace sunhorizon_velocity_supervisor {

// Receives ONE authenticated schema-2 wrapper, never a latest-message join.
// The legacy tracker checks upstream health/publication freshness; the spatial
// tracker additionally checks every contributing measurement's ORIGINAL expiry.
class SpatialCoverageDecisionTracker {
 public:
  explicit SpatialCoverageDecisionTracker(
      sunhorizon_obstacle_coverage::SpatialEvidenceTrackerConfig config,
      std::string source_mode="lidar-rgbd-v1",
      std::int64_t report_age_ns=CoverageDecisionTracker::kFreshnessNs)
      : upstream_(config.owner_epoch, config.contract, config.frame_id,std::move(source_mode),config.maximum_age_ns,report_age_ns), spatial_(config) {}

  bool observe(const CombinedCoverageEvidence& decision,
      const sunhorizon_obstacle_coverage::SpatialCoverageObservation& spatial,
      const std::string& owner, std::int64_t receipt_ros, std::int64_t receipt_steady) {
    // Schema 2 deliberately leaves obsolete ray-region fields empty. Reject
    // mixed contracts instead of interpreting a legacy payload as spatial.
    if (decision.schema != 2 || !decision.contract.empty() ||
        decision.region_code != 0 || decision.region_disposition != 0 ||
        decision.source_ros_ns != 0 || decision.frame_sequence != 0 ||
        decision.evaluation_sequence != 0 || decision.complete ||
        decision.frame_id != spatial.frame_id ||
        decision.header_ros_ns < spatial.header_ros_ns) {
      invalidate("spatial_wrapper_identity_mismatch"); return false;
    }
    if (!spatial_.observe(spatial, owner, receipt_ros, receipt_steady)) {
      upstream_.invalidate("spatial_evidence_rejected"); return false;
    }
    const auto checked = spatial_.evaluate(receipt_ros, receipt_steady);
    if (decision.disposition == 0 && !decision.non_region_healthy) {
      invalidate("spatial_upstream_hard_fault"); return false;
    }
    if (!checked.valid || checked.disposition != decision.disposition) {
      invalidate("spatial_wrapper_disposition_mismatch"); return false;
    }
    // Internal projection only: no schema-1 message is published or accepted
    // as spatial evidence. This shares the existing upstream health checks.
    auto upstream = decision;
    upstream.schema = 1;
    upstream.contract = spatial.contract;
    upstream.source_ros_ns = spatial.anchor_source_ros_ns;
    upstream.frame_sequence = spatial.anchor_frame_sequence;
    upstream.evaluation_sequence = spatial.evaluation_sequence;
    upstream.complete = spatial.evaluation_complete;
    upstream.region_disposition = decision.disposition;
    upstream.region_code = decision.disposition == 1 ? 1 : 3;
    if (!upstream_.observe(upstream, owner, receipt_ros, receipt_steady)) {
      spatial_.invalidate("spatial_upstream_evidence_rejected"); return false;
    }
    return true;
  }

  ValidatedCoverageDecision evaluate(std::int64_t ros, std::int64_t steady) {
    const auto spatial = spatial_.evaluate(ros, steady);
    if (!spatial.valid) return {false, 0, 0, 0, 0, 0, {}, {}, spatial.reason};
    auto output = upstream_.evaluate(ros, steady);
    if (!output.valid) { spatial_.invalidate(output.reason); return output; }
    if (output.disposition != spatial.disposition ||
        output.frame_sequence != spatial.frame_sequence ||
        output.evaluation_sequence != spatial.evaluation_sequence) {
      invalidate("spatial_wrapper_snapshot_changed");
      output.valid = false; output.reason = "spatial_wrapper_snapshot_changed";
      return output;
    }
    output.receipt_steady_ns = spatial.receipt_steady_ns;
    return output;
  }

  void invalidate(const std::string& reason) {
    upstream_.invalidate(reason); spatial_.invalidate(reason);
  }
 private:
  CoverageDecisionTracker upstream_;
  sunhorizon_obstacle_coverage::SpatialEvidenceTracker spatial_;
};
}  // namespace sunhorizon_velocity_supervisor
