#pragma once

#include "sunhorizon_obstacle_coverage/camera_freshness_policy.hpp"
#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include "sunhorizon_obstacle_coverage/spatial_delivery_result.hpp"

namespace sunhorizon_obstacle_coverage {

// Wire-independent mirror of SpatialSourceEvidence. Receipt clocks belong to
// the pinned robot boot. Contribution is query-specific; measurement identity
// (including its original expiry) is immutable across queries.
struct SpatialSourceObservation {
  std::uint8_t source_id{0}, contribution{0};
  std::string producer_epoch, frame_id;
  std::uint64_t frame_sequence{0};
  std::int64_t source_ros_ns{0}, first_receipt_ros_ns{0}, first_receipt_steady_ns{0};
  std::int64_t valid_until_ros_ns{0}, valid_until_steady_ns{0};
  bool geometry_valid{false};
};
struct SpatialCoverageObservation {
  std::uint8_t schema_version{0};
  std::string contract, query_digest, producer_epoch, clock_boot_id, frame_id, reason;
  std::int64_t header_ros_ns{0}, anchor_source_ros_ns{0}, evaluation_steady_ns{0};
  std::uint64_t anchor_frame_sequence{0}, evaluation_sequence{0}, input_generation{0};
  bool evaluation_complete{false};
  std::uint32_t total_cells{0}, free_cells{0}, occupied_cells{0}, unknown_cells{0};
  std::vector<SpatialSourceObservation> sources;
};
struct SpatialSourceIdentity {
  std::uint8_t source_id{0};
  std::string producer_epoch, frame_id;
};
struct SpatialEvidenceTrackerConfig {
  std::string owner_epoch, contract, query_digest, clock_boot_id, frame_id, producer_epoch;
  // Discovered/authenticated by the owner before construction, not auto-pinned
  // from an arbitrary arriving message. Belly (id 0) is mandatory.
  std::vector<SpatialSourceIdentity> source_identities;
  std::int64_t maximum_age_ns{kDefaultCameraAgeNs};
};
struct ValidatedSpatialEvidence {
  bool valid{false};
  // Same downstream meaning as GeometryRecoveryGate: 0 hard, 1 clear, 2 wait.
  std::uint8_t disposition{0};
  std::int64_t source_ros_ns{0}, receipt_steady_ns{0};
  std::uint64_t frame_sequence{0}, evaluation_sequence{0};
  std::string owner_epoch, contract, reason{"spatial_evidence_missing"};
  std::int64_t valid_until_ros_ns{0}, valid_until_steady_ns{0};
};

// Observation recovery never rearms the independent final supervisor, which
// retains Sticky. Occupancy recovery additionally requires a new fully Clear
// anchor; identity/structure faults and explicit revocation never recover.
enum class SpatialRecoveryPolicy { Sticky, RecoveryOnNewAnchor, CoverageObservations };

// One tracker per authenticated producer epoch; serialize calls. No reset,
// latest-message join, or clock reads here. Original upstream health MUST be
// combined outside this tracker;
// a valid spatial Wait never overrides an upstream hard fault.
class SpatialEvidenceTracker {
 public:
  static constexpr std::int64_t kMaximumAgeNs = 300000000;
  static constexpr std::int64_t kClockToleranceNs = 100000000;
  static constexpr std::uint8_t kBelly = 0, kChest = 1, kAiry = 2;
  static constexpr std::uint8_t kFreeContribution = 1, kOccupiedContribution = 2;

  explicit SpatialEvidenceTracker(SpatialEvidenceTrackerConfig config,
      SpatialRecoveryPolicy recovery = SpatialRecoveryPolicy::Sticky)
      : config_(std::move(config)), recovery_policy_(recovery) {
    validate_camera_age(config_.maximum_age_ns);
    if (!bounded(config_.owner_epoch) || !bounded(config_.contract, 16384) ||
        !bounded(config_.query_digest) || !bounded(config_.clock_boot_id) ||
        !bounded(config_.frame_id) || !bounded(config_.producer_epoch) ||
        config_.source_identities.empty() || config_.source_identities.size() > 3) {
      throw std::invalid_argument("spatial_tracker_config_invalid");
    }
    unsigned mask = 0;
    for (const auto& source : config_.source_identities) {
      if (config_.maximum_age_ns > kDefaultCameraAgeNs && source.source_id == kAiry)
        throw std::invalid_argument("extended_camera_age_cannot_extend_airy");
      if (source.source_id > kAiry || !bounded(source.producer_epoch) ||
          !bounded(source.frame_id) || (mask & (1U << source.source_id))) {
        throw std::invalid_argument("spatial_tracker_source_config_invalid");
      }
      mask |= 1U << source.source_id;
    }
    if (!(mask & 1U)) throw std::invalid_argument("spatial_tracker_belly_identity_required");
  }

  bool observe(const SpatialCoverageObservation& input, const std::string& owner,
               std::int64_t receipt_ros, std::int64_t receipt_steady) {
    if (!clock_valid(receipt_ros, receipt_steady)) return false;
    if (owner != config_.owner_epoch) return reject("spatial_owner_changed");
    if (input.schema_version != 1 || input.contract != config_.contract ||
        input.query_digest != config_.query_digest || input.clock_boot_id != config_.clock_boot_id ||
        input.frame_id != config_.frame_id || input.producer_epoch != config_.producer_epoch) {
      return reject("spatial_identity_changed");
    }
    if (input.total_cells == 0 || input.total_cells > 4096 ||
        std::uint64_t(input.free_cells) + input.occupied_cells + input.unknown_cells != input.total_cells ||
        input.reason.size() > 512 || input.sources.empty() || input.sources.size() > 3 ||
        !input.anchor_frame_sequence || !input.evaluation_sequence || !input.input_generation ||
        input.header_ros_ns <= 0 || input.header_ros_ns > receipt_ros ||
        input.evaluation_steady_ns <= 0 || input.evaluation_steady_ns > receipt_steady ||
        input.anchor_source_ros_ns <= 0 || input.anchor_source_ros_ns > input.header_ros_ns ||
        (!input.evaluation_complete && input.reason != "source_expired_before_egress")) {
      return reject("spatial_structure_or_evaluation_invalid");
    }
    if (seen_) {
      // Do not hide a missed validity interval merely because a new callback
      // runs before the timer notices the old result expired. The final
      // supervisor keeps this failure sticky even if the new anchor is fresh.
      if (!fault_) evaluate_validity(receipt_ros, receipt_steady);
      if (fault_ && failure_state_==SpatialDeliveryState::InternalFault) return false;
      if (input.anchor_frame_sequence == evidence_.anchor_frame_sequence) {
        if (!same(input, evidence_)) return reject("spatial_same_anchor_mutated");
        // Both transport and source validity expire from FIRST observation.
        // Even a byte-identical repeat cannot change either receipt or expiry.
        return evaluate_validity(receipt_ros, receipt_steady);
      }
      if (input.anchor_frame_sequence < evidence_.anchor_frame_sequence ||
          input.anchor_source_ros_ns <= evidence_.anchor_source_ros_ns ||
          input.evaluation_sequence <= evidence_.evaluation_sequence ||
          input.input_generation <= evidence_.input_generation ||
          input.header_ros_ns < evidence_.header_ros_ns ||
          input.evaluation_steady_ns < evidence_.evaluation_steady_ns) {
        return reject("spatial_generation_reordered");
      }
    }
    unsigned source_mask = 0;
    bool free_contributor = false, occupied_contributor = false;
    bool unavailable = false;
    auto next_highwater = highwater_;
    std::int64_t expiry_ros = input.header_ros_ns, expiry_steady = input.evaluation_steady_ns;
    // Addition is safe after this explicit bound, including adversarial int64.
    if (!can_add_budget(expiry_ros) || !can_add_budget(expiry_steady)) return reject("spatial_time_overflow");
    expiry_ros += config_.maximum_age_ns; expiry_steady += config_.maximum_age_ns;
    for (const auto& source : input.sources) {
      if (source.source_id > kAiry || (source_mask & (1U << source.source_id))) {
        return reject("spatial_duplicate_or_unknown_source");
      }
      source_mask |= 1U << source.source_id;
      const auto identity = std::find_if(config_.source_identities.begin(), config_.source_identities.end(),
          [&source](const auto& pin) { return pin.source_id == source.source_id; });
      if (identity == config_.source_identities.end() || source.producer_epoch != identity->producer_epoch ||
          source.frame_id != identity->frame_id || !source.frame_sequence || source.contribution > 3 ||
          (source.source_id == kAiry && (source.contribution & kFreeContribution))) {
        return reject("spatial_source_identity_or_contribution_invalid");
      }
      if (!source_times_valid(source, input)) return reject("spatial_source_times_invalid");
      const auto previous = std::find_if(next_highwater.begin(), next_highwater.end(),
          [&source](const auto& old) { return old.source_id == source.source_id; });
      if (previous != next_highwater.end()) {
        if (source.frame_sequence < previous->frame_sequence || source.source_ros_ns < previous->source_ros_ns ||
            source.first_receipt_ros_ns < previous->first_receipt_ros_ns ||
            source.first_receipt_steady_ns < previous->first_receipt_steady_ns) {
          return reject("spatial_source_reordered");
        }
        if (source.frame_sequence == previous->frame_sequence) {
          if (!same_measurement(source, *previous)) return reject("spatial_source_same_sequence_mutated");
        } else if (source.source_ros_ns <= previous->source_ros_ns) {
          return reject("spatial_source_sequence_without_new_measurement");
        }
        *previous = source;
      } else {
        next_highwater.push_back(source);
      }
      const bool mandatory = source.source_id == kBelly;
      if (mandatory && (source.source_ros_ns != input.anchor_source_ros_ns ||
                        source.frame_sequence != input.anchor_frame_sequence)) {
        return reject("spatial_belly_anchor_mismatch");
      }
      if (mandatory || source.contribution != 0) {
        if (!source_fresh(source, receipt_ros, receipt_steady)) unavailable=true;
        else if (!source.geometry_valid) return reject("spatial_required_evidence_stale_or_invalid");
        expiry_ros = std::min(expiry_ros, source.valid_until_ros_ns);
        expiry_steady = std::min(expiry_steady, source.valid_until_steady_ns);
      }
      free_contributor |= (source.contribution & kFreeContribution) != 0;
      occupied_contributor |= (source.contribution & kOccupiedContribution) != 0;
    }
    if (!(source_mask & 1U)) return reject("spatial_belly_missing");
    if (!fresh(receipt_ros,input.header_ros_ns) || !fresh(receipt_steady,input.evaluation_steady_ns) ||
        !fresh(input.header_ros_ns,input.anchor_source_ros_ns))
      unavailable=true;
    // An expiry marker cannot qualify a fresh but incomplete computation.
    if (!input.evaluation_complete && !unavailable) return reject("spatial_structure_or_evaluation_invalid");
    if ((input.free_cells > 0 && !free_contributor) ||
        (input.occupied_cells > 0 && !occupied_contributor) ||
        (occupied_contributor && input.occupied_cells == 0)) {
      return reject("spatial_cell_contribution_mismatch");
    }
    // Positive occupancy is dominant even when other cells are unknown/free.
    if (input.occupied_cells > 0) reject("spatial_occupied",SpatialDeliveryState::MotionUnavailable,false,true);
    else if (unavailable) reject("spatial_required_evidence_stale_or_invalid",SpatialDeliveryState::MotionUnavailable,true);
    evidence_ = input; highwater_ = std::move(next_highwater);
    seen_ = true;
    // Same-anchor repeats returned above; recovery keeps every measurement's
    // original receipt/expiry. Unknown may recover a deadline to Wait, but
    // cannot clear an occupancy latch. Only a complete new Clear can do that.
    const bool recoverable = deadline_only_ ||
      (recovery_policy_ == SpatialRecoveryPolicy::CoverageObservations &&
       occupancy_only_ && input.free_cells == input.total_cells);
    if (fault_ && recovery_policy_ != SpatialRecoveryPolicy::Sticky &&
        failure_state_ == SpatialDeliveryState::MotionUnavailable && recoverable &&
        input.evaluation_complete && !unavailable && input.occupied_cells == 0) {
      fault_ = false; deadline_only_ = false; occupancy_only_ = false; reason_.clear();
    }
    if (fault_) return false;
    first_receipt_ros_ = receipt_ros; first_receipt_steady_ = receipt_steady;
    valid_until_ros_ = expiry_ros; valid_until_steady_ = expiry_steady;
    return evaluate_validity(receipt_ros, receipt_steady);
  }

  ValidatedSpatialEvidence evaluate(std::int64_t now_ros, std::int64_t now_steady) {
    if (!clock_valid(now_ros, now_steady) || !seen_ || !evaluate_validity(now_ros, now_steady)) return failed();
    const bool clear = evidence_.free_cells == evidence_.total_cells;
    return {true, static_cast<std::uint8_t>(clear ? 1 : 2), evidence_.anchor_source_ros_ns,
            first_receipt_steady_, evidence_.anchor_frame_sequence, evidence_.evaluation_sequence,
            config_.owner_epoch, config_.contract, clear ? "spatial_clear" : "spatial_visibility_wait",
            valid_until_ros_, valid_until_steady_};
  }
  void invalidate(const std::string& reason) { reject(reason); }
  void invalidate_motion(const std::string& reason) { reject(reason,SpatialDeliveryState::MotionUnavailable); }
  void invalidate_deadline(const std::string& reason) {
    reject(reason,SpatialDeliveryState::MotionUnavailable,true);
  }
  SpatialDeliveryResult delivery_result() const {
    if (fault_) return {failure_state_,reason_};
    return seen_ ? SpatialDeliveryResult{SpatialDeliveryState::Accepted,{}} :
      SpatialDeliveryResult{SpatialDeliveryState::InternalFault,"spatial_evidence_missing"};
  }

 private:
  static bool bounded(const std::string& text, std::size_t maximum = 256) {
    return !text.empty() && text.size() <= maximum;
  }
  bool can_add_budget(std::int64_t t) const { return t > 0 && t <= std::numeric_limits<std::int64_t>::max() - config_.maximum_age_ns; }
  bool fresh(std::int64_t now, std::int64_t stamp) const {
    return stamp > 0 && now >= stamp && now - stamp <= config_.maximum_age_ns;
  }
  static bool same_measurement(const SpatialSourceObservation& a, const SpatialSourceObservation& b) {
    return a.source_id == b.source_id && a.producer_epoch == b.producer_epoch && a.frame_id == b.frame_id &&
        a.frame_sequence == b.frame_sequence && a.source_ros_ns == b.source_ros_ns &&
        a.first_receipt_ros_ns == b.first_receipt_ros_ns && a.first_receipt_steady_ns == b.first_receipt_steady_ns &&
        a.valid_until_ros_ns == b.valid_until_ros_ns && a.valid_until_steady_ns == b.valid_until_steady_ns;
  }
  static bool same(const SpatialCoverageObservation& a, const SpatialCoverageObservation& b) {
    if (a.schema_version != b.schema_version || a.contract != b.contract || a.query_digest != b.query_digest ||
        a.producer_epoch != b.producer_epoch || a.clock_boot_id != b.clock_boot_id || a.frame_id != b.frame_id ||
        a.reason != b.reason || a.header_ros_ns != b.header_ros_ns || a.anchor_source_ros_ns != b.anchor_source_ros_ns ||
        a.evaluation_steady_ns != b.evaluation_steady_ns || a.anchor_frame_sequence != b.anchor_frame_sequence ||
        a.evaluation_sequence != b.evaluation_sequence || a.input_generation != b.input_generation ||
        a.evaluation_complete != b.evaluation_complete || a.total_cells != b.total_cells ||
        a.free_cells != b.free_cells || a.occupied_cells != b.occupied_cells || a.unknown_cells != b.unknown_cells ||
        a.sources.size() != b.sources.size()) return false;
    for (std::size_t i = 0; i < a.sources.size(); ++i) {
      if (!same_measurement(a.sources[i], b.sources[i]) || a.sources[i].contribution != b.sources[i].contribution ||
          a.sources[i].geometry_valid != b.sources[i].geometry_valid) return false;
    }
    return true;
  }
  bool source_times_valid(const SpatialSourceObservation& s, const SpatialCoverageObservation& input) const {
    if (s.source_ros_ns <= 0 || s.source_ros_ns > input.anchor_source_ros_ns ||
        s.first_receipt_ros_ns < s.source_ros_ns || s.first_receipt_ros_ns > input.header_ros_ns ||
        s.first_receipt_steady_ns <= 0 || s.first_receipt_steady_ns > input.evaluation_steady_ns ||
        s.valid_until_ros_ns < s.source_ros_ns || s.valid_until_steady_ns <= 0 ||
        s.valid_until_ros_ns - s.source_ros_ns > config_.maximum_age_ns ||
        s.valid_until_ros_ns - s.first_receipt_ros_ns > config_.maximum_age_ns ||
        s.valid_until_steady_ns - s.first_receipt_steady_ns > config_.maximum_age_ns) return false;
    return s.valid_until_ros_ns - s.first_receipt_ros_ns ==
           s.valid_until_steady_ns - s.first_receipt_steady_ns;
  }
  bool source_fresh(const SpatialSourceObservation& s, std::int64_t ros, std::int64_t steady) const {
    return fresh(ros, s.source_ros_ns) && fresh(ros, s.first_receipt_ros_ns) &&
           fresh(steady, s.first_receipt_steady_ns) && ros <= s.valid_until_ros_ns && steady <= s.valid_until_steady_ns;
  }
  bool evaluate_validity(std::int64_t ros, std::int64_t steady) {
    if (fault_) return false;
    if (!seen_) return false;
    if (!fresh(ros, evidence_.anchor_source_ros_ns) || !fresh(ros, evidence_.header_ros_ns) ||
        !fresh(steady, evidence_.evaluation_steady_ns) || !fresh(ros, first_receipt_ros_) ||
        !fresh(steady, first_receipt_steady_) || ros > valid_until_ros_ || steady > valid_until_steady_) {
      return reject("spatial_frozen_evidence_expired",SpatialDeliveryState::MotionUnavailable,true);
    }
    return true;
  }
  bool reject(const std::string& reason, SpatialDeliveryState state=SpatialDeliveryState::InternalFault,
      bool deadline = false, bool occupancy = false) {
    const bool hardens_recoverable = fault_ && !deadline &&
      (deadline_only_ || (occupancy_only_ && !occupancy)) &&
      recovery_policy_ != SpatialRecoveryPolicy::Sticky;
    if (!fault_ || hardens_recoverable ||
        (failure_state_==SpatialDeliveryState::MotionUnavailable && state==SpatialDeliveryState::InternalFault)) {
      reason_ = reason;failure_state_=state;
    }
    // A later deadline cannot erase occupancy, and an explicit stop cannot be
    // disguised by an older occupancy reason and then cleared by new data.
    occupancy_only_ = state == SpatialDeliveryState::MotionUnavailable &&
      (occupancy || deadline) &&
      (fault_ ? (occupancy_only_ || (deadline_only_ && occupancy)) : occupancy);
    deadline_only_ = state == SpatialDeliveryState::MotionUnavailable && deadline && (!fault_ || deadline_only_);
    fault_ = true; return false;
  }
  bool clock_valid(std::int64_t ros, std::int64_t steady) {
    if (fault_ && failure_state_==SpatialDeliveryState::InternalFault) return false;
    if (ros <= 0 || steady <= 0 || ros < last_ros_ || steady < last_steady_) return reject("spatial_clock_invalid");
    if (!anchor_ros_) { anchor_ros_ = ros; anchor_steady_ = steady; }
    const auto dr = ros - anchor_ros_, ds = steady - anchor_steady_;
    if ((dr >= ds && dr - ds > kClockToleranceNs) || (ds > dr && ds - dr > kClockToleranceNs)) {
      return reject("spatial_clock_progress_invalid");
    }
    last_ros_ = ros; last_steady_ = steady; return true;
  }
  ValidatedSpatialEvidence failed() const {
    ValidatedSpatialEvidence value; value.reason = fault_ ? reason_ : "spatial_evidence_missing"; return value;
  }
  SpatialEvidenceTrackerConfig config_;
  SpatialRecoveryPolicy recovery_policy_;
  SpatialCoverageObservation evidence_;
  std::vector<SpatialSourceObservation> highwater_;
  bool seen_{false}, fault_{false}, deadline_only_{false}, occupancy_only_{false};
  SpatialDeliveryState failure_state_{SpatialDeliveryState::InternalFault};
  std::string reason_;
  std::int64_t first_receipt_ros_{0}, first_receipt_steady_{0}, valid_until_ros_{0}, valid_until_steady_{0};
  std::int64_t last_ros_{0}, last_steady_{0}, anchor_ros_{0}, anchor_steady_{0};
};

}  // namespace sunhorizon_obstacle_coverage
