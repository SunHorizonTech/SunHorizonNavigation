#pragma once

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace sunhorizon_velocity_supervisor {

// Wire-independent projection of ONE ObstacleCoverageDecision. The adapter must
// authenticate a unique publisher in the graph and its actual message GID
// before supplying owner_epoch. Never join independently received messages.
struct CombinedCoverageEvidence {
  std::uint8_t schema{0}, disposition{0}, region_code{0}, region_disposition{0};
  bool non_region_healthy{false}, complete{false};
  bool healthy{false}, forward{false}, near{false}, rear{false};
  std::int64_t header_ros_ns{0}, source_ros_ns{0};
  std::uint64_t frame_sequence{0}, evaluation_sequence{0};
  std::string frame_id, profile, contract;
  std::vector<std::string> required_sources, healthy_sources;
};

struct ValidatedCoverageDecision {
  bool valid{false};
  std::uint8_t disposition{0};
  std::int64_t source_ros_ns{0}, receipt_steady_ns{0};
  std::uint64_t frame_sequence{0}, evaluation_sequence{0};
  std::string owner_epoch, contract, reason{"coverage_decision_missing"};
};

// A fault is sticky for this authenticated producer epoch. Reconstruct only
// during explicit disarmed initialization, never to recover an active task.
class CoverageDecisionTracker {
 public:
  static constexpr std::int64_t kFreshnessNs = 300000000;
  CoverageDecisionTracker(std::string owner, std::string contract,
                          std::string frame = "base_footprint",
                          std::string source_mode = "lidar-rgbd-v1",
                          std::int64_t maximum_source_age_ns = kFreshnessNs,
                          std::int64_t report_age_ns = kFreshnessNs)
      : maximum_source_age_ns_(maximum_source_age_ns), report_age_ns_(report_age_ns), owner_(std::move(owner)), contract_(std::move(contract)), frame_(std::move(frame)),
        source_mode_(std::move(source_mode)) {
    if (report_age_ns_ != kFreshnessNs &&
        !(report_age_ns_ == 600000000 && maximum_source_age_ns_ == 1000000000 && source_mode_ == "rgbd-only-v1"))
      throw std::invalid_argument("coverage_decision_report_age_invalid");
    if(maximum_source_age_ns_!=kFreshnessNs && maximum_source_age_ns_!=500000000 &&
       !(maximum_source_age_ns_==1000000000 && source_mode_=="rgbd-only-v1"))
      throw std::invalid_argument("coverage_decision_source_age_invalid");
    if(source_mode_!="lidar-rgbd-v1" && source_mode_!="rgbd-only-v1")
      throw std::invalid_argument("coverage_decision_source_mode_invalid");
    if (owner_.empty() || contract_.empty() || frame_.empty()) {
      throw std::invalid_argument("coverage_decision_identity_required");
    }
  }

  bool observe(const CombinedCoverageEvidence& input, const std::string& owner,
               std::int64_t receipt_ros, std::int64_t receipt_steady) {
    if (!clock_valid(receipt_ros, receipt_steady)) return false;
    if (owner != owner_) return reject("coverage_decision_owner_changed");
    if (input.schema != 1 || input.disposition > 2 || input.region_disposition > 2 ||
        input.region_code > 3) return reject("coverage_decision_schema_invalid");
    if (input.contract != contract_ || input.frame_id != frame_ ||
        input.profile != (source_mode_=="rgbd-only-v1"?"visual":"fusion_shadow") || input.rear) {
      return reject("coverage_decision_contract_invalid");
    }
    if (!fresh(receipt_ros, input.header_ros_ns) ||
        input.source_ros_ns > input.header_ros_ns) {
      return reject("coverage_decision_header_invalid");
    }
    // HardFault may legitimately revoke earlier evidence or have no source yet.
    // It must latch immediately; its other fields can never authorize Hold.
    if (input.disposition == 0) return reject("coverage_decision_hard_fault");
    if (!input.non_region_healthy || !input.complete ||
        !fresh(receipt_ros, input.source_ros_ns, maximum_source_age_ns_) ||
        input.frame_sequence == 0 || input.evaluation_sequence == 0 ||
        !exact_sources(input.required_sources) || !exact_sources(input.healthy_sources)) {
      return reject("coverage_decision_evidence_invalid");
    }
    const bool clear = input.disposition == 1;
    if (input.region_disposition != input.disposition ||
        input.region_code != (clear ? 1 : 3) ||
        input.healthy != clear || input.forward != clear || input.near != clear) {
      return reject("coverage_decision_classification_invalid");
    }
    if (seen_) {
      if (input.header_ros_ns < evidence_.header_ros_ns ||
          input.source_ros_ns < evidence_.source_ros_ns ||
          input.frame_sequence < evidence_.frame_sequence ||
          input.evaluation_sequence < evidence_.evaluation_sequence) {
        return reject("coverage_decision_reordered");
      }
      const bool same_frame = input.frame_sequence == evidence_.frame_sequence;
      if (same_frame) {
        if (input.source_ros_ns != evidence_.source_ros_ns ||
            input.evaluation_sequence != evidence_.evaluation_sequence ||
            input.disposition != evidence_.disposition) {
          return reject("coverage_decision_same_frame_mutated");
        }
      } else if (input.source_ros_ns <= evidence_.source_ros_ns ||
                 input.evaluation_sequence <= evidence_.evaluation_sequence) {
        return reject("coverage_decision_source_identity_invalid");
      }
    }
    if (!seen_ || input.frame_sequence != evidence_.frame_sequence) {
      source_receipt_steady_ = receipt_steady;
    }
    evidence_ = input;
    receipt_ros_ = receipt_ros;
    receipt_steady_ = receipt_steady;
    seen_ = true;
    return true;
  }

  ValidatedCoverageDecision evaluate(std::int64_t now_ros, std::int64_t now_steady) {
    if (!clock_valid(now_ros, now_steady)) return failed();
    if (!seen_) return failed();
    if (!fresh(now_ros, evidence_.source_ros_ns, maximum_source_age_ns_) ||
        !fresh(now_ros, evidence_.header_ros_ns) ||
        !fresh(now_ros, receipt_ros_) || !fresh(now_steady, receipt_steady_) ||
        !fresh(now_steady, source_receipt_steady_, maximum_source_age_ns_)) {
      reject("coverage_decision_stale");
      return failed();
    }
    return {true, evidence_.disposition, evidence_.source_ros_ns, source_receipt_steady_,
            evidence_.frame_sequence, evidence_.evaluation_sequence, owner_, contract_, "ok"};
  }

  void invalidate(const std::string& reason) { reject(reason); }

 private:
  std::int64_t maximum_source_age_ns_{kFreshnessNs};
  std::int64_t report_age_ns_;
  bool fresh(std::int64_t now, std::int64_t stamp) const {return fresh(now, stamp, report_age_ns_);}
  static bool fresh(std::int64_t now, std::int64_t stamp, std::int64_t budget) {
    return stamp > 0 && now >= stamp && now - stamp <= budget;
  }
  bool exact_sources(std::vector<std::string> sources) const {
    std::sort(sources.begin(), sources.end());
    return sources == (source_mode_=="rgbd-only-v1" ? std::vector<std::string>{"belly", "chest"} :
      std::vector<std::string>{"airy", "belly", "chest"});
  }
  bool reject(const std::string& reason) {
    if (!fault_) reason_ = reason;
    fault_ = true;
    return false;
  }
  bool clock_valid(std::int64_t ros, std::int64_t steady) {
    if (fault_) return false;
    if (ros <= 0 || steady <= 0 || ros < last_ros_ || steady < last_steady_) {
      return reject("coverage_decision_clock_invalid");
    }
    if (anchor_ros_ == 0) {
      anchor_ros_ = ros;
      anchor_steady_ = steady;
    }
    const auto ros_delta = ros - anchor_ros_;
    const auto steady_delta = steady - anchor_steady_;
    // A fixed anchor detects cumulative freeze/skew; per-tick differences
    // would let a frozen ROS clock with frequent heartbeats evade this check.
    if ((ros_delta >= steady_delta && ros_delta - steady_delta > 100000000) ||
        (steady_delta > ros_delta && steady_delta - ros_delta > 100000000)) {
      return reject("coverage_decision_clock_progress_invalid");
    }
    last_ros_ = ros;
    last_steady_ = steady;
    return true;
  }
  ValidatedCoverageDecision failed() const {
    ValidatedCoverageDecision result;
    result.reason = fault_ ? reason_ : "coverage_decision_missing";
    return result;
  }
  std::string owner_, contract_, frame_, source_mode_, reason_;
  CombinedCoverageEvidence evidence_;
  bool fault_{false}, seen_{false};
  std::int64_t receipt_ros_{0}, receipt_steady_{0}, last_ros_{0}, last_steady_{0};
  std::int64_t anchor_ros_{0}, anchor_steady_{0};
  std::int64_t source_receipt_steady_{0};
};
}  // namespace sunhorizon_velocity_supervisor
