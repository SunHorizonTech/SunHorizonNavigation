#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace sunhorizon_navigation_core {

enum class GeometryCoverageClass { kHardFault, kClear, kGeometryHold };
enum class GeometryCommandKind { kController, kSyntheticZero, kOther };
enum class GeometryRecoveryState {
  kIdle, kRunning, kGeometryHold, kQualifyingClear, kAwaitingCommand, kFault
};

// Candidate qualification values only; no node enables this gate by default.
struct GeometryRecoveryConfig {
  std::size_t clear_frames{3U};
  std::int64_t clear_span_ns{100000000};
  std::int64_t max_frame_gap_ns{100000000};
  std::int64_t coverage_timeout_ns{300000000};
  std::int64_t command_timeout_ns{250000000};
  std::int64_t max_clock_progress_difference_ns{100000000};
};

// The caller must already authenticate and validate the COMPLETE combined
// decision, schema, typed health evidence, expected frame and producer identity.
// These fields never turn an unvalidated wire message into authorization.
struct GeometryCoverageEvidence {
  GeometryCoverageClass classification{GeometryCoverageClass::kHardFault};
  std::int64_t source_ros_ns{0};
  std::int64_t receipt_steady_ns{0};
  std::uint64_t frame_sequence{0U};
  std::uint64_t evaluation_sequence{0U};
};

struct GeometryCommandEvidence {
  GeometryCommandKind kind{GeometryCommandKind::kOther};
  std::uint64_t sequence{0U};
  std::string owner_epoch;
  // Actual controller computation sampling, not collision-filter header time.
  // Steady samples and receipts must share one host/boot CLOCK_MONOTONIC domain.
  std::int64_t compute_begin_ros_ns{0};
  std::int64_t compute_end_ros_ns{0};
  std::int64_t compute_begin_steady_ns{0};
  std::int64_t compute_end_steady_ns{0};
  std::int64_t receipt_steady_ns{0};
};

struct GeometryRecoveryInputs {
  std::int64_t now_ros_ns{0};
  std::int64_t now_steady_ns{0};
  bool hard_fault{false};
  // Existing outer task budget/lease. This gate never extends or resets either.
  bool task_budget_valid{false};
  std::string task_id;
  std::string producer_epoch;
  std::string coverage_contract;
  std::string command_owner_epoch;
  bool coverage_seen{false};
  GeometryCoverageEvidence coverage;
  bool command_seen{false};
  GeometryCommandEvidence command;
};

struct GeometryRecoveryDecision {
  GeometryRecoveryState state{GeometryRecoveryState::kIdle};
  bool hold{false};
  bool fault{false};
  bool ready{false};
  // One evaluation pulse only, when a previously held task qualifies recovery.
  bool recovered{false};
  std::string reason;
  std::size_t clear_frames{0U};
  std::int64_t barrier_ros_ns{0};
  std::int64_t barrier_steady_ns{0};
};

// Stateful pure C++ evidence gate, with no I/O, motion, lease renewal or goal
// manipulation. SafetyGate remains responsible for every hard safety check.
// Fault is sticky; replacement/reset requires the outer explicit reset policy.
class GeometryRecoveryGate {
 public:
  explicit GeometryRecoveryGate(GeometryRecoveryConfig config = {});
  GeometryRecoveryDecision evaluate(const GeometryRecoveryInputs& inputs);

 private:
  GeometryRecoveryDecision decision(const std::string& reason, bool recovered = false) const;
  GeometryRecoveryDecision fail(const std::string& reason);
  void clear_qualification();

  GeometryRecoveryConfig config_;
  GeometryRecoveryState state_{GeometryRecoveryState::kIdle};
  std::string fault_reason_;
  bool clocks_seen_{false};
  std::int64_t last_ros_ns_{0};
  std::int64_t last_steady_ns_{0};
  std::int64_t clock_anchor_ros_ns_{0};
  std::int64_t clock_anchor_steady_ns_{0};
  bool identity_seen_{false};
  std::string task_id_;
  std::string producer_epoch_;
  std::string coverage_contract_;
  std::string command_owner_epoch_;
  bool coverage_seen_{false};
  GeometryCoverageEvidence last_coverage_;
  bool command_seen_{false};
  GeometryCommandEvidence last_command_;
  bool held_{false};
  std::size_t clear_frames_{0U};
  std::int64_t first_clear_source_ns_{0};
  std::int64_t last_clear_source_ns_{0};
  std::int64_t last_clear_receipt_ns_{0};
  std::int64_t barrier_ros_ns_{0};
  std::int64_t barrier_steady_ns_{0};
  std::uint64_t barrier_command_sequence_{0U};
};

}  // namespace sunhorizon_navigation_core
