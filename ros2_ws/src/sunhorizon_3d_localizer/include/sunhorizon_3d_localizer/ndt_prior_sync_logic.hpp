#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace sunhorizon_3d_localizer {

struct PriorContinuityLimits {
  double maximum_gap_sec{0.25};
  double maximum_translation_step_m{0.30};
  double maximum_rotation_step_rad{0.2617993877991494};
};

struct PriorContinuityDecision {
  bool matchable{false};
  bool epoch_break{false};
  double dt_sec{0.0};
  double translation_step_m{0.0};
  double rotation_step_rad{0.0};
  std::string reason;
};

// The first prior anchors the gate and is not matchable. Spatial or timestamp
// discontinuities also begin a new epoch. A time-only gap may recover on its
// first fresh sample when that sample remains spatially continuous with the
// last accepted anchor; a spatial jump still needs a following continuous
// sample before it can become matchable.
class PriorContinuityGate {
 public:
  explicit PriorContinuityGate(PriorContinuityLimits limits);

  PriorContinuityDecision evaluate(
      std::int64_t stamp_ns, const Eigen::Matrix4d& odom_to_source);
  void reset();

 private:
  PriorContinuityLimits limits_;
  bool anchor_valid_{false};
  std::int64_t anchor_stamp_ns_{0};
  Eigen::Matrix4d anchor_{Eigen::Matrix4d::Identity()};
};

struct PriorSyncCandidate {
  std::int64_t stamp_ns{0};
  bool matchable{false};
};

struct PriorMatchDecision {
  bool matched{false};
  bool invalid_prior_within_tolerance{false};
  std::size_t index{0};
  std::int64_t delta_ns{0};
  std::string reason;
};

PriorMatchDecision select_nearest_matchable_prior(
    std::int64_t point_stamp_ns,
    const std::vector<PriorSyncCandidate>& candidates,
    std::int64_t tolerance_ns);

struct NdtPriorSyncDiagnosticSnapshot {
  std::uint64_t no_match_count{0};
  std::uint64_t invalid_prior_count{0};
  std::uint64_t epoch_break_count{0};
  std::uint64_t recovered_gap_count{0};
  std::uint64_t raw_scan_skip_count{0};
  std::uint64_t match_invoked_count{0};
  std::uint64_t match_accepted_count{0};
  std::uint64_t match_rejected_count{0};
  std::int64_t last_match_delta_ns{0};
  std::int64_t maximum_match_delta_ns{0};
};

class NdtPriorSyncDiagnostics {
 public:
  void record_no_match();
  void record_invalid_prior(bool epoch_break);
  void record_recovered_gap();
  void record_raw_scan_skip();
  void record_match(std::int64_t delta_ns);
  void record_match_invoked();
  void record_match_result(bool accepted);
  void reset();

  const NdtPriorSyncDiagnosticSnapshot& snapshot() const { return snapshot_; }

 private:
  NdtPriorSyncDiagnosticSnapshot snapshot_;
};

}  // namespace sunhorizon_3d_localizer
