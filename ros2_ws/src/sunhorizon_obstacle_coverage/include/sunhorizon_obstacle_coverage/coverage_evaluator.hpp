#pragma once

#include <string>
#include <vector>

namespace sunhorizon_obstacle_coverage {

enum class CoverageProfile {
  kLidar,
  kVisual,
  kFusionShadow,
};

struct CoverageFreshness {
  double airy_sec{0.30};
  double chest_sec{0.30};
  double belly_sec{0.30};
};

struct SourceObservation {
  bool seen{false};
  bool healthy{false};
  double stamp_sec{0.0};
  double receipt_stamp_sec{0.0};
};

// Keep RGB-D calibration evidence source-local. The upstream adapter's
// `*_fresh` / `*_pointcloud_healthy` booleans use its own timeout; this layer
// applies its configured freshness to the measured ages in the same status.
struct RgbdCoverageEvidence {
  bool chest_pair_fresh{false};
  bool chest_camera_info_valid{false};
  bool chest_alignment_accepted{false};
  bool chest_depth_scale_accepted{false};
  bool chest_pointcloud_healthy{false};
  bool belly_depth_fresh{false};
  bool belly_camera_info_valid{false};
  bool belly_depth_scale_accepted{false};
  bool belly_pointcloud_healthy{false};
};

struct RgbdCoverageHealth {
  bool chest{false};
  bool belly{false};
};

struct CoverageInputs {
  double now_sec{0.0};
  SourceObservation airy;
  SourceObservation chest;
  SourceObservation belly;
};

struct CoverageDecision {
  bool healthy{false};
  bool forward_coverage{false};
  bool near_field_coverage{false};
  bool rear_coverage{false};
  std::vector<std::string> required_sources;
  std::vector<std::string> healthy_sources;
  std::string reason;
};

CoverageProfile parse_profile(const std::string& profile);
std::string profile_name(CoverageProfile profile);
RgbdCoverageHealth evaluate_rgbd_coverage_health(
    const RgbdCoverageEvidence& evidence);
double oldest_source_stamp(
    double status_stamp_sec, double first_age_sec, double second_age_sec);
// Preserve both upstream calibration/health and processed-output freshness.
// Check each timeline before combining it, so a future/malformed stamp cannot
// be hidden by taking the minimum of two otherwise unrelated clocks.
SourceObservation require_processed_observation(
    const SourceObservation& upstream, const SourceObservation& processed,
    double now_sec, double upstream_freshness_sec, double processed_freshness_sec);

// Diagnostic text only; uses the caller's original evaluation time and snapshots.
// "raw_belly" means the RGBDStatus-derived oldest source, not a raw cloud.
// Does not modify either observation or the production merge/evaluation result.
std::string processed_observation_diagnostics(
    const SourceObservation& upstream, const SourceObservation& processed,
    double now_sec, double upstream_freshness_sec, double processed_freshness_sec,
    bool processed_required);

class CoverageEvaluator {
 public:
  CoverageEvaluator(CoverageProfile profile, CoverageFreshness freshness = {});

  CoverageDecision evaluate(const CoverageInputs& inputs) const;
  CoverageProfile profile() const { return profile_; }

 private:
  CoverageProfile profile_;
  CoverageFreshness freshness_;
};

}  // namespace sunhorizon_obstacle_coverage
