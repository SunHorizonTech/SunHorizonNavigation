#pragma once

#include "sunhorizon_obstacle_coverage/camera_freshness_policy.hpp"
#include <cmath>
#include <stdexcept>
#include <string>

#include "sunhorizon_obstacle_coverage/coverage_evaluator.hpp"

namespace sunhorizon_obstacle_coverage {

inline constexpr const char* kStrictRegionUpstreamDeadlineProfile = "strict-300ms-v1";
inline constexpr const char* kRawStatus350msRegionUpstreamDeadlineProfile = "raw-status-350ms-v1";

// Admission policy only. Never rewrites a configured budget or observation;
// evaluator/processed merge still enforce their source and receipt ages.
inline void validate_region_upstream_status_deadlines(
    const std::string& deadline_profile, CoverageProfile coverage_profile,
    bool region_required, bool typed_required, bool processed_required,
    double upstream_freshness_sec, double processed_freshness_sec,
    bool spatial_required=false) {
  const bool supported_profile=coverage_profile==CoverageProfile::kFusionShadow ||
    (spatial_required && coverage_profile==CoverageProfile::kVisual);
  if (deadline_profile == kLowSpeedCameraFreshness || deadline_profile == kOneSecondCameraFreshness) {
    const double age = camera_freshness_age_ns(deadline_profile) / 1e9;
    if (!spatial_required || !region_required || !typed_required || coverage_profile!=CoverageProfile::kVisual ||
        !processed_required || upstream_freshness_sec != age || processed_freshness_sec != age)
      throw std::runtime_error("attended camera freshness must match the selected spatial deadline");
    return;
  }
  if (deadline_profile == kRawStatus350msRegionUpstreamDeadlineProfile) {
    if (!region_required || !typed_required ||
        !supported_profile || !processed_required ||
        upstream_freshness_sec != .35 || processed_freshness_sec != .30) {
      throw std::runtime_error(
          "raw-status-350ms-v1 requires typed region fusion_shadow, processed belly, "
          "raw freshness exactly .35 and processed freshness exactly .30");
    }
    return;
  }
  if (deadline_profile != kStrictRegionUpstreamDeadlineProfile) {
    throw std::runtime_error("unknown region_upstream_status_deadline_profile");
  }
  // Preserve the historical non-region profiles and strict region upper bounds.
  if (region_required &&
      (!supported_profile || !processed_required ||
       !std::isfinite(upstream_freshness_sec) || upstream_freshness_sec <= 0. ||
       !std::isfinite(processed_freshness_sec) || processed_freshness_sec <= 0. ||
       upstream_freshness_sec > .30 || processed_freshness_sec > .30)) {
    throw std::runtime_error(
        "region support requires fusion_shadow and processed belly freshness <= .30");
  }
}

}  // namespace sunhorizon_obstacle_coverage
