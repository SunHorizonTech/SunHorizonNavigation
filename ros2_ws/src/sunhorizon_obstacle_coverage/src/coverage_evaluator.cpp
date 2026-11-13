#include "sunhorizon_obstacle_coverage/coverage_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace sunhorizon_obstacle_coverage {
namespace {

constexpr double kMaximumFutureAgeToleranceSec = 0.020;

enum class SourceState {
  kHealthy,
  kMissing,
  kInvalidHeaderStamp,
  kFutureHeaderStamp,
  kInvalidReceiptStamp,
  kFutureReceiptStamp,
  kQueueDelay,
  kStale,
  kReceiptStale,
  kUnhealthy,
};

struct RequiredSource {
  const char* name;
  const SourceObservation* observation;
  double freshness_sec;
};

SourceState source_state(
    const SourceObservation& observation, double now_sec, double freshness_sec) {
  if (!observation.seen) {
    return SourceState::kMissing;
  }
  if (!std::isfinite(observation.stamp_sec) || observation.stamp_sec <= 0.0) {
    return SourceState::kInvalidHeaderStamp;
  }
  if (observation.stamp_sec > now_sec) {
    return SourceState::kFutureHeaderStamp;
  }
  if (!std::isfinite(observation.receipt_stamp_sec) ||
      observation.receipt_stamp_sec <= 0.0) {
    return SourceState::kInvalidReceiptStamp;
  }
  if (observation.receipt_stamp_sec > now_sec) {
    return SourceState::kFutureReceiptStamp;
  }
  if (now_sec - observation.stamp_sec > freshness_sec) {
    if (now_sec - observation.receipt_stamp_sec <= freshness_sec) {
      return SourceState::kQueueDelay;
    }
    return SourceState::kStale;
  }
  if (now_sec - observation.receipt_stamp_sec > freshness_sec) {
    return SourceState::kReceiptStale;
  }
  if (!observation.healthy) {
    return SourceState::kUnhealthy;
  }
  return SourceState::kHealthy;
}

const char* state_suffix(SourceState state) {
  switch (state) {
    case SourceState::kHealthy:
      return "healthy";
    case SourceState::kMissing:
      return "missing";
    case SourceState::kInvalidHeaderStamp:
      return "invalid_header_stamp";
    case SourceState::kFutureHeaderStamp:
      return "future_header_stamp";
    case SourceState::kInvalidReceiptStamp:
      return "invalid_receipt_stamp";
    case SourceState::kFutureReceiptStamp:
      return "future_receipt_stamp";
    case SourceState::kQueueDelay:
      return "queue_delay";
    case SourceState::kStale:
      return "stale";
    case SourceState::kReceiptStale:
      return "receipt_stale";
    case SourceState::kUnhealthy:
      return "unhealthy";
  }
  return "unknown";
}

void append_reason(std::string* reason, const std::string& item) {
  if (!reason->empty()) {
    *reason += ';';
  }
  *reason += item;
}

}  // namespace

CoverageProfile parse_profile(const std::string& profile) {
  if (profile == "lidar") {
    return CoverageProfile::kLidar;
  }
  if (profile == "visual") {
    return CoverageProfile::kVisual;
  }
  if (profile == "fusion_shadow") {
    return CoverageProfile::kFusionShadow;
  }
  throw std::invalid_argument("profile must be lidar, visual, or fusion_shadow");
}

std::string profile_name(CoverageProfile profile) {
  switch (profile) {
    case CoverageProfile::kLidar:
      return "lidar";
    case CoverageProfile::kVisual:
      return "visual";
    case CoverageProfile::kFusionShadow:
      return "fusion_shadow";
  }
  throw std::invalid_argument("unknown coverage profile");
}

RgbdCoverageHealth evaluate_rgbd_coverage_health(
    const RgbdCoverageEvidence& evidence) {
  return {
      evidence.chest_camera_info_valid && evidence.chest_alignment_accepted &&
          evidence.chest_depth_scale_accepted,
      evidence.belly_camera_info_valid && evidence.belly_depth_scale_accepted};
}

double oldest_source_stamp(
    double status_stamp_sec, double first_age_sec, double second_age_sec) {
  const auto source_stamp = [status_stamp_sec](double age_sec) {
    if (!std::isfinite(status_stamp_sec) || status_stamp_sec <= 0.0 ||
        !std::isfinite(age_sec) ||
        age_sec < -kMaximumFutureAgeToleranceSec ||
        age_sec >= status_stamp_sec) {
      return 0.0;
    }
    return status_stamp_sec - std::max(0.0, age_sec);
  };
  const double first_stamp = source_stamp(first_age_sec);
  const double second_stamp = source_stamp(second_age_sec);
  if (first_stamp <= 0.0 || second_stamp <= 0.0) {
    return 0.0;
  }
  return std::min(first_stamp, second_stamp);
}

SourceObservation require_processed_observation(
    const SourceObservation& upstream, const SourceObservation& processed,
    double now_sec, double upstream_freshness_sec, double processed_freshness_sec) {
  if (source_state(upstream, now_sec, upstream_freshness_sec) != SourceState::kHealthy) return upstream;
  if (source_state(processed, now_sec, processed_freshness_sec) != SourceState::kHealthy) {
    auto rejected = processed;
    rejected.healthy = false;  // The outer camera deadline may be more permissive.
    return rejected;
  }
  return {true, true, std::min(upstream.stamp_sec, processed.stamp_sec),
          std::min(upstream.receipt_stamp_sec, processed.receipt_stamp_sec)};
}

std::string processed_observation_diagnostics(
    const SourceObservation& upstream, const SourceObservation& processed,
    double now_sec, double upstream_freshness_sec, double processed_freshness_sec,
    bool processed_required) {
  const auto raw_state = source_state(upstream, now_sec, upstream_freshness_sec);
  const auto processed_state = source_state(processed, now_sec, processed_freshness_sec);
  const char* branch = !processed_required ? "upstream_only" :
      raw_state != SourceState::kHealthy ? "upstream_rejected" :
      processed_state != SourceState::kHealthy ? "processed_rejected" : "combined_min";
  const auto age = [now_sec](double stamp) {
    return std::isfinite(now_sec) && std::isfinite(stamp) && stamp > 0.0
        ? now_sec - stamp : std::numeric_limits<double>::infinity();
  };
  std::ostringstream out;
  out << std::fixed << std::setprecision(9) << "belly_merge(now=" << now_sec
      << ",processed_required=" << processed_required << ",branch=" << branch;
  if (processed_required && raw_state == SourceState::kHealthy &&
      processed_state == SourceState::kHealthy) {
    const auto origin = [](double raw, double filtered) {
      return raw == filtered ? "equal" : raw < filtered ? "raw" : "processed";
    };
    out << ",stamp_from=" << origin(upstream.stamp_sec, processed.stamp_sec)
        << ",receipt_from=" << origin(upstream.receipt_stamp_sec, processed.receipt_stamp_sec);
  }
  out << ')';
  const auto append = [&](const char* name, const SourceObservation& value,
                          SourceState state, double freshness) {
    out << ';' << name << "(stamp=" << value.stamp_sec
        << ",receipt_stamp=" << value.receipt_stamp_sec
        << ",header_age=" << age(value.stamp_sec)
        << ",receipt_age=" << age(value.receipt_stamp_sec)
        << ",seen=" << value.seen << ",healthy=" << value.healthy
        << ",state=" << state_suffix(state) << ",freshness=" << freshness << ')';
  };
  append("raw_belly", upstream, raw_state, upstream_freshness_sec);
  append("processed_belly", processed, processed_state, processed_freshness_sec);
  return out.str();
}

CoverageEvaluator::CoverageEvaluator(
    CoverageProfile profile, CoverageFreshness freshness)
    : profile_(profile), freshness_(std::move(freshness)) {
  if (!std::isfinite(freshness_.airy_sec) || freshness_.airy_sec <= 0.0 ||
      !std::isfinite(freshness_.chest_sec) || freshness_.chest_sec <= 0.0 ||
      !std::isfinite(freshness_.belly_sec) || freshness_.belly_sec <= 0.0) {
    throw std::invalid_argument("coverage freshness values must be finite and positive");
  }
}

CoverageDecision CoverageEvaluator::evaluate(const CoverageInputs& inputs) const {
  CoverageDecision decision;
  std::vector<RequiredSource> required;
  switch (profile_) {
    case CoverageProfile::kLidar:
      required.push_back({"airy", &inputs.airy, freshness_.airy_sec});
      break;
    case CoverageProfile::kVisual:
      required.push_back({"chest", &inputs.chest, freshness_.chest_sec});
      required.push_back({"belly", &inputs.belly, freshness_.belly_sec});
      break;
    case CoverageProfile::kFusionShadow:
      required.push_back({"airy", &inputs.airy, freshness_.airy_sec});
      required.push_back({"chest", &inputs.chest, freshness_.chest_sec});
      required.push_back({"belly", &inputs.belly, freshness_.belly_sec});
      break;
  }

  bool all_healthy = std::isfinite(inputs.now_sec) && inputs.now_sec > 0.0;
  if (!all_healthy) {
    decision.reason = "invalid_now";
  }
  for (const auto& source : required) {
    decision.required_sources.emplace_back(source.name);
    const SourceState state = source_state(
        *source.observation, inputs.now_sec, source.freshness_sec);
    if (state == SourceState::kHealthy) {
      decision.healthy_sources.emplace_back(source.name);
      continue;
    }
    all_healthy = false;
    append_reason(
        &decision.reason, std::string(source.name) + '_' + state_suffix(state));
  }

  decision.healthy = all_healthy;
  decision.forward_coverage = all_healthy;
  decision.near_field_coverage = all_healthy;
  decision.rear_coverage = false;
  if (decision.reason.empty()) {
    decision.reason = "ok";
  }
  return decision;
}

}  // namespace sunhorizon_obstacle_coverage
