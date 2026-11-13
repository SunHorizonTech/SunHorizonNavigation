#include "sunhorizon_obstacle_coverage/region_upstream_deadline_policy.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>

using namespace sunhorizon_obstacle_coverage;
#define CHECK(x) do { if (!(x)) throw std::runtime_error(std::string(__func__) + ":" + std::to_string(__LINE__) + ": " #x); } while (false)

namespace {
bool admitted(const std::string& name, CoverageProfile profile, bool region,
              bool typed, bool processed, double raw, double filtered) {
  try {
    validate_region_upstream_status_deadlines(name, profile, region, typed, processed, raw, filtered);
    return true;
  } catch (const std::runtime_error&) {
    return false;
  }
}

void strict_default_and_legacy_profiles() {
  CHECK(std::string(kStrictRegionUpstreamDeadlineProfile) == "strict-300ms-v1");
  for (const auto profile : {CoverageProfile::kLidar, CoverageProfile::kVisual,
                             CoverageProfile::kFusionShadow}) {
    // This admission policy does not tighten historical non-region budgets.
    CHECK(admitted(kStrictRegionUpstreamDeadlineProfile, profile, false, false, false, 1., 1.));
  }
  CHECK(admitted(kStrictRegionUpstreamDeadlineProfile, CoverageProfile::kFusionShadow,
                 true, false, true, .30, .30));
  CHECK(admitted(kStrictRegionUpstreamDeadlineProfile, CoverageProfile::kFusionShadow,
                 true, true, true, .20, .25));
  for (const double invalid : {std::nextafter(.30, 1.), .35, 0., -1.,
                              std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::quiet_NaN()}) {
    CHECK(!admitted(kStrictRegionUpstreamDeadlineProfile, CoverageProfile::kFusionShadow,
                    true, true, true, invalid, .30));
    CHECK(!admitted(kStrictRegionUpstreamDeadlineProfile, CoverageProfile::kFusionShadow,
                    true, true, true, .30, invalid));
  }
  // Only exclusive spatial visibility permits visual; old typed/region paths
  // above still require their original fusion profile and identical budgets.
  validate_region_upstream_status_deadlines(kStrictRegionUpstreamDeadlineProfile,
    CoverageProfile::kVisual,true,true,true,.30,.30,true);
  validate_region_upstream_status_deadlines(kRawStatus350msRegionUpstreamDeadlineProfile,
    CoverageProfile::kVisual,true,true,true,.35,.30,true);
  CHECK(!admitted(kStrictRegionUpstreamDeadlineProfile, CoverageProfile::kVisual,
                  true, true, true, .30, .30));
  CHECK(!admitted(kStrictRegionUpstreamDeadlineProfile, CoverageProfile::kFusionShadow,
                  true, true, false, .30, .30));
}

void opt_in_has_one_exact_configuration() {
  for (const auto profile : {CoverageProfile::kLidar, CoverageProfile::kVisual,
                             CoverageProfile::kFusionShadow}) {
    for (const bool region : {false, true}) {
      for (const bool typed : {false, true}) {
        for (const bool processed : {false, true}) {
          CHECK(admitted(kRawStatus350msRegionUpstreamDeadlineProfile, profile,
                         region, typed, processed, .35, .30) ==
                (profile == CoverageProfile::kFusionShadow && region && typed && processed));
        }
      }
    }
  }
  for (const double invalid : {std::nextafter(.35, 0.), std::nextafter(.35, 1.),
                              .30, .40, 0., -1., std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::quiet_NaN()}) {
    CHECK(!admitted(kRawStatus350msRegionUpstreamDeadlineProfile, CoverageProfile::kFusionShadow,
                    true, true, true, invalid, .30));
  }
  for (const double invalid : {std::nextafter(.30, 0.), std::nextafter(.30, 1.),
                              .35, 0., -1., std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::quiet_NaN()}) {
    CHECK(!admitted(kRawStatus350msRegionUpstreamDeadlineProfile, CoverageProfile::kFusionShadow,
                    true, true, true, .35, invalid));
  }
  for (const auto* unknown : {"", "raw-status-350ms-v2", "raw-status-350ms-v1 ", "strict-350ms-v1"}) {
    CHECK(!admitted(unknown, CoverageProfile::kFusionShadow, true, true, true, .35, .30));
    CHECK(!admitted(unknown, CoverageProfile::kLidar, false, false, false, .30, .30));
  }
}

bool decision(const SourceObservation& raw, const SourceObservation& filtered, double raw_budget) {
  CoverageInputs inputs;
  inputs.now_sec = .5;
  inputs.airy = inputs.chest = {true, true, .5, .5};
  inputs.belly = require_processed_observation(raw, filtered, inputs.now_sec, raw_budget, .30);
  return CoverageEvaluator(CoverageProfile::kFusionShadow, {.30, .30, raw_budget}).evaluate(inputs).healthy;
}

void source_receipt_and_processed_gates_still_apply() {
  const SourceObservation raw{true, true, .15, .15}, filtered{true, true, .20, .20};
  CHECK(.5 - raw.stamp_sec == .35 && .5 - filtered.stamp_sec == .30);
  CHECK(!decision(raw, filtered, .30));
  CHECK(decision(raw, filtered, .35));
  auto changed = raw;
  changed.stamp_sec = .149999999; CHECK(!decision(changed, filtered, .35));
  changed = raw; changed.receipt_stamp_sec = .149999999; CHECK(!decision(changed, filtered, .35));
  changed = filtered; changed.stamp_sec = .199999999; CHECK(!decision(raw, changed, .35));
  changed = filtered; changed.receipt_stamp_sec = .199999999; CHECK(!decision(raw, changed, .35));
  changed = raw; changed.healthy = false; CHECK(!decision(changed, filtered, .35));
  changed = filtered; changed.healthy = false; CHECK(!decision(raw, changed, .35));
  changed = raw; changed.seen = false; CHECK(!decision(changed, filtered, .35));
  changed = filtered; changed.seen = false; CHECK(!decision(raw, changed, .35));
  changed = raw; changed.stamp_sec = .51; CHECK(!decision(changed, filtered, .35));
  changed = filtered; changed.receipt_stamp_sec = .51; CHECK(!decision(raw, changed, .35));
}
}  // namespace

int main() {
  CHECK(camera_freshness_age_ns(kStrictCameraFreshness)==300000000);
  CHECK(camera_freshness_age_ns(kLowSpeedCameraFreshness)==500000000);
  CHECK(camera_freshness_age_ns(kOneSecondCameraFreshness)==1000000000);
  validate_region_upstream_status_deadlines(kOneSecondCameraFreshness,CoverageProfile::kVisual,
    true,true,true,1.,1.,true);
  bool mismatched=false;
  try {validate_region_upstream_status_deadlines(kOneSecondCameraFreshness,CoverageProfile::kVisual,
      true,true,true,.5,.5,true);} catch(const std::runtime_error&){mismatched=true;}
  CHECK(mismatched);
  validate_low_speed_camera_limits(.05,0.,.03);
  for(const auto speed:{.051,std::numeric_limits<double>::quiet_NaN()}) {
    bool rejected=false;try{validate_low_speed_camera_limits(speed,0.,0.);}
    catch(const std::invalid_argument&){rejected=true;}CHECK(rejected);
  }
  validate_region_upstream_status_deadlines(kLowSpeedCameraFreshness,CoverageProfile::kVisual,
    true,true,true,.5,.5,true);
  bool airy_profile_rejected=false;
  try {validate_region_upstream_status_deadlines(kLowSpeedCameraFreshness,CoverageProfile::kFusionShadow,
    true,true,true,.5,.5,true);}catch(const std::runtime_error&){airy_profile_rejected=true;}
  CHECK(airy_profile_rejected);
  for(int mutation=0;mutation<4;++mutation) {
    bool rejected=false;
    try {validate_region_upstream_status_deadlines(kLowSpeedCameraFreshness,CoverageProfile::kVisual,
      true,true,mutation!=0,mutation==1?.3:.5,mutation==2?.3:.5,mutation!=3);}
    catch(const std::runtime_error&){rejected=true;}CHECK(rejected);
  }
  strict_default_and_legacy_profiles();
  opt_in_has_one_exact_configuration();
  source_receipt_and_processed_gates_still_apply();
  std::cout << "region upstream deadline policy: strict default, exact opt-in, dual ages and processed rejection passed\n";
}
