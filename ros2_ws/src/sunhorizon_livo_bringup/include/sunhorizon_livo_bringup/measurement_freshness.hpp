#pragma once

#include <cmath>
#include <cstdint>
#include <string>

namespace sunhorizon_livo_bringup
{

struct MeasurementFreshnessDecision
{
  bool accepted{false};
  double age_sec{0.0};
  std::string reason{"measurement_stamp_invalid"};
};

inline MeasurementFreshnessDecision evaluate_measurement_freshness(
  std::int64_t stamp_nanoseconds,
  std::int64_t now_nanoseconds,
  double maximum_age_sec)
{
  if (stamp_nanoseconds <= 0 || now_nanoseconds <= 0 ||
    !std::isfinite(maximum_age_sec) || maximum_age_sec <= 0.0)
  {
    return {false, 0.0, "measurement_stamp_invalid"};
  }
  const double age_sec = static_cast<double>(
    now_nanoseconds - stamp_nanoseconds) * 1.0e-9;
  if (!std::isfinite(age_sec)) {
    return {false, age_sec, "measurement_age_non_finite"};
  }
  if (age_sec < 0.0) {
    return {false, age_sec, "measurement_stamp_in_future"};
  }
  if (age_sec > maximum_age_sec) {
    return {false, age_sec, "measurement_stale"};
  }
  return {true, age_sec, "ok"};
}

}  // namespace sunhorizon_livo_bringup
