#include "sunhorizon_rgbd_adapter/rgbd_freshness.hpp"

#include <cmath>
#include <limits>

namespace sunhorizon_rgbd_adapter {

double measurement_age_seconds(double now_sec, double measurement_stamp_sec) {
  if (!std::isfinite(now_sec) || !std::isfinite(measurement_stamp_sec) ||
      measurement_stamp_sec <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return now_sec - measurement_stamp_sec;
}

bool measurement_fresh(double age_sec, double freshness_timeout_sec) {
  return std::isfinite(age_sec) && std::isfinite(freshness_timeout_sec) &&
      freshness_timeout_sec > 0.0 && age_sec >= 0.0 &&
      age_sec <= freshness_timeout_sec;
}

}  // namespace sunhorizon_rgbd_adapter
