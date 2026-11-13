#include "sunhorizon_3d_localizer/scan_range_gate.hpp"

#include <cmath>
#include <stdexcept>

namespace sunhorizon_3d_localizer {

ScanRangeGate::ScanRangeGate(
    double minimum_scan_range_m, double maximum_scan_range_m)
    : minimum_scan_range_m_(minimum_scan_range_m),
      maximum_scan_range_m_(maximum_scan_range_m) {
  if (!std::isfinite(minimum_scan_range_m_) || minimum_scan_range_m_ <= 0.0 ||
      !std::isfinite(maximum_scan_range_m_) ||
      maximum_scan_range_m_ <= minimum_scan_range_m_) {
    throw std::invalid_argument(
        "scan_range_limits_must_be_finite_positive_and_strictly_ordered");
  }
  minimum_squared_range_m2_ = minimum_scan_range_m_ * minimum_scan_range_m_;
  maximum_squared_range_m2_ = maximum_scan_range_m_ * maximum_scan_range_m_;
}

bool ScanRangeGate::accepts(double x, double y, double z) const {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return false;
  const double squared_range = x * x + y * y + z * z;
  return std::isfinite(squared_range) &&
      squared_range >= minimum_squared_range_m2_ &&
      squared_range <= maximum_squared_range_m2_;
}

double ScanRangeGate::minimum_scan_range_m() const {
  return minimum_scan_range_m_;
}

double ScanRangeGate::maximum_scan_range_m() const {
  return maximum_scan_range_m_;
}

}  // namespace sunhorizon_3d_localizer
