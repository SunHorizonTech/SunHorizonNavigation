#pragma once
#include <cmath>
#include <stdexcept>
#include <string>

namespace sunhorizon_navigation_runtime::detail {
// Latest-only delivery is legal only for an actual instantaneous upstream
// observation buffer. A persistent history needs a different explicit contract.
inline void require_instantaneous_raw_clearing(double persistence, const std::string& binding) {
  if (!std::isfinite(persistence) || persistence != 0.)
    throw std::invalid_argument("nav2_latest_raw_clearing_requires_zero_persistence:" + binding);
}
}  // namespace sunhorizon_navigation_runtime::detail
