#include <sunhorizon_3d_localizer/base_z_gate.hpp>

#include <cmath>

namespace sunhorizon_3d_localizer {

bool base_z_within_reference(
    const double candidate_base_z_m,
    const double reference_base_z_m,
    const double maximum_deviation_m) noexcept {
  return std::isfinite(candidate_base_z_m) &&
      std::isfinite(reference_base_z_m) &&
      std::isfinite(maximum_deviation_m) &&
      maximum_deviation_m > 0.0 &&
      std::abs(candidate_base_z_m - reference_base_z_m) <= maximum_deviation_m;
}

}  // namespace sunhorizon_3d_localizer
