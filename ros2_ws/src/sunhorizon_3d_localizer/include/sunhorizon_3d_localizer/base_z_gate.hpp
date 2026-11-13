#pragma once

namespace sunhorizon_3d_localizer {

bool base_z_within_reference(
    double candidate_base_z_m,
    double reference_base_z_m,
    double maximum_deviation_m) noexcept;

}  // namespace sunhorizon_3d_localizer
