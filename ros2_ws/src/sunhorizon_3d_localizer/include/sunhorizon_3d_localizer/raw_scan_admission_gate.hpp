#pragma once

#include <cstddef>
#include <string>

namespace sunhorizon_3d_localizer {

struct RawScanAdmissionDecision {
  std::size_t raw_points{0};
  std::size_t minimum_raw_points{0};
  bool accepted{false};
  std::string reason{"raw_scan_points_below_minimum"};
};

// A zero minimum disables this generic gate. A nonzero robot profile minimum
// skips incomplete frames before synchronization or NDT scheduling.
RawScanAdmissionDecision evaluate_raw_scan_admission(
    std::size_t raw_points, std::size_t minimum_raw_points);

}  // namespace sunhorizon_3d_localizer
