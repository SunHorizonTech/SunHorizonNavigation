#include "sunhorizon_3d_localizer/raw_scan_admission_gate.hpp"

namespace sunhorizon_3d_localizer {

RawScanAdmissionDecision evaluate_raw_scan_admission(
    std::size_t raw_points, std::size_t minimum_raw_points) {
  RawScanAdmissionDecision output;
  output.raw_points = raw_points;
  output.minimum_raw_points = minimum_raw_points;
  output.accepted = minimum_raw_points == 0U || raw_points >= minimum_raw_points;
  output.reason = output.accepted ? "ok" : "raw_scan_points_below_minimum";
  return output;
}

}  // namespace sunhorizon_3d_localizer
