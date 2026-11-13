#pragma once

namespace sunhorizon_3d_localizer {

// Inclusive Euclidean range gate applied before scan voxel filtering.
class ScanRangeGate {
 public:
  ScanRangeGate(double minimum_scan_range_m, double maximum_scan_range_m);

  bool accepts(double x, double y, double z) const;

  double minimum_scan_range_m() const;
  double maximum_scan_range_m() const;

 private:
  double minimum_scan_range_m_{0.5};
  double maximum_scan_range_m_{30.0};
  double minimum_squared_range_m2_{0.25};
  double maximum_squared_range_m2_{900.0};
};

}  // namespace sunhorizon_3d_localizer
