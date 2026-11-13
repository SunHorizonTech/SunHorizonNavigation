#pragma once
#include "sunhorizon_obstacle_coverage/coverage_evaluator.hpp"

namespace sunhorizon_obstacle_coverage {
// Only the authenticated direct spatial path owns processed-source freshness
// through mandatory belly source/receipt/expiry in SpatialEvidenceTracker.
// This removes a duplicate health check, not the spatial lease or stream stop.
class BellyHealthPolicy {
 public:
  BellyHealthPolicy(bool direct=false, bool spatial_required=false)
    : spatial_lease_(direct && spatial_required) {}
  bool require_processed(bool configured) const {return configured && !spatial_lease_;}
  double source_stamp(double status_stamp, double depth_age, double processed_age) const {
    // Derive the actual depth source time; never substitute publication now.
    return oldest_source_stamp(status_stamp,depth_age,spatial_lease_?depth_age:processed_age);
  }
 private:
  bool spatial_lease_;
};
}  // namespace sunhorizon_obstacle_coverage
