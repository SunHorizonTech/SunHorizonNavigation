#pragma once
#include <string>

namespace sunhorizon_obstacle_coverage {
// Delivery is distinct from qualification for motion. Unavailable evidence
// remains a sticky motion stop; it is not a damaged perception/localization owner.
enum class SpatialDeliveryState { Accepted, MotionUnavailable, InternalFault };
struct SpatialDeliveryResult {
  SpatialDeliveryState state{SpatialDeliveryState::InternalFault};
  std::string reason;
  bool accepted() const { return state == SpatialDeliveryState::Accepted; }
};
template<class Fault>
void propagate_spatial_internal_fault(const SpatialDeliveryResult& result, Fault fault) {
  if (result.state == SpatialDeliveryState::InternalFault) fault(result.reason);
}
}  // namespace sunhorizon_obstacle_coverage
