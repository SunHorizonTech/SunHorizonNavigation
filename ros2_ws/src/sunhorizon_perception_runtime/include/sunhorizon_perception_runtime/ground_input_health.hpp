#pragma once
#include <string>
#include "sunhorizon_perception_runtime/classified_obstacle_frame.hpp"

namespace sunhorizon_perception_runtime {
// Geometry-provider failure is upstream unavailability, not measured Unknown
// obstacle evidence. Missing support/fit with valid geometry remains consumable.
struct GroundInputHealth {
  bool available{true};
  std::string reason;
};
inline GroundInputHealth registration_ground_input_health(bool registration_required,
    const ground_relative::SourceTransformDiagnostic& diagnostic,
    ground_relative::Qualification result) {
  using Q=ground_relative::Qualification;
  if(!registration_required)return {};
  if(diagnostic.provider_state>0)
    return {false,"registration_ground_unavailable:"+diagnostic.reason};
  if(result==Q::ReferenceExpired || result==Q::ReferenceIdentityMismatch || result==Q::InvalidTransform)
    return {false,"registration_ground_invalid_original_geometry:"+std::to_string(static_cast<int>(result))};
  return {};
}
} // namespace sunhorizon_perception_runtime
