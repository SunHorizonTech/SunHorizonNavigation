#pragma once
#include <string>

namespace sunhorizon_camera_runtime {
// Constructed once from the actual opened SDK device, before capture starts.
// This is a direct owner identity, not a synthetic Image/CameraInfo DDS owner.
struct CameraSourceIdentity {
  std::string owner_fqn, opened_serial, capture_epoch;
};
}  // namespace sunhorizon_camera_runtime
