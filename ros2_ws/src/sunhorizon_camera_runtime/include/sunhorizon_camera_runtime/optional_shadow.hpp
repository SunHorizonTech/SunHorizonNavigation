#pragma once

namespace sunhorizon_camera_runtime {

// Optional analysis must not promote its own failures into primary sensor faults.
// Primary validation/publication errors deliberately retain their original handling.
enum class ShadowResult { kComplete, kFailed };

template<typename Primary, typename Shadow>
ShadowResult publish_primary_then_shadow(Primary && primary, Shadow && shadow)
{
  primary();
  try {
    shadow();
    return ShadowResult::kComplete;
  } catch (...) {
    return ShadowResult::kFailed;
  }
}

}  // namespace sunhorizon_camera_runtime
