#include "sunhorizon_obstacle_coverage/dynamic_self_geometry_cuda_contract.hpp"
namespace sunhorizon_obstacle_coverage::dynamic_self_geometry {
bool camera_cuda_arithmetic_contract_supported() noexcept {
#if defined(SUNHORIZON_CAMERA_ARM11_CONTRACT) && defined(__aarch64__) && __GNUC__ == 11 && __GNUC_MINOR__ == 4 && __GNUC_PATCHLEVEL__ == 0 && defined(__OPTIMIZE__) && !defined(__FAST_MATH__)
  return true;
#else
  return false;
#endif
}
}
