#pragma once

#include <cstddef>

namespace sunhorizon_airy_adapter
{

inline constexpr std::size_t kLiveInputPointQueueDepth = 4U;
inline constexpr std::size_t kLiveInputImuQueueDepth = 16U;
inline constexpr std::size_t kOutputPointQueueDepth = 32U;
inline constexpr std::size_t kOutputImuQueueDepth = 2000U;

struct InputQueueDepths
{
  std::size_t points;
  std::size_t imu;
};

inline InputQueueDepths input_queue_depths(
  bool live_tracking_clock, bool promoted_reliable_live_input = false)
{
  if (live_tracking_clock) {
    return {
      kLiveInputPointQueueDepth,
      promoted_reliable_live_input ? kOutputImuQueueDepth : kLiveInputImuQueueDepth};
  }
  return {kOutputPointQueueDepth, kOutputImuQueueDepth};
}

}  // namespace sunhorizon_airy_adapter
