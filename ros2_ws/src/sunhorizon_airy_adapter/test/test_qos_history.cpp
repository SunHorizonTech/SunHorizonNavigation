#include <iostream>

#include "sunhorizon_airy_adapter/qos_history.hpp"

int main()
{
  const auto live = sunhorizon_airy_adapter::input_queue_depths(true);
  if (live.points != 4U || live.imu != 16U) {
    std::cerr << "live tracking must use shallow input histories\n";
    return 1;
  }

  const auto promoted_live =
    sunhorizon_airy_adapter::input_queue_depths(true, true);
  if (promoted_live.points != 4U ||
    promoted_live.imu != sunhorizon_airy_adapter::kOutputImuQueueDepth)
  {
    std::cerr << "promoted reliable live input must retain shallow points and deep IMU\n";
    return 1;
  }

  const auto offline = sunhorizon_airy_adapter::input_queue_depths(false);
  if (offline.points != sunhorizon_airy_adapter::kOutputPointQueueDepth ||
    offline.imu != sunhorizon_airy_adapter::kOutputImuQueueDepth)
  {
    std::cerr << "fixed-clock replay must retain deep input histories\n";
    return 1;
  }

  std::cout << "All Airy QoS history tests passed\n";
  return 0;
}
