#pragma once

#include <string>

namespace sunhorizon_navigation_core {

struct Twist2D {
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct Pose3D {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

struct Point3D {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

}  // namespace sunhorizon_navigation_core
