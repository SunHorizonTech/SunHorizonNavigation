#pragma once
#include <cmath>
#include <string>
#include <stdexcept>
namespace sunhorizon_velocity_supervisor {
class AttendedTravelGuard {
 public:
  explicit AttendedTravelGuard(bool five_meter = false, double maximum_cross_track = .50)
      : maximum_displacement_(five_meter ? 5.30 : 3.30),
        maximum_cross_track_(maximum_cross_track),
        prefix_(five_meter ? "attended_five_meter" : "attended_three_meter") {
    if (!std::isfinite(maximum_cross_track_) || maximum_cross_track_ <= 0. ||
        maximum_cross_track_ > maximum_displacement_)
      throw std::invalid_argument("attended_cross_track_stop_m_invalid");
  }
  void begin(double x, double y, double yaw) {
    if (started_) return;
    started_ = true;
    x_ = x; y_ = y; yaw_ = yaw;
    observe(x, y);
  }
  std::string observe(double x, double y) {
    if (!started_ || !failure_.empty()) return failure_;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(x_) ||
        !std::isfinite(y_) || !std::isfinite(yaw_)) {
      failure_ = prefix_ + "_travel_pose_invalid";
    } else {
      const double dx = x-x_, dy = y-y_;
      if (std::hypot(dx, dy) > maximum_displacement_)
        failure_ = prefix_ + "_displacement_exceeded";
      else if (std::abs(-dx*std::sin(yaw_)+dy*std::cos(yaw_)) >= maximum_cross_track_)
        failure_ = prefix_ + "_cross_track_exceeded";
    }
    return failure_;
  }
 private:
  double maximum_displacement_;
  double maximum_cross_track_;
  std::string prefix_;
  bool started_{false};
  double x_{0}, y_{0}, yaw_{0};
  std::string failure_;
};
}  // namespace sunhorizon_velocity_supervisor
