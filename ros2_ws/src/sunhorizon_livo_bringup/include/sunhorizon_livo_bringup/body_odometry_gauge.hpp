#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>
namespace sunhorizon_livo_bringup {
struct BodyOdometryGaugeOrigin {
  bool valid{false};int64_t source_ns{0};uint64_t epoch{1};double raw_z_m{0.};
};
// One immutable vertical datum per explicit canonical epoch. This is NOT a
// floor/foot-height estimate. Never reset it for velocity warmup or a gap.
class BodyOdometryGauge {
 public:
  double relative_z(int64_t source_ns,double raw_z_m){
    if(source_ns<=0 || !std::isfinite(raw_z_m))throw std::invalid_argument("body_odometry_gauge_invalid_pose");
    if(!origin_.valid){origin_.valid=true;origin_.source_ns=source_ns;origin_.raw_z_m=raw_z_m;}
    return raw_z_m-origin_.raw_z_m;
  }
  void reset(){origin_={false,0,origin_.epoch+1,0.};}
  BodyOdometryGaugeOrigin origin()const{return origin_;}
 private:
  BodyOdometryGaugeOrigin origin_;
};
} // namespace sunhorizon_livo_bringup
