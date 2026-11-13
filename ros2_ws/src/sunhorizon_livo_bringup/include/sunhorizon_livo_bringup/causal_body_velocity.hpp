#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <vector>
#include <stdexcept>
#include <string>
#include "sunhorizon_navigation_core/body_velocity_evidence.hpp"

namespace sunhorizon_livo_bringup {
enum class BodyVelocityGeometryStatus {Ready, FuturePending, Unavailable};
struct BodyVelocitySample {
  int64_t source_ns{0};
  double x{0.0}, y{0.0}, yaw{0.0};
};
struct BodyVelocityEstimate {
  sunhorizon_navigation_core::BodyVelocityEvidence evidence;
  double vx{0.0}, vy{0.0}, wz{0.0};
  std::string reason{"body_velocity_warming_up"};
};
struct CausalBodyVelocityOptions {
  int64_t window_ns{500000000}, minimum_span_ns{350000000}, maximum_gap_ns{250000000};
  uint32_t minimum_samples{5};
  double maximum_translation_jump_m{0.25}, maximum_yaw_jump_rad{0.25};
  double velocity_variance_floor{0.0004}, yaw_variance_floor{0.0004};
  double maximum_speed_mps{1.5}, maximum_yaw_rate_rps{2.0};
};

// Bounded trailing least-squares slope. Only source times <= the current
// measurement enter the fit. No zero deadband/ZUPT or centered differentiation.
// Covariance includes fit residual and a floor; it is engineering uncertainty,
// not a certified physical error bound. Consumers must also account for age.
class CausalBodyVelocity {
 public:
  explicit CausalBodyVelocity(CausalBodyVelocityOptions options = {}) : options_(options) {
    if (options_.window_ns <= 0 || options_.window_ns > 1000000000 ||
        options_.minimum_span_ns <= 0 || options_.minimum_span_ns > options_.window_ns ||
        options_.maximum_gap_ns <= 0 || options_.minimum_samples < 3 || options_.minimum_samples > 4096 ||
        !positive(options_.maximum_translation_jump_m) || !positive(options_.maximum_yaw_jump_rad) ||
        !positive(options_.velocity_variance_floor) || !positive(options_.yaw_variance_floor) ||
        !positive(options_.maximum_speed_mps) || !positive(options_.maximum_yaw_rate_rps))
      throw std::invalid_argument("body_velocity_options_invalid");
  }
  void reset() {history_.clear(); ++epoch_;}
  BodyVelocityEstimate reject(const std::string& reason) {
    reset(); BodyVelocityEstimate result;result.reason=reason;result.evidence.reset_epoch=epoch_;return result;
  }
  // A normally asynchronous joint upper bound may arrive after this pose.
  // Reject that pose without inserting it or destroying earlier qualified
  // samples. The next qualified pose still passes the source gap/jump gates.
  BodyVelocityEstimate reject_geometry(BodyVelocityGeometryStatus status) {
    if (status==BodyVelocityGeometryStatus::Ready)
      throw std::invalid_argument("body_velocity_reject_ready_geometry");
    if (status!=BodyVelocityGeometryStatus::FuturePending)
      return reject("body_velocity_source_time_fk_unavailable");
    BodyVelocityEstimate result;
    result.reason="body_velocity_source_time_fk_pending";
    result.evidence.reset_epoch=epoch_;
    return result;
  }
  BodyVelocityEstimate observe(BodyVelocitySample value) {
    if (value.source_ns<=0 || !std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.yaw))
      return reject("body_velocity_nonfinite_pose_or_time");
    if (!history_.empty()) {
      const auto& previous=history_.back();
      const auto dt=value.source_ns-previous.source_ns;
      if (dt<=0) return reject("body_velocity_source_not_increasing");
      if (dt>options_.maximum_gap_ns) return reject("body_velocity_source_gap");
      const auto yaw_delta=wrap(value.yaw-previous.yaw);
      if (std::hypot(value.x-previous.x,value.y-previous.y)>options_.maximum_translation_jump_m ||
          std::abs(yaw_delta)>options_.maximum_yaw_jump_rad)
        return reject("body_velocity_pose_discontinuity");
      value.yaw=previous.yaw+yaw_delta;
    }
    history_.push_back(value);
    while (history_.size()>1 && value.source_ns-history_.front().source_ns>options_.window_ns)
      history_.pop_front();
    if (history_.size()>4096) return reject("body_velocity_history_overflow");
    BodyVelocityEstimate out;auto& evidence=out.evidence;
    evidence.window_begin_ns=history_.front().source_ns;evidence.window_end_ns=value.source_ns;
    evidence.reset_epoch=epoch_;evidence.sample_count=static_cast<uint32_t>(history_.size());
    double mt=0.,mx=0.,my=0.,ma=0.;
    for (const auto& p:history_) {
      mt+=double(p.source_ns-value.source_ns)*1e-9;mx+=p.x;my+=p.y;ma+=p.yaw;
    }
    const double count=history_.size();mt/=count;mx/=count;my/=count;ma/=count;
    evidence.effective_source_ns=value.source_ns+static_cast<int64_t>(std::llround(mt*1e9));
    if (history_.size()<options_.minimum_samples ||
        value.source_ns-history_.front().source_ns<options_.minimum_span_ns) return out;
    double tt=0.,ta=0.;
    for (const auto& p:history_) {
      const double dt=double(p.source_ns-value.source_ns)*1e-9-mt;
      tt+=dt*dt;ta+=dt*(p.yaw-ma);
    }
    if (!std::isfinite(tt) || tt<1e-12) return reject("body_velocity_singular_window");
    // Fit constant body velocity against integrated measured heading. This
    // avoids the artificial lateral velocity produced by rotating a historical
    // world slope with the newest heading during a turn. Still a window mean.
    std::vector<double> cs(history_.size(),0.),sn(history_.size(),0.);
    double mc=0.,ms=0.;
    for(std::size_t i=1;i<history_.size();++i){
      const auto& p=history_[i-1];const auto& q=history_[i];
      const double dt=double(q.source_ns-p.source_ns)*1e-9;
      cs[i]=cs[i-1]+.5*dt*(std::cos(p.yaw)+std::cos(q.yaw));
      sn[i]=sn[i-1]+.5*dt*(std::sin(p.yaw)+std::sin(q.yaw));
      mc+=cs[i];ms+=sn[i];
    }
    mc/=count;ms/=count;
    double denominator=0.,real=0.,imaginary=0.;
    for(std::size_t i=0;i<history_.size();++i){
      const double c=cs[i]-mc,s=sn[i]-ms,x=history_[i].x-mx,y=history_[i].y-my;
      denominator+=c*c+s*s;real+=c*x+s*y;imaginary+=c*y-s*x;
    }
    if(!std::isfinite(denominator)||denominator<1e-12)return reject("body_velocity_singular_heading_window");
    out.vx=real/denominator;out.vy=imaginary/denominator;out.wz=ta/tt;
    if (!std::isfinite(out.vx)||!std::isfinite(out.vy)||!std::isfinite(out.wz)||
        std::hypot(out.vx,out.vy)>options_.maximum_speed_mps || std::abs(out.wz)>options_.maximum_yaw_rate_rps)
      return reject("body_velocity_fit_out_of_bounds");
    double residual_xy=0.,eaa=0.;
    for(std::size_t i=0;i<history_.size();++i){
      const auto& p=history_[i];const double c=cs[i]-mc,s=sn[i]-ms;
      const double x=p.x-mx-out.vx*c+out.vy*s,y=p.y-my-out.vx*s-out.vy*c;
      const double dt=double(p.source_ns-value.source_ns)*1e-9-mt;
      const double a=p.yaw-ma-out.wz*dt;
      residual_xy+=x*x+y*y;eaa+=a*a;
    }
    // Correlated gait samples are not independent: two effective residual
    // degrees of freedom; isotropic trace bounds both body-axis fit variances.
    evidence.variance_vx=evidence.variance_vy=residual_xy/(2.*denominator)+options_.velocity_variance_floor;
    evidence.variance_wz=eaa/(2.*tt)+options_.yaw_variance_floor;
    evidence.valid=true;out.reason="ok";return out;
  }
 private:
  static bool positive(double x) {return std::isfinite(x)&&x>0.;}
  static double wrap(double x) {return std::atan2(std::sin(x),std::cos(x));}
  CausalBodyVelocityOptions options_;
  std::deque<BodyVelocitySample> history_;
  uint64_t epoch_{1};
};
}  // namespace sunhorizon_livo_bringup
