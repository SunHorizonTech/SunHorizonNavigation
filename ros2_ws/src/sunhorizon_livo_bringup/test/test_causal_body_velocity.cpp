#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include "sunhorizon_livo_bringup/causal_body_velocity.hpp"
#include "sunhorizon_livo_bringup/body_odometry_gauge.hpp"
using namespace sunhorizon_livo_bringup;
namespace {
void require(bool ok,const char* reason){if(!ok)throw std::runtime_error(reason);}
void near(double a,double b,double tolerance,const char* reason){require(std::abs(a-b)<=tolerance,reason);}
constexpr int64_t epoch=1792577491000000000LL;
BodyVelocitySample sample(int i,double x,double y=0.,double yaw=0.){return {epoch+int64_t(i)*10000000,x,y,yaw};}
void constant_turn_and_strafe(){
 CausalBodyVelocity fit;BodyVelocityEstimate v;
 const double vx=.4,vy=.12,w=.6;
 for(int i=0;i<=150;++i){const double t=.01*i,a=w*t;
  v=fit.observe(sample(i,(vx*std::sin(a)+vy*(std::cos(a)-1))/w,
   (vx*(1-std::cos(a))+vy*std::sin(a))/w,a));}
 require(v.evidence.valid,"constant turn valid");near(v.vx,vx,2e-6,"turn body vx");near(v.vy,vy,1e-6,"turn body vy");near(v.wz,w,1e-12,"turn wz");
 require(v.evidence.effective_source_ns==epoch+1250000000LL,"explicit 250ms phase age");
 require(v.evidence.window_end_ns==epoch+1500000000LL,"pose stamp remains newest source");
 require(v.evidence.sample_count==51,"bounded support count");
 require(v.evidence.variance_vx>=.0004&&v.evidence.variance_wz>=.0004,"nonzero uncertainty floor");
}
void invalid_is_not_still(){
 CausalBodyVelocity fit;BodyVelocityEstimate v;
 for(int i=0;i<=60;++i)v=fit.observe(sample(i,.002*i));
 require(v.evidence.valid,"warm fit");auto old=v.evidence.reset_epoch;
 v=fit.observe(sample(60,.12));require(!v.evidence.valid,"duplicate source invalid");require(v.evidence.reset_epoch>old,"duplicate resets epoch");
 v=fit.observe(sample(61,.122));require(!v.evidence.valid&&v.evidence.sample_count==1,"no old samples after reset");
 fit.reset();v=fit.observe(sample(70,std::numeric_limits<double>::quiet_NaN()));require(!v.evidence.valid,"NaN invalid");
 fit.observe(sample(80,0.));v=fit.observe(sample(81,.3));require(!v.evidence.valid&&v.reason=="body_velocity_pose_discontinuity","jump invalid");
 fit.observe(sample(100,0.));v=fit.observe(sample(130,.01));require(!v.evidence.valid&&v.reason=="body_velocity_source_gap","gap invalid");
 CausalBodyVelocityOptions options;options.window_ns=0;bool threw=false;
 try{CausalBodyVelocity bad(options);}catch(const std::invalid_argument&){threw=true;}require(threw,"invalid config rejected");
}
void phase_and_stop(){
 CausalBodyVelocity fit;BodyVelocityEstimate v;double first_low=-1.,at_release=0.;
 for(int i=0;i<=300;++i){const double t=i*.01;const double x=t<1?0.:(t<2?.4*(t-1):.4);
  v=fit.observe(sample(i,x));
  if(t<.35)require(!v.evidence.valid,"warmup cannot be stationary evidence");
  if(i==200)at_release=v.vx;
  if(i>200&&v.evidence.valid&&v.vx<.01&&first_low<0)first_low=t;
 }
 near(at_release,.4,1e-12,"release retains measured motion");
 require(first_low>=2.4&&first_low<=2.5,"finite trailing stop latency");
 require(v.evidence.valid,"static valid after full support");near(v.vx,0.,1e-12,"actual static regression");
}
void immutable_vertical_datum(){
 BodyOdometryGauge gauge;near(gauge.relative_z(epoch,-.215),0.,0.,"initial body Z datum");
 near(gauge.relative_z(epoch+100000000,-.195),.020,1e-12,"heave preserved");
 near(gauge.relative_z(epoch+200000000,-.225),-.010,1e-12,"vertical fluctuation not zeroed");
 const auto origin=gauge.origin();require(origin.valid&&origin.source_ns==epoch,"original source pin");
 CausalBodyVelocity fit;fit.reset();near(gauge.relative_z(epoch+300000000,-.2),.015,1e-12,"velocity reset cannot change datum");
 gauge.reset();require(!gauge.origin().valid&&gauge.origin().epoch>origin.epoch,"whole reset creates new datum epoch");
 near(gauge.relative_z(epoch+400000000,.32),0.,0.,"new explicit epoch rebase");
}
void causality_and_gait(){
 CausalBodyVelocity a,b;BodyVelocityEstimate one,two;double error=0.;int n=0;
 for(int i=0;i<=500;++i){const double t=.01*i;const double x=.3*t+.008*std::sin(2*3.141592653589793*4*t);
  one=a.observe(sample(i,x));two=b.observe(sample(i,x));
  require(one.evidence.valid==two.evidence.valid&&one.vx==two.vx,"prefix invariant");
  if(i>100&&one.evidence.valid){error+=(one.vx-.3)*(one.vx-.3);++n;}}
 require(std::sqrt(error/n)<.02,"gait oscillation attenuation");
 // Arbitrarily modifying a FUTURE input cannot retroactively affect prefix.
 auto saved=one;b.observe(sample(501,9.));near(saved.vx,one.vx,0.,"no future access");
}
void periodic_pending_geometry_does_not_starve_warmup(){
 CausalBodyVelocity fit;BodyVelocityEstimate v;int valid=0,qualified=0;
 uint64_t fit_epoch=0;std::vector<int64_t> sources;
 for(int i=0;i<=150;++i){
  if(i%3==1){
   const auto pending=fit.reject_geometry(BodyVelocityGeometryStatus::FuturePending);
   require(!pending.evidence.valid&&pending.evidence.sample_count==0,"pending is not a velocity measurement");
   require(pending.reason=="body_velocity_source_time_fk_pending","typed pending reason");
   if(fit_epoch)require(pending.evidence.reset_epoch==fit_epoch,"pending preserves qualified history epoch");
   continue; // No position, source timestamp or guessed FK enters the fit.
  }
  const auto measured=sample(i,.002*i);sources.push_back(measured.source_ns);++qualified;
  v=fit.observe(measured);
  if(!fit_epoch)fit_epoch=v.evidence.reset_epoch;
  require(v.evidence.reset_epoch==fit_epoch,"periodic pending cannot reset warmup");
  if(v.evidence.valid){
   ++valid;near(v.vx,.2,1e-12,"qualified-only speed");
   require(v.evidence.window_end_ns==measured.source_ns,"last actual qualified source");
   uint32_t support=0;
   for(const auto source:sources)if(measured.source_ns-source<=500000000)++support;
   require(v.evidence.sample_count==support,"support excludes all pending poses");
  }
 }
 require(qualified>90&&valid>60&&v.evidence.valid,"normal periodic pending still warms up");
}
void pending_geometry_does_not_hide_gap_or_time_regression(){
 CausalBodyVelocity fit;BodyVelocityEstimate v;
 for(int i=0;i<=60;++i)v=fit.observe(sample(i,.002*i));
 require(v.evidence.valid,"pre-gap valid fit");const auto before_gap=v.evidence.reset_epoch;
 for(int i=61;i<96;++i){const auto missing=fit.reject_geometry(BodyVelocityGeometryStatus::FuturePending);
  require(!missing.evidence.valid&&missing.evidence.reset_epoch==before_gap,"pending emits no valid old fit");}
 v=fit.observe(sample(96,.192));
 require(!v.evidence.valid&&v.reason=="body_velocity_source_gap"&&v.evidence.reset_epoch>before_gap,
  "qualified sample after actual gap still resets");
 for(int i=97;i<=150;++i)v=fit.observe(sample(i,.002*i));
 require(v.evidence.valid,"gap recovery requires fresh support");const auto before_reverse=v.evidence.reset_epoch;
 fit.reject_geometry(BodyVelocityGeometryStatus::FuturePending);
 v=fit.observe(sample(149,.298));
 require(!v.evidence.valid&&v.reason=="body_velocity_source_not_increasing"&&v.evidence.reset_epoch>before_reverse,
  "source regression still fails closed after pending");
}
void geometry_fault_and_explicit_reset_do_not_reuse_history(){
 CausalBodyVelocity fit;BodyVelocityEstimate v;
 for(int i=0;i<=60;++i)v=fit.observe(sample(i,.002*i));
 require(v.evidence.valid,"pre-fault valid fit");const auto before_fault=v.evidence.reset_epoch;
 v=fit.reject_geometry(BodyVelocityGeometryStatus::Unavailable);
 require(!v.evidence.valid&&v.evidence.reset_epoch>before_fault,"real FK fault clears history");
 v=fit.observe(sample(61,.122));
 require(!v.evidence.valid&&v.evidence.sample_count==1,"fault cannot reuse old support");
 for(int i=62;i<=110;++i)v=fit.observe(sample(i,.002*i));
 require(v.evidence.valid,"recovered fresh fit");const auto before_reset=v.evidence.reset_epoch;
 fit.reset();
 v=fit.reject_geometry(BodyVelocityGeometryStatus::FuturePending);
 require(!v.evidence.valid&&v.evidence.reset_epoch>before_reset,"whole reset epoch survives pending");
 v=fit.observe(sample(111,.222));
 require(!v.evidence.valid&&v.evidence.sample_count==1,"whole reset cannot reuse pre-epoch support");
}
}
int main(){try{constant_turn_and_strafe();invalid_is_not_still();phase_and_stop();causality_and_gait();immutable_vertical_datum();
 periodic_pending_geometry_does_not_starve_warmup();pending_geometry_does_not_hide_gap_or_time_regression();
 geometry_fault_and_explicit_reset_do_not_reuse_history();
 std::cout<<"8 deterministic causal body velocity cases passed\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
