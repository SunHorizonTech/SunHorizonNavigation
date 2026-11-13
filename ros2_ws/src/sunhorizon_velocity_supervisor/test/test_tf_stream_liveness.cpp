#include "sunhorizon_velocity_supervisor/tf_continuity.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace sunhorizon_velocity_supervisor;
void check(bool value) {if (!value) throw std::runtime_error("TF stream liveness");}
int main() {
  TfProgressTracker progress;
  check(progress.observe({7., 0., 0., 0.}, 10., 20.));
  check(!progress.current(10., 20., .25)); // A single dead cache is insufficient.
  for (int i=1; i<=20; ++i) {
    const double t = double(i)*.05;
    check(progress.observe({7.+t, 0., 0., 0.}, 10.+t, 20.+t));
    check(progress.current(10.+t, 20.+t, .25));
  }
  check(progress.source_sec()==8. && progress.receipt_ros_sec()==11.);
  // Repeated lookups, including a staged reset, cannot renew a frozen source.
  for (int i=1; i<=25; ++i) {
    const double t = double(i)*.01;
    check(progress.observe({8., 0., 0., 0.}, 11.+t, 21.+t));
    auto reset_copy=progress;
    check(reset_copy.receipt_ros_sec()==11.);
    check(reset_copy.current(11.+t, 21.+t, .25));
  }
  check(!progress.current(11.250000001,21.250000001,.25));
  check(!progress.current(11.25,21.250000001,.25));
  check(!progress.current(11.250000001,21.25,.25));
  for (int fault=0; fault<5; ++fault) {
    TfProgressTracker invalid;
    check(invalid.observe({7., 0., 0., 0.}, 10., 20.));
    auto value=PlanarTfObservation{7.1, 0., 0., 0.};
    double ros=10.1,steady=20.1;
    if (fault==0) value.stamp_sec=6.9;
    if (fault==1) value.stamp_sec=11.;
    if (fault==2) value.x=std::numeric_limits<double>::quiet_NaN();
    if (fault==3) ros=9.9;
    if (fault==4) steady=19.9;
    check(!invalid.observe(value,ros,steady));
    check(!invalid.current(10.2,20.2,.25));
  }
  TfContinuityTracker continuity;
  check(continuity.observe({7.,0.,0.,0.},10.,.1,.1));
  check(!continuity.observe({7.05,.3,0.,0.},10.05,.1,.1));
  std::cout<<"delayed live TF accepted; frozen cache, reset, clocks and jumps protected\n";
}
