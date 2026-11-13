#include "sunhorizon_perception_runtime/rgbd_health_core.hpp"
#include "sunhorizon_perception_runtime/role_completion.hpp"
#include "sunhorizon_camera_runtime/latest_frame_worker.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace sunhorizon_perception_runtime;
namespace {
void check(bool value,const char * message) {if (!value) throw std::runtime_error(message);}
RgbdHealthConfig config() {
  RgbdHealthConfig c;c.target_rate_hz=10.;c.minimum_healthy_pair_rate_hz=1.;
  c.statistics_window_sec=1.;c.maximum_rejected_pair_ratio=.25;
  c.chest_alignment_calibration_accepted=true;c.chest_depth_scale_accepted=true;
  c.belly_projection_calibration_accepted=true;c.belly_depth_scale_accepted=true;
  c.calibration_report_sha256=std::string(64,'a');return c;
}
RgbdHealthObservation frame(const std::string & role,std::uint64_t sequence,std::int64_t ns) {
  RgbdHealthObservation f;f.role=role;f.sequence=sequence;
  f.depth_source_ns=ns;f.color_source_ns=ns;f.receipt_ros_ns=ns;f.receipt_steady_ns=ns;
  f.source_validated=true;f.depth_shape_valid=true;f.color_shape_valid=true;
  f.depth_info_matches=true;f.color_info_matches=true;f.depth_scale_m=.001;f.valid_depth_ratio=1.;
  f.color={double(ns)/1.e9,640,480,"rgb8","optical"};
  f.depth={double(ns)/1.e9,640,480,"16UC1","optical"};
  f.color_info={640,480,500.,500.,320.,240.,"optical"};f.depth_info=f.color_info;return f;
}
void accepted_pair_does_not_commit_cloud() {
  RgbdHealthCore h(config());auto chest=frame("chest",1,10000000000LL),belly=frame("belly",1,10000000000LL);
  check(h.admit(chest,chest.depth_source_ns,chest.receipt_steady_ns).computation_allowed,"chest admission");
  check(h.admit(belly,belly.depth_source_ns,belly.receipt_steady_ns).computation_allowed,"belly admission");
  auto s=h.snapshot(10000000000LL,10000000000LL);
  check(s.chest_pair_fresh && s.belly_depth_fresh,"source freshness independent");
  check(!s.chest_pointcloud_healthy && !s.belly_pointcloud_healthy,"admission must not commit a cloud");
  check(s.state!=2,"pending computation cannot be healthy");
  check(h.complete("chest",1,10000000000LL,10000000001LL,10000000001LL),"chest completion");
  check(h.complete("belly",1,10000000000LL,10000000001LL,10000000001LL),"belly completion");
  check(h.snapshot(10000000002LL,10000000002LL).state==2,"both completions healthy");
  // New healthy camera arrivals cannot renew a stalled computation.
  for (int i=2;i<=5;++i) {
    const auto ns=10000000000LL+(i-1)*100000001LL;
    check(h.admit(frame("chest",i,ns),ns,ns).accepted,"camera keeps arriving");
    check(h.admit(frame("belly",i,ns),ns,ns).accepted,"belly keeps arriving");
  }
  s=h.snapshot(10400000004LL,10400000004LL);
  check(s.chest_pair_fresh && s.belly_depth_fresh,"fresh arrivals remain fresh");
  check(!s.chest_pointcloud_healthy && !s.belly_pointcloud_healthy,"stalled computation expires");
  check(s.state!=2,"stalled computation is unhealthy");
}
void pair_rate_rejection_info_and_scale_preserved() {
  auto c=config();c.minimum_healthy_pair_rate_hz=3.;RgbdHealthCore h(c);
  auto f=frame("chest",1,10000000000LL);check(h.admit(f,f.depth_source_ns,f.receipt_steady_ns).accepted,"first accepted");
  auto s=h.snapshot(f.depth_source_ns,f.receipt_steady_ns);check(s.reason.find("pair_rate_low")!=std::string::npos,"pair rate floor");
  f=frame("chest",2,10200000000LL);f.color.stamp_sec+=.01;f.color_source_ns+=10000000LL;
  check(!h.admit(f,10210000000LL,10210000000LL).accepted,"strict pair delta");
  s=h.snapshot(10210000000LL,10210000000LL);
  check(s.source.accepted_pairs==1 && s.source.rejected_pairs==1,"rolling counters");
  check(s.rejected_pair_ratio==.5 && s.reason.find("pair_rejection_ratio_high")!=std::string::npos,"rejection ratio");
  f=frame("chest",3,10400000000LL);f.depth_info.fx=0.;
  check(!h.admit(f,10400000000LL,10400000000LL).accepted,"invalid focal length");
  check(!h.snapshot(10400000000LL,10400000000LL).source.chest_info_valid,"bad info is unhealthy");
  f=frame("belly",1,10400000000LL);f.depth_scale_m=.01;
  check(!h.admit(f,10400000000LL,10400000000LL).accepted,"scale mismatch rejected");
  f=frame("belly",2,10500000000LL);f.valid_depth_ratio=0.;
  check(!h.admit(f,10500000000LL,10500000000LL).accepted,"valid ratio retained");
}
void exact_completion_fault_and_steady_deadlines() {
  RgbdHealthCore h(config());auto f=frame("belly",1,10000000000LL);
  check(h.admit(f,10000000000LL,10000000000LL).accepted,"accepted belly");
  check(!h.complete("belly",1,10000000001LL,10000000002LL,10000000002LL),"cloud source stamp must be exact");
  check(!h.complete("belly",1,10000000000LL,10300000001LL,10300000001LL),"late cloud cannot refresh health");
  check(h.complete("belly",1,10000000000LL,10000000001LL,10000000001LL),"valid cloud commit");
  check(!h.complete("belly",1,10000000000LL,10000000002LL,10000000002LL),"duplicate completion rejected");
  RgbdHealthCore delayed(config());check(delayed.admit(f,10000000000LL,10000000000LL).accepted,"delayed admitted");
  check(delayed.complete("belly",1,10000000000LL,10000000001LL,10250000000LL),"bounded delayed completion");
  check(!delayed.snapshot(10000000002LL,10400000000LL).belly_pointcloud_healthy,
    "cloud keeps original receipt expiry even after a recent completion");
  auto s=h.snapshot(10000000002LL,10400000000LL);
  check(!s.belly_depth_fresh && !s.belly_pointcloud_healthy,"paused ROS clock cannot extend source/computation");
  f=frame("belly",2,10200000000LL);check(h.admit(f,10200000000LL,10200000000LL).accepted,"next frame");
  h.reject_computation("belly",2,"model_unavailable");
  check(!h.complete("belly",2,10200000000LL,10200000001LL,10200000001LL),"failed calculation cannot ack later");
  s=h.snapshot(10200000001LL,10200000001LL);check(!s.belly_pointcloud_healthy,"failed computation clears health");
  h.fault("capture_timestamp_regression");
  check(h.snapshot(10200000001LL,10200000001LL).state==4,"fatal source fault immediately unhealthy");
  f=frame("belly",3,10300000000LL);check(!h.admit(f,10300000000LL,10300000000LL).accepted,"fault stays latched");
}
void asynchronous_admission_keeps_exact_older_completion() {
  RgbdHealthCore h(config());
  const auto first=frame("belly",1,10000000000LL),next=frame("belly",2,10100000000LL);
  check(h.admit(first,10000000000LL,10000000000LL).computation_allowed,"first asynchronous admission");
  check(h.admit(next,10100000000LL,10100000000LL).computation_allowed,"newer asynchronous admission");
  check(h.complete("belly",1,10000000000LL,10110000000LL,10110000000LL),
    "newer admission must not reject older valid cloud completion");
  check(h.complete("belly",2,10100000000LL,10110000001LL,10110000001LL),"newer completion commits");
  check(!h.complete("belly",1,10000000000LL,10110000002LL,10110000002LL),"older completion cannot roll cloud stamp back");
}
void registration_failure_requires_new_successful_completion() {
  RgbdHealthCore h(config());
  auto first=frame("belly",544,1792665060465484000LL);
  auto failed=frame("belly",545,1792665060565484000LL);
  auto next=frame("belly",546,1792665060665484000LL);
  check(h.admit(first,first.depth_source_ns,first.receipt_steady_ns).accepted,"old original admission");
  check(h.complete("belly",544,first.depth_source_ns,first.depth_source_ns,first.receipt_steady_ns),"old complete");
  check(h.admit(failed,failed.depth_source_ns,failed.receipt_steady_ns).accepted,"P545 admission");
  h.reject_computation("belly",545,"registration_ground_unavailable:pcd_registration_original_evidence_expired",true);
  auto status=h.snapshot(failed.depth_source_ns,failed.receipt_steady_ns);
  check(!status.belly_pointcloud_healthy,"expired registration immediately unhealthy");
  check(h.admit(next,next.depth_source_ns,next.receipt_steady_ns).accepted,"new SDK admission");
  status=h.snapshot(next.depth_source_ns,next.receipt_steady_ns);
  check(!status.belly_pointcloud_healthy,"SDK admission cannot renew upstream health");
  check(status.reason.find("pcd_registration_original_evidence_expired")!=std::string::npos,"original failure retained");
  check(!h.complete("belly",545,failed.depth_source_ns,next.depth_source_ns,next.receipt_steady_ns),"failed token never completes");
  check(h.complete("belly",546,next.depth_source_ns,next.depth_source_ns,next.receipt_steady_ns),"fresh classified output restores");
  status=h.snapshot(next.depth_source_ns,next.receipt_steady_ns);
  check(status.belly_pointcloud_healthy,"new successful completion restores source");
  h.reject_computation("belly",545,"late old failure",true);
  status=h.snapshot(next.depth_source_ns,next.receipt_steady_ns);
  // Generic failure may retain its existing conservative disposition, but an
  // already superseded failure must not become the new upstream epoch barrier.
  check(status.reason.find("belly_upstream=")==std::string::npos,"old failure cannot relatch upstream barrier");
}
void calibration_and_throttle() {
  auto c=config();c.chest_alignment_calibration_accepted=false;c.belly_projection_calibration_accepted=false;
  RgbdHealthCore h(c);auto chest=frame("chest",1,10000000000LL),belly=frame("belly",1,10000000000LL);
  auto a=h.admit(chest,10000000000LL,10000000000LL);
  check(a.accepted && !a.computation_allowed,"unaccepted chest calibration can count stream but no cloud");
  a=h.admit(belly,10000000000LL,10000000000LL);check(a.accepted && !a.computation_allowed,"belly calibration gate");
  chest=frame("chest",2,10010000000LL);a=h.admit(chest,10010000000LL,10010000000LL);
  check(!a.accepted && a.reason=="target_rate_throttled","target rate selection");
  const auto s=h.snapshot(10010000000LL,10010000000LL);
  check(s.source.accepted_pairs==1 && s.source.rejected_pairs==0,"throttle does not inflate rejection ratio");
  bool threw=false;try {auto bad=config();bad.calibration_report_sha256="";RgbdHealthCore fail(bad);}
  catch (const std::invalid_argument &) {threw=true;}check(threw,"calibration receipt mandatory");
}
void completion_notifies_existing_worker_without_renewing_clocks() {
  RgbdHealthCore h(config());
  constexpr int64_t source=10000000000LL,receipt=20000000000LL;
  std::atomic<int64_t> ros{source+100000000},steady{receipt+100000000};
  std::mutex mutex;std::condition_variable changed;
  bool blocked=false,entered=false;int deliveries=0;
  std::atomic<bool> worker_failed{false};RgbdHealthSnapshot delivered;
  sunhorizon_camera_runtime::LatestFrameWorker<RgbdHealthSnapshot> worker(
    [&](auto& trigger){
      check(trigger.source.chest_points_stamp==0.,"completion wakeup carries no cached healthy state");
      std::unique_lock<std::mutex> lock(mutex);
      entered=true;changed.notify_all();
      check(changed.wait_for(lock,std::chrono::seconds(2),[&]{return !blocked;}),"release delayed egress");
      delivered=h.snapshot(ros.load(),steady.load());++deliveries;changed.notify_all();
    },[]{return true;},[&](auto){worker_failed.store(true);changed.notify_all();});
  const auto notify=[&]{worker.replace(std::make_unique<RgbdHealthSnapshot>());};
  auto chest=frame("chest",1,source),belly=frame("belly",1,source);
  chest.receipt_steady_ns=receipt;belly.receipt_steady_ns=receipt;
  check(h.admit(chest,source,receipt).computation_allowed,"chest admitted for notification");
  check(h.admit(belly,source,receipt).computation_allowed,"belly admitted for notification");
  bool handed_off=false;
  detail::complete_role(true,[]{return true;},[&]{handed_off=true;},[&]{
    detail::complete_health_and_notify([&]{
      check(handed_off,"health completion follows direct handoff");
      return h.complete("chest",1,source,ros.load(),steady.load());
    },notify);
  },[]{},[](const auto&){throw std::runtime_error("unexpected handoff failure");});
  // The DDS path uses the same successful-completion notification contract.
  detail::complete_health_and_notify([&]{return h.complete("belly",1,source,ros.load(),steady.load());},notify);
  {
    std::unique_lock<std::mutex> lock(mutex);
    check(changed.wait_for(lock,std::chrono::seconds(2),[&]{
      return delivered.chest_pointcloud_healthy && delivered.belly_pointcloud_healthy;
    }),"successful completion wakes egress without any timer tick");
    check(std::abs(delivered.chest_pointcloud_age_sec-.1)<1e-8,"egress preserves original source age");
  }
  const auto admitted=worker.stats().admitted;
  detail::complete_health_and_notify([&]{return h.complete("chest",1,source,ros.load(),steady.load());},notify);
  detail::complete_health_and_notify([&]{return h.complete("belly",1,source+1,ros.load(),steady.load());},notify);
  check(worker.stats().admitted==admitted,"duplicate or mismatched completion must not notify");
  bool failed=false;
  try {
    detail::complete_role(true,[]{return true;},[]{throw std::runtime_error("handoff failed");},[&]{
      detail::complete_health_and_notify([]{return true;},notify);
    },[]{},[&](const auto&){failed=true;});
  } catch(const std::runtime_error&) {}
  check(failed && worker.stats().admitted==admitted,"failed handoff cannot notify health");

  auto next=frame("chest",2,source+200000000);next.receipt_steady_ns=receipt+200000000;
  check(h.admit(next,next.depth_source_ns,next.receipt_steady_ns).computation_allowed,"next original input");
  ros.store(source+210000000);steady.store(receipt+210000000);
  int before;
  {std::lock_guard<std::mutex> lock(mutex);blocked=true;entered=false;before=deliveries;}
  detail::complete_health_and_notify([&]{return h.complete("chest",2,next.depth_source_ns,ros.load(),steady.load());},notify);
  {
    std::unique_lock<std::mutex> lock(mutex);
    check(changed.wait_for(lock,std::chrono::seconds(2),[&]{return entered;}),"completion wakes blocked observer");
    // Delayed delivery with a nearly paused ROS clock must still expire the
    // original steady receipt. A trigger must never act as new evidence.
    ros.store(source+220000000);steady.store(receipt+510000000);blocked=false;changed.notify_all();
    check(changed.wait_for(lock,std::chrono::seconds(2),[&]{return deliveries>before;}),"delayed egress completes");
    check(!delivered.chest_pointcloud_healthy,"notification cannot refresh expired original steady receipt");
    check(std::abs(delivered.chest_pointcloud_age_sec-.02)<1e-8,"original source still retained after delayed notification");
  }
  next=frame("chest",3,source+600000000);next.receipt_steady_ns=receipt+600000000;
  check(h.admit(next,next.depth_source_ns,next.receipt_steady_ns).computation_allowed,"next input before fault");
  ros.store(source+610000000);steady.store(receipt+610000000);
  {std::lock_guard<std::mutex> lock(mutex);blocked=true;entered=false;before=deliveries;}
  detail::complete_health_and_notify([&]{return h.complete("chest",3,next.depth_source_ns,ros.load(),steady.load());},notify);
  {
    std::unique_lock<std::mutex> lock(mutex);
    check(changed.wait_for(lock,std::chrono::seconds(2),[&]{return entered;}),"fault test reaches delayed observer");
    h.fault("fault_after_completion");blocked=false;changed.notify_all();
    check(changed.wait_for(lock,std::chrono::seconds(2),[&]{return deliveries>before;}),"fault observer completes");
    check(delivered.state==4 && !delivered.chest_pointcloud_healthy &&
      delivered.reason.find("fault_after_completion")!=std::string::npos,
      "queued completion cannot publish pre-fault healthy state");
  }
  worker.request_stop();worker.join();check(!worker_failed.load(),"health observer must not fail");
}
}  // namespace
int main() {
  try {accepted_pair_does_not_commit_cloud();pair_rate_rejection_info_and_scale_preserved();
    exact_completion_fault_and_steady_deadlines();asynchronous_admission_keeps_exact_older_completion();calibration_and_throttle();registration_failure_requires_new_successful_completion();
    completion_notifies_existing_worker_without_renewing_clocks();
    std::cout<<"RgbdHealth: 7 deterministic scenarios passed\n";return EXIT_SUCCESS;}
  catch (const std::exception & e) {std::cerr<<e.what()<<'\n';return EXIT_FAILURE;}
}
