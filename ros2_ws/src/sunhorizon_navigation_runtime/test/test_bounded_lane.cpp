#include "sunhorizon_navigation_runtime/bounded_lane.hpp"
#include "sunhorizon_navigation_runtime/detail/direct_cloud_admission.hpp"
#include "sunhorizon_navigation_runtime/detail/peripheral_startup.hpp"
#include "sunhorizon_navigation_runtime/detail/raw_clearing_contract.hpp"
#include "sunhorizon_camera_runtime/latest_frame_worker.hpp"
#include <nav2_util/direct_observation.hpp>
#include <future>
#include <atomic>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <limits>
using sunhorizon_navigation_runtime::detail::BoundedLane;
void check(bool value){if(!value)throw std::runtime_error("navigation lane contract failure");}
void peripheral_startup_contract(){
  using Startup=sunhorizon_navigation_runtime::detail::PeripheralStartup;
  using Step=Startup::Step;
  const auto begin=Startup::Clock::time_point{},deadline=begin+std::chrono::seconds(1);
  Startup pending(deadline);
  check(pending.advance(false,true,{},begin)==Step::Waiting);
  check(pending.advance(true,false,{},begin)==Step::Waiting);
  check(pending.advance(true,true,{},begin)==Step::Request);
  for(int i=1;i<9;++i)check(pending.advance(true,true,{},begin+std::chrono::milliseconds(100*i))==Step::Waiting);
  check(pending.advance(true,true,true,deadline-std::chrono::nanoseconds(1))==Step::Complete);
  check(pending.advance(true,true,{},deadline)==Step::Complete);
  const auto rejected=[&](auto action){bool refused=false;try{action();}catch(const std::runtime_error&){refused=true;}check(refused);};
  Startup absent(deadline);rejected([&]{absent.advance(false,false,{},deadline);});
  Startup hung(deadline);check(hung.advance(true,true,{},begin)==Step::Request);
  rejected([&]{hung.advance(true,true,{},deadline);});
  Startup late(deadline);check(late.advance(true,true,{},begin)==Step::Request);
  rejected([&]{late.advance(true,true,true,deadline);});
  Startup refused(deadline);check(refused.advance(true,true,{},begin)==Step::Request);
  rejected([&]{refused.advance(true,true,false,begin);});
  Startup lost(deadline);check(lost.advance(true,true,{},begin)==Step::Request);
  rejected([&]{lost.advance(false,true,true,begin);});
}
void instantaneous_raw_clearing_contract(){
  using sunhorizon_navigation_runtime::detail::require_instantaneous_raw_clearing;
  require_instantaneous_raw_clearing(0.,"belly:global.voxel/floor");
  for(double invalid:{.1,-.1,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}){
    bool refused=false;try{require_instantaneous_raw_clearing(invalid,"chest:global.voxel/floor");}
    catch(const std::invalid_argument& error){refused=std::string(error.what()).find("chest:global.voxel/floor")!=std::string::npos;}
    check(refused);
  }
  struct Frame {nav2_util::DirectObservationMetadata metadata;std::shared_ptr<const int> cloud;};
  using Worker=sunhorizon_camera_runtime::LatestFrameWorker<Frame>;
  constexpr int64_t base=1000000000LL,budget=300000000LL;
  const nav2_util::DirectSourceContract contract{"chest","epoch","optical",budget};
  nav2_util::DirectObservationGate ingress(contract),slow_gate(contract),fast_gate(contract);
  std::promise<void> slow_entered,slow_release,slow_third,fast_first,fast_third;
  auto released=slow_release.get_future().share();
  std::vector<Frame> slow_seen,fast_seen;
  const auto failure=[](auto){check(false);};
  Worker slow([&](Frame& frame){
    if(frame.metadata.sequence==1){slow_entered.set_value();released.wait();}
    check(slow_gate.admit(frame.metadata,base+10000,base+10000)==nav2_util::DirectAdmission::Accepted);
    slow_seen.push_back(frame);if(frame.metadata.sequence==3)slow_third.set_value();
  },[]{return true;},failure);
  Worker fast([&](Frame& frame){
    check(fast_gate.admit(frame.metadata,base+10000,base+10000)==nav2_util::DirectAdmission::Accepted);
    fast_seen.push_back(frame);
    if(frame.metadata.sequence==1)fast_first.set_value();if(frame.metadata.sequence==3)fast_third.set_value();
  },[]{return true;},failure);
  const auto frame=[](uint64_t sequence){const auto source=base+static_cast<int64_t>(sequence)*100;
    return Frame{{"chest","epoch","optical",sequence,source,source+10,source+20,source-5},std::make_shared<const int>(int(sequence))};};
  std::vector<Frame> originals;
  const auto submit=[&](const Frame& input){
    // This is the same real metadata gate used before raw queue replacement.
    // Rejected frames never replace a previously validated pending owner.
    if(ingress.admit(input.metadata,base+1000,base+1000)!=nav2_util::DirectAdmission::Accepted)return false;
    slow.replace(std::make_unique<Frame>(input));fast.replace(std::make_unique<Frame>(input));return true;
  };
  for(uint64_t sequence=1;sequence<=3;++sequence){
    originals.push_back(frame(sequence));check(submit(originals.back()));
    if(sequence==1){slow_entered.get_future().wait();fast_first.get_future().wait();}
  }
  // A blocked sink cannot force the other actual sink to wait behind it.
  fast_third.get_future().wait();check(slow.stats().in_flight);check(slow.stats().replaced==1);
  auto invalid=frame(4);invalid.metadata.epoch="wrong_owner";
  const auto admitted=slow.stats().admitted;check(!submit(invalid));check(slow.stats().admitted==admitted);
  slow_release.set_value();slow_third.get_future().wait();
  slow.request_stop();fast.request_stop();slow.join();fast.join();
  check(slow_seen.size()==2 && slow_seen[0].metadata.sequence==1 && slow_seen[1].metadata.sequence==3);
  check(fast_seen.back().metadata.sequence==3);
  for(const auto& seen:slow_seen){const auto& original=originals.at(seen.metadata.sequence-1);
    check(seen.cloud==original.cloud);
    check(seen.metadata.source_ros_ns==original.metadata.source_ros_ns);
    check(seen.metadata.raw_source_ns==original.metadata.raw_source_ns);
    check(seen.metadata.first_receipt_ros_ns==original.metadata.first_receipt_ros_ns);
    check(seen.metadata.first_receipt_steady_ns==original.metadata.first_receipt_steady_ns);
  }
  // Expiry is still rejected by the original downstream gate; latest delivery
  // never substitutes a fresh receipt or turns a stale sink result into success.
  nav2_util::DirectObservationGate expired(contract);const auto stale=frame(1);
  check(expired.admit(stale.metadata,stale.metadata.source_ros_ns+budget+1,
    stale.metadata.first_receipt_steady_ns+budget+1)==nav2_util::DirectAdmission::Rejected);
  auto rejected=std::make_unique<Frame>(frame(5));const auto* offered=rejected.get();
  auto returned=slow.replace(std::move(rejected));check(returned.get()==offered);check(slow.stats().closed);
  std::promise<void> failed;
  Worker errors([](Frame&){throw std::runtime_error("original raw sink rejected");},[]{return true;},
    [&](auto error){check(bool(error));failed.set_value();});
  errors.replace(std::make_unique<Frame>(frame(1)));failed.get_future().wait();errors.join();
  check(errors.stats().closed && errors.stats().errors==1);
}
void airy_obstacle_latest_keeps_original_evidence(){
  struct Scan {nav2_util::DirectObservationMetadata metadata;int64_t queued_ns;std::shared_ptr<const int> rays;};
  using Input=std::shared_ptr<const Scan>;
  using Worker=sunhorizon_camera_runtime::LatestFrameWorker<Input>;
  constexpr int64_t base=1000000000LL,budget=300000000LL;
  const nav2_util::DirectSourceContract contract{"airy","epoch","lidar",budget};
  nav2_util::DirectObservationGate ingress(contract),filter(contract);
  std::promise<void> entered,release,last;const auto released=release.get_future().share();
  std::vector<Input> processed;
  Worker worker([&](Input& input){
    if(input->metadata.sequence==1){entered.set_value();released.wait();}
    check(filter.admit(input->metadata,base+10000,base+10000)==nav2_util::DirectAdmission::Accepted);
    processed.push_back(input);if(input->metadata.sequence==10)last.set_value();
  },[]{return true;},[](auto){check(false);});
  const auto make=[](uint64_t sequence){const auto source=base+int64_t(sequence)*100;
    return std::make_shared<const Scan>(Scan{{"airy","epoch","lidar",sequence,source,source+10,source+20,
      source-5,source+21,source+22,1},source+30,std::make_shared<const int>(int(sequence))});};
  std::vector<Input> originals;
  for(uint64_t sequence=1;sequence<=10;++sequence){
    auto input=make(sequence);originals.push_back(input);
    check(ingress.admit(input->metadata,base+10000,base+10000)==nav2_util::DirectAdmission::Accepted);
    auto pending=std::make_unique<Input>(input);const auto* offered=pending.get();
    const auto retired=worker.replace(std::move(pending));check(retired.get()!=offered);
    if(sequence==1)entered.get_future().wait();
    if(sequence>=3)check(*retired==originals.at(sequence-2));
  }
  const auto queued=worker.stats();
  check(queued.admitted==10 && queued.taken==1 && queued.completed==0 && queued.replaced==8);
  check(queued.pending && queued.in_flight && !queued.closed && !queued.errors);
  release.set_value();last.get_future().wait();worker.request_stop();worker.join();
  check(processed==std::vector<Input>({originals.front(),originals.back()}));
  // Sharing the original immutable owner preserves raw/source/receipt clocks,
  // normalization timing, original queue time, ray ownership and sequence gaps.
  check(processed.back()->queued_ns==originals.back()->queued_ns && processed.back()->rays==originals.back()->rays);
  const auto completed=worker.stats();check(completed.completed==2 && completed.replaced==8 && completed.taken==2);
  const auto& latest=processed.back()->metadata;
  check(!filter.fresh(latest.source_ros_ns+budget+1,latest.first_receipt_steady_ns+budget+1));
  auto pending=std::make_unique<Input>(make(11));const auto* offered=pending.get();
  auto rejected=worker.replace(std::move(pending));check(rejected.get()==offered && worker.stats().closed);
  std::promise<void> failed;
  Worker errors([](Input&){throw std::runtime_error("original airy filter failed");},[]{return true;},
    [&](auto error){check(bool(error));failed.set_value();});
  errors.replace(std::make_unique<Input>(make(1)));failed.get_future().wait();errors.join();
  check(errors.stats().closed && errors.stats().errors==1 && errors.stats().completed==0);
}
void typed_admission_keeps_lane_alive_without_current_evidence(){
  using namespace nav2_util;
  using sunhorizon_navigation_runtime::detail::require_nonfatal_cloud_admission;
  constexpr int64_t base=1000000000LL,budget=300000000LL;
  DirectObservationGate gate({"chest","epoch","optical",budget});
  std::promise<void> dropped,recovered,failed;
  std::atomic<bool> faulted{false};
  BoundedLane<int> lane(2,[&](int event){
    const int64_t now=base+budget+1+event;
    DirectObservationMetadata metadata{"chest","epoch","optical",uint64_t(event+1),
      event==0?base:now,now,now};
    if(event==2)metadata.epoch="wrong_owner";
    const auto result=gate.admitResult(metadata,now,now);
    require_nonfatal_cloud_admission(result,"test.actual_lane");
    if(event==0){
      check(result.state==DirectAdmissionState::MotionUnavailable);
      check(result.reason==DirectRejectionReason::SourceExpired);
      check(!gate.fresh(now,now)); // Runtime lives; unavailable evidence cannot drive motion.
      check(!gate.faulted());dropped.set_value();
    }
    if(event==1){check(result.accepted());check(gate.fresh(now,now));recovered.set_value();}
  },[&](auto error){check(bool(error));faulted.store(true);failed.set_value();});
  check(lane.admit(0));dropped.get_future().wait();check(!faulted.load());
  check(lane.admit(1));recovered.get_future().wait();check(!faulted.load());
  check(lane.admit(2));failed.get_future().wait();lane.join();check(faulted.load());
  check(gate.faulted());check(!lane.admit(3));
}
int main(){
  typed_admission_keeps_lane_alive_without_current_evidence();
  peripheral_startup_contract();
  instantaneous_raw_clearing_contract();
  airy_obstacle_latest_keeps_original_evidence();
  std::promise<void> first_started,release,done;auto released=release.get_future().share();
  std::vector<int> received;
  BoundedLane<int> ordered(2,[&](int value){received.push_back(value);
    if(value==0){first_started.set_value();released.wait();}if(value==2)done.set_value();},[](auto){});
  check(ordered.pending_size()==0);
  check(ordered.admit(0));first_started.get_future().wait();check(ordered.pending_size()==0);
  check(ordered.admit(1));check(ordered.admit(2));check(!ordered.admit(3));check(ordered.pending_size()==2);
  release.set_value();done.get_future().wait();ordered.request_stop();ordered.join();check(ordered.pending_size()==0);
  check(received==std::vector<int>({0,1,2}));check(!ordered.admit(4));
  std::promise<void> stop_started,stop_release;auto stop_ready=stop_release.get_future().share();int count=0;
  BoundedLane<int> closing(2,[&](int){++count;stop_started.set_value();stop_ready.wait();},[](auto){});
  check(closing.admit(0));stop_started.get_future().wait();check(closing.admit(1));
  closing.request_stop();stop_release.set_value();closing.join();check(count==1);
  std::promise<void> failed;
  BoundedLane<int> errors(1,[](int){throw std::runtime_error("input lane failure");},[&](auto error){check(bool(error));failed.set_value();});
  check(errors.admit(0));failed.get_future().wait();errors.join();check(!errors.admit(1));
  std::cout<<"navigation lanes, instantaneous raw sinks, and one-shot startup/deadline contracts passed\n";
}
