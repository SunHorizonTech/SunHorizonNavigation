#include "sunhorizon_navigation_runtime/vendor_admission.hpp"
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <future>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
using namespace fast_livo_direct;
using sunhorizon_navigation_runtime::detail::handle_vendor_admission;
using namespace std::chrono_literals;
void check(bool condition,const char* reason){if(!condition)throw std::runtime_error(reason);}
void race(const char* cause,bool overflow,bool poll_first=false) {
  FirstFaultLatch first;std::mutex ingress,reason_mutex;
  std::atomic<bool> closed{false},faulted{false},outer_revoked{false},callback_ran{false};
  std::atomic<unsigned> getter_calls{0};
  struct Owner {
    FirstFaultLatch& first;std::mutex& ingress;std::atomic<bool>& faulted;
    std::atomic<unsigned>& getter_calls;
    bool failed()const{return faulted.load();}
    std::optional<FirstFaultInfo> first_fault_snapshot()const {
      std::lock_guard<std::mutex> lock(ingress);++getter_calls;return first.snapshot();
    }
  } owner{first,ingress,faulted,getter_calls};
  using sunhorizon_navigation_runtime::detail::vendor_owner_failure_reason;
  using sunhorizon_navigation_runtime::detail::vendor_fault_cause;
  check(vendor_owner_failure_reason(owner).empty()&&!getter_calls.load(),"healthy_path_read_fault_snapshot");
  std::string outer_reason;
  const auto expected=std::string("livo_vendor:")+cause;
  auto revoke=[&](const FirstFaultInfo& info){
    std::lock_guard<std::mutex> lock(reason_mutex);
    if(!outer_revoked.exchange(true))outer_reason=vendor_fault_cause(info);
  };
  std::atomic<bool> generic_won{false},closed_seen{false};
  auto unexpected=[&](const char*){generic_won.store(true);};
  std::promise<void> published,release_callback;auto pub=published.get_future(),release=release_callback.get_future();
  auto primary=std::async(std::launch::async,[&]{
    std::optional<FirstFaultInfo> info;
    {std::lock_guard<std::mutex> lock(ingress);
      DirectInputOverflow snapshot;snapshot.kind=1;snapshot.occupancy.counts[1]=2048;
      first.publish_and_close(closed,faulted,cause,overflow?&snapshot:nullptr);info=first.snapshot();}
    published.set_value();release.wait();callback_ran.store(true);
    if(info)revoke(*info);else generic_won.store(true);
  });
  pub.wait();
  bool poll_revoked=true;
  if(poll_first) {
    auto polling=std::async(std::launch::async,[&]{
      // Same production sequence as main.cpp -> NavigationRuntime polling.
      if(owner.failed()) {
        const auto reason=vendor_owner_failure_reason(owner);
        std::lock_guard<std::mutex> lock(reason_mutex);
        if(!outer_revoked.exchange(true))outer_reason=reason;
      }
    });
    if(polling.wait_for(2s)!=std::future_status::ready)std::quick_exit(4);
    polling.get();poll_revoked=outer_revoked.load()&&!callback_ran.load()&&outer_reason==expected;
  }
  auto imu=std::async(std::launch::async,[&]{
    AdmissionResult rejected;
    {std::lock_guard<std::mutex> lock(ingress);closed_seen.store(closed.load());
      rejected={AdmissionStatus::Closed,first.snapshot()};}
    return handle_vendor_admission(rejected,false,revoke,unexpected);
  });
  if(imu.wait_for(2s)!=std::future_status::ready)std::quick_exit(4);
  const bool rejected=!imu.get(),revoked_before_callback=outer_revoked.load();
  const bool before_callback=!callback_ran.load(),same_cause=outer_reason==expected;
  release_callback.set_value();primary.get();
  check(closed_seen.load()&&rejected&&revoked_before_callback,"follower_did_not_revoke_immediately");
  check(before_callback&&same_cause&&!generic_won.load(),"follower_waited_for_callback_or_lost_cause");
  check(poll_revoked&&getter_calls.load()==unsigned(poll_first),"main_poll_lost_published_cause_or_waited_for_callback");
  check(outer_reason==expected,"late_callback_changed_first_reason");
}
int main() {
  race("direct_input_queue_overflow",true);
  race("direct_input_identity_or_timestamp_violation",false);
  race("IMU timestamp jump reset application queue",false);
  race("direct_input_queue_overflow",true,true);
  race("direct_input_identity_or_timestamp_violation",false,true);
  race("IMU timestamp jump reset application queue",false,true);
  int faults=0;std::string unexpected;
  auto info=[&](const FirstFaultInfo&){++faults;};
  auto other=[&](const char* reason){unexpected=reason;};
  check(!handle_vendor_admission({AdmissionStatus::Closed,std::nullopt},true,info,other),"stop_accepted");
  check(!faults&&unexpected.empty(),"normal_stop_invented_fault");
  handle_vendor_admission({AdmissionStatus::Closed,std::nullopt},false,info,other);
  check(unexpected=="vendor_runtime_closed_without_fault","unexpected_close_not_distinct");
  handle_vendor_admission({AdmissionStatus::NullInput,std::nullopt},false,info,other);
  check(unexpected=="vendor_null_input","null_input_not_distinct");
  unexpected.clear();check(handle_vendor_admission({},false,info,other),"accepted_input_failed");
  check(!faults&&unexpected.empty(),"accepted_hot_path_invoked_fault");
  std::cout<<"vendor_admission_cases=10 passed=true no_ros=true\n";
}
