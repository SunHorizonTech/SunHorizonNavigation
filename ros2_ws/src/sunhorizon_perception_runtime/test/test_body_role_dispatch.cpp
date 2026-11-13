#include "sunhorizon_perception_runtime/body_role_dispatch.hpp"
#include "sunhorizon_camera_runtime/latest_frame_worker.hpp"
#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

namespace detail=sunhorizon_perception_runtime::detail;
template<class T>using Worker=sunhorizon_camera_runtime::LatestFrameWorker<T>;
using namespace std::chrono_literals;
struct Frame {int sequence;long source_ns;long receipt_ns;};
void require(bool ok,const char* reason){if(!ok)throw std::runtime_error(reason);}

void independent_latest_owner() {
  std::promise<void> belly_enter,chest_enter,release_belly,release_chest,chest_latest;
  const auto belly_gate=release_belly.get_future().share();
  const auto chest_gate=release_chest.get_future().share();
  auto b=belly_enter.get_future(),c=chest_enter.get_future(),latest=chest_latest.get_future();
  std::atomic<int> failures{0},retained{0};std::vector<Frame> chest_seen;
  Worker<Frame> belly([&](Frame&){belly_enter.set_value();belly_gate.wait();},[]{return true;},[&](auto){++failures;});
  Worker<Frame> chest([&](Frame& f){
    chest_seen.push_back(f);
    if(f.sequence==1){chest_enter.set_value();chest_gate.wait();}
    if(f.sequence==3)chest_latest.set_value();
  },[]{return true;},[&](auto){++failures;});
  auto submit=[&](std::size_t role,Frame f){detail::dispatch_prepared_body(role,false,f,
    [&](const Frame& x){belly.replace(std::make_unique<Frame>(x));},
    [&](const Frame& x){chest.replace(std::make_unique<Frame>(x));},
    [&](const Frame&){++retained;});};
  submit(0,{10,1000,1100});const bool belly_started=b.wait_for(1s)==std::future_status::ready;
  submit(1,{1,2000,2100});const bool chest_started=c.wait_for(1s)==std::future_status::ready;
  submit(1,{2,3000,3100});submit(1,{3,4000,4100});
  release_chest.set_value();const bool chest_advanced=latest.wait_for(1s)==std::future_status::ready;
  // The body-only chest advanced while belly remained blocked. It discarded
  // only the pending middle frame and preserved the actual source identities.
  release_belly.set_value();belly.request_stop();chest.request_stop();belly.join();chest.join();
  require(belly_started&&chest_started&&chest_advanced,"body_only_chest_waited_for_belly");
  require(chest_seen.size()==2&&chest_seen[0].sequence==1&&chest_seen[1].sequence==3,"latest_pending_owner_not_bounded");
  require(chest_seen[1].source_ns==4000&&chest_seen[1].receipt_ns==4100,"original_clocks_changed");
  require(retained==0&&failures==0,"independent_role_loaned_to_spatial_pair");
}
void dual_source_retains_original_pairing() {
  int belly=0,independent=0;std::vector<Frame> history;
  const Frame chest{7,7000,7100};
  detail::dispatch_prepared_body(1,true,chest,[&](const Frame&){++belly;},
    [&](const Frame&){++independent;},[&](const Frame& f){history.push_back(f);});
  require(belly==0&&independent==0&&history.size()==1,"dual_source_chest_escaped_anchor_history");
  require(history.front().source_ns==7000&&history.front().receipt_ns==7100,"dual_source_identity_changed");
  bool rejected=false;
  try{detail::dispatch_prepared_body(2,false,chest,[](auto&){},[](auto&){},[](auto&){});}
  catch(const std::invalid_argument&){rejected=true;}
  require(rejected,"invalid_role_accepted");
}
void independent_failure_reaches_original_failure_owner() {
  std::promise<void> failed;auto event=failed.get_future();std::atomic<int> count{0};
  Worker<Frame> chest([](Frame&){throw std::runtime_error("original_ground_failure");},[]{return true;},
    [&](std::exception_ptr e){try{std::rethrow_exception(e);}catch(const std::runtime_error& x){
      require(std::string(x.what())=="original_ground_failure","failure_identity_changed");++count;failed.set_value();}});
  const Frame frame{8,8000,8100};detail::dispatch_prepared_body(1,false,frame,[](auto&){},
    [&](const Frame& f){chest.replace(std::make_unique<Frame>(f));},[](auto&){});
  const bool observed=event.wait_for(1s)==std::future_status::ready;chest.join();
  require(observed&&count==1&&chest.stats().closed&&chest.stats().errors==1,"independent_exception_hidden_in_future");
}
int main(){try{independent_latest_owner();dual_source_retains_original_pairing();independent_failure_reaches_original_failure_owner();
  std::cout<<"body-role dispatch: independent progress, bounded ownership, unchanged dual pairing and failure propagation passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}return 0;}
