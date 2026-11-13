#include "sunhorizon_navigation_runtime/detail/original_source_geometry_wait.hpp"
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <vector>
namespace d=sunhorizon_navigation_runtime::detail;
enum class State {Ready,FuturePending,Stale,Fault,Missing};
struct Result {State state;int64_t source_ns;};
void check(bool value){if(!value)throw std::runtime_error("original source geometry wait");}
struct Fixture {
  d::OriginalSourceGeometryLease lease{1000000000,1020000000,2020000000,250000000};
  d::GeometryWaitClocks clocks{1025000000,2025000000};
  std::vector<State> states{State::FuturePending,State::Ready};
  int query_count{0},wait_count{0};bool stopped{false},regress{false};
  int64_t wait_ns{75000000},query_ns{0};
  std::optional<Result> run(){return d::wait_for_original_source_geometry(lease,
    [&]{return clocks;},[&]{
      clocks.ros_ns+=query_ns;clocks.steady_ns+=query_ns;
      const auto i=std::min(static_cast<std::size_t>(query_count++),states.size()-1);
      return Result{states[i],lease.source_ns};},
    [](const auto& r){return r.state==State::FuturePending;},
    [&]{++wait_count;clocks.ros_ns+=regress?-wait_ns:wait_ns;clocks.steady_ns+=wait_ns;},
    [&]{return stopped;});}
};
int main(){
  Fixture f;
  // Before this change a 75 ms late upper bracket discarded the real sample.
  // It now resolves the SAME source epoch without changing either receipt.
  const auto original=f.lease;auto result=f.run();
  check(result&&result->state==State::Ready&&result->source_ns==original.source_ns);
  check(f.wait_count==1&&f.query_count==2&&f.lease.source_ns==original.source_ns&&
    f.lease.first_receipt_ros_ns==original.first_receipt_ros_ns&&
    f.lease.first_receipt_steady_ns==original.first_receipt_steady_ns);
  // A FIFO of REAL 5 ms epochs accumulated while the independent joint writer
  // was 75 ms late. One-shot lookup loses those epochs even though a later
  // latest pose is fresh. Waiting preserves each original epoch for the same
  // unchanged 50 ms correction-history interpolation gate.
  const auto gap_accepted=[](const std::vector<int64_t>& history,int64_t target){
    const auto upper=std::lower_bound(history.begin(),history.end(),target);
    if(upper==history.end()||upper==history.begin())return false;
    return target-*std::prev(upper)<=50000000 && *upper-target<=50000000;
  };
  std::vector<int64_t> preserved{995000000};
  check(!gap_accepted({995000000,1075000000},1022000000));
  for(int64_t source=1000000000;source<=1075000000;source+=5000000){
    Fixture item;
    item.lease={source,source+20000000,source+1020000000,250000000};
    if(source!=1000000000){item.clocks={1100000000,2100000000};item.states={State::Ready};}
    const auto kept=item.run();check(kept&&kept->source_ns==source);preserved.push_back(kept->source_ns);
  }
  check(preserved.size()==17&&gap_accepted(preserved,1022000000));
  for(auto state:{State::Stale,State::Fault,State::Missing}){
    f={};f.states={state};result=f.run();check(result&&result->state==state&&f.wait_count==0);
  }
  f={};f.states={State::FuturePending};check(!f.run()&&f.wait_count==4&&f.query_count==4);
  f={};f.lease.source_ns=1000000;result=f.run();
  check(result&&result->source_ns==1000000);
  // Receipt lease is inclusive; source transport age no longer consumes it.
  f={};f.clocks={1270000000,2270000000};f.states={State::Ready};check(f.run().has_value());
  f={};f.clocks={1270000001,2270000001};f.states={State::Ready};check(!f.run()&&f.query_count==0);
  f={};f.clocks={1269000000,2269000000};f.states={State::Ready};f.query_ns=1000001;
  check(!f.run()&&f.query_count==1);
  f={};f.clocks.steady_ns=f.lease.first_receipt_steady_ns+250000001;check(!f.run());
  f={};f.regress=true;check(!f.run()&&f.query_count==1);
  f={};f.stopped=true;check(!f.run()&&f.query_count==0);
  f={};f.lease.first_receipt_ros_ns=f.lease.source_ns-1;check(!f.run());
  f={};f.lease.source_ns=0;check(!f.run());
  std::cout<<"original source geometry: late bracket recovered; original clocks, deadline, faults and stop preserved\n";
}
