#include <stdexcept>
#include <string>
#include "sunhorizon_livo_bringup/propagation_causal_audit.hpp"
#include "sunhorizon_livo_bringup/propagation_watchdog.hpp"

using namespace sunhorizon_livo_bringup;
namespace {
void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
PropagationAuditStamp stamp(int64_t source,uint64_t sequence){
  return {source,source+123,source+456,sequence,"/vendor#corrected",{}};
}
}
int main(){
  PropagationCausalAudit audit;
  PropagationWatchdog watchdog(0.5,1.0,0.5);
  const auto corrected=[&](int64_t source,uint64_t sequence){
    const auto input=stamp(source,sequence);
    const auto decision=watchdog.observe_corrected(source);
    if(decision.accepted)audit.accepted_corrected(input);
    else audit.rejected_corrected(input,decision.reason);
    return decision.accepted;
  };
  require(corrected(10'000'000'000LL,1),"initial correction");
  require(corrected(10'100'000'000LL,2),"second correction");
  require(!watchdog.evaluate_propagated(10'110'000'000LL).accepted,"initial propagation epoch");
  require(corrected(10'500'000'000LL,3),"accepted correction");
  require(watchdog.evaluate_propagated(10'610'000'000LL).accepted,"stable epoch");
  audit.propagated_accepted();
  // A newer received correction rejected before the watchdog must not replace
  // the last accepted identity. Original nanosecond receipts remain unchanged.
  auto rejected=stamp(10'800'000'000LL,4);
  rejected.delivery_timing=std::array<int64_t,4>{{91,92,93,94}};
  audit.rejected_corrected(rejected,"body_geometry_future_pending");
  auto propagated=stamp(11'010'000'000LL,10);propagated.identity="/vendor#propagated";
  const auto decision=watchdog.evaluate_propagated(propagated.source_ns);
  require(decision.reason=="propagation_unanchored","real watchdog transition");
  auto event=audit.unanchored(propagated,decision.propagation_horizon_sec,
    watchdog.corrected_stamp_nanoseconds());
  require(event && event->accepted_corrected->sequence==3,"accepted is not last received");
  require(event->accepted_corrected->receipt_ros_ns==10'500'000'123LL,"original receipt preserved");
  require(event->rejected_corrected->sequence==4,"rejected identity retained");
  require((*event->rejected_corrected->delivery_timing)[2]==93,"wall observations preserved");
  require(event->watchdog_anchor_source_ns==10'500'000'000LL,"original horizon basis");
  require(!audit.unanchored(propagated,0.6,watchdog.corrected_stamp_nanoseconds()),
    "one event per unanchored cycle");
  // observe_corrected may install a recovery anchor yet reject it. Only its
  // accepted decision may advance this specifically named accepted identity.
  require(!corrected(11'100'000'000LL,5),"recovery anchor rejected by watchdog");
  require(watchdog.corrected_stamp_nanoseconds()==11'100'000'000LL,
    "rejected recovery input still updates actual watchdog horizon basis");
  require(event->rejected_corrected->sequence==4,"frozen event not overwritten");
  require(event->text().find("propagated_delivery_timing_available=0")!=std::string::npos,
    "missing timing explicit, not invented zeros");
  audit.propagated_accepted();
  const auto next=audit.unanchored(propagated,0.7,watchdog.corrected_stamp_nanoseconds());
  require(next && next->event==2 && next->accepted_corrected->sequence==3,
    "recovery rejection cannot renew last accepted");
  require(next->rejected_corrected->sequence==5,"new event sees latest rejection");
  require(next->watchdog_anchor_source_ns!=next->accepted_corrected->source_ns,
    "actual watchdog anchor is distinct from last accepted input");
  require(next->text().find("watchdog_anchor_source_ns=11100000000")!=std::string::npos,
    "actual horizon basis is serialized without receipt renewal");
  require(event->watchdog_anchor_source_ns==10'500'000'000LL,"frozen horizon basis unchanged");
  PropagationCausalAudit empty;
  const auto unavailable=empty.unanchored(propagated,0.5,0);
  require(!unavailable->accepted_corrected && !unavailable->rejected_corrected,"missing identity explicit");
  return 0;
}
