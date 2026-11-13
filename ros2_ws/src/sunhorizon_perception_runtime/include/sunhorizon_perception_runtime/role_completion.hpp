#pragma once

#include <stdexcept>

namespace sunhorizon_perception_runtime::detail {
// Notify only after the health core accepted the original completion. The
// notification carries no snapshot: the existing egress owner re-samples ages.
template<class Complete,class Notify>
void complete_health_and_notify(Complete complete,Notify notify) {
  if(complete())notify();
}

// One role invokes this exactly once after freezing its immutable result.
// No coverage decision belongs here. Handoffs are bounded ownership transfers;
// failure is latched before a packaged_task can defer the exception to its join.
template<class Active,class Handoff,class Credit,class Observe,class Failure>
void complete_role(bool eligible,Active active,Handoff handoff,Credit credit,
    Observe observe,Failure failure) {
  if(!eligible || !active())return;
  try {
    handoff();
    if(!active())return;
    credit();
    if(active())observe();
  }catch(const std::exception& error){
    failure(error.what());
    // A sink throwing a TF exception is fatal, unlike a missing decode TF.
    throw std::runtime_error(error.what());
  }catch(...){
    failure("unknown_worker_failure");
    throw std::runtime_error("unknown_worker_failure");
  }
}
}  // namespace sunhorizon_perception_runtime::detail
