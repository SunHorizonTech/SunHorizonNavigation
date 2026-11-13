#pragma once
#include <fast_livo/direct_fault.hpp>

namespace sunhorizon_navigation_runtime { namespace detail {
inline std::string vendor_fault_cause(const fast_livo_direct::FirstFaultInfo& info) {
  return std::string("livo_vendor:")+info.cause();
}
// This is the main failed()->failure_reason() polling path as well as its test
// seam. The getter reads the published value, never waits for the callback.
template<class Owner>
std::string vendor_owner_failure_reason(const Owner& owner) {
  if(!owner.failed())return {};
  const auto info=owner.first_fault_snapshot();
  return info?vendor_fault_cause(*info):"livo_vendor_fault_snapshot_missing";
}
// Both the direct callback and a rejected follower use the frozen owner cause.
// A normal shutdown is not an invented vendor fault. Unexpected closure/null
// input remain distinct failures, never guessed to be queue overflow.
template<class FaultInfo,class Unexpected>
bool handle_vendor_admission(const fast_livo_direct::AdmissionResult& result,
                             bool stopping,FaultInfo fault,Unexpected unexpected) {
  using fast_livo_direct::AdmissionStatus;
  if(result.accepted())return true;
  if(result.fault)fault(*result.fault);
  else if(result.status==AdmissionStatus::Closed) {
    if(!stopping)unexpected("vendor_runtime_closed_without_fault");
  } else if(result.status==AdmissionStatus::NullInput)unexpected("vendor_null_input");
  else unexpected("vendor_admission_rejected_without_fault");
  return false;
}
}} // namespace sunhorizon_navigation_runtime::detail
