#pragma once
#include <cstdint>
#include <optional>

namespace sunhorizon_navigation_runtime { namespace detail {
struct OriginalSourceGeometryLease {
  int64_t source_ns{0}, first_receipt_ros_ns{0}, first_receipt_steady_ns{0};
  int64_t maximum_age_ns{0};
};
struct GeometryWaitClocks {int64_t ros_ns{0}, steady_ns{0};};

inline bool original_geometry_lease_current(const OriginalSourceGeometryLease& lease,
    const GeometryWaitClocks& clocks) {
  return lease.source_ns>0 && lease.maximum_age_ns>0 &&
    lease.first_receipt_ros_ns>=lease.source_ns && lease.first_receipt_steady_ns>0 &&
    clocks.ros_ns>=lease.first_receipt_ros_ns && clocks.steady_ns>=lease.first_receipt_steady_ns &&
    clocks.ros_ns-lease.first_receipt_ros_ns<=lease.maximum_age_ns &&
    clocks.steady_ns-lease.first_receipt_steady_ns<=lease.maximum_age_ns;
}

// Retain the original output owner on its delivery worker, before any adapter
// writer lock. Only an explicitly future-pending source-time query may wait.
// No replacement sample, extrapolation, new receipt or refreshed lease exists.
template<class Clock,class Query,class Pending,class Wait,class Stopped>
auto wait_for_original_source_geometry(const OriginalSourceGeometryLease& lease,
    Clock clock,Query query,Pending pending,Wait wait,Stopped stopped)
    -> std::optional<decltype(query())> {
  auto previous=clock();
  for(;;) {
    const auto before=clock();
    if(stopped() || before.ros_ns<previous.ros_ns || before.steady_ns<previous.steady_ns ||
        !original_geometry_lease_current(lease,before))return std::nullopt;
    auto result=query();
    const auto after=clock();
    if(stopped() || after.ros_ns<before.ros_ns || after.steady_ns<before.steady_ns ||
        !original_geometry_lease_current(lease,after))return std::nullopt;
    if(!pending(result))return result;
    previous=after;
    wait();
  }
}
}}  // namespace sunhorizon_navigation_runtime::detail
