#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>

namespace sunhorizon_perception_runtime::detail {
// Only a spatial contributor must precede the belly query. BODY-only geometry
// is evaluated at the selected chest frame's own acquisition time.
inline int64_t chest_selection_limit(bool spatial_contributor,int64_t belly_source_ns,int64_t now_ros_ns) {
  return spatial_contributor ? std::min(belly_source_ns,now_ros_ns) : now_ros_ns;
}

// Matches RgbdHealthCore's existing admitted-token bound: older tokens cannot
// receive BODY completion credit even if an owner retains their image buffers.
constexpr std::size_t kMaximumChestHistoryEntries=64;
struct ChestCandidateHistoryPolicy {
  int64_t maximum_source_span_ns;
  std::size_t capacity;
  ChestCandidateHistoryPolicy(int64_t maximum_age_ns,double admitted_rate_hz,double pair_delta_sec)
      :maximum_source_span_ns(maximum_age_ns),capacity(2) {
    if(maximum_age_ns<=0 || !std::isfinite(admitted_rate_hz) || admitted_rate_hz<=0. ||
        !std::isfinite(pair_delta_sec) || pair_delta_sec<=0.)
      throw std::invalid_argument("invalid_chest_history_health_contract");
    // Chest admission spaces COLOR sources by 1/target_rate. Original accepted
    // DEPTH sources can differ from color by +/-max_pair_delta, at both ends of
    // the age window. Ceil + one includes endpoints without rounding a slot away.
    const long double slots=std::ceil((static_cast<long double>(maximum_age_ns)/1.e9L+
      2.L*static_cast<long double>(pair_delta_sec))*admitted_rate_hz)+1.L;
    capacity=slots>=kMaximumChestHistoryEntries?kMaximumChestHistoryEntries:
      std::max<std::size_t>(2,static_cast<std::size_t>(slots));
  }
};
template<class Work> struct ChestCandidateRetirements {
  // Moving owners here cannot destroy large image/mask buffers under state_mutex.
  std::array<Work,kMaximumChestHistoryEntries> owners;
  std::size_t count{0};
};
// Sole chest preparation owner calls under existing state_mutex. Queue identities
// are strictly source/sequence ordered by the unchanged RGBD admission contract.
// This changes storage only; caller retains its original selector/freshness gate.
template<class Queue,class Work,class IdentityOf,class Evicted>
void retain_chest_candidate(Queue& queue,const Work& incoming,const ChestCandidateHistoryPolicy& policy,
    int64_t now_ros_ns,ChestCandidateRetirements<Work>& retired,IdentityOf identity_of,Evicted evicted) {
  const auto newest=identity_of(incoming);
  if(newest.source_ns>now_ros_ns) {
    // An observed ROS rollback must not let a now-future source advance the
    // history watermark or displace a legal older candidate. Original runtime
    // health/freshness/clock owners retain all fault and completion authority.
    retired.owners[retired.count++]=incoming;evicted(newest);return;
  }
  queue.push_back(incoming);
  while(queue.size()>policy.capacity || (!queue.empty() &&
      newest.source_ns-identity_of(queue.front()).source_ns>policy.maximum_source_span_ns)) {
    evicted(identity_of(queue.front()));
    retired.owners[retired.count++]=std::move(queue.front());queue.pop_front();
  }
}
} // namespace sunhorizon_perception_runtime::detail
