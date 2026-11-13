#pragma once

#include <cstddef>
#include <cstdint>

namespace sunhorizon_perception_runtime::detail {
// Private runtime diagnostic values: no frame ownership, heap allocation or clocks.
struct BodyPairIdentity {
  int64_t source_ns{-1};
  uint64_t sequence{0};
};
struct BodyPairEvictions {
  uint64_t count{0};
  BodyPairIdentity last;
  void note(BodyPairIdentity value) noexcept {++count;last=value;}
};
struct BodyPairWaitTiming {
  int64_t enqueue_steady_ns{0},process_entry_steady_ns{0};
  int64_t lock_attempt_steady_ns{0},lock_acquired_steady_ns{0};
};
struct BodyPairWitness {
  BodyPairIdentity belly,oldest,newest,chosen_identity;
  int64_t anchor_source_ns{-1},selection_steady_ns{0};
  BodyPairWaitTiming wait;
  std::size_t deque_count{0},too_new_count{0},causal_already_processed_count{0};
  uint64_t last_processed_sequence{0};
  bool chosen{false},chosen_fresh{false};
  const char* selection_reason{"no_prepared_frame"};
  BodyPairEvictions evictions;
};
// Call under the existing queue lock, AFTER the unchanged selector has run.
// This observes its choice; it never supplies or changes a frame for processing.
// Queue capacity is bounded by the chest history policy. No frame ownership or
// per-point data is retained in this diagnostic snapshot.
template<class Queue,class Work,class IdentityOf>
BodyPairWitness freeze_body_pair_witness(const Queue& queue,BodyPairIdentity belly,
    BodyPairWaitTiming wait,int64_t anchor_source_ns,uint64_t last_processed_sequence,
    const Work& chosen,const BodyPairEvictions& evictions,IdentityOf identity_of) {
  BodyPairWitness result;
  result.belly=belly;result.anchor_source_ns=anchor_source_ns;
  result.wait=wait;result.last_processed_sequence=last_processed_sequence;
  result.deque_count=queue.size();result.evictions=evictions;
  if(!queue.empty()) {result.oldest=identity_of(queue.front());result.newest=identity_of(queue.back());}
  for(const auto& work:queue) {
    const auto identity=identity_of(work);
    if(identity.source_ns>anchor_source_ns)++result.too_new_count;
    else if(identity.sequence<=last_processed_sequence)++result.causal_already_processed_count;
  }
  result.chosen=bool(chosen);
  if(chosen) {result.chosen_identity=identity_of(chosen);result.selection_reason="chosen";}
  else if(queue.empty())result.selection_reason="no_prepared_frame";
  else if(result.too_new_count==queue.size())result.selection_reason="too_new";
  else result.selection_reason="already_processed";
  return result;
}
} // namespace sunhorizon_perception_runtime::detail
