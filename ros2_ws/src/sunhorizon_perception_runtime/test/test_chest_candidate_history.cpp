#include "../src/chest_candidate_history.hpp"
#include "../src/body_pair_witness.hpp"
#include <deque>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

namespace detail=sunhorizon_perception_runtime::detail;
using Work=std::shared_ptr<const detail::BodyPairIdentity>;
using Queue=std::deque<Work>;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
Work frame(int64_t source,uint64_t sequence){return std::make_shared<const detail::BodyPairIdentity>(detail::BodyPairIdentity{source,sequence});}
auto identity=[](const Work& value){return *value;};
void append(Queue& queue,const Work& work,const detail::ChestCandidateHistoryPolicy& policy,int64_t now,
    detail::BodyPairEvictions& evictions){
  detail::ChestCandidateRetirements<Work> retired;
  detail::retain_chest_candidate(queue,work,policy,now,retired,identity,
    [&](detail::BodyPairIdentity value){evictions.note(value);});
}
Work select(const Queue& queue,int64_t anchor,uint64_t last){
  for(auto it=queue.rbegin();it!=queue.rend();++it)
    if((*it)->source_ns<=anchor && (*it)->sequence>last)return *it;
  return {};
}
int main(){
  Queue history{frame(100,37313),frame(233,37317),frame(320,37319)};
  require(select(history,detail::chest_selection_limit(false,140,300),37311)->sequence==37317,
    "BODY-only may use a newer original chest frame, never a future one");
  require(select(history,detail::chest_selection_limit(true,140,300),37311)->sequence==37313,
    "fusion keeps its belly anchor");
  require(!select(history,detail::chest_selection_limit(true,140,90),0),
    "clock rollback cannot admit a future sample");
  constexpr int64_t base=1000000000LL,age=500000000LL;
  const detail::ChestCandidateHistoryPolicy policy(age,15.,.005);
  require(policy.capacity==9 && policy.maximum_source_span_ns==age,"capacity follows existing age/rate/pair contract");
  require(detail::ChestCandidateHistoryPolicy(300000000,15.,.005).capacity==6,"strict age profile follows same derivation");
  require(detail::ChestCandidateHistoryPolicy(age,1.e300,1.e300).capacity==64,"large existing legal config remains bounded without new rejection");
  Queue queue;detail::BodyPairEvictions evictions;
  // The old two slots would both be newer than anchor; history retains the
  // newest causal, unprocessed identity without changing the original selector.
  for(uint64_t i=0;i<8;++i)append(queue,frame(base+int64_t(i)*66666667,i+1),policy,base+int64_t(i)*66666667+10000000,evictions);
  auto chosen=select(queue,base+200000001,1);
  require(chosen && chosen->sequence==4,"recover newest causal frame past original two slots");
  require(!select(queue,base+200000001,4),"already-processed restriction unchanged");
  chosen.reset();
  Queue boundary;detail::BodyPairEvictions boundary_evictions;
  append(boundary,frame(base,1),policy,base,boundary_evictions);
  append(boundary,frame(base+age,2),policy,base+age,boundary_evictions);
  require(boundary.size()==2 && boundary.front()->sequence==1,"inclusive source-age boundary retained");
  append(boundary,frame(base+age+1,3),policy,base+age+1,boundary_evictions);
  require(boundary.front()->sequence==2 && boundary_evictions.count==1,"only strictly expired source is pruned");
  // A future source following a ROS rollback must not advance the history
  // watermark or evict a currently legal candidate. No new health credit exists.
  Queue rollback{frame(base,1)};detail::BodyPairEvictions rollback_evictions;
  append(rollback,frame(base+age+1,2),policy,base+100000000,rollback_evictions);
  require(rollback.size()==1 && rollback.front()->sequence==1,"future incoming preserves older legal owner");
  require(rollback_evictions.last.sequence==2,"diagnostic identifies the actually retired incoming frame");
  // Dense/adversarial arrivals are capped at the existing health token bound.
  Queue dense;detail::BodyPairEvictions dense_evictions;
  const detail::ChestCandidateHistoryPolicy capped(age,1000.,.005);
  for(uint64_t i=0;i<1000;++i)append(dense,frame(base+int64_t(i),i+1),capped,base+int64_t(i),dense_evictions);
  require(dense.size()==64 && dense.front()->sequence==937 && dense_evictions.count==936,"hard owner bound");
  // Retire many large-frame owners in one admission. Their deleters must run
  // after the caller releases its lock; moving shared_ptrs must not destroy them.
  bool locked=false,destroyed_under_lock=false;std::size_t destroyed=0;
  Queue owned;detail::BodyPairEvictions owned_evictions;
  for(uint64_t i=0;i<64;++i){
    Work work(new detail::BodyPairIdentity{base+int64_t(i),i+1},[&](const auto* ptr){destroyed_under_lock|=locked;++destroyed;delete ptr;});
    append(owned,work,capped,base+int64_t(i),owned_evictions);
  }
  {
    detail::ChestCandidateRetirements<Work> retired;const auto incoming=frame(base+age+100,65);
    locked=true;
    detail::retain_chest_candidate(owned,incoming,capped,base+age+100,retired,identity,
      [&](detail::BodyPairIdentity value){owned_evictions.note(value);});
    require(retired.count==64 && destroyed==0 && owned.size()==1,"batch owner transfer does not destroy under lock");
    locked=false;
  }
  require(!destroyed_under_lock && destroyed==64,"batch destructors execute outside lock");
  // Exercise the rate/pair-derived bound at both +/-pair extrema in source time.
  for(int direction:{-1,1}){
    Queue jitter;detail::BodyPairEvictions jitter_evictions;
    for(uint64_t i=0;i<100;++i){
      const auto source=base+int64_t(i)*66666667+(i%2?direction*5000000:-direction*5000000);
      append(jitter,frame(source,i+1),policy,source+20000000,jitter_evictions);
      require(jitter.size()<=policy.capacity,"accepted color cadence and pair extrema fit derived capacity");
      require(jitter.back()->source_ns-jitter.front()->source_ns<=age,"retained source window bounded");
    }
  }
  Queue extreme;detail::BodyPairEvictions extreme_evictions;
  const auto high=std::numeric_limits<int64_t>::max()-1;
  append(extreme,frame(high-age,1),policy,high-age,extreme_evictions);
  append(extreme,frame(high,2),policy,high,extreme_evictions);
  require(extreme.size()==2,"positive source subtraction preserves int64 boundary");
  std::cout<<"chest_candidate_history: policy, causal selection, inclusive expiry, future-clock preservation, hard cap, out-of-lock batch destruction and pair extrema passed\n";
}
