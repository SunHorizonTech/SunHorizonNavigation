#include "../src/body_pair_witness.hpp"
#include <deque>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace detail=sunhorizon_perception_runtime::detail;
using Work=std::shared_ptr<const detail::BodyPairIdentity>;
using Queue=std::deque<Work>;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
Work frame(int64_t source,uint64_t sequence){return std::make_shared<const detail::BodyPairIdentity>(detail::BodyPairIdentity{source,sequence});}
// Exact production selector predicate/order, deliberately separate from witness.
Work select(const Queue& queue,int64_t anchor,uint64_t last){
  for(auto it=queue.rbegin();it!=queue.rend();++it)
    if((*it)->source_ns<=anchor && (*it)->sequence>last)return *it;
  return {};
}
detail::BodyPairWitness observe(const Queue& queue,int64_t anchor,uint64_t last,
    const detail::BodyPairEvictions& evictions={}){
  const Work chosen=select(queue,anchor,last);
  return detail::freeze_body_pair_witness(queue,{anchor,99},{100,150,175,200},anchor,last,chosen,evictions,
    [](const Work& work){return *work;});
}
int main(){
  static_assert(std::is_trivially_copyable<detail::BodyPairWitness>::value,"frozen scalar witness only");
  require(std::string(observe({},20,1).selection_reason)=="no_prepared_frame","empty reason");
  require(std::string(observe({frame(21,2),frame(22,3)},20,1).selection_reason)=="too_new","too-new reason");
  const auto mixed=observe({frame(10,1),frame(21,2)},20,1);
  require(std::string(mixed.selection_reason)=="already_processed","causal processed + too new reason");
  require(mixed.too_new_count==1 && mixed.causal_already_processed_count==1,"mixed counts preserve both blockers");
  const auto boundary=observe({frame(19,2),frame(20,3)},20,1);
  require(boundary.chosen && boundary.chosen_identity.sequence==3,"equal anchor accepted; reverse order retained");
  require(boundary.belly.sequence==99 && boundary.belly.source_ns==20 && boundary.wait.enqueue_steady_ns==100,"original belly identity/time");
  require(boundary.oldest.sequence==2 && boundary.newest.sequence==3 && boundary.deque_count==2,"queue ends");
  require(boundary.wait.process_entry_steady_ns==150 && boundary.wait.lock_attempt_steady_ns==175 &&
    boundary.wait.lock_acquired_steady_ns==200,"entry and lock boundaries frozen exactly");
  require(boundary.wait.process_entry_steady_ns-boundary.wait.enqueue_steady_ns==50 &&
    boundary.wait.lock_acquired_steady_ns-boundary.wait.lock_attempt_steady_ns==25,
    "worker backlog and mutex wait are independently recoverable");
  // Model delayed belly processing while two new chest preparations retire a
  // causal candidate. Snapshot names the retired identity, without calling it fresh.
  Queue queue{frame(19,2),frame(20,3)};detail::BodyPairEvictions evictions;
  for(const auto source:{21,22}){
    queue.push_back(frame(source,uint64_t(source-17)));
    evictions.note(*queue.front());queue.pop_front();
  }
  const auto frozen=observe(queue,20,1,evictions);
  require(!frozen.chosen && frozen.too_new_count==2,"two-slot starvation scenario is observable");
  require(frozen.evictions.count==2 && frozen.evictions.last.source_ns==20 && frozen.evictions.last.sequence==3,"latest retired causal identity preserved");
  queue.clear();evictions.note({23,6});
  require(frozen.deque_count==2 && frozen.evictions.count==2 && frozen.newest.source_ns==22,"later mutation cannot rewrite frozen witness");
  std::size_t cases=0;
  for(int64_t first=0;first<4;++first)for(int64_t second=0;second<4;++second)
    for(uint64_t first_seq=0;first_seq<4;++first_seq)for(uint64_t second_seq=0;second_seq<4;++second_seq)
      for(int64_t anchor=0;anchor<4;++anchor)for(uint64_t last=0;last<4;++last){
        Queue values{frame(first,first_seq),frame(second,second_seq)};
        const auto use_first=values[0].use_count(),use_second=values[1].use_count();
        const auto got=observe(values,anchor,last);const auto expected=select(values,anchor,last);
        require(got.chosen==bool(expected),"witness matches unchanged selector");
        require(!expected || (got.chosen_identity.sequence==expected->sequence && got.chosen_identity.source_ns==expected->source_ns),"chosen identity exact");
        require(got.deque_count==2 && values[0]->source_ns==first && values[1]->source_ns==second,"observer cannot mutate queue");
        // expected itself temporarily owns one frame; witness must own neither.
        require(values[0].use_count()==use_first+(expected==values[0]) && values[1].use_count()==use_second+(expected==values[1]),"witness retains no frames");
        const std::size_t too_new=std::size_t(first>anchor)+std::size_t(second>anchor);
        const std::size_t processed=std::size_t(first<=anchor&&first_seq<=last)+std::size_t(second<=anchor&&second_seq<=last);
        require(got.too_new_count==too_new && got.causal_already_processed_count==processed,"exact blocker counts");
        if(!expected)require(std::string(got.selection_reason)==(too_new==2?"too_new":"already_processed"),"reason uses original AND predicate");
        ++cases;
      }
  std::cout<<"body_pair_witness: "<<cases<<" exhaustive predicate cases + boundary/eviction/frozen ownership cases passed; witness bytes="<<sizeof(detail::BodyPairWitness)<<"\n";
}
