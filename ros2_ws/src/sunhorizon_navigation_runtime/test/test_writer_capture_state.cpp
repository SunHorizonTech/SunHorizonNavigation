#include "sunhorizon_navigation_runtime/detail/writer_capture_state.hpp"
#include <stdexcept>
#define CHECK(v) do {if(!(v))throw std::runtime_error("writer capture check failed: " #v);}while(false)
#include <iostream>
#include <thread>
#include <vector>
namespace d=sunhorizon_navigation_runtime::detail;
int main(){
 CHECK(!d::WriterCaptureRequest::valid("../escape"));CHECK(!d::WriterCaptureRequest::valid(""));
 CHECK(d::WriterCaptureRequest::valid("actual-writer-01"));
 auto request=std::make_shared<d::WriterCaptureRequest>("request");
 // Merely offering/accepting projected frames does not claim a writer request.
 for(unsigned i=0;i<100;++i)CHECK(request->states[0].load()==d::WriterCaptureState::Armed);
 std::atomic<unsigned> claimed{0};std::vector<std::thread> threads;
 for(int i=0;i<16;++i)threads.emplace_back([&]{if(request->claim(0))++claimed;});
 for(auto& thread:threads)thread.join();CHECK(claimed==1);CHECK(!request->claim(0));
 struct Event {std::shared_ptr<const int> source;std::shared_ptr<d::WriterCaptureRequest> request;};
 d::BoundedWriterAuditQueue<Event,1> queue;auto source=std::make_shared<const int>(42);std::weak_ptr<const int> weak=source;
 CHECK(queue.tryPush({source,request}));CHECK(!queue.tryPush({source,request}));source.reset();CHECK(!weak.expired());
 auto events=queue.take();CHECK(*events[0]->source==42);CHECK(queue.tryPush({events[0]->source,request}));
 // Worker can complete immediately after enqueue, before the producer's ack.
 request->states[0].store(d::WriterCaptureState::Written);
 auto expected=d::WriterCaptureState::Claimed;CHECK(!request->states[0].compare_exchange_strong(expected,d::WriterCaptureState::Queued));
 CHECK(request->states[0].load()==d::WriterCaptureState::Written);
 auto failed=std::make_shared<d::WriterCaptureRequest>("failed");CHECK(failed->claim(1));
 CHECK(!queue.tryPush({events[0]->source,failed}));failed->states[1].store(d::WriterCaptureState::FailedQueue);CHECK(!failed->claim(1));
 // Superseded/expired never silently retry against a later source.
 for(auto terminal:{d::WriterCaptureState::SupersededUnwritten,d::WriterCaptureState::ExpiredUnwritten}) {
  auto r=std::make_shared<d::WriterCaptureRequest>("old");auto from=d::WriterCaptureState::Armed;
  CHECK(r->states[2].compare_exchange_strong(from,terminal));CHECK(!r->claim(2));
 }
 events={};auto final=queue.take();CHECK(!weak.expired());final={};CHECK(weak.expired());
 std::cout<<"PASS actual-writer CAS, bounded-full failure, terminal race and owner lifetime\n";
}
