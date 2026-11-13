#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <vector>
#include "sunhorizon_camera_runtime/latest_frame_worker.hpp"
#include "sunhorizon_camera_runtime/runtime_contract.hpp"
#include "sunhorizon_camera_runtime/callback_timing.hpp"
using namespace sunhorizon_camera_runtime;
namespace {
void require(bool value,const char* reason) {if (!value) throw std::runtime_error(reason);}
struct Frame {
  int sequence;uint64_t color_global_us,depth_global_us;
  CallbackFrameTiming timing;std::function<void()> release;
  ~Frame() {if (release) release();}
};
struct Block {
  std::mutex mutex;std::condition_variable cv;bool entered{false},released{false};
  void enter() {std::unique_lock<std::mutex> lock(mutex);entered=true;cv.notify_all();
    if (!cv.wait_for(lock,std::chrono::seconds(2),[this]{return released;})) throw std::runtime_error("fake publisher block timed out");}
  void wait() {std::unique_lock<std::mutex> lock(mutex);
    require(cv.wait_for(lock,std::chrono::seconds(2),[this]{return entered;}),"worker never entered publisher");}
  void release() {std::lock_guard<std::mutex> lock(mutex);released=true;cv.notify_all();}
};
std::unique_ptr<Frame> frame(int seq) {
  auto f=std::make_unique<Frame>();f->sequence=seq;f->color_global_us=1000+seq;
  f->depth_global_us=1001+seq;f->timing.entry_system_ns=10000+seq;
  f->timing.entry={20000+seq,30000+seq};return f;
}
void latest_and_owned_destruction() {
  Block block;std::mutex gate,records_mutex;std::condition_variable done;
  std::vector<int> seen;std::vector<uint64_t> stamps;std::vector<int64_t> original_entries;
  bool released_unlocked=false;std::atomic<bool> fatal{false};
  LatestFrameWorker<Frame> worker([&](Frame& f){
    if (f.sequence==1) block.enter();
    std::lock_guard<std::mutex> lock(records_mutex);seen.push_back(f.sequence);
    stamps.push_back(f.color_global_us);stamps.push_back(f.depth_global_us);
    original_entries.push_back(f.timing.entry_system_ns);done.notify_all();
  },[&]{return !fatal.load();},[&](std::exception_ptr){fatal=true;});
  StrictTimestampGate timestamps;
  auto ingress=[&](std::unique_ptr<Frame> value) {
    std::unique_ptr<Frame> retired;
    {std::lock_guard<std::mutex> lock(gate);
      require(timestamps.observe(value->depth_global_us)==TimestampDecision::kAccepted,"ingress clock rejected");
      retired=worker.replace(std::move(value));}
    retired.reset();
  };
  ingress(frame(1));block.wait();
  auto second=frame(2);second->release=[&]{
    released_unlocked=gate.try_lock();if (released_unlocked) gate.unlock();
    (void)worker.stats(); // Must not be called under queue lock during destruction.
  };
  ingress(std::move(second));ingress(frame(3));
  const auto busy=worker.stats();
  require(busy.in_flight&&busy.pending&&busy.admitted==3&&busy.replaced==1,"queue capacity/latest replacement changed");
  require(released_unlocked,"replaced SDK frame destroyed under admission lock");
  {std::lock_guard<std::mutex> lock(records_mutex);require(seen.empty(),"publication claimed complete while blocked");}
  block.release();
  {std::unique_lock<std::mutex> lock(records_mutex);require(done.wait_for(lock,std::chrono::seconds(2),[&]{return seen.size()==2;}),"latest frame did not complete");}
  auto retired=worker.request_stop();retired.reset();worker.join();
  require(seen==std::vector<int>({1,3}),"middle pending frame was published");
  require(stamps==std::vector<uint64_t>({1001,1002,1003,1004}),"chest pair/source times split or refreshed");
  require(original_entries==std::vector<int64_t>({10001,10003}),"worker refreshed SDK ingress time");
  require(!fatal.load()&&worker.stats().completed==2,"successful worker failed");
}
void regression_while_blocked_still_revokes_pending() {
  Block block;std::atomic<bool> fatal{false};std::vector<int> seen;std::mutex records;
  LatestFrameWorker<Frame> worker([&](Frame& f){if (f.sequence==1) block.enter();
    std::lock_guard<std::mutex> lock(records);seen.push_back(f.sequence);
  },[&]{return !fatal.load();},[&](std::exception_ptr){fatal=true;});
  StrictTimestampGate timestamps;
  require(timestamps.observe(100)==TimestampDecision::kAccepted,"first timestamp");
  worker.replace(frame(1));block.wait();
  require(timestamps.observe(200)==TimestampDecision::kAccepted,"pending timestamp");worker.replace(frame(2));
  const auto bad=timestamps.observe(150);require(bad==TimestampDecision::kRegression,"overwritten/blocked input hid clock regression");
  fatal=true;block.release();
  const auto limit=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while (worker.stats().skipped==0 && std::chrono::steady_clock::now()<limit) std::this_thread::yield();
  require(worker.stats().skipped==1&&worker.stats().completed==1,"fatal pending was not skipped distinctly");
  auto retired=worker.request_stop();retired.reset();worker.join();
  require(seen==std::vector<int>({1}),"pending processing started after fatal revocation");
}
void worker_failure_closes_and_discards_pending() {
  Block block;std::atomic<bool> fatal{false};std::atomic<int> calls{0};
  LatestFrameWorker<Frame> worker([&](Frame&){++calls;block.enter();throw std::runtime_error("fake conversion/publication failed");},
    [&]{return !fatal.load();},[&](std::exception_ptr error){require(bool(error),"failure lost exception");fatal=true;});
  worker.replace(frame(1));block.wait();worker.replace(frame(2));block.release();worker.join();
  const auto state=worker.stats();require(fatal&&state.closed&&state.errors==1&&!state.pending&&calls==1,"worker failure continued pending");
  auto rejected=worker.replace(frame(3));require(rejected&&rejected->sequence==3,"closed worker took new input");
}
void stop_discards_pending_then_joins_in_flight() {
  Block block;std::vector<int> seen;
  LatestFrameWorker<Frame> worker([&](Frame& f){block.enter();seen.push_back(f.sequence);},[]{return true;},[](std::exception_ptr){});
  worker.replace(frame(1));block.wait();worker.replace(frame(2));
  auto retired=worker.request_stop();require(retired&&retired->sequence==2,"stop did not release pending owner");retired.reset();
  require(worker.stats().in_flight&&worker.stats().closed,"stop pretended in-flight publish was joined");
  block.release();worker.join();require(seen==std::vector<int>({1}),"stop admitted pending frame");
}
void partial_start_requires_stop_attempt() {
  CaptureStartAttempt state;int stop_calls=0;bool callback_installed=false;
  require(!state.take_stop(),"unattempted SDK start stopped");
  try {state.mark();callback_installed=true;throw std::runtime_error("SDK start failed after installing callback");}
  catch (const std::runtime_error&) {if (state.take_stop()) {++stop_calls;callback_installed=false;}}
  require(stop_calls==1&&!callback_installed&&!state.take_stop(),"partial SDK start left callback ownership alive");
}
void modes_and_watchdogs_are_separate() {
  require(valid_capture_dispatch_mode("inline-v1",true,true),"default inline shadow compatibility lost");
  require(valid_capture_dispatch_mode("latest-worker-v1",false,false),"fixed worker mode refused");
  require(!valid_capture_dispatch_mode("latest-worker-v1",true,false)&&!valid_capture_dispatch_mode("latest-worker-v1",false,true)&&
    !valid_capture_dispatch_mode("arbitrary",false,false),"unknown/mixed worker mode accepted");
  StreamFreshnessObservation o;o.started=true;o.frame_arrival_seen=true;o.publication_seen=true;
  o.callbacks_in_flight=0;o.last_frame_arrival_age_sec=1.1;o.last_publication_age_sec=0.1;
  require(evaluate_stream_freshness(o,3.,1.,2.)==StreamFreshnessDecision::kFrameArrivalStale,"worker busy renewed SDK capture");
  o.last_frame_arrival_age_sec=0.1;o.last_publication_age_sec=2.1;
  require(evaluate_stream_freshness(o,3.,1.,2.)==StreamFreshnessDecision::kPublicationStale,"new arrival hid blocked publication");
}
}
int main() {
  latest_and_owned_destruction();regression_while_blocked_still_revokes_pending();worker_failure_closes_and_discards_pending();
  stop_discards_pending_then_joins_in_flight();partial_start_requires_stop_attempt();modes_and_watchdogs_are_separate();
  std::cout<<"latest worker: 6 deterministic scenarios passed (throwing checks active under NDEBUG)\n";
}
