#pragma once

#include <condition_variable>
#include <atomic>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <string_view>
#include <thread>

namespace sunhorizon_camera_runtime {
// A throwing SDK start can still have installed callbacks. Ownership of a
// stop attempt begins before start(), not only after its successful return.
class CaptureStartAttempt {
 public:
  void mark() noexcept {attempted_.store(true);}
  bool take_stop() noexcept {return attempted_.exchange(false);}
 private:
  std::atomic<bool> attempted_{false};
};
inline bool valid_capture_dispatch_mode(std::string_view mode,bool shadow,bool ir) {
  return mode=="inline-v1" || (mode=="latest-worker-v1" && !shadow && !ir);
}
// One in-flight item plus one replaceable pending item. The caller validates
// every arrival before admission; a replacement never authorizes that arrival.
template<class T> class LatestFrameWorker {
 public:
  struct Stats {uint64_t admitted{0},replaced{0},taken{0},completed{0},skipped{0},errors{0};bool pending{false},in_flight{false},closed{false};};
  LatestFrameWorker(std::function<void(T&)> process,std::function<bool()> allowed,
      std::function<void(std::exception_ptr)> failure)
      :process_(std::move(process)),allowed_(std::move(allowed)),failure_(std::move(failure)),thread_([this]{run();}) {}
  ~LatestFrameWorker() {
    auto retired=request_stop(); // Destroy outside the queue mutex.
    retired.reset();join();
  }
  LatestFrameWorker(const LatestFrameWorker&)=delete;
  LatestFrameWorker& operator=(const LatestFrameWorker&)=delete;
  // Return the replaced/rejected owner to the caller. In particular it must
  // also be destroyed outside any caller-side ingress/gate lock.
  std::unique_ptr<T> replace(std::unique_ptr<T> value) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stats_.closed) return value;
      ++stats_.admitted;if (pending_) ++stats_.replaced;
      pending_.swap(value);stats_.pending=bool(pending_);
    }
    ready_.notify_one();return value;
  }
  std::unique_ptr<T> request_stop() {
    std::unique_ptr<T> retired;
    {std::lock_guard<std::mutex> lock(mutex_);stats_.closed=true;pending_.swap(retired);stats_.pending=false;}
    ready_.notify_all();return retired;
  }
  void join() {if (thread_.joinable()) thread_.join();} // Owned external shutdown only; no hard timeout claim.
  Stats stats() const {std::lock_guard<std::mutex> lock(mutex_);return stats_;}
 private:
  void run() noexcept {
    for (;;) {
      std::unique_ptr<T> value;
      {
        std::unique_lock<std::mutex> lock(mutex_);ready_.wait(lock,[this]{return stats_.closed || bool(pending_);});
        if (stats_.closed) return;
        value.swap(pending_);stats_.pending=false;stats_.in_flight=true;++stats_.taken;
      }
      std::exception_ptr error;
      bool processed=false;
      try {if (allowed_()) {processed=true;process_(*value);}} catch (...) {error=std::current_exception();}
      std::unique_ptr<T> retired;
      {
        std::lock_guard<std::mutex> lock(mutex_);stats_.in_flight=false;
        if (error) {++stats_.errors;stats_.closed=true;pending_.swap(retired);stats_.pending=false;}
        else if (processed) ++stats_.completed;
        else ++stats_.skipped;
      }
      // No queue lock across processing, failure handling or frame destruction.
      if (error) {try {failure_(error);} catch (...) {} return;}
    }
  }
  std::function<void(T&)> process_;std::function<bool()> allowed_;
  std::function<void(std::exception_ptr)> failure_;
  mutable std::mutex mutex_;std::condition_variable ready_;std::unique_ptr<T> pending_;
  Stats stats_;std::thread thread_;
};
} // namespace sunhorizon_camera_runtime
