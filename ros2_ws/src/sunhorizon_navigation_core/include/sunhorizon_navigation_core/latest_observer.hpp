#pragma once

#include <condition_variable>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

namespace sunhorizon_navigation_core {

// One immutable owner in flight and one replaceable pending owner. Publication
// or recording is exclusively observer work, never acquisition/estimation work.
// External shutdown owns destruction/join; callbacks must not destroy this lane.
template <typename T> class LatestObserver {
 public:
  using Owner = std::shared_ptr<const T>;
  LatestObserver(std::function<void(Owner)> process,
                 std::function<void(std::exception_ptr)> failure)
      : process_(std::move(process)), failure_(std::move(failure)), worker_([this]() {run();}) {}
  ~LatestObserver() {close(); if (worker_.joinable()) worker_.join();}
  LatestObserver(const LatestObserver&) = delete;
  LatestObserver& operator=(const LatestObserver&) = delete;

  bool enqueue(Owner owner) {
    if (!owner) return false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (closed_) return false;
      pending_.swap(owner);
    }
    ready_.notify_one();
    return true;  // Retired owner destruction occurs outside the queue lock.
  }
  void close() {
    Owner retired;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
      retired.swap(pending_);
    }
    ready_.notify_one();
  }

 private:
  void run() noexcept {
    while (true) {
      Owner owner;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait(lock, [this]() {return closed_ || pending_;});
        if (closed_) return;
        owner.swap(pending_);
      }
      try {process_(std::move(owner));}
      catch (...) {
        const auto error = std::current_exception();
        close();
        try {failure_(error);} catch (...) {}
        return;
      }
    }
  }
  std::function<void(Owner)> process_;
  std::function<void(std::exception_ptr)> failure_;
  std::mutex mutex_;
  std::condition_variable ready_;
  Owner pending_;
  bool closed_{false};
  std::thread worker_;
};
}  // namespace sunhorizon_navigation_core
