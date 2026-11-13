#pragma once
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <exception>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

namespace sunhorizon_navigation_runtime { namespace detail {
// FIFO, one compute owner. Overload is visible to the caller; unlike an
// observer mailbox it never silently replaces IMU/correction continuity.
template<class T> class BoundedLane {
 public:
  BoundedLane(std::size_t capacity,std::function<void(T)> process,
    std::function<void(std::exception_ptr)> failure)
    :capacity_(capacity),process_(std::move(process)),failure_(std::move(failure)) {
    if(!capacity_||!process_||!failure_)throw std::invalid_argument("invalid bounded lane");
    worker_=std::thread([this]{run();});
  }
  ~BoundedLane(){request_stop();join();}
  bool admit(T item){
    {std::lock_guard<std::mutex> lock(mutex_);
      if(closed_||pending_.size()==capacity_)return false;
      pending_.push_back(std::move(item));}
    ready_.notify_one();return true;
  }
  std::size_t pending_size(){std::lock_guard<std::mutex> lock(mutex_);return pending_.size();}
  void request_stop(){
    std::deque<T> retired;
    {std::lock_guard<std::mutex> lock(mutex_);closed_=true;retired.swap(pending_);}
    ready_.notify_all();
  }
  void join(){if(worker_.joinable())worker_.join();}
 private:
  void run()noexcept{
    try{for(;;){
      T item;
      {std::unique_lock<std::mutex> lock(mutex_);ready_.wait(lock,[this]{return closed_||!pending_.empty();});
        if(closed_)return;
        item=std::move(pending_.front());pending_.pop_front();}
      process_(std::move(item));
    }}catch(...){const auto error=std::current_exception();request_stop();try{failure_(error);}catch(...){}}
  }
  const std::size_t capacity_;std::function<void(T)> process_;
  std::function<void(std::exception_ptr)> failure_;
  std::mutex mutex_;std::condition_variable ready_;std::deque<T> pending_;
  bool closed_{false};std::thread worker_;
};
}}  // namespace sunhorizon_navigation_runtime::detail
