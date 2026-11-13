#pragma once
#include <atomic>
#include <cstdint>
namespace sunhorizon_camera_runtime {
class ObserverStride {
 public:
  static bool valid(int64_t value)noexcept{return value>=1&&value<=60;}
  bool set(int64_t value)noexcept {if(!valid(value))return false;value_.store(int(value),std::memory_order_release);return true;}
  int get()const noexcept{return value_.load(std::memory_order_acquire);}
 private:std::atomic<int> value_{6};
};
}
