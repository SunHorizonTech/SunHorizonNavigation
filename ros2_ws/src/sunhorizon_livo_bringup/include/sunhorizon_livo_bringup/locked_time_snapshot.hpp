#pragma once

#include <mutex>
#include <utility>

namespace sunhorizon_livo_bringup
{

// Receipt times and the clock used to judge them must belong to one locked
// snapshot. Sampling before waiting for the lock can make fresh input appear
// to have arrived in the future. Do not return references to protected state.
template<typename Mutex, typename Clock, typename Reader>
decltype(auto) with_locked_time(Mutex & mutex, Clock && clock, Reader && reader)
{
  std::lock_guard<Mutex> lock(mutex);
  return std::forward<Reader>(reader)(std::forward<Clock>(clock)());
}

}  // namespace sunhorizon_livo_bringup
