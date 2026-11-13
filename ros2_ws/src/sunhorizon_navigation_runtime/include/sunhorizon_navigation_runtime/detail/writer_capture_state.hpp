#pragma once
#include <array>
#include <cstdint>
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <type_traits>
namespace sunhorizon_navigation_runtime::detail {
enum class WriterCaptureState : unsigned {Armed,Claimed,Queued,FailedQueue,Written,FailedWrite,SupersededUnwritten,ExpiredUnwritten};
struct WriterCaptureRequest {
  explicit WriterCaptureRequest(std::string value,int64_t now=0):token(std::move(value)),armed_steady_ns(now) {
    for(auto& s:states)s.store(WriterCaptureState::Armed);
  }
  const std::string token;
  const int64_t armed_steady_ns;
  std::array<std::atomic<WriterCaptureState>,4> states;
  bool claim(std::size_t slot)noexcept {
    auto expected=WriterCaptureState::Armed;
    return slot<states.size()&&states[slot].compare_exchange_strong(expected,WriterCaptureState::Claimed);
  }
  static bool valid(const std::string& token)noexcept {
    if(token.empty()||token.size()>64)return false;
    for(char c:token)if(!((c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='_'||c=='-'))return false;
    return true;
  }
};
// Diagnostic callback never waits for this lock. Full/contended is an explicit
// failed request, not a retry against a newer source. Slots are allocated once.
template<class T,std::size_t N=4> class BoundedWriterAuditQueue {
public:
  static_assert(std::is_nothrow_move_constructible<T>::value,"audit queue moves must not throw");
  bool tryPush(T event) noexcept {
    std::unique_lock<std::mutex> guard(mutex_,std::try_to_lock);
    if(!guard.owns_lock())return false;
    for(auto& slot:slots_)if(!slot){slot.emplace(std::move(event));return true;}
    return false;
  }
  std::array<std::optional<T>,N> take() {
    std::array<std::optional<T>,N> out;
    {std::lock_guard<std::mutex> guard(mutex_);out.swap(slots_);}
    return out;
  }
private:
  std::mutex mutex_;std::array<std::optional<T>,N> slots_;
};
}
