#pragma once

#include <future>
#include <utility>

namespace sunhorizon_perception_runtime {
// A role task must finish (or be cancelled) before the anchor coordinator
// unwinds. In particular an exception in the other role cannot lend the next
// anchor this role's engine or region state while it is still in use.
template<class T> class JoinedFuture {
 public:
  explicit JoinedFuture(std::future<T> future) : future_(std::move(future)) {}
  ~JoinedFuture() { if (future_.valid()) future_.wait(); }
  JoinedFuture(const JoinedFuture&) = delete;
  JoinedFuture& operator=(const JoinedFuture&) = delete;
  bool valid() const { return future_.valid(); }
  T get() { return future_.get(); }
 private:
  std::future<T> future_;
};
}  // namespace sunhorizon_perception_runtime
