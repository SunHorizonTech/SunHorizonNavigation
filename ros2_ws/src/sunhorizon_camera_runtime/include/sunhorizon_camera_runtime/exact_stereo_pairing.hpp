#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <utility>

namespace sunhorizon_camera_runtime {
// Bounded optional-analysis mailbox. Never waits for a missing stream, never
// pairs different device stamps, and never releases older completed work later.
template<typename Frame>
class ExactStereoPairing {
public:
  using Triple = std::array<Frame, 3>;
  Triple insert(std::size_t channel, std::uint64_t stamp, Frame frame)
  {
    if (channel >= 3 || !stamp || stamp <= completed_ || !frame) return {};
    auto & triple = pending_[stamp];
    // A duplicate must not replace a frame already associated with this stamp.
    if (!triple[channel]) triple[channel] = std::move(frame);
    Triple ready{};
    if (triple[0] && triple[1] && triple[2]) {
      ready = std::move(triple);
      completed_ = stamp;
      pending_.erase(pending_.begin(), pending_.upper_bound(stamp));
    }
    while (pending_.size() > 4) { pending_.erase(pending_.begin()); ++evicted_; }
    return ready;
  }
  std::size_t pending() const { return pending_.size(); }
  std::size_t evicted() const { return evicted_; }
private:
  std::map<std::uint64_t, Triple> pending_;
  std::uint64_t completed_{0};
  std::size_t evicted_{0};
};
}  // namespace sunhorizon_camera_runtime
