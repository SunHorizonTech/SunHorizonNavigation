#include <memory>
#include <stdexcept>

#include "sunhorizon_3d_localizer/detail/ndt_pending_clouds.hpp"

namespace {
void require(bool value, const char* reason) {
  if (!value) throw std::runtime_error(reason);
}
// Immutable stand-in for the owner's shared input and original delivery clocks.
struct Input {
  int64_t source_ns;
  uint64_t sequence;
  int64_t receipt_ros_ns, receipt_steady_ns;
};
using Cloud = std::shared_ptr<const Input>;
using Queue = sunhorizon_3d_localizer::detail::NdtPendingClouds<Cloud>;
Cloud input(int64_t source) {
  return std::make_shared<const Input>(Input{source, static_cast<uint64_t>(source),
      source + 123, source + 456});
}
bool push(Queue& queue, const Cloud& cloud, std::size_t points,
    std::size_t minimum = 85000) {
  return queue.push(cloud, cloud->source_ns, points, minimum);
}
void expect_next(Queue& queue, const Cloud& original) {
  const auto next = queue.take();
  require(next && *next == original, "must dequeue the original immutable input");
  require((*next)->receipt_ros_ns == original->source_ns + 123 &&
      (*next)->receipt_steady_ns == original->source_ns + 456 &&
      (*next)->sequence == static_cast<uint64_t>(original->source_ns),
      "must preserve sequence and original receipt clocks");
}
}

int main() {
  {
    Queue queue;
    const auto complete = input(1856), incomplete = input(1857);
    require(!push(queue, complete, 86304), "initial complete input");
    require(!push(queue, incomplete, 82656), "incomplete must not evict complete");
    require(queue.counters().complete_preserved == 1 &&
        queue.counters().complete_overwritten == 0, "preservation must be counted");
    expect_next(queue, complete);
    expect_next(queue, incomplete);
    require(queue.empty() && !queue.take(), "both bounded slots drained");
  }
  {
    Queue queue;
    const auto old_incomplete = input(1), complete = input(2), incomplete = input(3);
    push(queue, old_incomplete, 82656);
    push(queue, complete, 85000);
    require(push(queue, incomplete, 84999), "only old incomplete is coalesced");
    expect_next(queue, complete);
    expect_next(queue, incomplete);
    require(queue.counters().incomplete_submitted == 2 &&
        queue.counters().incomplete_overwritten == 1 &&
        queue.counters().complete_submitted == 1, "exact incomplete counts");
  }
  {
    Queue queue;
    const auto old_complete = input(1), incomplete = input(2), complete = input(3);
    push(queue, old_complete, 86304);
    push(queue, incomplete, 82656);
    require(push(queue, complete, 85000), "latest complete replaces old complete");
    // Running incomplete first preserves on_points' source watermark ordering.
    expect_next(queue, incomplete);
    expect_next(queue, complete);
    require(queue.counters().complete_overwritten == 1 &&
        queue.counters().incomplete_overwritten == 0, "exact complete overwrites");
  }
  {
    Queue queue;
    const auto first = input(1), latest = input(2);
    push(queue, first, 0, 0);
    require(push(queue, latest, 1, 0), "disabled minimum keeps latest-only policy");
    expect_next(queue, latest);
    require(queue.empty() && queue.counters().incomplete_submitted == 0,
        "zero threshold must not invent incomplete input");
  }
  {
    Queue queue;
    std::weak_ptr<const Input> evicted;
    {
      const auto old = input(1);
      evicted = old;
      push(queue, old, 85000);
    }
    for (int64_t i = 2; i <= 1001; ++i) push(queue, input(i), i % 2 ? 85000 : 84999);
    require(evicted.expired(), "queue must release overwritten input");
    require(queue.counters().complete_submitted == 501 &&
        queue.counters().incomplete_submitted == 500 &&
        queue.counters().complete_overwritten == 500 &&
        queue.counters().incomplete_overwritten == 499 &&
        queue.counters().complete_preserved == 500, "burst counters must be exact");
    const auto first = queue.take(), second = queue.take();
    require(first && second && (*first)->source_ns == 1000 &&
        (*second)->source_ns == 1001 && queue.empty(), "burst retains only two latest inputs");
    push(queue, input(1002), 85000);
    push(queue, input(1003), 84999);
    queue.clear();
    require(queue.empty() && !queue.take() &&
        queue.counters().complete_submitted == 502 &&
        queue.counters().incomplete_submitted == 501,
        "stop/input failure clears both slots without erasing audit history");
  }
}
