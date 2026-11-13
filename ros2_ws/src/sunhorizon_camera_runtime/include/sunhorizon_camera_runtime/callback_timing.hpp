#pragma once

#include <cstdint>

namespace sunhorizon_camera_runtime
{

// Clock reads are supplied by the caller. A failed/unavailable clock stays -1;
// in particular an unavailable thread CPU clock must not look like zero work.
struct CallbackClockPoint
{
  std::int64_t steady_ns{-1};
  std::int64_t thread_cpu_ns{-1};
};

inline std::int64_t diagnostic_clock_delta(std::int64_t begin, std::int64_t end) noexcept
{
  return begin < 0 || end < begin ? -1 : end - begin;
}

struct CallbackTimingPhase
{
  CallbackClockPoint begin;
  CallbackClockPoint end;

  std::int64_t wall_ns() const noexcept
  {
    return diagnostic_clock_delta(begin.steady_ns, end.steady_ns);
  }

  std::int64_t thread_cpu_ns() const noexcept
  {
    return diagnostic_clock_delta(begin.thread_cpu_ns, end.thread_cpu_ns);
  }
};

struct CallbackFrameTiming
{
  bool worker_dispatch{false};
  std::uint64_t ingress_sequence{0};
  CallbackTimingPhase ingress_validate;
  CallbackTimingPhase worker_convert;
  std::int64_t queue_enter_steady_ns{-1};
  std::int64_t worker_start_steady_ns{-1};
  std::int64_t entry_ros_ns{-1};
  std::int64_t entry_system_ns{-1};
  CallbackClockPoint entry;
  CallbackTimingPhase lock_wait;
  CallbackTimingPhase validate_convert;
  CallbackTimingPhase publish;
  std::int64_t publish_complete_system_ns{-1};
  bool completed{false};

  std::int64_t queue_wait_wall_ns() const noexcept
  {
    return diagnostic_clock_delta(queue_enter_steady_ns, worker_start_steady_ns);
  }

  void finish_publication(CallbackClockPoint end, std::int64_t system_ns) noexcept
  {
    publish.end = end;
    publish_complete_system_ns = system_ns;
    completed = true;
  }
};

// Commit this snapshot only in record_accepted, under the same role mutex as
// accepted_frames and that frame's SDK/global/device timestamps. A periodic
// diagnostic samples the last completed frame; it is not an every-frame trace.
struct CompletedCallbackTiming
{
  bool valid{false};
  std::uint64_t accepted_frames{0};
  CallbackFrameTiming frame;

  bool accept(std::uint64_t sequence, const CallbackFrameTiming & candidate) noexcept
  {
    if (!candidate.completed || sequence == 0 || sequence <= accepted_frames) {
      return false;
    }
    frame = candidate;
    accepted_frames = sequence;
    valid = true;
    return true;
  }
};

}  // namespace sunhorizon_camera_runtime
