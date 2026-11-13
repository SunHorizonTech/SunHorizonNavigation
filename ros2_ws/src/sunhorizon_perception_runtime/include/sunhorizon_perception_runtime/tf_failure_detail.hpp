#pragma once

#include <ctime>
#include <sstream>
#include <string>
#include <tf2/exceptions.hpp>

#include "sunhorizon_perception_runtime/thread_scheduling.hpp"

namespace sunhorizon_perception_runtime::detail {

inline bool transient_tf_unavailable(const tf2::TransformException& error) {
  return dynamic_cast<const tf2::TimeoutException*>(&error) ||
         dynamic_cast<const tf2::LookupException*>(&error) ||
         dynamic_cast<const tf2::ConnectivityException*>(&error) ||
         dynamic_cast<const tf2::ExtrapolationException*>(&error);
}

inline int64_t diagnostic_thread_cpu_ns() noexcept {
#if defined(CLOCK_THREAD_CPUTIME_ID)
  timespec value{};
  if (::clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value) == 0) {
    return int64_t(value.tv_sec) * 1000000000LL + value.tv_nsec;
  }
#endif
  return -1;
}

// Owner-side boundaries only, never claimed to be tf2-internal timings. Stage
// boundaries remain -1 if the operation threw before that boundary was reached.
struct TfFailureDetail {
  const char* role{"?"};
  uint64_t sequence{0};
  int64_t source_ns{-1}, query_ns{-1}, fk_end_ns{-1}, deadline_ns{-1};
  int64_t begin_ns{-1}, failure_ns{-1}, current_ros_ns{-1};
  int64_t camera_end_ns{-1}, observations_end_ns{-1}, spatial_begin_ns{-1};
  int64_t cpu_begin_ns{-1}, cpu_end_ns{-1};
  ThreadSchedulingDelta scheduling;

  std::string message(const char* reason, const char* kind) const {
    const auto cpu = cpu_begin_ns >= 0 && cpu_end_ns >= cpu_begin_ns ?
        cpu_end_ns - cpu_begin_ns : -1;
    std::ostringstream detail;
    // r/n=role/sequence, s/q=source/query ROS ns, fk/d/f=FK end/deadline/failure
    // steady ns, ros=current ROS ns, b=CPU-window start steady ns, cpu=thread
    // CPU delta ns, rq=runqueue wait ns, rs=SchedstatStatus, sc=sampling wall ns,
    // ca/ob/sp=camera end/observations end/spatial begin steady ns, k=TF class.
    detail << " tf{r=" << std::string(role).substr(0, 8) << ",n=" << sequence
           << ",s=" << source_ns << ",q=" << query_ns << ",fk=" << fk_end_ns
           << ",d=" << deadline_ns << ",f=" << failure_ns << ",ros=" << current_ros_ns
           << ",b=" << begin_ns << ",cpu=" << cpu << ",rq=" << scheduling.runqueue_wait_ns
           << ",rs=" << static_cast<int>(scheduling.status) << ",sc=" << scheduling.sampling_wall_ns
           << ",ca=" << camera_end_ns << ",ob=" << observations_end_ns
           << ",sp=" << spatial_begin_ns << ",k=" << kind << '}';
    const auto suffix = detail.str();
    // The existing first-fault latch stores 512 bytes. Keep every diagnostic
    // field and the original reason prefix within that unchanged limit.
    return std::string(reason).substr(0, 512 - suffix.size()) + suffix;
  }
};

[[noreturn]] inline void rethrow_tf_failure(
    const tf2::TransformException& error, const TfFailureDetail& detail) {
  if (dynamic_cast<const tf2::TimeoutException*>(&error)) {
    throw tf2::TimeoutException(detail.message(error.what(), "timeout"));
  }
  if (dynamic_cast<const tf2::LookupException*>(&error)) {
    throw tf2::LookupException(detail.message(error.what(), "lookup"));
  }
  if (dynamic_cast<const tf2::ConnectivityException*>(&error)) {
    throw tf2::ConnectivityException(detail.message(error.what(), "connectivity"));
  }
  if (dynamic_cast<const tf2::ExtrapolationException*>(&error)) {
    throw tf2::ExtrapolationException(detail.message(error.what(), "extrapolation"));
  }
  if (dynamic_cast<const tf2::InvalidArgumentException*>(&error)) {
    throw tf2::InvalidArgumentException(detail.message(error.what(), "invalid_argument"));
  }
  throw tf2::TransformException(detail.message(error.what(), "transform"));
}

}  // namespace sunhorizon_perception_runtime::detail
