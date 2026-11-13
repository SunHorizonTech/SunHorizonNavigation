#pragma once

#include <cerrno>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string_view>
#include "sunhorizon_3d_localizer/ndt_match_audit.hpp"
#if defined(__linux__)
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace sunhorizon_3d_localizer::detail {
inline void validate_direct_worker_nice(int64_t requested) {
  if (requested != 0 && requested != -5)
    throw std::invalid_argument("direct_worker_nice_must_be_0_or_minus_5");
}
struct DirectWorkerState {
  int64_t tid{-1};
  int policy{-1}, nice{-100};
};
struct DirectWorkerSyscalls {
  int64_t self_tid() const {
#if defined(__linux__)
    return static_cast<int64_t>(::syscall(SYS_gettid));
#else
    return -1;
#endif
  }
  int policy(int64_t tid) const {
#if defined(__linux__)
    static_assert(SCHED_OTHER == 0, "direct NDT requires SCHED_OTHER");
    return ::sched_getscheduler(static_cast<pid_t>(tid));
#else
    (void)tid; return -1;
#endif
  }
  int nice(int64_t tid, int& error) const {
#if defined(__linux__)
    errno = 0;
    const int value = ::getpriority(PRIO_PROCESS, static_cast<id_t>(tid));
    error = errno;
    return value;
#else
    (void)tid; error = ENOTSUP; return -100;
#endif
  }
  int name_self() const {
#if defined(__linux__)
    return ::pthread_setname_np(::pthread_self(), "shn_ndt");
#else
    return ENOTSUP;
#endif
  }
  int set_nice(int64_t tid, int requested) const {
#if defined(__linux__)
    return ::setpriority(PRIO_PROCESS, static_cast<id_t>(tid), requested) == 0 ?
        0 : (errno ? errno : EIO);
#else
    (void)tid; (void)requested; return ENOTSUP;
#endif
  }
};
// Runtime scheduling observations are optional; startup uses strict readback.
template<class Syscalls> DirectWorkerState read_direct_worker(Syscalls& calls,bool strict=true) {
  DirectWorkerState state;
  state.tid = calls.self_tid();
  if (state.tid <= 0) throw std::runtime_error("ndt_worker_tid_unavailable");
  state.policy = calls.policy(state.tid);
  if (state.policy < 0 && strict) throw std::runtime_error("ndt_worker_policy_read_failed");
  int error = 0;
  state.nice = calls.nice(state.tid, error);
  if (error) {
    if (strict) throw std::runtime_error("ndt_worker_nice_read_failed");
    state.nice=-100;
  }
  return state;
}
inline void require_direct_worker_identity(const DirectWorkerState& actual,int64_t tid) {
  if (tid<=0 || actual.tid!=tid)
    throw std::runtime_error("ndt_worker_tid_changed");
}
inline void require_direct_worker(const DirectWorkerState& actual, int64_t tid, int nice) {
  if (tid <= 0 || actual.tid != tid || actual.policy != 0 || actual.nice != nice)
    throw std::runtime_error("ndt_worker_tid_policy_or_nice_changed");
}
template<class Syscalls> DirectWorkerState configure_direct_worker(Syscalls& calls, int64_t requested) {
  validate_direct_worker_nice(requested);
  const auto nice = static_cast<int>(requested);
  const auto before = read_direct_worker(calls);
  if (before.policy != 0) throw std::runtime_error("ndt_worker_requires_sched_other");
  if (calls.name_self() != 0) throw std::runtime_error("ndt_worker_name_failed");
  if (calls.set_nice(before.tid, nice) != 0)
    throw std::runtime_error("ndt_worker_setpriority_failed");
  const auto after = read_direct_worker(calls);
  require_direct_worker(after, before.tid, nice);
  return after;
}

// Observation only. Missing/inactive/regressed counters are never zero delay.
enum class NdtSchedstatStatus : int {
  Unsupported=0, Available=1, OpenFailed=2, ReadFailed=3, Malformed=4,
  Inactive=5, Regressed=6, ThreadChanged=7, Overflow=8, ClockRegressed=9
};
struct NdtSchedstatSample {
  int64_t tid{-1}, begin_ns{-1}, end_ns{-1};
  uint64_t running_ns{0}, runqueue_ns{0}, timeslices{0};
  NdtSchedstatStatus status{NdtSchedstatStatus::Unsupported};
};
struct NdtSchedstatDelta {
  bool available{false};
  NdtSchedstatStatus status{NdtSchedstatStatus::Unsupported};
  int64_t tid{-1}, begin_ns{-1}, end_ns{-1}, sampling_ns{-1};
  int64_t running_ns{-1}, runqueue_ns{-1};
};
inline bool parse_ndt_schedstat(std::string_view text, NdtSchedstatSample& sample) noexcept {
  const auto space = [](char c) {return c==' ' || c=='\t' || c=='\r' || c=='\n';};
  if (text.empty()) {sample.status=NdtSchedstatStatus::Malformed; return false;}
  uint64_t values[3]{};
  const char* p=text.data(); const char* end=p+text.size();
  for (auto& value : values) {
    while (p!=end && space(*p)) ++p;
    const auto result=std::from_chars(p,end,value);
    if (result.ec!=std::errc{} || result.ptr==p || (result.ptr!=end && !space(*result.ptr))) {
      sample.status=NdtSchedstatStatus::Malformed; return false;
    }
    p=result.ptr;
  }
  while (p!=end && space(*p)) ++p;
  if (p!=end) {sample.status=NdtSchedstatStatus::Malformed; return false;}
  sample.running_ns=values[0]; sample.runqueue_ns=values[1]; sample.timeslices=values[2];
  sample.status=values[2] ? NdtSchedstatStatus::Available : NdtSchedstatStatus::Inactive;
  return true;
}
inline NdtSchedstatDelta ndt_schedstat_delta(const NdtSchedstatSample& before,
    const NdtSchedstatSample& after) noexcept {
  NdtSchedstatDelta out;
  out.tid=before.tid; out.begin_ns=before.begin_ns; out.end_ns=after.end_ns;
  if (before.begin_ns<0 || before.end_ns<before.begin_ns || after.begin_ns<before.end_ns || after.end_ns<after.begin_ns) {
    out.status=NdtSchedstatStatus::ClockRegressed; return out;
  }
  const auto a=before.end_ns-before.begin_ns, b=after.end_ns-after.begin_ns;
  if (a<=std::numeric_limits<int64_t>::max()-b) out.sampling_ns=a+b;
  if (before.tid<=0 || before.tid!=after.tid) {
    out.status=before.status==NdtSchedstatStatus::Unsupported ? before.status : NdtSchedstatStatus::ThreadChanged;
    return out;
  }
  out.status=before.status!=NdtSchedstatStatus::Available ? before.status : after.status;
  if (out.status!=NdtSchedstatStatus::Available) return out;
  if (after.running_ns<before.running_ns || after.runqueue_ns<before.runqueue_ns || after.timeslices<before.timeslices) {
    out.status=NdtSchedstatStatus::Regressed; return out;
  }
  const auto run=after.running_ns-before.running_ns, wait=after.runqueue_ns-before.runqueue_ns;
  const auto maximum=static_cast<uint64_t>(std::numeric_limits<int64_t>::max());
  if (run>maximum || wait>maximum) {out.status=NdtSchedstatStatus::Overflow; return out;}
  out.available=true; out.running_ns=static_cast<int64_t>(run); out.runqueue_ns=static_cast<int64_t>(wait);
  return out;
}
inline NdtStageSchedulingAudit ndt_stage_scheduling_audit(
    const NdtSchedstatSample& before, const NdtSchedstatSample& after) noexcept {
  const auto delta = ndt_schedstat_delta(before, after);
  return {delta.available, static_cast<int>(delta.status), delta.tid,
    delta.begin_ns, delta.end_ns, delta.sampling_ns, delta.running_ns, delta.runqueue_ns};
}
inline int64_t ndt_scheduling_steady_ns() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
class NdtSchedstatReader {
 public:
  NdtSchedstatReader() noexcept {
#if defined(__linux__)
    tid_=static_cast<int64_t>(::syscall(SYS_gettid));
    fd_=::open("/proc/thread-self/schedstat",O_RDONLY|O_CLOEXEC);
#endif
  }
  ~NdtSchedstatReader() {
#if defined(__linux__)
    if (fd_>=0) ::close(fd_);
#endif
  }
  NdtSchedstatReader(const NdtSchedstatReader&)=delete;
  NdtSchedstatReader& operator=(const NdtSchedstatReader&)=delete;
  NdtSchedstatSample sample() const noexcept {
    NdtSchedstatSample out;
    out.tid=tid_; out.begin_ns=ndt_scheduling_steady_ns();
#if defined(__linux__)
    out.status=NdtSchedstatStatus::OpenFailed;
    if (fd_>=0) {
      char text[128]; const auto size=::pread(fd_,text,sizeof(text),0);
      out.status=NdtSchedstatStatus::ReadFailed;
      if (size>0 && static_cast<size_t>(size)<sizeof(text))
        (void)parse_ndt_schedstat({text,static_cast<size_t>(size)},out);
    }
#endif
    out.end_ns=ndt_scheduling_steady_ns();
    return out;
  }
 private:
  int64_t tid_{-1};
#if defined(__linux__)
  int fd_{-1};
#endif
};
inline NdtSchedstatReader& current_ndt_schedstat() noexcept {
  thread_local NdtSchedstatReader reader;
  return reader;
}
}  // namespace sunhorizon_3d_localizer::detail
