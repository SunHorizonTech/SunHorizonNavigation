#pragma once

#include <charconv>
#include <chrono>
#include <cstdint>
#include <limits>
#include <string_view>
#if defined(__linux__)
#include <fcntl.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace sunhorizon_perception_runtime::detail {
enum class SchedstatStatus : int {
  Unsupported=0, Available=1, OpenFailed=2, ReadFailed=3, Malformed=4,
  CountersInactive=5, CounterRegressed=6, ThreadChanged=7, DeltaOverflow=8
};
struct ThreadSchedulingSample {
  int64_t tid{-1}, read_begin_steady_ns{0}, read_end_steady_ns{0};
  uint64_t running_ns{0}, runqueue_wait_ns{0}, timeslices{0};
  int64_t voluntary_switches{-1}, involuntary_switches{-1};
  SchedstatStatus status{SchedstatStatus::Unsupported};
  bool rusage_available{false};
};
struct ThreadSchedulingDelta {
  bool schedstat_available{false}, rusage_available{false};
  SchedstatStatus status{SchedstatStatus::Unsupported};
  int64_t tid{-1}, window_begin_steady_ns{-1}, window_end_steady_ns{-1}, sampling_wall_ns{-1};
  int64_t running_ns{-1}, runqueue_wait_ns{-1}, timeslices{-1};
  int64_t voluntary_switches{-1}, involuntary_switches{-1};
};
inline bool schedstat_space(char c) {return c==' ' || c=='\t' || c=='\r' || c=='\n';}
inline bool parse_schedstat(std::string_view text, ThreadSchedulingSample& output) noexcept {
  if(text.empty()){output.status=SchedstatStatus::Malformed;return false;}
  uint64_t values[3]{};
  const char* p=text.data();const char* end=p+text.size();
  for(auto& value:values){
    while(p!=end && schedstat_space(*p))++p;
    const auto result=std::from_chars(p,end,value);
    if(result.ec!=std::errc{} || result.ptr==p){output.status=SchedstatStatus::Malformed;return false;}
    p=result.ptr;
    if(p!=end && !schedstat_space(*p)){output.status=SchedstatStatus::Malformed;return false;}
  }
  while(p!=end && schedstat_space(*p))++p;
  if(p!=end){output.status=SchedstatStatus::Malformed;return false;}
  output.running_ns=values[0];output.runqueue_wait_ns=values[1];output.timeslices=values[2];
  // Linux emits all zeros when sched_info accounting is inactive. A zero WAIT
  // with a real timeslice count, however, is a valid zero, not missing data.
  output.status=values[2] ? SchedstatStatus::Available : SchedstatStatus::CountersInactive;
  return true;
}
inline ThreadSchedulingDelta scheduling_delta(const ThreadSchedulingSample& before,
    const ThreadSchedulingSample& after) noexcept {
  ThreadSchedulingDelta out;out.tid=before.tid;
  out.window_begin_steady_ns=before.read_begin_steady_ns;out.window_end_steady_ns=after.read_end_steady_ns;
  if(before.read_begin_steady_ns>=0 && after.read_begin_steady_ns>=0 &&
      before.read_end_steady_ns>=before.read_begin_steady_ns && after.read_end_steady_ns>=after.read_begin_steady_ns){
    const auto first=before.read_end_steady_ns-before.read_begin_steady_ns,
      second=after.read_end_steady_ns-after.read_begin_steady_ns;
    if(first<=std::numeric_limits<int64_t>::max()-second)out.sampling_wall_ns=first+second;
  }
  if(before.tid<=0 || after.tid!=before.tid){
    out.status=before.status==SchedstatStatus::Unsupported ? before.status : SchedstatStatus::ThreadChanged;
    return out;
  }
  if(before.rusage_available && after.rusage_available && before.voluntary_switches>=0 &&
      before.involuntary_switches>=0 && after.voluntary_switches>=before.voluntary_switches &&
      after.involuntary_switches>=before.involuntary_switches){
    out.rusage_available=true;
    out.voluntary_switches=after.voluntary_switches-before.voluntary_switches;
    out.involuntary_switches=after.involuntary_switches-before.involuntary_switches;
  }
  out.status=before.status!=SchedstatStatus::Available ? before.status : after.status;
  if(out.status!=SchedstatStatus::Available)return out;
  if(after.running_ns<before.running_ns || after.runqueue_wait_ns<before.runqueue_wait_ns ||
      after.timeslices<before.timeslices){out.status=SchedstatStatus::CounterRegressed;return out;}
  const auto running=after.running_ns-before.running_ns,wait=after.runqueue_wait_ns-before.runqueue_wait_ns,
    slices=after.timeslices-before.timeslices;
  const auto maximum=static_cast<uint64_t>(std::numeric_limits<int64_t>::max());
  if(running>maximum || wait>maximum || slices>maximum){out.status=SchedstatStatus::DeltaOverflow;return out;}
  out.schedstat_available=true;out.running_ns=static_cast<int64_t>(running);
  out.runqueue_wait_ns=static_cast<int64_t>(wait);out.timeslices=static_cast<int64_t>(slices);
  return out;
}
inline int64_t scheduling_steady_ns() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
// Linux 5.10 proc schedstat uses CONFIG_SCHED_INFO's per-task run_delay;
// sched_info_on() is always true with CONFIG_SCHEDSTATS, independently of the
// kernel.sched_schedstats switch. Read/parse/counter validity is checked here.
// Observation only. No sysctl/priority changes, retries, evidence clocks or
// exceptions. Reuse the SAME thread's proc FD until that engine worker exits.
class ThreadSchedulingReader {
 public:
  ThreadSchedulingReader() noexcept {
#if defined(__linux__)
    tid_=static_cast<int64_t>(::syscall(SYS_gettid));
    fd_=::open("/proc/thread-self/schedstat",O_RDONLY|O_CLOEXEC);
#endif
  }
  ~ThreadSchedulingReader(){
#if defined(__linux__)
    if(fd_>=0)::close(fd_);
#endif
  }
  ThreadSchedulingReader(const ThreadSchedulingReader&)=delete;
  ThreadSchedulingReader& operator=(const ThreadSchedulingReader&)=delete;
  ThreadSchedulingSample sample() const noexcept {
    ThreadSchedulingSample out;out.tid=tid_;out.read_begin_steady_ns=scheduling_steady_ns();
#if defined(__linux__)
#if defined(RUSAGE_THREAD)
    rusage usage{};
    if(::getrusage(RUSAGE_THREAD,&usage)==0 && usage.ru_nvcsw>=0 && usage.ru_nivcsw>=0){
      out.rusage_available=true;out.voluntary_switches=usage.ru_nvcsw;out.involuntary_switches=usage.ru_nivcsw;
    }
#endif
    out.status=SchedstatStatus::OpenFailed;
    if(fd_>=0){
      char text[128];const auto size=::pread(fd_,text,sizeof(text),0);
      out.status=SchedstatStatus::ReadFailed;
      if(size>0 && static_cast<size_t>(size)<sizeof(text))
        (void)parse_schedstat({text,static_cast<size_t>(size)},out);
    }
#endif
    out.read_end_steady_ns=scheduling_steady_ns();return out;
  }
 private:
  int64_t tid_{-1};
#if defined(__linux__)
  int fd_{-1};
#endif
};
inline ThreadSchedulingReader& current_thread_scheduling() noexcept {
  thread_local ThreadSchedulingReader reader;return reader;
}
}  // namespace sunhorizon_perception_runtime::detail
