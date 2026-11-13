#pragma once

#include <cerrno>
#include <cstdint>
#include <stdexcept>
#include <string_view>
#include <vector>
#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace sunhorizon_perception_runtime::detail {
struct ComputeThreadState {
  bool available{false};
  int64_t tid{-1};
  int policy{-1}, nice{-100};
  bool affinity_available{false};
  std::vector<int64_t> cpu_set;
};
inline void validate_compute_thread_nice(int64_t nice) {
  if(nice!=0 && nice!=-10)throw std::invalid_argument("engine_thread_nice_must_be_0_or_minus_10");
}
inline void validate_compute_thread_cpu_set(const std::vector<int64_t>& cpus) {
  if(!cpus.empty() && cpus!=std::vector<int64_t>{8,9,10,11})
    throw std::invalid_argument("engine_thread_cpu_set_must_be_empty_or_8_9_10_11");
}
inline std::vector<int64_t> compute_owner_cpu_set(const char* name,const std::vector<int64_t>& cpus) {
  validate_compute_thread_cpu_set(cpus);
  const std::string_view role=name ? name : "";
  if(role!="shn_body_belly" && role!="shn_body_chest" &&
      role!="shn_prep_belly" && role!="shn_prep_chest" &&
      role!="shn_geom_belly" && role!="shn_geom_chest")
    throw std::invalid_argument("compute_thread_owner_role_invalid");
  // This is the allowed compute pool, not a per-role singleton assignment.
  // Keep all six owners eligible for spare capacity when independent RT work
  // occupies a core. Kernel scheduling never changes frame/worker ownership.
  return cpus;
}
// This API never enumerates threads or changes policy. Priority and optional
// affinity target only the executing owner's ID obtained from self_tid().
struct ComputeThreadSyscalls {
  int64_t self_tid() const {
#if defined(__linux__)
    return static_cast<int64_t>(::syscall(SYS_gettid));
#else
    return -1;
#endif
  }
  int policy(int64_t tid) const {
#if defined(__linux__)
    static_assert(SCHED_OTHER==0,"compute workers require the Linux SCHED_OTHER policy");
    return ::sched_getscheduler(static_cast<pid_t>(tid));
#else
    (void)tid;return -1;
#endif
  }
  int nice(int64_t tid,int& error) const {
#if defined(__linux__)
    errno=0;const auto value=::getpriority(PRIO_PROCESS,static_cast<id_t>(tid));error=errno;return value;
#else
    (void)tid;error=ENOTSUP;return -100;
#endif
  }
  int name_self(const char* name) const {
#if defined(__linux__)
    return ::pthread_setname_np(::pthread_self(),name);
#else
    (void)name;return ENOTSUP;
#endif
  }
  std::vector<int64_t> affinity(int64_t tid,int& error) const {
#if defined(__linux__)
    cpu_set_t mask;CPU_ZERO(&mask);
    if(::sched_getaffinity(static_cast<pid_t>(tid),sizeof(mask),&mask)!=0){
      error=errno ? errno : EIO;return {};
    }
    std::vector<int64_t> result;
    for(int cpu=0;cpu<CPU_SETSIZE;++cpu)if(CPU_ISSET(cpu,&mask))result.push_back(cpu);
    error=0;return result;
#else
    (void)tid;error=ENOTSUP;return {};
#endif
  }
  int set_affinity(int64_t tid,const std::vector<int64_t>& cpus) const {
#if defined(__linux__)
    cpu_set_t mask;CPU_ZERO(&mask);
    for(const auto cpu:cpus){
      if(cpu<0 || cpu>=CPU_SETSIZE)return EINVAL;
      CPU_SET(static_cast<int>(cpu),&mask);
    }
    return ::sched_setaffinity(static_cast<pid_t>(tid),sizeof(mask),&mask)==0 ? 0 : (errno ? errno : EIO);
#else
    (void)tid;(void)cpus;return ENOTSUP;
#endif
  }
  int set_nice(int64_t tid,int value) const {
#if defined(__linux__)
    return ::setpriority(PRIO_PROCESS,static_cast<id_t>(tid),value)==0 ? 0 : (errno ? errno : EIO);
#else
    (void)tid;(void)value;return ENOTSUP;
#endif
  }
};
// Runtime scheduling observations are optional; startup uses strict readback.
template<class Syscalls> ComputeThreadState read_self_compute_thread(Syscalls& calls,bool read_affinity=false,
    bool strict=true) {
  ComputeThreadState state;state.tid=calls.self_tid();
  if(state.tid<=0)throw std::runtime_error("compute_thread_self_tid_unavailable");
  state.policy=calls.policy(state.tid);
  if(state.policy<0 && strict)throw std::runtime_error("compute_thread_policy_read_failed");
  int error=0;state.nice=calls.nice(state.tid,error);
  if(error){
    if(strict)throw std::runtime_error("compute_thread_nice_read_failed");
    state.nice=-100;
  }
  if(read_affinity){
    state.cpu_set=calls.affinity(state.tid,error);
    if(error || state.cpu_set.empty()){
      if(strict)throw std::runtime_error("compute_thread_affinity_read_failed");
      state.cpu_set.clear();
    }else state.affinity_available=true;
  }
  state.available=true;return state;
}
inline void require_compute_thread_identity(const ComputeThreadState& state,int64_t self_tid) {
  if(!state.available || self_tid<=0 || state.tid!=self_tid)
    throw std::runtime_error("compute_thread_actual_tid_mismatch");
}
inline void require_compute_thread_state(const ComputeThreadState& state,int64_t self_tid,int requested,
    const std::vector<int64_t>& cpus={}) {
  if(!state.available || state.tid!=self_tid || state.policy!=0 || state.nice!=requested)
    throw std::runtime_error("compute_thread_actual_tid_policy_or_nice_mismatch");
  if(!cpus.empty() && (!state.affinity_available || state.cpu_set!=cpus))
    throw std::runtime_error("compute_thread_actual_cpu_set_mismatch");
}
template<class Syscalls> ComputeThreadState configure_self_compute_thread(Syscalls& calls,int requested,
    const char* name,const std::vector<int64_t>& cpus={}) {
  validate_compute_thread_nice(requested);
  const auto owner_cpus=compute_owner_cpu_set(name,cpus);
  const auto before=read_self_compute_thread(calls);
  if(before.policy!=0)throw std::runtime_error("compute_thread_requires_sched_other");
  if(calls.name_self(name)!=0)throw std::runtime_error("compute_thread_name_failed");
  if(calls.set_nice(before.tid,requested)!=0)throw std::runtime_error("compute_thread_setpriority_failed");
  if(!owner_cpus.empty() && calls.set_affinity(before.tid,owner_cpus)!=0)
    throw std::runtime_error("compute_thread_setaffinity_failed");
  const auto after=read_self_compute_thread(calls,!owner_cpus.empty());
  require_compute_thread_state(after,before.tid,requested,owner_cpus);return after;
}
}  // namespace sunhorizon_perception_runtime::detail
