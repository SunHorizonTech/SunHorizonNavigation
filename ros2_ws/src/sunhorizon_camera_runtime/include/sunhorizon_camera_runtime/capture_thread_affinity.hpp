// Copyright (c) 2026 SunHorizon. Apache-2.0.
#pragma once
#include <cstdint>
#include <exception>
#include <stdexcept>
#include <vector>
#ifdef __linux__
#include <cerrno>
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace sunhorizon_camera_runtime { namespace capture_affinity {
inline const std::vector<int64_t> & approvedMask() {
  static const std::vector<int64_t> cpus{0,1,2,3,6,7,8,9,10,11};return cpus;
}
inline const std::vector<int64_t> & serialLivoCompanionMask() {
  static const std::vector<int64_t> cpus{0,1,2,3,7,8,9,10,11};return cpus;
}
inline void validate(const std::vector<int64_t> & cpus) {
  if(!cpus.empty()&&cpus!=approvedMask()&&cpus!=serialLivoCompanionMask())
    throw std::invalid_argument("camera_capture_affinity_requires_fixed_robot_a_ten_core_mask_or_minus6_nine_core_mask");
}
struct State {
  std::vector<int64_t> cpus;
  long tid{-1};int policy{-1},nice{0};
};
// All native operations target the calling thread (pid0). No process/global or
// other-owner mutation, no policy/nice write. Empty configuration never calls these.
struct NativeOps {
  std::vector<int64_t> readAffinity() const {
#ifdef __linux__
    cpu_set_t mask;CPU_ZERO(&mask);
    if(::sched_getaffinity(0,sizeof(mask),&mask)!=0)
      throw std::runtime_error("camera_capture_affinity_read_failed");
    std::vector<int64_t> cpus;
    for(int cpu=0;cpu<CPU_SETSIZE;++cpu)if(CPU_ISSET(cpu,&mask))cpus.push_back(cpu);
    return cpus;
#else
    throw std::runtime_error("camera_capture_affinity_requires_linux");
#endif
  }
  State read() const {
#ifdef __linux__
    State state;state.tid=static_cast<long>(::syscall(SYS_gettid));
    state.cpus=readAffinity();state.policy=::sched_getscheduler(0);errno=0;
    state.nice=::getpriority(PRIO_PROCESS,0);const int error=errno;
    if(state.tid<=0||state.policy<0||error)
      throw std::runtime_error("camera_capture_affinity_owner_read_failed");
    return state;
#else
    throw std::runtime_error("camera_capture_affinity_requires_linux");
#endif
  }
  void setAffinity(const std::vector<int64_t> & cpus) const {
#ifdef __linux__
    cpu_set_t mask;CPU_ZERO(&mask);
    for(const auto cpu:cpus) {
      if(cpu<0||cpu>=CPU_SETSIZE)throw std::invalid_argument("camera_capture_affinity_cpu_range");
      CPU_SET(static_cast<int>(cpu),&mask);
    }
    if(::sched_setaffinity(0,sizeof(mask),&mask)!=0)
      throw std::runtime_error("camera_capture_affinity_apply_failed");
#else
    (void)cpus;throw std::runtime_error("camera_capture_affinity_requires_linux");
#endif
  }
};
// Explicit scope: restoration failures propagate, including on construction failure.
// A failed set may have changed the mask, so restoration is attempted after any set.
template<class Ops, class Create> void with_affinity(
    const std::vector<int64_t>& cpus, const Ops& ops, Create create) {
  validate(cpus);
  if(cpus.empty()){create();return;}
  const auto before=ops.read();
  if(before.tid<=0 || before.policy!=0 || before.cpus.empty())
    throw std::runtime_error("camera_capture_affinity_requires_sched_other_owner");
  auto require_state=[&](const State& state,const std::vector<int64_t>& expected) {
    if(state.tid!=before.tid || state.policy!=before.policy ||
       state.nice!=before.nice || state.cpus!=expected)
      throw std::runtime_error("camera_capture_affinity_readback_mismatch");
  };
  std::exception_ptr failure;
  try {
    ops.setAffinity(cpus);
    require_state(ops.read(),cpus);
    create();
    require_state(ops.read(),cpus);
  } catch(...) {failure=std::current_exception();}
  try {
    ops.setAffinity(before.cpus);
    require_state(ops.read(),before.cpus);
  } catch(...) {
    std::throw_with_nested(std::runtime_error("camera_capture_affinity_restore_failed"));
  }
  if(failure)std::rethrow_exception(failure);
}
}}  // namespace sunhorizon_camera_runtime::capture_affinity
