#include <gtest/gtest.h>

#include <future>
#include <limits>
#include <thread>
#include <vector>

#include "sunhorizon_3d_localizer/ndt_worker_control.hpp"

namespace sunhorizon_3d_localizer::detail {
namespace {
TEST(NdtStageSchedulingAudit, AdjacentStagesKeepTheirOwnWindowAndUnavailableIsNotZero) {
  NdtSchedstatSample before, boundary, after;
  before.tid = boundary.tid = after.tid = 42;
  before.begin_ns = 100; before.end_ns = 102;
  boundary.begin_ns = 200; boundary.end_ns = 203;
  after.begin_ns = 300; after.end_ns = 304;
  ASSERT_TRUE(parse_ndt_schedstat("10 20 1", before));
  ASSERT_TRUE(parse_ndt_schedstat("30 50 2", boundary));
  ASSERT_TRUE(parse_ndt_schedstat("40 50 3", after));
  const auto preprocess = ndt_stage_scheduling_audit(before, boundary);
  const auto gates = ndt_stage_scheduling_audit(boundary, after);
  ASSERT_TRUE(preprocess.available); ASSERT_TRUE(gates.available);
  EXPECT_EQ(preprocess.begin_steady_ns, 100); EXPECT_EQ(preprocess.end_steady_ns, 203);
  EXPECT_EQ(preprocess.running_ns, 20); EXPECT_EQ(preprocess.runqueue_wait_ns, 30);
  EXPECT_EQ(preprocess.sampling_wall_ns, 5);
  EXPECT_EQ(gates.begin_steady_ns, 200); EXPECT_EQ(gates.end_steady_ns, 304);
  EXPECT_EQ(gates.running_ns, 10); EXPECT_EQ(gates.runqueue_wait_ns, 0);
  EXPECT_EQ(gates.owner_tid, 42); EXPECT_EQ(gates.sampling_wall_ns, 7);
  after.status = NdtSchedstatStatus::ReadFailed;
  const auto unavailable = ndt_stage_scheduling_audit(boundary, after);
  EXPECT_FALSE(unavailable.available);
  EXPECT_EQ(unavailable.status, static_cast<int>(NdtSchedstatStatus::ReadFailed));
  EXPECT_EQ(unavailable.running_ns, -1); EXPECT_EQ(unavailable.runqueue_wait_ns, -1);
  EXPECT_EQ(unavailable.begin_steady_ns, 200); EXPECT_EQ(unavailable.end_steady_ns, 304);
}

struct FakeWorkerCalls {
  int64_t tid{42};
  int actual_policy{0}, actual_nice{0}, nice_error{0}, name_error{0}, set_error{0};
  int set_calls{0}, name_calls{0};
  bool wrong_tid_after{false}, wrong_policy_after{false}, wrong_nice_after{false};
  std::vector<int64_t> targets;
  int64_t self_tid() const {return tid + (wrong_tid_after && set_calls ? 1 : 0);}
  int policy(int64_t target) {
    targets.push_back(target);
    return wrong_policy_after && set_calls ? 1 : actual_policy;
  }
  int nice(int64_t target, int& error) {
    targets.push_back(target); error=nice_error; return actual_nice;
  }
  int name_self() {++name_calls; return name_error;}
  int set_nice(int64_t target, int value) {
    targets.push_back(target); ++set_calls;
    if (!set_error && !wrong_nice_after) actual_nice=value;
    return set_error;
  }
};

TEST(NdtWorkerControl, OnlySelfIsConfiguredAndLaterReadsNeverRepairDrift) {
  for (const int requested : {0,-5}) {
    FakeWorkerCalls calls;
    const auto state=configure_direct_worker(calls,requested);
    EXPECT_EQ(state.tid,42); EXPECT_EQ(state.policy,0); EXPECT_EQ(state.nice,requested);
    EXPECT_EQ(calls.set_calls,1); EXPECT_EQ(calls.name_calls,1);
    for (const auto target : calls.targets) EXPECT_EQ(target,42);
    EXPECT_NO_THROW(require_direct_worker_identity(read_direct_worker(calls,false),state.tid));
    calls.actual_nice=5;
    EXPECT_NO_THROW(require_direct_worker_identity(read_direct_worker(calls,false),state.tid));
    EXPECT_EQ(read_direct_worker(calls,false).nice,5);
    calls.actual_nice=requested; calls.actual_policy=1;
    EXPECT_NO_THROW(require_direct_worker_identity(read_direct_worker(calls,false),state.tid));
    EXPECT_EQ(read_direct_worker(calls,false).policy,1);
    calls.nice_error=EIO; calls.actual_policy=-1;
    const auto unavailable=read_direct_worker(calls,false);
    EXPECT_EQ(unavailable.policy,-1); EXPECT_EQ(unavailable.nice,-100);
    EXPECT_NO_THROW(require_direct_worker_identity(unavailable,state.tid));
    calls.nice_error=0; calls.actual_policy=0; calls.tid=43;
    EXPECT_THROW(require_direct_worker_identity(read_direct_worker(calls,false),state.tid),std::runtime_error);
    EXPECT_EQ(calls.set_calls,1); EXPECT_EQ(calls.name_calls,1);
  }
}

TEST(NdtWorkerControl, InvalidValuesAndEveryFailedSyscallRejectBeforeWork) {
  for (const int64_t requested : {int64_t{-10},int64_t{-4},int64_t{1},int64_t{4294967291},
                                 std::numeric_limits<int64_t>::max()}) {
    FakeWorkerCalls calls;
    EXPECT_THROW(configure_direct_worker(calls,requested),std::invalid_argument);
    EXPECT_TRUE(calls.targets.empty()); EXPECT_EQ(calls.set_calls,0); EXPECT_EQ(calls.name_calls,0);
  }
  for (int failure=0; failure<9; ++failure) {
    FakeWorkerCalls calls;
    if (failure==0) calls.tid=-1;
    if (failure==1) calls.actual_policy=-1;
    if (failure==2) calls.actual_policy=1;
    if (failure==3) calls.nice_error=EACCES;
    if (failure==4) calls.name_error=EINVAL;
    if (failure==5) calls.set_error=EPERM;
    if (failure==6) calls.wrong_tid_after=true;
    if (failure==7) calls.wrong_policy_after=true;
    if (failure==8) calls.wrong_nice_after=true;
    EXPECT_THROW(configure_direct_worker(calls,-5),std::runtime_error);
    EXPECT_LE(calls.set_calls,1);
    if (failure<5) EXPECT_EQ(calls.set_calls,0);
  }
}

void exercise_native_worker(int requested) {
#if defined(__linux__)
  DirectWorkerSyscalls main_calls;
  const auto before=read_direct_worker(main_calls);
  if (before.policy!=0 || before.nice!=0)
    GTEST_SKIP()<<"requires inherited SCHED_OTHER/nice0; never changes test main priority";
  if (requested==-5 && ::geteuid()!=0) {
    rlimit limit{};
    if (::getrlimit(RLIMIT_NICE,&limit)!=0 || limit.rlim_cur<25)
      GTEST_SKIP()<<"native -5 requires root or RLIMIT_NICE >=25";
  }
  std::promise<DirectWorkerState> completed;
  auto result=completed.get_future();
  std::thread worker([&] {
    try {
      DirectWorkerSyscalls calls;
      const auto configured=configure_direct_worker(calls,requested);
      const auto actual=read_direct_worker(calls);
      require_direct_worker(actual,configured.tid,requested);
      char name[16]{};
      if (::pthread_getname_np(::pthread_self(),name,sizeof(name))!=0 || std::string_view(name)!="shn_ndt")
        throw std::runtime_error("ndt_worker_actual_name_mismatch");
      completed.set_value(actual);
    } catch (...) {completed.set_exception(std::current_exception());}
  });
  worker.join();
  const auto after=read_direct_worker(main_calls);
  EXPECT_EQ(after.tid,before.tid); EXPECT_EQ(after.policy,before.policy); EXPECT_EQ(after.nice,before.nice);
  const auto actual=result.get();
  EXPECT_NE(actual.tid,before.tid); EXPECT_EQ(actual.nice,requested); EXPECT_EQ(actual.policy,0);
#else
  (void)requested;
  GTEST_SKIP()<<"native Linux self-priority test unavailable";
#endif
}
TEST(NdtWorkerControl, NativeDefaultZeroOnlyChangesDedicatedThread) {exercise_native_worker(0);}
TEST(NdtWorkerControl, NativeOptInMinusFiveOnlyChangesDedicatedThread) {exercise_native_worker(-5);}

TEST(NdtWorkerScheduling, ExactDeltasAndValidZeroWaitUseOneThreadAndRealWindow) {
  NdtSchedstatSample before,after;
  before.tid=after.tid=42; before.begin_ns=100; before.end_ns=105;
  after.begin_ns=200; after.end_ns=207;
  ASSERT_TRUE(parse_ndt_schedstat("1000 2000 10\n",before));
  ASSERT_TRUE(parse_ndt_schedstat("1400 2600 15\n",after));
  const auto delta=ndt_schedstat_delta(before,after);
  EXPECT_TRUE(delta.available); EXPECT_EQ(delta.status,NdtSchedstatStatus::Available);
  EXPECT_EQ(delta.tid,42); EXPECT_EQ(delta.begin_ns,100); EXPECT_EQ(delta.end_ns,207);
  EXPECT_EQ(delta.sampling_ns,12); EXPECT_EQ(delta.running_ns,400); EXPECT_EQ(delta.runqueue_ns,600);
  after.runqueue_ns=before.runqueue_ns;
  EXPECT_TRUE(ndt_schedstat_delta(before,after).available);
  EXPECT_EQ(ndt_schedstat_delta(before,after).runqueue_ns,0);
}

TEST(NdtWorkerScheduling, MissingMalformedRegressedOrDifferentThreadIsExplicitlyUnavailable) {
  for (const auto* text : {"","1 2","1 2 3 4","-1 2 3","1x 2 3","18446744073709551616 2 3"}) {
    NdtSchedstatSample sample;
    EXPECT_FALSE(parse_ndt_schedstat(text,sample));
    EXPECT_EQ(sample.status,NdtSchedstatStatus::Malformed);
  }
  NdtSchedstatSample before,after;
  before.tid=after.tid=42; before.begin_ns=10; before.end_ns=11;
  after.begin_ns=20; after.end_ns=21;
  ASSERT_TRUE(parse_ndt_schedstat("0 0 0",before));
  EXPECT_EQ(before.status,NdtSchedstatStatus::Inactive);
  after=before; after.begin_ns=20; after.end_ns=21;
  EXPECT_FALSE(ndt_schedstat_delta(before,after).available);
  before.status=after.status=NdtSchedstatStatus::Available;
  before.running_ns=10; after.running_ns=9;
  EXPECT_EQ(ndt_schedstat_delta(before,after).status,NdtSchedstatStatus::Regressed);
  after.running_ns=20; before.runqueue_ns=10; after.runqueue_ns=9;
  EXPECT_EQ(ndt_schedstat_delta(before,after).status,NdtSchedstatStatus::Regressed);
  after.runqueue_ns=20; after.tid=43;
  EXPECT_EQ(ndt_schedstat_delta(before,after).status,NdtSchedstatStatus::ThreadChanged);
  after.tid=42; after.begin_ns=0;
  EXPECT_EQ(ndt_schedstat_delta(before,after).status,NdtSchedstatStatus::ClockRegressed);
  after.begin_ns=20; after.status=NdtSchedstatStatus::ReadFailed;
  auto missing=ndt_schedstat_delta(before,after);
  EXPECT_FALSE(missing.available); EXPECT_EQ(missing.running_ns,-1); EXPECT_EQ(missing.runqueue_ns,-1);
  EXPECT_EQ(missing.status,NdtSchedstatStatus::ReadFailed);
  after.status=NdtSchedstatStatus::Available; before.running_ns=0;
  after.running_ns=std::numeric_limits<uint64_t>::max();
  EXPECT_EQ(ndt_schedstat_delta(before,after).status,NdtSchedstatStatus::Overflow);
}
}  // namespace
}  // namespace sunhorizon_3d_localizer::detail
