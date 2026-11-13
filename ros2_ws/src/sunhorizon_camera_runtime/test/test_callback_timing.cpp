#include <stdexcept>

#include "sunhorizon_camera_runtime/callback_timing.hpp"

using sunhorizon_camera_runtime::CallbackFrameTiming;
using sunhorizon_camera_runtime::CallbackTimingPhase;
using sunhorizon_camera_runtime::CompletedCallbackTiming;

namespace
{
void require(bool condition, const char * message)
{
  // Keep checks effective in the production Release/NDEBUG build.
  if (!condition) {
    throw std::runtime_error(message);
  }
}
}  // namespace

int main()
{
  CompletedCallbackTiming snapshot;
  require(!snapshot.valid && snapshot.accepted_frames == 0, "startup snapshot is valid");
  CallbackFrameTiming frame;
  frame.entry_system_ns = 1000000;
  frame.entry = {100, 20};
  frame.lock_wait = {{110, 25}, {310, 30}};
  frame.validate_convert = {{310, 30}, {410, 110}};
  frame.publish.begin = {410, 110};
  require(!snapshot.accept(1, frame), "partial/startup-dropped frame accepted");
  require(!snapshot.valid, "partial frame created valid snapshot");
  frame.finish_publication({710, 120}, 1000610);
  require(snapshot.accept(1, frame), "completed frame rejected");
  require(snapshot.valid && snapshot.accepted_frames == 1, "accepted sequence lost");
  const auto & saved = snapshot.frame;
  require(saved.entry_system_ns == 1000000, "entry system clock changed");
  require(saved.entry.steady_ns == 100 && saved.entry.thread_cpu_ns == 20,
    "entry clock domains mixed");
  require(saved.lock_wait.wall_ns() == 200 && saved.lock_wait.thread_cpu_ns() == 5,
    "lock wall and CPU time mixed");
  require(saved.validate_convert.wall_ns() == 100 &&
    saved.validate_convert.thread_cpu_ns() == 80, "conversion clocks mixed");
  require(saved.publish.wall_ns() == 300 && saved.publish.thread_cpu_ns() == 10,
    "publication clocks mixed");
  require(saved.publish_complete_system_ns == 1000610 && saved.publish.end.steady_ns == 710,
    "publication completion lost");

  // Subsequent arrival/exception and repeated diagnostics cannot relabel the
  // last completed frame, nor add later record_accepted lock delay to publish.
  CallbackFrameTiming next;
  next.entry_system_ns = 2000000;
  next.entry = {900, 130};
  require(!snapshot.accept(2, next), "exception/incomplete frame replaced snapshot");
  frame.finish_publication({9999, 999}, 9000000);
  require(snapshot.frame.publish.end.steady_ns == 710, "snapshot retained mutable frame");
  require(snapshot.frame.publish.wall_ns() == 300, "post-publication tail counted as publish");
  require(!snapshot.accept(1, frame) && !snapshot.accept(0, frame),
    "duplicate or invalid sequence accepted");
  require(snapshot.accepted_frames == 1 && snapshot.frame.entry_system_ns == 1000000,
    "failed frame or repeated snapshot invented a new completion");
  next.publish.begin = {910, -1};
  next.finish_publication({940, -1}, 2000040);
  require(snapshot.accept(2, next), "next completed frame rejected");
  require(snapshot.accepted_frames == 2 && snapshot.frame.entry_system_ns == 2000000,
    "new accepted sequence not associated with new clocks");
  require(snapshot.frame.publish.wall_ns() == 30 &&
    snapshot.frame.publish.thread_cpu_ns() == -1, "failed CPU clock became zero work");
  require(!snapshot.accept(1, frame), "older accepted sequence replaced latest");

  require(CallbackTimingPhase{{10, -1}, {20, 30}}.thread_cpu_ns() == -1,
    "failed begin CPU clock ignored");
  require(CallbackTimingPhase{{10, 30}, {20, -1}}.thread_cpu_ns() == -1,
    "failed end CPU clock ignored");
  require(CallbackTimingPhase{{20, 30}, {10, 20}}.wall_ns() == -1 &&
    CallbackTimingPhase{{20, 30}, {10, 20}}.thread_cpu_ns() == -1,
    "backwards clocks presented as a duration");
  CallbackFrameTiming worker;
  worker.worker_dispatch=true;worker.ingress_sequence=7;
  worker.entry_system_ns=1000000;worker.entry={100,50000};
  worker.ingress_validate={{110,50001},{120,50010}};
  worker.queue_enter_steady_ns=130;worker.worker_start_steady_ns=230;
  worker.worker_convert={{240,10},{250,30}};worker.publish.begin={250,30};
  worker.finish_publication({270,40},1000170);
  require(worker.queue_wait_wall_ns()==100,"queue wait lost original enqueue point");
  require(worker.validate_convert.thread_cpu_ns()==-1 && worker.validate_convert.wall_ns()==-1,
    "worker mode invented a cross-thread validate_convert duration");
  require(worker.ingress_validate.thread_cpu_ns()==9 && worker.worker_convert.thread_cpu_ns()==20 && worker.publish.thread_cpu_ns()==10,
    "SDK and worker CPU clock domains mixed");
  require(snapshot.accept(3,worker) && snapshot.frame.entry_system_ns==1000000 && snapshot.frame.ingress_sequence==7,
    "worker reset original ingress timestamps/sequence");
  return 0;
}
