#include "sunhorizon_obstacle_coverage/rgbd_upstream_diagnostics.hpp"
#include "sunhorizon_rgbd_adapter/rgbd_status_snapshot.hpp"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <thread>

namespace ud = sunhorizon_obstacle_coverage::upstream_diagnostics;

int main() {
  unsetenv("SUNHORIZON_RGBD_TRACE_SOCKET");
  assert(ud::endpoint_from_environment().empty());
  setenv("SUNHORIZON_RGBD_TRACE_SOCKET", "", 1);
  assert(ud::endpoint_from_environment().empty());
  setenv("SUNHORIZON_RGBD_TRACE_SOCKET", "/data/navigation/evidence/test.sock", 1);
  assert(ud::endpoint_from_environment() == "/data/navigation/evidence/test.sock");
  for (const auto* bad : {"/tmp/test.sock", "/data/navigation/evidence/../bad.sock",
       "/data/navigation/evidence//bad.sock", "/data/navigation/evidence/./bad.sock", "/data/navigation/evidence/bad"}) {
    bool rejected = false;
    try { ud::validate_endpoint(bad); } catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
  }
  unsetenv("SUNHORIZON_RGBD_TRACE_SOCKET");
  assert(ud::double_bits(0.) != ud::double_bits(-0.));
  uint64_t nan_bits = 0x7ff8000000000123ULL; double nan;
  std::memcpy(&nan, &nan_bits, 8); assert(ud::double_bits(nan) == nan_bits);
  assert(ud::double_bits(std::numeric_limits<double>::infinity()) == 0x7ff0000000000000ULL);
  ud::HeaderIdentity headers;
  assert(!headers.observe(100)); assert(headers.observe(100));
  assert(headers.observe(90)); assert(headers.observe(95)); assert(!headers.observe(101));

  ud::Queue<2> tiny; ud::Record a; a.callback_sequence = 1;
  assert(tiny.try_push(a)); a.callback_sequence = 2; assert(tiny.try_push(a));
  assert(!tiny.try_push(a) && tiny.drops() == 1 && tiny.pending() == 2);
  assert(tiny.front()->callback_sequence == 1); tiny.pop();
  assert(tiny.front()->callback_sequence == 2); tiny.pop(); assert(!tiny.front());

  // The identities used by status are the same immutable scalar snapshot as ages.
  sunhorizon_rgbd_adapter::RgbdStatusSnapshot status;
  status.update([](auto& s) { s.belly_depth_stamp = 1.; s.belly_depth_source_ns = 1000000001;
    s.belly_depth_callback_sequence = 1; s.belly_points_stamp = 1.;
    s.belly_points_source_ns = 1000000001; s.belly_points_callback_sequence = 1; });
  status.update([](auto& s) { s.belly_depth_stamp = 2.; s.belly_depth_source_ns = 2000000003;
    s.belly_depth_callback_sequence = 2; });
  const auto mixed = status.snapshot(3., 10.);
  status.update([](auto& s) { s.belly_points_stamp = 2.; s.belly_points_source_ns = 2000000003;
    s.belly_points_callback_sequence = 2; });
  assert(mixed.belly_depth_source_ns == 2000000003 && mixed.belly_points_source_ns == 1000000001);
  assert(mixed.belly_depth_callback_sequence == 2 && mixed.belly_points_callback_sequence == 1);

  auto first = std::make_unique<ud::Queue<>>(); auto second = std::make_unique<ud::Queue<>>();
  {
    ud::Stage stage(*first, 4, 1000000007, 1000000100);
    stage.emit(ud::Event::kPointsPublishBegin);
    // Before a blocked publication returns, entry and begin already exist.
    assert(first->front()->event == ud::Event::kEntry); first->pop();
    assert(first->front()->event == ud::Event::kPointsPublishBegin); first->pop();
  }
  assert(first->front()->event == ud::Event::kFinish && first->front()->outcome == 3); first->pop();

  // Concurrent public pending samples never underflow/exceed capacity.
  ud::Queue<64> threaded;
  std::atomic<bool> finished{false};
  std::thread writer([&] { for (unsigned i = 0; i < 50000; ++i) {
      ud::Record r; r.sequence = i; while (!threaded.try_push(r)) std::this_thread::yield();
    } finished = true; });
  std::thread consumer([&] { while (!finished || threaded.front()) {
      if (threaded.front()) threaded.pop(); else std::this_thread::yield();
    } });
  while (!finished) assert(threaded.pending() <= 64);
  writer.join(); consumer.join();

  const auto path = (std::filesystem::temp_directory_path() / ("upstream-" + std::to_string(getpid()) + ".sock")).string();
  const auto start = std::chrono::steady_clock::now();
  // Missing receiver retains events, no consumer/socket wait during destruction.
  first->try_push(a);
  {
    ud::Exporter exporter(*first, *second, path, "producer");
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    assert(exporter.pending_counts()[0] == 1);
  }
  assert(std::chrono::steady_clock::now() - start < std::chrono::seconds(1));

  // A receiver that never reads eventually fills; nonblocking sender still stops.
  int receiver = socket(AF_UNIX, SOCK_DGRAM, 0); assert(receiver >= 0);
  sockaddr_un addr{}; addr.sun_family = AF_UNIX;
  std::memcpy(addr.sun_path, path.c_str(), path.size() + 1);
  assert(bind(receiver, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);
  for (unsigned i = 0; i < 100; ++i) first->try_push(a);
  {
    ud::Exporter exporter(*first, *second, path, "producer");
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    assert(first->pending() > 0);
    const auto stop = std::chrono::steady_clock::now(); exporter.stop();
    assert(std::chrono::steady_clock::now() - stop < std::chrono::seconds(1));
  }
  close(receiver); unlink(path.c_str());
  std::cout << "exact snapshot, bits, queue, no/full receiver and shutdown checks passed\n";
}
