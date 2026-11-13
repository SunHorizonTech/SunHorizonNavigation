#include "sunhorizon_navigation_runtime/detail/peripheral_startup.hpp"
#include "sunhorizon_perception_runtime/spatial_startup_gate.hpp"
#include <iostream>
#include <stdexcept>

using Startup = sunhorizon_navigation_runtime::detail::PeripheralStartup;
using Spatial = sunhorizon_perception_runtime::SpatialStartupGate;
void check(bool value) {if (!value) throw std::runtime_error("startup motion hold contract");}
int main() {
  using namespace std::chrono;
  const Startup::Clock::time_point begin{seconds(1)};
  const auto deadline = begin + seconds(1);
  const auto ns = [](auto time) {return duration_cast<nanoseconds>(time.time_since_epoch()).count();};
  Startup startup(deadline), not_started(deadline), late(deadline);
  Spatial spatial; spatial.defer(); spatial.note_input();
  check(!not_started.should_poll(true,begin)); // Hold cannot issue new STARTUP.
  check(startup.advance(true,true,{},begin)==Startup::Step::Request);
  check(startup.should_poll(true,begin+milliseconds(100)));
  check(startup.advance(true,true,{},begin+milliseconds(100))==Startup::Step::Waiting);
  check(startup.advance(true,true,true,begin+milliseconds(200))==Startup::Step::Complete);
  spatial.owner_ready(ns(deadline));
  check(spatial.snapshot().owner_ready && !spatial.snapshot().opened);
  const auto opened=ns(begin+milliseconds(300));
  check(!spatial.qualify(false,opened,opened)); // Startup success is not coverage health.
  check(spatial.qualify(true,opened,opened));
  check(!spatial.allows(opened-1,opened+1,opened+1)); // Cached source cannot qualify.
  check(!spatial.allows(opened+1,opened-1,opened+1));
  check(!spatial.allows(opened+1,opened+1,opened-1));
  check(!spatial.published(opened+1,opened+1,opened+1,1,ns(deadline)));
  check(spatial.published(opened+1,opened+1,opened+1,1,opened+2));
  check(!spatial.published(opened+3,opened+3,opened+3,2,opened+4));
  check(startup.advance(true,true,{},begin+milliseconds(400))==Startup::Step::Complete);
  bool duplicate=false; try {spatial.owner_ready(ns(deadline));} catch(const std::logic_error&) {duplicate=true;}
  check(duplicate);
  check(late.advance(true,true,{},begin)==Startup::Step::Request);
  check(late.should_poll(true,deadline-nanoseconds(1)));
  check(!late.should_poll(true,deadline));
  check(!late.should_poll(true,deadline+seconds(30))); // No renewed grace under hold.
  check(!startup.should_poll(true,deadline)); // Completion does not extend first publication.
  check(late.should_poll(false,deadline)); // Ordinary startup retains timeout failure.
  bool expired=false; try {late.advance(true,true,true,deadline);} catch(const std::runtime_error&) {expired=true;}
  check(expired);
  std::cout << "startup hold, original deadline and original RGB-D contracts passed\n";
}
