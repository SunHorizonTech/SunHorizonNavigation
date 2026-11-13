#include "sunhorizon_rgbd_adapter/region_depth_pairing.hpp"
#define CHECK(condition) do { if (!(condition)) throw std::runtime_error(#condition); } while (false)
#include <limits>
#include <iostream>
using namespace sunhorizon_rgbd_adapter;
int main() {
  using Pairing = RegionExactPairing<int, int>;
  const auto object = std::make_shared<const int>(3);
  auto key = [](int64_t n) { return RegionFrameKey{n, "optical", 640, 480}; };
  Pairing p;
  CHECK(p.image({key(10), object, 1, 20, 30}));
  CHECK(!p.match(key(10)) && p.pending()->sequence == 1);
  CHECK(p.info(key(9), "cal", object) && !p.match(key(10)));
  CHECK(p.info(key(10), "cal", object));
  CHECK(p.match(key(10)) && !p.match(key(10))->conflict);
  p.clear(); CHECK(!p.pending());
  // No approximate match, and replacing pending cannot bring the old frame back.
  CHECK(p.image({key(11), object, 2, 21, 31}));
  CHECK(p.image({key(12), object, 3, 22, 32}));
  CHECK(p.info(key(11), "cal", object));
  CHECK(p.pending()->key.stamp_ns == 12 && !p.match(key(12)));
  auto wrong = key(12); wrong.frame = "another";
  p.info(wrong, "cal", object); CHECK(!p.match(key(12)));
  wrong = key(12); wrong.width = 639;
  p.info(wrong, "cal", object); CHECK(!p.match(key(12)));
  p.info(key(12), "cal", object); p.info(key(12), "cal", object);
  CHECK(!p.match(key(12))->conflict);
  p.info(key(12), "different", object); p.info(key(12), "cal", object);
  CHECK(p.match(key(12))->conflict); // Conflict is sticky; no last-writer wins.
  CHECK(!p.image({key(12), object, 4, 23, 33}) && !p.pending());
  CHECK(!p.image({key(11), object, 5, 24, 34}) && !p.pending());
  for (int i = 20; i < 60; ++i) p.info(key(i), "cal", object);
  CHECK(p.info_count() == 8 && !p.match(key(12)) && p.match(key(59)));
  wrong = key(60); wrong.width = 4097;
  CHECK(!p.info(wrong, "cal", object));
  CHECK(!p.info(key(60), std::string(8193, 'x'), object));
  CHECK(!p.image({wrong, object, 6, 25, 35}) && !p.pending());
  // Exact age boundary passes; either local elapsed clock or source age can fail.
  CHECK(region_times_fresh(1000000000, 1000000000, 100, 1300000000, 300000100, .3));
  CHECK(!region_times_fresh(1000000000, 1000000000, 100, 1300000001, 200, .3));
  CHECK(!region_times_fresh(1000000000, 1000000000, 100, 1000000000, 300000101, .3));
  CHECK(!region_times_fresh(1000000001, 1000000000, 100, 1000000001, 200, .3));
  CHECK(!region_times_fresh(1000000000, 1000000001, 100, 1000000000, 200, .3));
  CHECK(!region_times_fresh(1000000000, 1000000000, 100, 1000000000, 99, .3));
  CHECK(!region_times_fresh(1, 1, 1, INT64_MAX, INT64_MAX, .3));
  CHECK(!region_times_fresh(0, 0, 1, 1, 1, .3));
  RegionClockGuard clock;
  CHECK(clock.observe(100, 100)); CHECK(clock.observe(100, 101));
  CHECK(!clock.observe(99, 102)); CHECK(!clock.observe(200, 200));
  RegionClockGuard steady_clock;
  CHECK(steady_clock.observe(1, 2)); CHECK(!steady_clock.observe(2, 1));
  RegionCanonical a, b, zero, negzero;
  a.add("a").add("bc"); b.add("ab").add("c"); CHECK(a.str() != b.str());
  zero.number(0.); negzero.number(-0.); CHECK(zero.str() == negzero.str());
  CHECK(zero.str() == "23:0.00000000000000000e+00");
  bool threw = false;
  try { zero.number(std::numeric_limits<double>::quiet_NaN()); }
  catch (const std::invalid_argument&) { threw = true; } CHECK(threw);
  threw = false;
  try { RegionCanonical{}.add(std::string(8193, 'x')); }
  catch (const std::invalid_argument&) { threw = true; } CHECK(threw);
  std::cout << "region pairing: exact identity, ordering, bounded cache, poison, clocks, canonical passed\n";
}
