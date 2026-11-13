#include "sunhorizon_camera_runtime/exact_stereo_pairing.hpp"
#include <memory>
#include <stdexcept>
using namespace sunhorizon_camera_runtime;
void require(bool x) { if (!x) throw std::runtime_error("stereo pairing contract"); }
int main()
{
  ExactStereoPairing<std::shared_ptr<int>> p;
  auto frame = [](int value) { return std::make_shared<int>(value); };
  require(!p.insert(2, 10, frame(3))[0]);
  require(!p.insert(0, 10, frame(1))[0]);
  require(!p.insert(0, 10, frame(99))[0]);
  require(!p.insert(1, 11, frame(20))[0]); // Wrong stamp cannot complete 10.
  auto ready = p.insert(1, 10, frame(2));
  require(ready[0] && *ready[0] == 1 && *ready[1] == 2 && *ready[2] == 3);
  require(!p.insert(2, 10, frame(3))[0]); // No duplicate completion.
  require(!p.insert(1, 0, frame(1))[0]);
  require(!p.insert(3, 12, frame(1))[0]);
  for (int i = 12; i < 100; ++i) {
    require(!p.insert(0, i, frame(i))[0]);
    require(p.pending() <= 4); // Missing IR cannot grow memory or block depth.
  }
  require(p.evicted() > 0);
  require(!p.insert(2, 99, frame(3))[0]);
  require(p.insert(1, 99, frame(2))[0] != nullptr);
  require(p.pending() == 0);
  require(!p.insert(1, 98, frame(2))[0]); // A late older pair never emits.
}
