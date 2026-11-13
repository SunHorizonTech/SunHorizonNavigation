#include "sunhorizon_camera_runtime/optional_shadow.hpp"
#include <stdexcept>
#include <vector>

using namespace sunhorizon_camera_runtime;
void require(bool value) { if (!value) throw std::runtime_error("shadow isolation failed"); }

int main()
{
  std::vector<int> order;
  const auto primary = [&]() { order.push_back(1); };
  require(publish_primary_then_shadow(primary, [&]() { order.push_back(2); }) ==
    ShadowResult::kComplete);
  require(order == std::vector<int>({1, 2}));
  // Missing or mismatched IR must not suppress this or the next raw depth frame.
  for (int i = 0; i < 3; ++i) {
    require(publish_primary_then_shadow(primary, []() {
      throw std::runtime_error("missing IR or stereo timestamp mismatch");
    }) == ShadowResult::kFailed);
  }
  require(publish_primary_then_shadow(primary, []() { throw 1; }) == ShadowResult::kFailed);
  require(publish_primary_then_shadow(primary, []() {}) == ShadowResult::kComplete);
  require(order.size() == 7);
  // A real primary sensor failure still escapes to the runtime fail-stop handler.
  bool shadow_called = false, primary_failure = false;
  try {
    publish_primary_then_shadow([]() { throw std::runtime_error("invalid depth"); },
      [&]() { shadow_called = true; });
  } catch (const std::runtime_error &) { primary_failure = true; }
  require(primary_failure && !shadow_called);

  // A newly added IR Image/receipt publication error is optional even after
  // quality depth has published. Continue raw publication on the next callback.
  for (int publication = 0; publication < 3; ++publication) {
    bool raw_published = false, quality_published = false;
    const auto result = publish_primary_then_shadow([&]() { raw_published = true; }, [&]() {
      quality_published = true;
      for (int current = 0; current < 3; ++current) {
        if (current == publication) throw std::runtime_error("IR observation publication");
      }
    });
    require(raw_published && quality_published && result == ShadowResult::kFailed);
    raw_published = false;
    require(publish_primary_then_shadow([&]() { raw_published = true; }, []() {}) ==
      ShadowResult::kComplete);
    require(raw_published);
  }
}
