#include "sunhorizon_camera_runtime/context_startup.hpp"

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
void require(bool condition)
{
  if (!condition) throw std::runtime_error("context startup contract failed");
}

struct SdkFailure {int code;};

struct FakeContext
{
  std::vector<std::string> & events;
  bool fail;
  FakeContext(std::vector<std::string> & trace, bool reject)
  : events(trace), fail(reject) {events.push_back("context");}
  ~FakeContext() {events.push_back("destroy");}
  void enableNetDeviceEnumeration(bool enabled)
  {
    events.push_back(enabled ? "network_on" : "network_off");
    if (fail) throw SdkFailure{37};
  }
};
}  // namespace

int main()
{
  using sunhorizon_camera_runtime::create_context_with_network_enumeration;
  for (bool enabled : {false, true}) {
    std::vector<std::string> events;
    auto context = create_context_with_network_enumeration(enabled, [&] {
      return std::make_shared<FakeContext>(events, false);
    });
    events.push_back("workers");
    events.push_back("bind");
    require(context != nullptr);
    require(events == std::vector<std::string>{
      "context", enabled ? "network_on" : "network_off", "workers", "bind"});
  }

  // Rejection must never return a usable context to pipeline/worker startup.
  std::vector<std::string> events;
  bool rejected = false;
  try {
    auto context = create_context_with_network_enumeration(false, [&] {
      return std::make_shared<FakeContext>(events, true);
    });
    events.push_back("workers");
  } catch (const SdkFailure & error) {
    rejected = error.code == 37;
  }
  require(rejected);
  require(events == std::vector<std::string>{"context", "network_off", "destroy"});

  rejected = false;
  try {
    auto context = create_context_with_network_enumeration(false,
      []() -> std::shared_ptr<FakeContext> {throw SdkFailure{41};});
    events.push_back("workers");
  } catch (const SdkFailure & error) {
    rejected = error.code == 41;
  }
  require(rejected);
  require(events == std::vector<std::string>{"context", "network_off", "destroy"});
  std::cout << "context enumeration ordering and startup failure tests passed\n";
}
