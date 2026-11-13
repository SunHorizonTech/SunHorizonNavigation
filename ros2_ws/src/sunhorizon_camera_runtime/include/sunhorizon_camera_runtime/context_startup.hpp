#pragma once

#include <utility>

namespace sunhorizon_camera_runtime
{

// Apply the explicit SDK discovery policy before the context reaches any
// pipeline or worker. SDK exceptions propagate and destroy the local context.
template<typename ContextFactory>
auto create_context_with_network_enumeration(
  bool enabled, ContextFactory && factory)
{
  auto context = std::forward<ContextFactory>(factory)();
  context->enableNetDeviceEnumeration(enabled);
  return context;
}

}  // namespace sunhorizon_camera_runtime
