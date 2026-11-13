#pragma once

#include <string>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace sunhorizon_velocity_supervisor {

inline rcl_interfaces::msg::ParameterDescriptor read_only_parameter(
    const std::string& description) {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.read_only = true;
  return descriptor;
}

}  // namespace sunhorizon_velocity_supervisor
