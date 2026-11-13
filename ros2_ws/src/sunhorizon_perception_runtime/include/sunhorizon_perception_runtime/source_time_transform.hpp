#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <tf2/buffer_core.h>

namespace sunhorizon_perception_runtime::detail {
// The direct owner already supplies this shared BufferCore. A zero-wait query
// needs one lookup, not the ROS timeout wrapper's canTransform plus lookup.
// Keep the exact source time, BufferCore interpolation and original exceptions;
// never substitute latest-time TF or create another transform cache.
inline geometry_msgs::msg::TransformStamped source_time_transform(
    const tf2::BufferCore& buffer, const std::string& target,
    const std::string& source, int64_t source_ns) {
  if(source_ns<=0)throw std::invalid_argument("source_time_transform_requires_positive_source");
  return buffer.lookupTransform(target,source,tf2::TimePoint(tf2::Duration(source_ns)));
}
}  // namespace sunhorizon_perception_runtime::detail
