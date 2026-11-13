#pragma once

#include "sunhorizon_rgbd_adapter/dynamic_self_model.hpp"

namespace sunhorizon_rgbd_adapter {
enum class SpatialLegAvailability {ExactReady, FuturePending};
// Observational only; populated from the existing exact/latest lookups.
struct SpatialLegPendingTiming {
  int first_link_index{-1};int64_t source_query_ns{0},latest_upper_ns{0};
};

// Nonblocking admission only. A latest transform is never returned or used for
// geometry; its timestamp only distinguishes a missing future upper bound.
inline SpatialLegAvailability spatial_leg_availability(tf2_ros::Buffer& buffer,
    const std::vector<dynamic_self_model::AssetSpec>& specs, int64_t source_ns,
    rcl_clock_type_t clock_type, SpatialLegPendingTiming* timing=nullptr) {
  if (timing) *timing={};
  if (source_ns <= 0 || specs.size() != 12)
    throw std::invalid_argument("spatial_leg_pairing_contract");
  const rclcpp::Time source(source_ns, clock_type);
  const auto zero = rclcpp::Duration::from_nanoseconds(0);
  bool future = false;
  for (std::size_t index=0;index<specs.size();++index) {
    const auto& spec=specs[index];
    try {
      (void)dynamic_self_model::rigid(buffer.lookupTransform(
          "base_link", spec.link_name, source, zero).transform);
    } catch (const tf2::ExtrapolationException&) {
      const auto latest = buffer.lookupTransform("base_link", spec.link_name,
          rclcpp::Time(int64_t(0), clock_type), zero);
      const auto latest_ns = rclcpp::Time(latest.header.stamp, clock_type).nanoseconds();
      if (latest_ns > 0 && latest_ns < source_ns) {
        if (timing && !future) *timing={static_cast<int>(index),source_ns,latest_ns};
        future = true;
      } else {
        // TF may have arrived between the exact failure and latest read. Only
        // another successful exact lookup can admit that race; past remains hard.
        (void)dynamic_self_model::rigid(buffer.lookupTransform(
            "base_link", spec.link_name, source, zero).transform);
      }
    }
  }
  return future ? SpatialLegAvailability::FuturePending : SpatialLegAvailability::ExactReady;
}
} // namespace sunhorizon_rgbd_adapter
