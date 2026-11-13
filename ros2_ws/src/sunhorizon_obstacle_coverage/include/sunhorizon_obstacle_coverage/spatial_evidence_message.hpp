#pragma once

#include <sunhorizon_navigation_interfaces/msg/spatial_coverage_evidence.hpp>
#include "sunhorizon_obstacle_coverage/spatial_evidence_tracker.hpp"

namespace sunhorizon_obstacle_coverage {

// Exact integer conversion. Malformed ROS stamps become invalid input, never
// normalized into a plausible newer stamp. This header does not read clocks.
inline SpatialCoverageObservation spatial_evidence_observation(
    const sunhorizon_navigation_interfaces::msg::SpatialCoverageEvidence& input) {
  SpatialCoverageObservation out;
  out.schema_version = input.schema_version;
  out.contract = input.contract; out.query_digest = input.query_digest;
  out.producer_epoch = input.producer_epoch; out.clock_boot_id = input.clock_boot_id;
  out.frame_id = input.header.frame_id; out.reason = input.reason;
  if (input.header.stamp.sec > 0 && input.header.stamp.nanosec < 1000000000U) {
    out.header_ros_ns = static_cast<std::int64_t>(input.header.stamp.sec) * 1000000000LL +
        input.header.stamp.nanosec;
  }
  out.anchor_source_ros_ns = input.anchor_source_ros_ns;
  out.evaluation_steady_ns = input.evaluation_steady_ns;
  out.anchor_frame_sequence = input.anchor_frame_sequence;
  out.evaluation_sequence = input.evaluation_sequence;
  out.input_generation = input.input_generation;
  out.evaluation_complete = input.evaluation_complete;
  out.total_cells = input.total_cells; out.free_cells = input.free_cells;
  out.occupied_cells = input.occupied_cells; out.unknown_cells = input.unknown_cells;
  out.sources.reserve(input.sources.size());
  for (const auto& s : input.sources) {
    out.sources.push_back({s.source_id, s.contribution, s.producer_epoch, s.frame_id,
        s.frame_sequence, s.source_ros_ns, s.first_receipt_ros_ns, s.first_receipt_steady_ns,
        s.valid_until_ros_ns, s.valid_until_steady_ns, s.geometry_valid});
  }
  return out;
}

}  // namespace sunhorizon_obstacle_coverage
