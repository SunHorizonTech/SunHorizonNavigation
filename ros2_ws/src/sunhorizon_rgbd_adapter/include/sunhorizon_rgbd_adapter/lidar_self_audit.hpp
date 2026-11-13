#pragma once
#include <array>
#include <vector>
#include <cstdint>
#include "sunhorizon_obstacle_coverage/dynamic_self_geometry.hpp"
namespace sunhorizon_rgbd_adapter {
template<class Work> bool try_lidar_diagnostic(Work&& work) noexcept {
  try {work();return true;}catch(...){return false;}
}
struct LidarSelfAuditPoint {
  uint64_t sample_id{0}; // original raw record, never retained/finite index
  int64_t source_ns{0};
  std::array<float,3> position{},ray_origin{};
  sunhorizon_obstacle_coverage::dynamic_self_geometry::Result result;
  std::vector<sunhorizon_obstacle_coverage::dynamic_self_geometry::PreparedModel::DiagnosticLink> links;
};
// At most 32 per Self/External/Conflict result, selected by horizontal distance
// in the lower source hemisphere. Diagnostic selection has no filter authority.
struct LidarSelfAudit {std::vector<LidarSelfAuditPoint> points;};
}
