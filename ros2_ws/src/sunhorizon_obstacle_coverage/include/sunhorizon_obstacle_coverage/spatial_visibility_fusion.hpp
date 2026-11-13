#pragma once

#include "sunhorizon_obstacle_coverage/camera_freshness_policy.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <variant>
#include <vector>

#include "sunhorizon_obstacle_coverage/region_depth_support.hpp"

namespace sunhorizon_obstacle_coverage::spatial_visibility {

constexpr std::size_t kMaximumCells = 4096;
constexpr std::size_t kMaximumSources = 3;
constexpr std::size_t kMaximumRasterElements = 4 * 1024 * 1024;
constexpr std::size_t kMaximumPointReturns = 1024 * 1024;
constexpr std::size_t kMaximumChecks = 16 * 1024 * 1024;
// Historical default remains unchanged for legacy producers.
constexpr int64_t kMaximumEvidenceAgeNs = kDefaultCameraAgeNs;
constexpr int64_t kSupportedEvidenceAgeNs = kOneSecondCameraAgeNs;
inline bool supported_evidence_age(int64_t age) {
  return (age > 0 && age <= kSupportedCameraAgeNs) || age == kOneSecondCameraAgeNs;
}

struct AxisAlignedCell {
  uint32_t id{0};
  std::array<double, 3> min{}, max{};  // metres in Query.common_frame
};
struct SourceContract {
  std::string source_id;
  std::string expected_epoch;  // Fixed by owner, never inferred from the sample.
  bool required_for_evaluation{false};
};
struct Query {
  std::vector<AxisAlignedCell> cells;
  std::string common_frame, query_id, contract_id;
  int64_t query_stamp_ns{0};
  std::vector<SourceContract> source_contracts;
};
struct RigidTransform {
  std::array<double, 9> rotation{1.,0.,0.,0.,1.,0.,0.,0.,1.};
  std::array<double, 3> translation{};
  std::string source_frame, common_frame;
  int64_t source_stamp_ns{0}, query_stamp_ns{0};
  // Must map source(t_source) to common(t_query). When times differ, the
  // caller attests actual motion compensation, not a latest/static TF guess.
};
// A verified SelfMatched endpoint is not an external obstacle. Its surface
// still occludes the space beyond it. ModelConflict retains the measured
// endpoint/ray exactly like External; a disagreeing model cannot delete it.
enum class ReturnKind : uint8_t { kExternal, kSelfMatched, kInvalid, kModelConflict };
struct DepthRaster {
  uint32_t width{0}, height{0};
  double fx{0}, fy{0}, cx{0}, cy{0};
  const float* depth_m{nullptr};  // optical axial Z, not Euclidean range
  std::size_t depth_elements{0}, row_stride_elements{0};
  const ReturnKind* return_kinds{nullptr};  // Null means all External (fail-safe).
  std::size_t kind_elements{0};  // Same stride as depth.
  RigidTransform optical_to_common;
  double minimum_depth_m{0}, maximum_depth_m{0}, clear_margin_m{0};
  RegionCalibrationContract calibration;
};
struct PointReturn {
  std::array<double, 3> position{};  // source-frame positive measured endpoint
  ReturnKind kind{ReturnKind::kExternal};
};
struct PointReturns {
  std::vector<PointReturn> points;
  RigidTransform sensor_to_common;
  double minimum_range_m{0}, maximum_range_m{0};
  bool metric_returns_verified{false};
  bool source_time_and_transform_verified{false};
};
struct Source {
  std::string source_id, epoch, frame_id;
  uint64_t frame_sequence{0};
  int64_t source_stamp_ros_ns{0}, local_receipt_ros_ns{0}, local_receipt_steady_ns{0};
  int64_t expiry_budget_ns{0};
  bool required_for_evaluation{false};  // Must match pinned SourceContract.
  std::variant<DepthRaster, PointReturns> data;
};
enum class CellState : uint8_t { kUnknown, kFree, kOccupied };
enum class Aggregate : uint8_t { kHardInvalid, kVisibilityWait, kClear, kOccupied };
enum class SourceAssessment : uint8_t { kFresh, kUnavailableExpired, kUnavailableMissing, kInvalid };
struct Counts {
  std::size_t tested_samples{0}, intersecting_rays{0}, clear_rays{0};
  std::size_t obstacle_returns{0}, invalid_returns{0}, self_returns{0};
  std::size_t self_blocked_rays{0}, front_occluded_rays{0}, margin_band_rays{0};
  std::size_t model_conflict_returns{0};
};
struct CellEvidence {
  uint32_t id{0};
  CellState state{CellState::kUnknown};
  uint8_t free_sources{0}, occupied_sources{0}, unknown_sources{0}, unavailable_sources{0};
  bool conflicting_sources{false};
  std::array<Counts, kMaximumSources> counts{};  // Query.source_contracts order.
};
struct SourceEvidence {
  std::string source_id, expected_epoch, epoch, reason;
  uint64_t frame_sequence{0};
  int64_t source_stamp_ros_ns{0}, local_receipt_ros_ns{0}, local_receipt_steady_ns{0};
  int64_t remaining_valid_ns{0};
  bool required_for_evaluation{false};
  SourceAssessment assessment{SourceAssessment::kUnavailableMissing};
};
struct Result {
  std::string query_id, contract_id, reason;
  int64_t query_stamp_ns{0};
  Aggregate aggregate{Aggregate::kHardInvalid};
  bool evaluation_complete{false};
  std::vector<CellEvidence> cells;
  std::vector<SourceEvidence> sources;
  int64_t min_remaining_valid_ns{0};
  std::size_t checks{0};
};

// Stateless, bounded, no ROS/clocks/I/O. Inputs are immutable for this call.
// Free is sampled-ray support for the whole projected cell, not proof of a
// continuous empty volume, minimum detectable obstacle size or motion safety.
// Mandatory stale/missing/invalid sources hard-fail; optional unusable sources
// cannot contribute any spatial evidence. Original upstream health gates stay
// independent. Re-evaluation never changes epoch/generation/receipt timestamps.
// The wrapper must enforce epoch/sequence high water and clock rollback across
// calls, real source-time TF/self-match attestations, and recovery policy.
Result evaluate(const Query&, const std::vector<Source>&, int64_t now_ros_ns, int64_t now_steady_ns);

}  // namespace sunhorizon_obstacle_coverage::spatial_visibility
