#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace sunhorizon_obstacle_coverage {

// These attestations belong to the caller. The pure core neither calibrates nor
// substitutes its own source-age/TF gates. Unverified distortion is not pinhole.
struct RegionCalibrationContract {
  bool rectified_pinhole_verified{false};
  bool metric_axial_depth_verified{false};
  bool depth_intrinsics_registration_verified{false};
  bool optical_to_base_extrinsics_verified{false};
  bool source_time_and_transform_verified{false};
};

struct RegionDepthInput {
  std::array<double, 3> box_min{};  // base frame, metres
  std::array<double, 3> box_max{};
  // Integer (u,v) denotes the centre of a pixel. The image's outer boundaries
  // are exactly [-0.5,width-0.5] x [-0.5,height-0.5]; no additional tolerance.
  std::uint32_t width{0}, height{0};
  double fx{0}, fy{0}, cx{0}, cy{0};
  // Immutable for this call; elements are optical axial Z in metres, NOT range.
  const float* depth_m{nullptr};
  std::size_t depth_elements{0}, row_stride_elements{0};
  std::array<double, 9> optical_to_base_rotation{};  // row major
  std::array<double, 3> optical_to_base_translation{};
  double minimum_depth_m{0}, maximum_depth_m{0};
  double clear_margin_m{0};  // optical axial Z beyond box exit, NOT Euclidean ray distance
  RegionCalibrationContract contract;
};

enum class RegionSupportState { kUnknown, kClearSupported, kObstacleSupported };

enum class RegionEvaluationCode { kRejected = 0, kClear = 1, kObstacle = 2, kIncompleteRays = 3 };
enum class RegionDisposition { kHardFault = 0, kClear = 1, kGeometryHold = 2 };

struct RegionDepthSupport {
  RegionSupportState state{RegionSupportState::kUnknown};
  std::string reason;
  std::size_t tested_pixels{0}, intersecting_rays{0};
  std::size_t clear_rays{0}, obstacle_rays{0}, invalid_depth_rays{0};
  std::size_t front_occluded_rays{0}, margin_band_rays{0};
  // Appended evidence only: no permission to move or resume. HardFault means
  // ineligible for geometry-only recovery, not necessarily a hardware fault.
  bool evaluation_complete{false};
  RegionEvaluationCode evaluation_code{RegionEvaluationCode::kRejected};
  RegionDisposition disposition{RegionDisposition::kHardFault};
};

// Bounded, no ROS/I/O/publishers; every pixel centre in the conservative
// projected bounding rectangle is tested, with no stride/subsampling/median.
// ANY unknown ray prevents a region from being declared supported, even when
// other rays see an obstacle. Obstacle counts remain available to the caller.
// This proves support at sampled rays only, never continuous-volume visibility,
// detection of subpixel objects, free space behind a surface, or motion safety.
RegionDepthSupport evaluate_region_depth_support(const RegionDepthInput& input);

}  // namespace sunhorizon_obstacle_coverage
