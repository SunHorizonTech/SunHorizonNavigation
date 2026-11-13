#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace sunhorizon_rgbd_adapter {

// Shared startup/core limit. Separate local volumes must not become a hull over
// the robot footprint. Raising capacity does not enable any new volume.
inline constexpr std::size_t kMaxSelfExclusionBoxes = 8;

struct ExclusionBox {
  std::array<double, 3> minimum;
  std::array<double, 3> maximum;
};

// Row-major rigid transform from the input cloud frame to the exclusion frame.
struct ExclusionTransform {
  std::array<double, 9> rotation{1., 0., 0., 0., 1., 0., 0., 0., 1.};
  std::array<double, 3> translation{0., 0., 0.};
};

struct ExclusionResult {
  std::optional<sensor_msgs::msg::PointCloud2> cloud;
  std::size_t input_count{0};
  std::size_t removed_count{0};
  std::size_t retained_count{0};
  std::string reason;
  // Optional per-record CAD uncertainty: retained finite endpoints still mark,
  // but never contribute ground support/clearing. Compacted with whole records.
  std::vector<uint8_t> ground_blocked;
  std::vector<std::array<float,3>> ray_origins;
  std::vector<uint64_t> sample_ids;
};

// Local self volumes only: finite, strictly positive spans <= 1 m per axis,
// wholly within 2 m of the base origin. Bounds are inclusive.
bool valid_exclusion_box(const ExclusionBox& box);

// A low removed fraction guards against an unexpectedly broad crop or bad TF.
bool exclusion_fraction_allowed(std::size_t removed, std::size_t input, double limit);

// Missing transforms and malformed layouts produce no output, never a raw fallback.
// Finite points inside the box are removed. Nonfinite points remain untouched.
// Retained point records, fields, frame and timestamp are byte-preserved; rows are
// compacted into an unorganized cloud, without input row padding.
// Union of at most kMaxSelfExclusionBoxes independently bounded self volumes;
// never their hull. Bounds require separate physical evidence before enabling.
ExclusionResult exclude_self_volumes(
    const sensor_msgs::msg::PointCloud2& input, const std::vector<ExclusionBox>& boxes,
    const std::optional<ExclusionTransform>& cloud_to_base);

// Each local body volume may follow a different articulated link. Every
// transform must describe the cloud's measurement time; an unavailable link
// rejects the whole cloud, never a partial/raw fallback. No union hull is used.
// Optional shadow testing assumes the cloud frame origin is the optical sensor
// origin. Only points within shadow_sensor_range_m may be removed for having
// a self volume between that origin and the measured point. Zero disables it.
ExclusionResult exclude_self_framed_volumes(
    const sensor_msgs::msg::PointCloud2& input, const std::vector<ExclusionBox>& boxes,
    const std::vector<std::optional<ExclusionTransform>>& cloud_to_volume_frames,
    double shadow_sensor_range_m = 0.,
    const std::vector<uint8_t>* ground_blocked = nullptr,
    const std::vector<std::array<float,3>>* ray_origins = nullptr,
    const std::vector<uint64_t>* sample_ids = nullptr);

ExclusionResult exclude_self_points(
    const sensor_msgs::msg::PointCloud2& input, const ExclusionBox& box,
    const std::optional<ExclusionTransform>& cloud_to_base);

}  // namespace sunhorizon_rgbd_adapter
