#pragma once
#include <array>
#include <algorithm>
#include <cstring>
#include <cstddef>
#include <vector>
#include <limits>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace sunhorizon_rgbd_adapter {
// Validated, borrowed traversal used by materialized and fused projection.
// Each callback sees the exact FLOAT32 XYZ the materialized cloud would contain.
// Returning false stops traversal; no cloud or per-pixel workspace is allocated.
class DepthProjectionTraversal {
 public:
  DepthProjectionTraversal(const sensor_msgs::msg::Image& depth,
      const sensor_msgs::msg::CameraInfo& info, double scale, double minimum,
      double maximum, uint32_t stride, const std::vector<uint8_t>* keep = nullptr,
      double mask_max_depth = std::numeric_limits<double>::infinity());
  std::size_t sampled_count() const noexcept { return sampled_count_; }
  sensor_msgs::msg::PointCloud2 empty_cloud() const;
  template<class Visitor> bool for_each_point(Visitor&& visitor) const {
    return for_each_sample([&](const std::array<float, 3>& values, std::size_t) {return visitor(values);});
  }
  // Original raster identity, before invalid/masked/self compaction.
  template<class Visitor> bool for_each_sample(Visitor&& visitor) const {
    for (uint32_t row = 0; row < depth_.height; row += stride_) {
      for (uint32_t col = 0; col < depth_.width; col += stride_) {
        const auto offset = static_cast<std::size_t>(row) * depth_.step + col * 2;
        const uint16_t raw = depth_.is_bigendian ?
            (uint16_t(depth_.data[offset]) << 8) | depth_.data[offset + 1] :
            uint16_t(depth_.data[offset]) | (uint16_t(depth_.data[offset + 1]) << 8);
        const double z = raw * scale_;
        if (!raw || z < minimum_ || z > maximum_) continue;
        if (keep_ && z <= mask_max_depth_ &&
            !(*keep_)[static_cast<std::size_t>(row) * depth_.width + col]) continue;
        const std::array<float, 3> values{{
            static_cast<float>((col - info_.k[2]) * z / info_.k[0]),
            static_cast<float>((row - info_.k[5]) * z / info_.k[4]), static_cast<float>(z)}};
        if (!visitor(values, static_cast<std::size_t>(row) * depth_.width + col)) return false;
      }
    }
    return true;
  }
  std::size_t point_count() const {
    std::size_t count = 0;
    for_each_point([&](const std::array<float, 3>&) { ++count; return true; });
    return count;
  }
 private:
  const sensor_msgs::msg::Image& depth_;
  const sensor_msgs::msg::CameraInfo& info_;
  double scale_, minimum_, maximum_;
  uint32_t stride_;
  const std::vector<uint8_t>* keep_;
  double mask_max_depth_;
  std::size_t sampled_count_;
};
// Encodes only one record, matching project_depth's little-endian FLOAT32 XYZ.
inline std::array<uint8_t, 12> depth_xyz_bytes(const std::array<float, 3>& values) {
  static_assert(sizeof(float) == 4, "PointCloud2 FLOAT32 requires four-byte float");
  std::array<uint8_t, 12> bytes;
  std::memcpy(bytes.data(), values.data(), bytes.size());
  const uint16_t endian = 1;
  if (*reinterpret_cast<const uint8_t*>(&endian) == 0) {
    for (std::size_t i = 0; i < 3; ++i)
      std::reverse(bytes.begin() + i * 4, bytes.begin() + (i + 1) * 4);
  }
  return bytes;
}
// Rectified/aligned depth projection shared by normalization and obstacle-only
// processing. Throws on invalid layout/intrinsics/mask, never returns raw data.
// A finite mask_max_depth applies the supplied component mask only up to that
// optical depth, preserving distant thin objects regardless of component size.
sensor_msgs::msg::PointCloud2 project_depth(
    const sensor_msgs::msg::Image& depth, const sensor_msgs::msg::CameraInfo& info,
    double scale, double minimum, double maximum, uint32_t stride,
    const std::vector<uint8_t>* keep = nullptr,
    double mask_max_depth = std::numeric_limits<double>::infinity());
}
