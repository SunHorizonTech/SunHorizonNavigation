#include "sunhorizon_rgbd_adapter/depth_projection.hpp"
#include <cmath>
#include <stdexcept>
#include "sunhorizon_rgbd_adapter/depth_components.hpp"

namespace sunhorizon_rgbd_adapter {
DepthProjectionTraversal::DepthProjectionTraversal(
    const sensor_msgs::msg::Image& depth, const sensor_msgs::msg::CameraInfo& info,
    double scale, double minimum, double maximum, uint32_t stride,
    const std::vector<uint8_t>* keep, double mask_max_depth)
    : depth_(depth), info_(info), scale_(scale), minimum_(minimum), maximum_(maximum),
      stride_(stride), keep_(keep), mask_max_depth_(mask_max_depth) {
  const auto count = static_cast<std::size_t>(depth.width) * depth.height;
  if (depth.encoding != "16UC1" || !count || count > kMaxDepthComponentPixels ||
      depth.width > depth.step / 2 || depth.data.size() / depth.height < depth.step ||
      info.width != depth.width || info.height != depth.height ||
      info.header.frame_id.empty() || depth.header.frame_id != info.header.frame_id ||
      !std::isfinite(scale) || scale <= 0 || !std::isfinite(minimum) || minimum < 0 ||
      !std::isfinite(maximum) || maximum <= minimum || !stride ||
      !std::isfinite(info.k[0]) || info.k[0] <= 0 ||
      !std::isfinite(info.k[4]) || info.k[4] <= 0 ||
      !std::isfinite(info.k[2]) || !std::isfinite(info.k[5]) ||
      (keep && keep->size() != count) || std::isnan(mask_max_depth) || mask_max_depth <= 0.) {
    throw std::invalid_argument("invalid_depth_projection_input");
  }
  // Subtract before dividing, preserving the original overflow-safe bound.
  sampled_count_ = (1 + (depth.width - 1) / stride) *
      static_cast<std::size_t>(1 + (depth.height - 1) / stride);
}
sensor_msgs::msg::PointCloud2 DepthProjectionTraversal::empty_cloud() const {
  sensor_msgs::msg::PointCloud2 output;
  output.header = depth_.header;
  output.height = 1;
  output.is_bigendian = false;
  output.is_dense = true;
  output.point_step = 12;
  for (std::size_t i = 0; i < 3; ++i) {
    sensor_msgs::msg::PointField field;
    field.name = std::string(1, "xyz"[i]); field.offset = i * 4;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32; field.count = 1;
    output.fields.push_back(field);
  }
  return output;
}
sensor_msgs::msg::PointCloud2 project_depth(
    const sensor_msgs::msg::Image& depth, const sensor_msgs::msg::CameraInfo& info,
    double scale, double minimum, double maximum, uint32_t stride,
    const std::vector<uint8_t>* keep, double mask_max_depth) {
  const DepthProjectionTraversal projection(depth, info, scale, minimum, maximum,
      stride, keep, mask_max_depth);
  auto output = projection.empty_cloud();
  if (projection.sampled_count() > output.data.max_size() / output.point_step)
    throw std::length_error("depth_projection_output_too_large");
  output.data.reserve(projection.sampled_count() * output.point_step);
  projection.for_each_point([&](const std::array<float, 3>& values) {
    const auto bytes = depth_xyz_bytes(values);
    output.data.insert(output.data.end(), bytes.begin(), bytes.end());
    ++output.width;
    return true;
  });
  output.row_step = output.width * output.point_step;
  return output;
}
}
