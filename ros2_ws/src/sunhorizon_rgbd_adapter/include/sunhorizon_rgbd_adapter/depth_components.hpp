#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sunhorizon_rgbd_adapter {

// Counts original image pixels, before pointcloud stride. No dilation or filling.
struct DepthImageView {
  const uint8_t* data{nullptr};
  std::size_t data_size{0};
  uint32_t width{0}, height{0}, step{0};
  bool bigendian{false};
};
struct DepthComponentMask {
  bool valid{false};
  std::string reason;
  std::vector<uint8_t> keep;
  std::size_t valid_pixels{0}, removed_pixels{0}, removed_components{0};
};
constexpr std::size_t kMaxDepthComponentPixels = 4194304;
// Per-caller, single-threaded scratch storage. Capacity grows only for valid
// images within kMaxDepthComponentPixels; contents reset on every call, including
// failures. The returned reference remains valid only until this workspace's
// next call or destruction. No caller may retain/mutate it concurrently.
struct DepthComponentWorkspace {
  std::vector<uint16_t> depth;
  std::vector<std::size_t> queue;
  DepthComponentMask result;
};
const DepthComponentMask& depth_component_mask(
    const DepthImageView& image, double scale_m, double min_depth_m,
    double max_depth_m, double neighbor_delta_m, std::size_t minimum_component_pixels,
    DepthComponentWorkspace& workspace);
// The stronger threshold applies only to whole components entirely within
// near_max_depth_m. Zero disables it, preserving the original API behavior.
const DepthComponentMask& depth_component_mask(
    const DepthImageView& image, double scale_m, double min_depth_m,
    double max_depth_m, double neighbor_delta_m, std::size_t minimum_component_pixels,
    DepthComponentWorkspace& workspace, std::size_t near_minimum_component_pixels,
    double near_max_depth_m);
DepthComponentMask depth_component_mask(
    const DepthImageView& image, double scale_m, double min_depth_m,
    double max_depth_m, double neighbor_delta_m, std::size_t minimum_component_pixels);
DepthComponentMask depth_component_mask(
    const DepthImageView& image, double scale_m, double min_depth_m,
    double max_depth_m, double neighbor_delta_m, std::size_t minimum_component_pixels,
    std::size_t near_minimum_component_pixels, double near_max_depth_m);

}  // namespace sunhorizon_rgbd_adapter
