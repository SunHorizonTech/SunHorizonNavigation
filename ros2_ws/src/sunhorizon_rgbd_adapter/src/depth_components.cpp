#include "sunhorizon_rgbd_adapter/depth_components.hpp"

#include <cmath>
#include <limits>

namespace sunhorizon_rgbd_adapter {
const DepthComponentMask& depth_component_mask(
    const DepthImageView& image, double scale_m, double min_depth_m,
    double max_depth_m, double neighbor_delta_m, std::size_t minimum_component_pixels,
    DepthComponentWorkspace& workspace, std::size_t near_minimum_component_pixels,
    double near_max_depth_m) {
  auto& result = workspace.result;
  result.valid = false; result.reason.clear(); result.keep.clear();
  result.valid_pixels = result.removed_pixels = result.removed_components = 0;
  workspace.depth.clear(); workspace.queue.clear();
  if (!std::isfinite(scale_m) || scale_m <= 0 || !std::isfinite(min_depth_m) ||
      min_depth_m < 0 || !std::isfinite(max_depth_m) || max_depth_m <= min_depth_m ||
      !std::isfinite(neighbor_delta_m) || neighbor_delta_m <= 0 ||
      minimum_component_pixels == 0 || minimum_component_pixels > kMaxDepthComponentPixels ||
      near_minimum_component_pixels > kMaxDepthComponentPixels ||
      (near_minimum_component_pixels &&
       (near_minimum_component_pixels < minimum_component_pixels ||
        !std::isfinite(near_max_depth_m) || near_max_depth_m < min_depth_m ||
        near_max_depth_m > max_depth_m))) {
    result.reason = "invalid_component_parameters";
    return result;
  }
  if (!image.data || !image.width || !image.height ||
      image.width > kMaxDepthComponentPixels / image.height ||
      image.width > image.step / 2 ||
      image.step > image.data_size / image.height) {
    result.reason = "invalid_depth_layout_or_size";
    return result;
  }
  const std::size_t count = static_cast<std::size_t>(image.width) * image.height;
  auto& depth = workspace.depth;
  depth.reserve(count); depth.assign(count, 0);
  result.keep.reserve(count); result.keep.assign(count, 0);
  for (uint32_t row = 0; row < image.height; ++row) {
    for (uint32_t col = 0; col < image.width; ++col) {
      const auto offset = static_cast<std::size_t>(row) * image.step + col * 2;
      const uint16_t raw = image.bigendian ?
          (uint16_t(image.data[offset]) << 8) | image.data[offset + 1] :
          uint16_t(image.data[offset]) | (uint16_t(image.data[offset + 1]) << 8);
      const double meters = raw * scale_m;
      if (raw && meters >= min_depth_m && meters <= max_depth_m) {
        depth[static_cast<std::size_t>(row) * image.width + col] = raw;
        ++result.valid_pixels;
      }
    }
  }
  // The queue is also the member list. Each valid pixel is enqueued once.
  // keep=2 means visited, keep=1 means retained. Invalid/removed pixels are 0.
  auto& queue = workspace.queue;
  queue.reserve(count);
  for (std::size_t seed = 0; seed < count; ++seed) {
    if (!depth[seed] || result.keep[seed]) continue;
    queue.clear(); queue.push_back(seed); result.keep[seed] = 2;
    for (std::size_t head = 0; head < queue.size(); ++head) {
      const auto current = queue[head];
      const int row = static_cast<int>(current / image.width);
      const int col = static_cast<int>(current % image.width);
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const int y = row + dy, x = col + dx;
          if ((!dy && !dx) || y < 0 || x < 0 || y >= static_cast<int>(image.height) ||
              x >= static_cast<int>(image.width)) continue;
          const auto next = static_cast<std::size_t>(y) * image.width + x;
          if (!depth[next] || result.keep[next] ||
              std::abs(int(depth[current]) - int(depth[next])) * scale_m > neighbor_delta_m + 1e-12)
            continue;
          result.keep[next] = 2;
          queue.push_back(next);
        }
      }
    }
    // Preserve c7's whole-surface rule; never crop the near part of a component
    // that extends beyond the stronger threshold's range.
    bool entirely_near = near_minimum_component_pixels != 0;
    if (entirely_near) {
      for (auto member : queue) {
        if (depth[member] * scale_m > near_max_depth_m) {
          entirely_near = false;
          break;
        }
      }
    }
    const auto minimum = entirely_near ? near_minimum_component_pixels : minimum_component_pixels;
    if (queue.size() < minimum) {
      result.removed_pixels += queue.size(); ++result.removed_components;
      // Preserve the visited marker until the scan finishes: setting it to zero
      // here would make every rejected component be traversed again.
    } else {
      for (auto member : queue) result.keep[member] = 1;
    }
  }
  for (auto& value : result.keep) if (value == 2) value = 0;
  result.valid = true;
  result.reason = "ok";
  return result;
}
const DepthComponentMask& depth_component_mask(
    const DepthImageView& image, double scale_m, double min_depth_m,
    double max_depth_m, double neighbor_delta_m, std::size_t minimum_component_pixels,
    DepthComponentWorkspace& workspace) {
  return depth_component_mask(image, scale_m, min_depth_m, max_depth_m,
                              neighbor_delta_m, minimum_component_pixels, workspace, 0, 0.);
}
DepthComponentMask depth_component_mask(
    const DepthImageView& image, double scale_m, double min_depth_m,
    double max_depth_m, double neighbor_delta_m, std::size_t minimum_component_pixels) {
  return depth_component_mask(image, scale_m, min_depth_m, max_depth_m,
                              neighbor_delta_m, minimum_component_pixels, 0, 0.);
}
DepthComponentMask depth_component_mask(
    const DepthImageView& image, double scale_m, double min_depth_m,
    double max_depth_m, double neighbor_delta_m, std::size_t minimum_component_pixels,
    std::size_t near_minimum_component_pixels, double near_max_depth_m) {
  DepthComponentWorkspace workspace;
  depth_component_mask(image, scale_m, min_depth_m, max_depth_m, neighbor_delta_m,
                       minimum_component_pixels, workspace,
                       near_minimum_component_pixels, near_max_depth_m);
  return std::move(workspace.result);
}
}  // namespace sunhorizon_rgbd_adapter
