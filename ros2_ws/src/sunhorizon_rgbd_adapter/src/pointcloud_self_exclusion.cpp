#include "sunhorizon_rgbd_adapter/pointcloud_self_exclusion.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <utility>

#include <sensor_msgs/msg/point_field.hpp>

namespace sunhorizon_rgbd_adapter {
namespace {
using Field = sensor_msgs::msg::PointField;

bool host_big_endian() {
  const uint16_t probe = 1;
  return *reinterpret_cast<const uint8_t*>(&probe) == 0;
}

double coordinate(const uint8_t* point, const Field& field, bool swap_bytes) {
  uint8_t bytes[8];
  const std::size_t size = field.datatype == Field::FLOAT32 ? 4U : 8U;
  std::memcpy(bytes, point + field.offset, size);
  if (swap_bytes) std::reverse(bytes, bytes + size);
  if (size == 4U) {
    float value;
    std::memcpy(&value, bytes, sizeof(value));
    return value;
  }
  double value;
  std::memcpy(&value, bytes, sizeof(value));
  return value;
}

bool segment_intersects_box(const std::array<double, 3>& origin,
                            const std::array<double, 3>& end, const ExclusionBox& box) {
  double entry = 0., exit = 1.;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double direction = end[axis] - origin[axis];
    if (std::abs(direction) < 1.e-12) {
      if (origin[axis] < box.minimum[axis] || origin[axis] > box.maximum[axis]) return false;
      continue;
    }
    const double a = (box.minimum[axis] - origin[axis]) / direction;
    const double b = (box.maximum[axis] - origin[axis]) / direction;
    entry = std::max(entry, std::min(a, b));
    exit = std::min(exit, std::max(a, b));
    if (entry > exit) return false;
  }
  return true;
}
}  // namespace

bool valid_exclusion_box(const ExclusionBox& box) {
  for (std::size_t i = 0; i < 3; ++i) {
    const double lo = box.minimum[i], hi = box.maximum[i];
    if (!std::isfinite(lo) || !std::isfinite(hi) || lo >= hi ||
        hi - lo > 1.0 || lo < -2.0 || hi > 2.0) return false;
  }
  return true;
}

bool exclusion_fraction_allowed(std::size_t removed, std::size_t input, double limit) {
  if (!std::isfinite(limit) || limit <= 0. || limit > .10 || removed > input) return false;
  return input == 0 || static_cast<double>(removed) / static_cast<double>(input) <= limit;
}

ExclusionResult exclude_self_points(
    const sensor_msgs::msg::PointCloud2& input, const ExclusionBox& box,
    const std::optional<ExclusionTransform>& cloud_to_base) {
  return exclude_self_volumes(input, {box}, cloud_to_base);
}

ExclusionResult exclude_self_volumes(
    const sensor_msgs::msg::PointCloud2& input, const std::vector<ExclusionBox>& boxes,
    const std::optional<ExclusionTransform>& cloud_to_base) {
  return exclude_self_framed_volumes(
      input, boxes, std::vector<std::optional<ExclusionTransform>>(boxes.size(), cloud_to_base));
}

ExclusionResult exclude_self_framed_volumes(
    const sensor_msgs::msg::PointCloud2& input, const std::vector<ExclusionBox>& boxes,
    const std::vector<std::optional<ExclusionTransform>>& transforms, double shadow_sensor_range_m,
    const std::vector<uint8_t>* ground_blocked,
    const std::vector<std::array<float,3>>* ray_origins,
    const std::vector<uint64_t>* sample_ids) {
  ExclusionResult result;
  if (!std::isfinite(shadow_sensor_range_m) || shadow_sensor_range_m < 0. || shadow_sensor_range_m > 2.) {
    result.reason = "invalid_shadow_sensor_range";
    return result;
  }
  result.input_count = static_cast<std::size_t>(input.width) * input.height;
  if(ground_blocked && (ground_blocked->size()!=result.input_count ||
      std::any_of(ground_blocked->begin(),ground_blocked->end(),[](auto value){return value>1;}))){
    result.reason="ground_block_mask_layout";return result;
  }
  if (result.input_count == 0) {
    result.reason = "empty_input_cloud";
    return result;
  }
  if (boxes.empty() || boxes.size() > kMaxSelfExclusionBoxes ||
      !std::all_of(boxes.begin(), boxes.end(), valid_exclusion_box)) {
    result.reason = "invalid_exclusion_box";
    return result;
  }
  if (transforms.size() != boxes.size() ||
      std::any_of(transforms.begin(), transforms.end(), [](const auto& tf) { return !tf; })) {
    result.reason = "cloud_stamp_transform_unavailable";
    return result;
  }
  for (const auto& transform : transforms) {
    const auto& tf = *transform;
    for (const auto value : tf.rotation) {
      if (!std::isfinite(value)) {
        result.reason = "invalid_transform";
        return result;
      }
    }
    for (const auto value : tf.translation) {
      if (!std::isfinite(value)) {
        result.reason = "invalid_transform";
        return result;
      }
    }
  }
  const auto& first = *transforms.front();
  const bool common_frame = std::all_of(transforms.begin(), transforms.end(),
      [&first](const auto& tf) {
        return tf->rotation == first.rotation && tf->translation == first.translation;
      });
  // Every local box lies inside a sphere centred at the optical origin.
  // For rigid transforms, distant scene points cannot be self returns. Bound
  // them before the per-link transforms; fall back for non-rigid core inputs.
  double sensor_radius = shadow_sensor_range_m;
  bool rigid = true;
  for (std::size_t i = 0; i < boxes.size(); ++i) {
    const auto& tf = *transforms[i];
    for (std::size_t a = 0; a < 3; ++a) {
      for (std::size_t b = 0; b < 3; ++b) {
        double dot = 0.;
        for (std::size_t k = 0; k < 3; ++k) dot += tf.rotation[k * 3 + a] * tf.rotation[k * 3 + b];
        rigid = rigid && std::abs(dot - (a == b ? 1. : 0.)) < 1.e-10;
      }
    }
    std::array<double, 3> extent;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      extent[axis] = std::max(std::abs(boxes[i].minimum[axis] - tf.translation[axis]),
                              std::abs(boxes[i].maximum[axis] - tf.translation[axis]));
    }
    sensor_radius = std::max(sensor_radius, std::hypot(extent[0], extent[1], extent[2]) + 1.e-6);
  }
  const double radius_squared = rigid ? sensor_radius * sensor_radius :
      std::numeric_limits<double>::infinity();
  const double shadow_range_squared = shadow_sensor_range_m * shadow_sensor_range_m;
  const std::size_t row_bytes = static_cast<std::size_t>(input.width) * input.point_step;
  const std::size_t data_bytes = static_cast<std::size_t>(input.row_step) * input.height;
  if (input.point_step == 0 || input.row_step < row_bytes ||
      input.data.size() != data_bytes || result.input_count >
      std::numeric_limits<uint32_t>::max() / input.point_step) {
    result.reason = "invalid_pointcloud_layout";
    return result;
  }
  std::array<const Field*, 3> xyz{};
  const std::array<std::string, 3> names{"x", "y", "z"};
  for (const auto& field : input.fields) {
    for (std::size_t i = 0; i < 3; ++i) {
      if (field.name != names[i]) continue;
      const std::size_t size = field.datatype == Field::FLOAT32 ? 4U :
          (field.datatype == Field::FLOAT64 ? 8U : 0U);
      if (xyz[i] || size == 0 || field.count != 1 ||
          static_cast<std::size_t>(field.offset) + size > input.point_step) {
        result.reason = "invalid_xyz_fields";
        return result;
      }
      xyz[i] = &field;
    }
  }
  if (!xyz[0] || !xyz[1] || !xyz[2]) {
    result.reason = "missing_xyz_fields";
    return result;
  }
  for (std::size_t i = 0; i < 3; ++i) {
    const auto size_i = xyz[i]->datatype == Field::FLOAT32 ? 4U : 8U;
    for (std::size_t j = i + 1; j < 3; ++j) {
      const auto size_j = xyz[j]->datatype == Field::FLOAT32 ? 4U : 8U;
      if (xyz[i]->offset < xyz[j]->offset + size_j &&
          xyz[j]->offset < xyz[i]->offset + size_i) {
        result.reason = "overlapping_xyz_fields";
        return result;
      }
    }
  }

  if(sample_ids && sample_ids->size()!=result.input_count) {
    result.reason="sample_id_layout_mismatch";return result;
  }
  if(ray_origins && ray_origins->size()!=result.input_count) {
    result.reason="ray_origin_layout_mismatch";return result;
  }
  // Copy metadata only; avoid copying the entire source data before compacting.
  sensor_msgs::msg::PointCloud2 output;
  output.header = input.header;
  output.fields = input.fields;
  output.is_bigendian = input.is_bigendian;
  output.point_step = input.point_step;
  output.is_dense = input.is_dense;
  output.height = 1;
  output.data.reserve(result.input_count * input.point_step);
  const bool swap = input.is_bigendian != host_big_endian();
  for (std::size_t row = 0; row < input.height; ++row) {
    for (std::size_t col = 0; col < input.width; ++col) {
      const auto* point = input.data.data() + row * input.row_step + col * input.point_step;
      const double x = coordinate(point, *xyz[0], swap);
      const double y = coordinate(point, *xyz[1], swap);
      const double z = coordinate(point, *xyz[2], swap);
      bool inside = false;
      const double squared_range = x * x + y * y + z * z;
      const bool in_shadow_range = shadow_sensor_range_m > 0. && squared_range <= shadow_range_squared;
      if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && squared_range <= radius_squared) {
        std::array<double, 3> base{};
        for (std::size_t i = 0; i < boxes.size(); ++i) {
          const auto& box = boxes[i];
          if (i == 0 || !common_frame) {
            const auto& tf = *transforms[i];
            for (std::size_t axis = 0; axis < 3; ++axis) {
              base[axis] = tf.rotation[axis * 3] * x +
                  tf.rotation[axis * 3 + 1] * y + tf.rotation[axis * 3 + 2] * z +
                  tf.translation[axis];
            }
          }
          bool in_box = true;
          for (std::size_t axis = 0; axis < 3; ++axis) {
            in_box = in_box && std::isfinite(base[axis]) &&
                base[axis] >= box.minimum[axis] && base[axis] <= box.maximum[axis];
          }
          if (in_box || (in_shadow_range &&
              segment_intersects_box(transforms[i]->translation, base, box))) {
            inside = true;
            break;
          }
        }
      }
      if (inside) {
        ++result.removed_count;
      } else {
        output.data.insert(output.data.end(), point, point + input.point_step);
        if(ground_blocked)result.ground_blocked.push_back((*ground_blocked)[row*input.width+col]);
        if(ray_origins)result.ray_origins.push_back((*ray_origins)[row*input.width+col]);
        if(sample_ids)result.sample_ids.push_back((*sample_ids)[row*input.width+col]);
        ++result.retained_count;
      }
    }
  }
  output.width = static_cast<uint32_t>(result.retained_count);
  output.row_step = output.width * output.point_step;
  result.cloud = std::move(output);
  result.reason = "filtered";
  return result;
}
}  // namespace sunhorizon_rgbd_adapter
