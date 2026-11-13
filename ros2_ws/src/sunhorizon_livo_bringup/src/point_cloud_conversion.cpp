#include "sunhorizon_livo_bringup/point_cloud_conversion.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace sunhorizon_livo_bringup
{
namespace
{

using sensor_msgs::msg::PointField;

const PointField * find_field(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  const auto found = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [&name](const PointField & field) {return field.name == name;});
  return found == cloud.fields.end() ? nullptr : &*found;
}

bool field_matches(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name,
  std::uint8_t datatype)
{
  const auto * field = find_field(cloud, name);
  return field != nullptr && field->count == 1U && field->datatype == datatype;
}

bool valid_point_layout(
  const sensor_msgs::msg::PointCloud2 & cloud, std::string * reason)
{
  if (cloud.width == 0U || cloud.height == 0U || cloud.data.empty()) {
    *reason = "airy_pointcloud_empty";
    return false;
  }
  for (const auto * name : {"x", "y", "z"}) {
    if (!field_matches(cloud, name, PointField::FLOAT32)) {
      *reason = std::string("airy_point_field_invalid:") + name;
      return false;
    }
  }
  if (!field_matches(cloud, "intensity", PointField::UINT8)) {
    *reason = "airy_point_field_invalid:intensity_uint8_required";
    return false;
  }
  if (!field_matches(cloud, "timestamp", PointField::FLOAT32)) {
    *reason = "airy_point_field_invalid:timestamp_float32_required";
    return false;
  }
  if (!field_matches(cloud, "ring", PointField::UINT16)) {
    *reason = "airy_point_field_invalid:ring_uint16_required";
    return false;
  }
  return true;
}

bool finite_point(float x, float y, float z, float timestamp)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
         std::isfinite(timestamp);
}

}  // namespace

bool convert_airy_points(
  const sensor_msgs::msg::PointCloud2 & input,
  sensor_msgs::msg::PointCloud2 * output,
  std::string * reason)
{
  if (output == nullptr || reason == nullptr) {
    return false;
  }
  if (!valid_point_layout(input, reason)) {
    return false;
  }
  try {
    std::size_t finite_count = 0U;
    sensor_msgs::PointCloud2ConstIterator<float> count_x(input, "x");
    sensor_msgs::PointCloud2ConstIterator<float> count_y(input, "y");
    sensor_msgs::PointCloud2ConstIterator<float> count_z(input, "z");
    sensor_msgs::PointCloud2ConstIterator<float> count_timestamp(input, "timestamp");
    for (; count_x != count_x.end();
      ++count_x, ++count_y, ++count_z, ++count_timestamp)
    {
      if (finite_point(*count_x, *count_y, *count_z, *count_timestamp)) {
        ++finite_count;
      }
    }
    if (finite_count == 0U) {
      *reason = "airy_pointcloud_no_finite_points";
      return false;
    }

    output->header = input.header;
    output->is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(*output);
    modifier.setPointCloud2Fields(
      6,
      "x", 1, PointField::FLOAT32,
      "y", 1, PointField::FLOAT32,
      "z", 1, PointField::FLOAT32,
      "intensity", 1, PointField::FLOAT32,
      "timestamp", 1, PointField::FLOAT64,
      "ring", 1, PointField::UINT16);
    modifier.resize(finite_count);
    output->height = 1U;
    output->width = static_cast<std::uint32_t>(finite_count);
    output->row_step = output->point_step * output->width;

    sensor_msgs::PointCloud2ConstIterator<float> in_x(input, "x");
    sensor_msgs::PointCloud2ConstIterator<float> in_y(input, "y");
    sensor_msgs::PointCloud2ConstIterator<float> in_z(input, "z");
    sensor_msgs::PointCloud2ConstIterator<std::uint8_t> in_intensity(input, "intensity");
    sensor_msgs::PointCloud2ConstIterator<float> in_timestamp(input, "timestamp");
    sensor_msgs::PointCloud2ConstIterator<std::uint16_t> in_ring(input, "ring");
    sensor_msgs::PointCloud2Iterator<float> out_x(*output, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(*output, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(*output, "z");
    sensor_msgs::PointCloud2Iterator<float> out_intensity(*output, "intensity");
    sensor_msgs::PointCloud2Iterator<double> out_timestamp(*output, "timestamp");
    sensor_msgs::PointCloud2Iterator<std::uint16_t> out_ring(*output, "ring");

    for (; in_x != in_x.end();
      ++in_x, ++in_y, ++in_z, ++in_intensity, ++in_timestamp, ++in_ring)
    {
      if (!finite_point(*in_x, *in_y, *in_z, *in_timestamp)) {
        continue;
      }
      *out_x = *in_x;
      *out_y = *in_y;
      *out_z = *in_z;
      *out_intensity = static_cast<float>(*in_intensity);
      *out_timestamp = static_cast<double>(*in_timestamp);
      *out_ring = *in_ring;
      ++out_x;
      ++out_y;
      ++out_z;
      ++out_intensity;
      ++out_timestamp;
      ++out_ring;
    }
    reason->clear();
  } catch (const std::runtime_error &) {
    *reason = "airy_pointcloud_iterator_error";
    return false;
  }
  return true;
}

}  // namespace sunhorizon_livo_bringup
