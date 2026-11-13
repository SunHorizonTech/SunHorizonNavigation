#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>
#include <utility>
#include <nav2_util/direct_cloud.hpp>

namespace sunhorizon_navigation_runtime::detail {

inline nav2_util::DirectCloud exclude_near_body_endpoints(
    nav2_util::DirectCloud frame, const std::vector<std::size_t>& indices) {
  if (indices.empty()) return frame;
  if (!frame.classes) throw std::invalid_argument("near_body_mask_requires_classes");
  auto classes = std::make_shared<std::vector<uint8_t>>(*frame.classes);
  using Class = nav2_util::DirectPointClass;
  for (const auto index : indices) {
    auto& value = classes->at(index);
    if (value != uint8_t(Class::Obstacle) && value != uint8_t(Class::Uncertain))
      throw std::invalid_argument("near_body_endpoint_class_mismatch");
    value = uint8_t(Class::Rejected);
  }
  frame.classes = std::move(classes);
  return frame;
}
}  // namespace sunhorizon_navigation_runtime::detail
