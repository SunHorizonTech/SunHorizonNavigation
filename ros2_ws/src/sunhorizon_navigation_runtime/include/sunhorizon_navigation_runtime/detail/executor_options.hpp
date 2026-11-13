#pragma once

#include <rclcpp/executor_options.hpp>
#include <memory>
#include <utility>

namespace sunhorizon_navigation_runtime::detail {
// ExecutorOptions owns a shared_ptr to mutable wait-set memory strategy.
// Construct defaults per executor; copying one options object between spinning
// executors would share that strategy even when their node sets are disjoint.
inline rclcpp::ExecutorOptions make_executor_options(rclcpp::Context::SharedPtr context) {
  rclcpp::ExecutorOptions options;
  options.context=std::move(context);
  return options;
}
}  // namespace sunhorizon_navigation_runtime::detail
