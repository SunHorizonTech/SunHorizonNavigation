#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace sunhorizon_navigation_server {

struct FirstClickPoint {
  double x;
  double y;
};

// A cell-center planner on the accepted 0.05 m grid can quantize the goal by
// at most half a cell diagonal. This is deliberately a fixed upper bound:
// coarser maps do not silently expand the first-click endpoint allowance.
constexpr double kFirstClickEndpointToleranceM = 0.03535533905932738;
constexpr double kFirstClickGeometryEpsilonM = 1e-9;

inline bool first_click_pose_time_valid(
    std::int64_t now_ns, std::int64_t stamp_ns, double max_age_sec) {
  return std::isfinite(max_age_sec) && max_age_sec > 0.0 && stamp_ns > 0 &&
         stamp_ns <= now_ns && static_cast<double>(now_ns - stamp_ns) <= max_age_sec * 1e9;
}

// A planner's cell-center poses/orientations describe its path, not the measured
// robot pose. The caller must supply the actual map -> base_footprint TF pose.
inline bool validate_first_click_goal_geometry(
    const FirstClickPoint& measured_start, double measured_start_yaw,
    const FirstClickPoint& goal, double goal_yaw, double min_distance_m,
    double max_distance_m, double max_heading_error_rad,
    double max_goal_yaw_delta_rad, std::string* reason) {
  const double dx = goal.x - measured_start.x;
  const double dy = goal.y - measured_start.y;
  const double distance = std::hypot(dx, dy);
  if (!std::isfinite(distance) || distance < min_distance_m || distance > max_distance_m) {
    *reason = "first_click_distance_out_of_range";
    return false;
  }
  if (!std::isfinite(measured_start_yaw) || !std::isfinite(goal_yaw)) {
    *reason = "first_click_orientation_invalid";
    return false;
  }
  constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
  if (std::abs(std::remainder(std::atan2(dy, dx) - measured_start_yaw, kTwoPi)) >
      max_heading_error_rad) {
    *reason = "first_click_requires_straight_forward_goal";
    return false;
  }
  if (std::abs(std::remainder(goal_yaw - measured_start_yaw, kTwoPi)) >
      max_goal_yaw_delta_rad) {
    *reason = "first_click_goal_yaw_must_match_start";
    return false;
  }
  return true;
}

inline bool validate_first_click_path_geometry(
    const std::vector<FirstClickPoint>& points, const FirstClickPoint& goal,
    double max_length_m, std::string* reason) {
  if (points.size() < 2U) {
    *reason = "first_click_no_feasible_path";
    return false;
  }
  for (const auto& point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      *reason = "first_click_path_length_out_of_range";
      return false;
    }
  }
  const auto& start = points.front();
  const auto& end = points.back();
  const double endpoint_error = std::hypot(end.x - goal.x, end.y - goal.y);
  if (!std::isfinite(endpoint_error) ||
      endpoint_error > kFirstClickEndpointToleranceM + kFirstClickGeometryEpsilonM) {
    *reason = "first_click_path_endpoint_out_of_range";
    return false;
  }
  const double chord = std::hypot(end.x - start.x, end.y - start.y);
  const double requested_distance = std::hypot(goal.x - start.x, goal.y - start.y);
  if (!std::isfinite(requested_distance) || requested_distance <= 0.0 || chord <= 0.0) {
    *reason = "first_click_path_length_out_of_range";
    return false;
  }
  const double forward_x = (goal.x - start.x) / requested_distance;
  const double forward_y = (goal.y - start.y) / requested_distance;
  double length = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    const double dx = points[index].x - points[index - 1U].x;
    const double dy = points[index].y - points[index - 1U].y;
    if (dx * forward_x + dy * forward_y < -kFirstClickGeometryEpsilonM) {
      *reason = "first_click_path_must_not_reverse";
      return false;
    }
    length += std::hypot(dx, dy);
  }
  // Compare quantities belonging to the same path, not to an unquantized goal.
  if (!std::isfinite(length) || length + kFirstClickGeometryEpsilonM < chord ||
      length > max_length_m) {
    *reason = "first_click_path_length_out_of_range";
    return false;
  }
  return true;
}

}  // namespace sunhorizon_navigation_server
