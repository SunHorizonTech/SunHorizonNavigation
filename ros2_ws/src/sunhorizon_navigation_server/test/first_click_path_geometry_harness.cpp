#include <iostream>
#include <string>
#include <vector>
#include "sunhorizon_navigation_server/first_click_path_geometry.hpp"

int main(int argc, char** argv) {
  using sunhorizon_navigation_server::FirstClickPoint;
  if (argc == 2 && std::string(argv[1]) == "time") {
    std::int64_t now_ns, stamp_ns;
    double max_age;
    if (!(std::cin >> now_ns >> stamp_ns >> max_age)) return 2;
    const bool valid = sunhorizon_navigation_server::first_click_pose_time_valid(
        now_ns, stamp_ns, max_age);
    std::cout << (valid ? "accepted" : "invalid") << std::endl;
    return valid ? 0 : 1;
  }
  if (argc == 2 && std::string(argv[1]) == "goal") {
    FirstClickPoint start{}, goal{};
    double start_yaw, goal_yaw, min_distance, max_distance, max_heading, max_yaw;
    if (!(std::cin >> start.x >> start.y >> start_yaw >> goal.x >> goal.y >> goal_yaw
          >> min_distance >> max_distance >> max_heading >> max_yaw)) return 2;
    std::string reason;
    const bool valid = sunhorizon_navigation_server::validate_first_click_goal_geometry(
        start, start_yaw, goal, goal_yaw, min_distance, max_distance, max_heading, max_yaw,
        &reason);
    std::cout << (valid ? "accepted" : reason) << std::endl;
    return valid ? 0 : 1;
  }
  FirstClickPoint goal{};
  double max_length = 0.0;
  std::size_t count = 0U;
  if (!(std::cin >> goal.x >> goal.y >> max_length >> count)) return 2;
  std::vector<FirstClickPoint> points(count);
  for (auto& point : points) {
    if (!(std::cin >> point.x >> point.y)) return 2;
  }
  std::string reason;
  const bool accepted = sunhorizon_navigation_server::validate_first_click_path_geometry(
      points, goal, max_length, &reason);
  std::cout << (accepted ? "accepted" : reason) << std::endl;
  return accepted ? 0 : 1;
}
