cmake_minimum_required(VERSION 3.8)
project(sunhorizon_navigation_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sunhorizon_navigation_core REQUIRED)
find_package(sunhorizon_navigation_interfaces REQUIRED)
find_path(JSON_C_INCLUDE_DIR json-c/json.h REQUIRED)
find_library(JSON_C_LIBRARY NAMES json-c REQUIRED)

add_library(map_bundle_validator STATIC src/map_bundle_validator.cpp)
target_include_directories(map_bundle_validator
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(map_bundle_validator
  SYSTEM PRIVATE ${JSON_C_INCLUDE_DIR})
target_link_libraries(map_bundle_validator PUBLIC OpenSSL::Crypto ${JSON_C_LIBRARY})
target_compile_options(map_bundle_validator PRIVATE -Wall -Wextra -Wpedantic)

add_executable(navigation_server_node src/navigation_server_node.cpp)
ament_target_dependencies(navigation_server_node
  geometry_msgs nav2_msgs nav_msgs rclcpp rclcpp_action std_srvs tf2 tf2_ros sunhorizon_navigation_core
  sunhorizon_navigation_interfaces)
target_link_libraries(navigation_server_node map_bundle_validator)
target_compile_options(navigation_server_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS navigation_server_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(
    test_localization_health_gate
    test/test_localization_health_gate.cpp)
  target_include_directories(test_localization_health_gate
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_cancel_goal_if_known test/test_cancel_goal_if_known.cpp)
  target_include_directories(test_cancel_goal_if_known
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_cancel_goal_if_known nav2_msgs rclcpp rclcpp_action)
  ament_add_pytest_test(test_startup_map_provider_delay
    test/test_startup_map_provider_delay.py
    TIMEOUT 30
    ENV "NAVIGATION_SERVER_TEST_EXECUTABLE=$<TARGET_FILE:navigation_server_node>")
  ament_add_pytest_test(test_first_click_path_geometry
    test/test_first_click_path_geometry.py TIMEOUT 30)
  ament_add_pytest_test(test_terminal_zero_guard
    test/test_terminal_zero_guard.py TIMEOUT 30)
endif()

ament_package()
