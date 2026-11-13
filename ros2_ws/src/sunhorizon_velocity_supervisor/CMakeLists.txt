cmake_minimum_required(VERSION 3.8)
project(sunhorizon_velocity_supervisor)

set(SUNHORIZON_RELEASE_SHA "development" CACHE STRING
  "Full committed release SHA embedded in the supervisor binary")
if(NOT SUNHORIZON_RELEASE_SHA STREQUAL "development")
  string(LENGTH "${SUNHORIZON_RELEASE_SHA}" SUNHORIZON_RELEASE_SHA_LENGTH)
  if(NOT SUNHORIZON_RELEASE_SHA_LENGTH EQUAL 40 OR
     NOT SUNHORIZON_RELEASE_SHA MATCHES "^[0-9a-f]+$")
    message(FATAL_ERROR
      "SUNHORIZON_RELEASE_SHA must be development or 40 lowercase hex")
  endif()
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sunhorizon_navigation_core REQUIRED)
find_package(sunhorizon_obstacle_coverage REQUIRED)
find_package(sunhorizon_navigation_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(velocity_supervisor_node src/velocity_supervisor_node.cpp)
ament_target_dependencies(velocity_supervisor_node
  geometry_msgs rcl_interfaces rclcpp std_srvs sunhorizon_navigation_core
  sunhorizon_navigation_interfaces sunhorizon_obstacle_coverage tf2 tf2_geometry_msgs tf2_ros)
target_include_directories(velocity_supervisor_node PRIVATE include
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/patches")
target_compile_options(velocity_supervisor_node PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(velocity_supervisor_node PRIVATE
  SUNHORIZON_RELEASE_SHA="${SUNHORIZON_RELEASE_SHA}")

install(TARGETS velocity_supervisor_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_tf_stream_liveness test/test_tf_stream_liveness.cpp)
  target_include_directories(test_tf_stream_liveness PRIVATE include)
  add_test(NAME test_tf_stream_liveness COMMAND test_tf_stream_liveness)
  add_executable(test_spatial_coverage_decision_tracker test/test_spatial_coverage_decision_tracker.cpp)
  target_include_directories(test_spatial_coverage_decision_tracker PRIVATE include)
  ament_target_dependencies(test_spatial_coverage_decision_tracker sunhorizon_obstacle_coverage)
  target_compile_options(test_spatial_coverage_decision_tracker PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_spatial_coverage_decision_tracker COMMAND test_spatial_coverage_decision_tracker)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_coverage_decision_tracker test/test_coverage_decision_tracker.cpp)
  target_include_directories(test_coverage_decision_tracker PRIVATE include)

  ament_add_gtest(test_parameter_contract test/test_parameter_contract.cpp)
  target_include_directories(test_parameter_contract PRIVATE include)
  ament_target_dependencies(test_parameter_contract rcl_interfaces)

  add_executable(test_heading_correction_contract test/test_heading_correction_contract.cpp)
  target_include_directories(test_heading_correction_contract PRIVATE include)
  ament_target_dependencies(test_heading_correction_contract sunhorizon_navigation_core)
  # Assertions remain active in Release builds; this executable is a test only.
  target_compile_options(test_heading_correction_contract PRIVATE -UNDEBUG)
  add_test(NAME test_heading_correction_contract COMMAND test_heading_correction_contract)

  ament_add_pytest_test(
    test_read_only_motion_parameter_contract
    test/test_read_only_motion_parameter_contract.py)
  ament_add_pytest_test(test_spatial_visibility_supervisor_node
    test/test_spatial_visibility_supervisor_node.py
    ENV "SUPERVISOR_EXECUTABLE=$<TARGET_FILE:velocity_supervisor_node>"
    TIMEOUT 90)
  ament_add_pytest_test(test_geometry_hold_supervisor_node
    test/test_geometry_hold_supervisor_node.py
    ENV "SUPERVISOR_EXECUTABLE=$<TARGET_FILE:velocity_supervisor_node>"
    TIMEOUT 180)
  # Vendor actor paths must be supplied explicitly by the pinned trial runner.
  # Missing controller/collision ELFs produce a visible pytest skip.
  ament_add_pytest_test(test_geometry_hold_actual_pipeline
    test/test_geometry_hold_actual_pipeline.py
    ENV "SUPERVISOR_EXECUTABLE=$<TARGET_FILE:velocity_supervisor_node>"
    TIMEOUT 60)
endif()

ament_package()
