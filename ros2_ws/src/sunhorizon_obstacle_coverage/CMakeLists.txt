cmake_minimum_required(VERSION 3.8)
project(sunhorizon_obstacle_coverage)

option(SUNHORIZON_ENABLE_CAMERA_BODY_CUDA "Enable explicit camera BODY CUDA backend" OFF)
if(SUNHORIZON_ENABLE_CAMERA_BODY_CUDA)
  if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR
     NOT CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL "11.4.0" OR
     NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    message(FATAL_ERROR "Camera CUDA counterpart requires GCC11.4.0 AArch64")
  endif()
  # Match the archived 859 geometry flags; enabling CUDA must not retune CPU math.
  if(NOT CMAKE_BUILD_TYPE STREQUAL "Release" OR NOT CMAKE_CXX_FLAGS STREQUAL "" OR
     NOT CMAKE_CXX_FLAGS_RELEASE STREQUAL "-O3 -DNDEBUG")
    message(FATAL_ERROR "Camera CUDA requires original Release flags: empty CXX_FLAGS, -O3 -DNDEBUG")
  endif()
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sunhorizon_navigation_interfaces REQUIRED)
find_package(sunhorizon_navigation_core REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(coverage_evaluator STATIC src/coverage_evaluator.cpp)
target_include_directories(coverage_evaluator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(coverage_evaluator PRIVATE -Wall -Wextra -Wpedantic)

# Candidate pure geometry core. Not linked into the production ROS node.
add_library(region_depth_support src/region_depth_support.cpp)
target_include_directories(region_depth_support PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(region_depth_support PRIVATE -Wall -Wextra -Wpedantic)

add_library(region_support_tracker STATIC src/region_support_tracker.cpp)
target_include_directories(region_support_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(region_support_tracker PRIVATE -Wall -Wextra -Wpedantic)

# Spatial visibility candidate cores. Legacy region evaluation remains separate.
add_library(dynamic_self_geometry src/dynamic_self_geometry.cpp src/dynamic_self_geometry_cuda_contract.cpp)
if(SUNHORIZON_ENABLE_CAMERA_BODY_CUDA)
  target_compile_definitions(dynamic_self_geometry PRIVATE SUNHORIZON_CAMERA_ARM11_CONTRACT=1)
endif()
target_include_directories(dynamic_self_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(dynamic_self_geometry PRIVATE -Wall -Wextra -Wpedantic)

add_library(spatial_visibility_fusion src/spatial_visibility_fusion.cpp)
target_include_directories(spatial_visibility_fusion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(spatial_visibility_fusion PRIVATE -Wall -Wextra -Wpedantic)

# The same node owns both standalone DDS ingress and explicit direct ingress.
add_library(coverage_runtime SHARED src/obstacle_coverage_node.cpp)
target_include_directories(coverage_runtime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(coverage_runtime
  rclcpp rcl_interfaces sensor_msgs diagnostic_msgs sunhorizon_navigation_interfaces sunhorizon_navigation_core tf2_ros)
target_link_libraries(coverage_runtime coverage_evaluator region_support_tracker)
set_target_properties(coverage_evaluator region_support_tracker PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(coverage_runtime PRIVATE -Wall -Wextra -Wpedantic)
add_executable(obstacle_coverage_node src/obstacle_coverage_main.cpp)
ament_target_dependencies(obstacle_coverage_node rclcpp)
target_link_libraries(obstacle_coverage_node coverage_runtime)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(TARGETS coverage_runtime coverage_evaluator region_depth_support region_support_tracker
  dynamic_self_geometry spatial_visibility_fusion
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS obstacle_coverage_node RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_spatial_evidence_tracker test/test_spatial_evidence_tracker.cpp)
  target_include_directories(test_spatial_evidence_tracker PRIVATE include)
  target_compile_options(test_spatial_evidence_tracker PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_spatial_evidence_tracker COMMAND test_spatial_evidence_tracker)
  add_executable(test_dynamic_self_geometry test/test_dynamic_self_geometry.cpp)
  target_link_libraries(test_dynamic_self_geometry dynamic_self_geometry)
  target_compile_options(test_dynamic_self_geometry PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_dynamic_self_geometry COMMAND test_dynamic_self_geometry)
  set(frame1258_shoe "${CMAKE_CURRENT_SOURCE_DIR}/../../../config/robot_a/measured_surfaces/left_leg_ankle_roll_link.measured-visible-20261023.STL")
  if(EXISTS "${frame1258_shoe}")
    add_test(NAME test_frame1258_map_exclusion COMMAND test_dynamic_self_geometry --frame1258 "${frame1258_shoe}")
  endif()
  add_executable(test_spatial_visibility_fusion test/test_spatial_visibility_fusion.cpp)
  target_link_libraries(test_spatial_visibility_fusion spatial_visibility_fusion region_depth_support)
  target_compile_options(test_spatial_visibility_fusion PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_spatial_visibility_fusion COMMAND test_spatial_visibility_fusion)
  add_executable(test_rgbd_fault_reference test/test_rgbd_fault_reference.cpp)
  target_include_directories(test_rgbd_fault_reference PRIVATE include)
  target_compile_options(test_rgbd_fault_reference PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_rgbd_fault_reference COMMAND test_rgbd_fault_reference)
  add_executable(test_region_upstream_deadline_policy test/test_region_upstream_deadline_policy.cpp)
  target_link_libraries(test_region_upstream_deadline_policy coverage_evaluator)
  target_compile_options(test_region_upstream_deadline_policy PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_region_upstream_deadline_policy COMMAND test_region_upstream_deadline_policy)
  add_executable(test_processed_observation_diagnostics test/test_processed_observation_diagnostics.cpp)
  target_link_libraries(test_processed_observation_diagnostics coverage_evaluator)
  target_compile_options(test_processed_observation_diagnostics PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_processed_observation_diagnostics COMMAND test_processed_observation_diagnostics)
  add_executable(test_belly_health_policy test/test_belly_health_policy.cpp)
  target_link_libraries(test_belly_health_policy coverage_evaluator)
  target_compile_options(test_belly_health_policy PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_belly_health_policy COMMAND test_belly_health_policy)
  add_executable(test_region_support_tracker test/test_region_support_tracker.cpp)
  target_link_libraries(test_region_support_tracker region_support_tracker)
  target_compile_options(test_region_support_tracker PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_region_support_tracker COMMAND test_region_support_tracker)
  add_executable(test_region_depth_support test/test_region_depth_support.cpp)
  target_link_libraries(test_region_depth_support region_depth_support)
  target_compile_options(test_region_depth_support PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_region_depth_support COMMAND test_region_depth_support)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_spatial_coverage_binding test/test_spatial_coverage_binding.cpp TIMEOUT 30)
  ament_target_dependencies(test_spatial_coverage_binding
    rclcpp rcl_interfaces sensor_msgs diagnostic_msgs sunhorizon_navigation_interfaces sunhorizon_navigation_core tf2_ros)
  target_include_directories(test_spatial_coverage_binding PRIVATE ../sunhorizon_velocity_supervisor/include)
  target_link_libraries(test_spatial_coverage_binding coverage_evaluator region_support_tracker)
  set_tests_properties(test_spatial_coverage_binding PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=217;ROS_LOCALHOST_ONLY=1")
  ament_add_gtest(test_rgbd_fault_reference_binding test/test_rgbd_fault_reference_binding.cpp TIMEOUT 20)
  ament_target_dependencies(test_rgbd_fault_reference_binding
    rclcpp rcl_interfaces sensor_msgs diagnostic_msgs sunhorizon_navigation_interfaces sunhorizon_navigation_core tf2_ros)
  target_link_libraries(test_rgbd_fault_reference_binding coverage_evaluator region_support_tracker)
  set_tests_properties(test_rgbd_fault_reference_binding PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=216;ROS_LOCALHOST_ONLY=1;RCUTILS_LOGGING_USE_STDOUT=0")
  ament_add_gtest(test_rgbd_upstream_binding test/test_rgbd_upstream_binding.cpp TIMEOUT 20)
  ament_target_dependencies(test_rgbd_upstream_binding
    rclcpp rcl_interfaces sensor_msgs diagnostic_msgs sunhorizon_navigation_interfaces sunhorizon_navigation_core tf2_ros)
  target_link_libraries(test_rgbd_upstream_binding coverage_evaluator region_support_tracker)
  set_tests_properties(test_rgbd_upstream_binding PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=216;ROS_LOCALHOST_ONLY=1")
  ament_add_gtest(test_coverage_evaluator test/test_coverage_evaluator.cpp)
  target_link_libraries(test_coverage_evaluator coverage_evaluator)
  ament_add_pytest_test(test_obstacle_coverage_node_contract
    test/test_obstacle_coverage_node_contract.py)
  ament_add_pytest_test(test_processed_belly_coverage_node
    test/test_processed_belly_coverage_node.py
    ENV "COVERAGE_EXECUTABLE=$<TARGET_FILE:obstacle_coverage_node>"
    TIMEOUT 45)
  ament_add_pytest_test(test_typed_region_coverage_node
    test/test_typed_region_coverage_node.py
    ENV "COVERAGE_EXECUTABLE=$<TARGET_FILE:obstacle_coverage_node>"
    TIMEOUT 100)
  ament_add_pytest_test(test_region_upstream_deadline_node
    test/test_region_upstream_deadline_node.py
    ENV "COVERAGE_EXECUTABLE=$<TARGET_FILE:obstacle_coverage_node>"
    TIMEOUT 100)
  ament_add_pytest_test(test_region_coverage_node
    test/test_region_coverage_node.py
    ENV "COVERAGE_EXECUTABLE=$<TARGET_FILE:obstacle_coverage_node>"
    TIMEOUT 45)
endif()

ament_export_dependencies(rclcpp rcl_interfaces sensor_msgs diagnostic_msgs sunhorizon_navigation_interfaces tf2_ros)
ament_export_include_directories(include)
ament_export_libraries(coverage_evaluator region_depth_support region_support_tracker
  dynamic_self_geometry spatial_visibility_fusion)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
