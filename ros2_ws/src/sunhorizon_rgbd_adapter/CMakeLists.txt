cmake_minimum_required(VERSION 3.18)
project(sunhorizon_rgbd_adapter)

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
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
find_package(sunhorizon_navigation_core REQUIRED)
find_package(sunhorizon_navigation_interfaces REQUIRED)
find_package(sunhorizon_obstacle_coverage REQUIRED)

add_library(rgbd_freshness src/rgbd_freshness.cpp src/depth_components.cpp src/depth_projection.cpp)
ament_target_dependencies(rgbd_freshness sensor_msgs)
target_include_directories(rgbd_freshness PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(rgbd_freshness PRIVATE -Wall -Wextra -Wpedantic)

add_executable(rgbd_adapter_node src/rgbd_adapter_node.cpp)
ament_target_dependencies(rgbd_adapter_node
  rclcpp sensor_msgs sunhorizon_navigation_core sunhorizon_navigation_interfaces sunhorizon_obstacle_coverage)
target_link_libraries(rgbd_adapter_node rgbd_freshness)
target_compile_options(rgbd_adapter_node PRIVATE -Wall -Wextra -Wpedantic)

add_library(pointcloud_self_exclusion src/pointcloud_self_exclusion.cpp)
target_include_directories(pointcloud_self_exclusion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pointcloud_self_exclusion sensor_msgs)
target_compile_options(pointcloud_self_exclusion PRIVATE -Wall -Wextra -Wpedantic)

add_library(dynamic_self_point_filter src/dynamic_self_point_filter.cpp)
target_include_directories(dynamic_self_point_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(dynamic_self_point_filter sunhorizon_obstacle_coverage)
target_link_libraries(dynamic_self_point_filter Threads::Threads)
target_compile_options(dynamic_self_point_filter PRIVATE -Wall -Wextra -Wpedantic)

set(camera_cuda_source src/body_geometry_cuda_stub.cpp)
if(SUNHORIZON_ENABLE_CAMERA_BODY_CUDA)
  enable_language(CUDA)
  find_package(CUDAToolkit 12.6 REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  if(NOT CMAKE_CUDA_COMPILER_VERSION VERSION_GREATER_EQUAL "12.6" OR CMAKE_CUDA_COMPILER_VERSION VERSION_GREATER_EQUAL "12.7")
    message(FATAL_ERROR "Camera CUDA arithmetic requires CUDA12.6")
  endif()
  if(NOT CMAKE_CUDA_FLAGS STREQUAL "" OR NOT CMAKE_CUDA_FLAGS_RELEASE STREQUAL "-O3 -DNDEBUG")
    message(FATAL_ERROR "Camera CUDA requires reviewed empty CUDA_FLAGS and Release -O3 -DNDEBUG")
  endif()
  set(camera_geometry_source "${CMAKE_CURRENT_SOURCE_DIR}/../sunhorizon_obstacle_coverage")
  set(camera_device_header "${CMAKE_CURRENT_BINARY_DIR}/camera_cuda/device_geometry.cuh")
  add_custom_command(OUTPUT "${camera_device_header}"
    COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/src/cuda/generate_geometry.py"
      --header "${camera_geometry_source}/include/sunhorizon_obstacle_coverage/dynamic_self_geometry.hpp"
      --source "${camera_geometry_source}/src/dynamic_self_geometry.cpp"
      --arm-helper "${CMAKE_CURRENT_SOURCE_DIR}/src/cuda/arm_nominal.cuh"
      --output "${camera_device_header}"
    DEPENDS src/cuda/generate_geometry.py src/cuda/arm_nominal.cuh
      "${camera_geometry_source}/include/sunhorizon_obstacle_coverage/dynamic_self_geometry.hpp"
      "${camera_geometry_source}/src/dynamic_self_geometry.cpp"
    VERBATIM)
  set(camera_cuda_source src/cuda/body_geometry_cuda.cu "${camera_device_header}")
endif()
add_library(body_geometry_cuda ${camera_cuda_source})
target_include_directories(body_geometry_cuda PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(body_geometry_cuda sunhorizon_obstacle_coverage)
target_link_libraries(body_geometry_cuda Threads::Threads)
if(SUNHORIZON_ENABLE_CAMERA_BODY_CUDA)
  set_target_properties(body_geometry_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED ON CUDA_ARCHITECTURES 87)
  target_include_directories(body_geometry_cuda PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/camera_cuda")
  target_compile_options(body_geometry_cuda PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-O3;--fmad=false;--prec-div=true;--prec-sqrt=true;--expt-relaxed-constexpr;-Xcompiler=-ffp-contract=off>)
  target_link_libraries(body_geometry_cuda CUDA::cudart CUDA::cuda_driver)
  ament_export_dependencies(CUDAToolkit)
endif()

add_library(fused_depth_self_filter src/fused_depth_self_filter.cpp)
target_include_directories(fused_depth_self_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(fused_depth_self_filter sensor_msgs sunhorizon_obstacle_coverage)
target_link_libraries(fused_depth_self_filter rgbd_freshness dynamic_self_point_filter body_geometry_cuda)
target_compile_options(fused_depth_self_filter PRIVATE -Wall -Wextra -Wpedantic)

add_executable(spatial_visibility_node src/spatial_visibility_node.cpp)
ament_target_dependencies(spatial_visibility_node
  rclcpp rcl_interfaces sensor_msgs diagnostic_msgs tf2 tf2_ros tf2_msgs
  sunhorizon_obstacle_coverage sunhorizon_navigation_interfaces)
target_link_libraries(spatial_visibility_node region_depth_producer OpenSSL::Crypto)
target_compile_options(spatial_visibility_node PRIVATE -Wall -Wextra -Wpedantic)

add_library(pointcloud_self_exclusion_runtime src/pointcloud_self_exclusion_node.cpp)
target_include_directories(pointcloud_self_exclusion_runtime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pointcloud_self_exclusion_runtime
  rclcpp rcl_interfaces sensor_msgs diagnostic_msgs tf2 tf2_ros tf2_msgs sunhorizon_obstacle_coverage sunhorizon_navigation_core)
target_link_libraries(pointcloud_self_exclusion_runtime pointcloud_self_exclusion rgbd_freshness dynamic_self_point_filter OpenSSL::Crypto)
target_compile_options(pointcloud_self_exclusion_runtime PRIVATE -Wall -Wextra -Wpedantic)
add_executable(pointcloud_self_exclusion_node src/pointcloud_self_exclusion_main.cpp)
ament_target_dependencies(pointcloud_self_exclusion_node rclcpp)
target_link_libraries(pointcloud_self_exclusion_node pointcloud_self_exclusion_runtime)

add_library(region_depth_producer src/region_depth_producer.cpp)
target_include_directories(region_depth_producer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(region_depth_producer
  rclcpp sensor_msgs diagnostic_msgs tf2 tf2_ros sunhorizon_obstacle_coverage)
target_compile_options(region_depth_producer PRIVATE -Wall -Wextra -Wpedantic)

add_executable(depth_obstacle_cloud_node src/depth_obstacle_cloud_node.cpp)
ament_target_dependencies(depth_obstacle_cloud_node rclcpp rcl_interfaces sensor_msgs diagnostic_msgs tf2 tf2_ros sunhorizon_obstacle_coverage)
target_link_libraries(depth_obstacle_cloud_node rgbd_freshness region_depth_producer)
target_compile_options(depth_obstacle_cloud_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS body_geometry_cuda pointcloud_self_exclusion_runtime fused_depth_self_filter dynamic_self_point_filter region_depth_producer rgbd_freshness pointcloud_self_exclusion
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS spatial_visibility_node rgbd_adapter_node pointcloud_self_exclusion_node depth_obstacle_cloud_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_libraries(body_geometry_cuda pointcloud_self_exclusion_runtime rgbd_freshness pointcloud_self_exclusion region_depth_producer dynamic_self_point_filter fused_depth_self_filter)
ament_export_dependencies(Threads sensor_msgs rclcpp diagnostic_msgs tf2 tf2_ros tf2_msgs OpenSSL sunhorizon_obstacle_coverage sunhorizon_navigation_core)
ament_export_targets(export_${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  if(SUNHORIZON_ENABLE_CAMERA_BODY_CUDA)
    add_executable(test_camera_cuda_replay test/test_camera_cuda_replay.cpp)
    target_link_libraries(test_camera_cuda_replay fused_depth_self_filter OpenSSL::Crypto)
    # Explicit fixture paths are required; this is never an automatic live test.
  endif()
  add_executable(test_body_geometry_cuda test/test_body_geometry_cuda.cpp)
  target_link_libraries(test_body_geometry_cuda body_geometry_cuda)
  if(SUNHORIZON_ENABLE_CAMERA_BODY_CUDA)
    target_compile_definitions(test_body_geometry_cuda PRIVATE CAMERA_CUDA_TEST_CONTEXT=1)
    target_link_libraries(test_body_geometry_cuda CUDA::cuda_driver)
  endif()
  add_test(NAME test_body_geometry_cuda COMMAND test_body_geometry_cuda)
  add_executable(test_native_projection_edge_filter test/test_native_projection_edge_filter.cpp)
  target_link_libraries(test_native_projection_edge_filter fused_depth_self_filter)
  target_compile_options(test_native_projection_edge_filter PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_native_projection_edge_filter COMMAND test_native_projection_edge_filter)
  add_executable(test_fused_depth_self_filter test/test_fused_depth_self_filter.cpp)
  target_link_libraries(test_fused_depth_self_filter fused_depth_self_filter)
  target_compile_options(test_fused_depth_self_filter PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_fused_depth_self_filter COMMAND test_fused_depth_self_filter)
  add_test(NAME test_fused_self_support_only COMMAND test_fused_depth_self_filter --self-support-only)
  add_executable(test_spatial_visibility_sampling test/test_spatial_visibility_sampling.cpp)
  target_include_directories(test_spatial_visibility_sampling PRIVATE include)
  ament_target_dependencies(test_spatial_visibility_sampling sunhorizon_obstacle_coverage)
  target_compile_options(test_spatial_visibility_sampling PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_spatial_visibility_sampling COMMAND test_spatial_visibility_sampling)
  add_executable(test_lidar_self_filter test/test_lidar_self_filter.cpp)
  ament_target_dependencies(test_lidar_self_filter rclcpp sensor_msgs tf2_ros sunhorizon_obstacle_coverage)
  target_link_libraries(test_lidar_self_filter pointcloud_self_exclusion_runtime)
  target_compile_options(test_lidar_self_filter PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_lidar_self_filter COMMAND test_lidar_self_filter)
  add_executable(test_dynamic_self_point_filter test/test_dynamic_self_point_filter.cpp)
  target_link_libraries(test_dynamic_self_point_filter dynamic_self_point_filter)
  target_compile_options(test_dynamic_self_point_filter PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_dynamic_self_point_filter COMMAND test_dynamic_self_point_filter)
  add_executable(test_rgbd_upstream_diagnostics test/test_rgbd_upstream_diagnostics.cpp)
  target_include_directories(test_rgbd_upstream_diagnostics PRIVATE include)
  ament_target_dependencies(test_rgbd_upstream_diagnostics sunhorizon_obstacle_coverage)
  target_compile_options(test_rgbd_upstream_diagnostics PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
  add_test(NAME test_rgbd_upstream_diagnostics COMMAND test_rgbd_upstream_diagnostics)
  add_executable(test_belly_stage_diagnostics test/test_belly_stage_diagnostics.cpp)
  target_include_directories(test_belly_stage_diagnostics PRIVATE include)
  target_compile_options(test_belly_stage_diagnostics PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_belly_stage_diagnostics COMMAND test_belly_stage_diagnostics)
  add_executable(region_contract_fixture test/region_contract_fixture.cpp)
  target_link_libraries(region_contract_fixture region_depth_producer)
  add_executable(test_region_depth_pairing test/test_region_depth_pairing.cpp)
  target_include_directories(test_region_depth_pairing PRIVATE include)
  target_compile_options(test_region_depth_pairing PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_region_depth_pairing COMMAND test_region_depth_pairing)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_dynamic_self_model_deadline test/test_dynamic_self_model_deadline.cpp)
  target_include_directories(test_dynamic_self_model_deadline PRIVATE include)
  ament_target_dependencies(test_dynamic_self_model_deadline rclcpp tf2 tf2_ros geometry_msgs sunhorizon_obstacle_coverage)
  target_link_libraries(test_dynamic_self_model_deadline OpenSSL::Crypto)
  ament_add_pytest_test(test_spatial_visibility_node
    test/test_spatial_visibility_node.py
    ENV "SPATIAL_VISIBILITY_EXECUTABLE=$<TARGET_FILE:spatial_visibility_node>"
    TIMEOUT 90)
  ament_add_pytest_test(test_dynamic_self_point_filter_node
    test/test_dynamic_self_point_filter_node.py
    ENV "SELF_EXCLUSION_EXECUTABLE=$<TARGET_FILE:pointcloud_self_exclusion_node>"
    TIMEOUT 120)

  ament_add_gtest(test_region_tf_diagnostics test/test_region_tf_diagnostics.cpp)
  target_include_directories(test_region_tf_diagnostics PRIVATE include)
  ament_target_dependencies(test_region_tf_diagnostics tf2)
  target_link_libraries(test_region_tf_diagnostics region_depth_producer)
  ament_add_gtest(test_rgbd_adapter_concurrency test/test_rgbd_adapter_concurrency.cpp
    TIMEOUT 30)
  ament_target_dependencies(test_rgbd_adapter_concurrency
    rclcpp sensor_msgs sunhorizon_navigation_core sunhorizon_navigation_interfaces sunhorizon_obstacle_coverage)
  target_link_libraries(test_rgbd_adapter_concurrency rgbd_freshness)
  set_tests_properties(test_rgbd_adapter_concurrency PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=215;ROS_LOCALHOST_ONLY=1")
  ament_add_gtest(test_depth_components test/test_depth_components.cpp)
  target_link_libraries(test_depth_components rgbd_freshness)
  ament_add_gtest(test_depth_projection test/test_depth_projection.cpp)
  target_link_libraries(test_depth_projection rgbd_freshness)
  add_executable(depth_components_replay test/depth_components_replay.cpp)
  target_link_libraries(depth_components_replay rgbd_freshness)
  ament_add_gtest(test_rgbd_freshness test/test_rgbd_freshness.cpp)
  target_link_libraries(test_rgbd_freshness rgbd_freshness)
  ament_add_gtest(test_pointcloud_self_exclusion_direct test/test_pointcloud_self_exclusion_direct.cpp)
  ament_target_dependencies(test_pointcloud_self_exclusion_direct rclcpp sensor_msgs tf2_ros)
  target_link_libraries(test_pointcloud_self_exclusion_direct pointcloud_self_exclusion_runtime)
  ament_add_gtest(test_pointcloud_self_exclusion test/test_pointcloud_self_exclusion.cpp)
  target_link_libraries(test_pointcloud_self_exclusion pointcloud_self_exclusion)
  ament_add_pytest_test(test_rgbd_adapter_contract
    test/test_rgbd_adapter_contract.py)
  ament_add_pytest_test(test_belly_obstacle_launch test/test_belly_obstacle_launch.py)
  ament_add_pytest_test(test_pointcloud_self_exclusion_node
    test/test_pointcloud_self_exclusion_node.py
    ENV "SELF_EXCLUSION_EXECUTABLE=$<TARGET_FILE:pointcloud_self_exclusion_node>"
    TIMEOUT 45)
  ament_add_pytest_test(test_region_depth_producer_node
    test/test_region_depth_producer_node.py
    ENV "DEPTH_OBSTACLE_EXECUTABLE=$<TARGET_FILE:depth_obstacle_cloud_node>"
    TIMEOUT 40)
  ament_add_pytest_test(test_region_complete_measurement_node
    test/test_region_complete_measurement_node.py
    ENV "DEPTH_OBSTACLE_EXECUTABLE=$<TARGET_FILE:depth_obstacle_cloud_node>"
    TIMEOUT 80)
  ament_add_pytest_test(test_region_shadow_only_node
    test/test_region_shadow_only_node.py
    ENV "DEPTH_OBSTACLE_EXECUTABLE=$<TARGET_FILE:depth_obstacle_cloud_node>"
    TIMEOUT 45)
  ament_add_pytest_test(test_depth_obstacle_timing_node
    test/test_depth_obstacle_timing_node.py
    ENV "DEPTH_OBSTACLE_EXECUTABLE=$<TARGET_FILE:depth_obstacle_cloud_node>"
    TIMEOUT 35)
endif()

ament_package()
