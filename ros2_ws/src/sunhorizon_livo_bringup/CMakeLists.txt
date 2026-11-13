cmake_minimum_required(VERSION 3.8)
project(sunhorizon_livo_bringup)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sunhorizon_navigation_core REQUIRED)
find_package(sunhorizon_navigation_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(point_cloud_conversion src/point_cloud_conversion.cpp)
target_include_directories(point_cloud_conversion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(point_cloud_conversion sensor_msgs)
target_compile_options(point_cloud_conversion PRIVATE -Wall -Wextra -Wpedantic)

add_executable(livo_adapter_node src/livo_adapter_node.cpp)
add_library(livo_direct_adapter SHARED src/livo_adapter_node.cpp)
target_compile_definitions(livo_direct_adapter PRIVATE SUNHORIZON_LIVO_ADAPTER_NO_MAIN=1)
target_include_directories(livo_direct_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
foreach(livo_owner livo_adapter_node livo_direct_adapter)
target_include_directories(${livo_owner} PRIVATE include)
target_link_libraries(${livo_owner} point_cloud_conversion)
ament_target_dependencies(${livo_owner}
  diagnostic_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  sunhorizon_navigation_interfaces
  sunhorizon_navigation_core
  tf2
  tf2_geometry_msgs)
target_compile_options(${livo_owner} PRIVATE -Wall -Wextra -Wpedantic)
endforeach()
# Keep implementation-only dependencies private, but export the core evidence
# header dependency used by the public direct adapter/causal velocity headers.
set_property(TARGET livo_direct_adapter PROPERTY INTERFACE_LINK_LIBRARIES
  sunhorizon_navigation_core::sunhorizon_navigation_core)
install(TARGETS livo_direct_adapter EXPORT export_livo_direct_adapter
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(export_livo_direct_adapter HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(nav_msgs rclcpp sensor_msgs sunhorizon_navigation_interfaces sunhorizon_navigation_core)

add_library(geometry_voxel_map src/geometry_voxel_map.cpp)
target_include_directories(geometry_voxel_map
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_include_directories(geometry_voxel_map SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_compile_definitions(geometry_voxel_map PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(geometry_voxel_map
  PUBLIC ${PCL_LIBRARIES} nlohmann_json::nlohmann_json OpenSSL::Crypto)
target_compile_options(geometry_voxel_map PRIVATE -Wall -Wextra -Wpedantic)

add_executable(livo_geometry_voxelizer src/livo_geometry_voxelizer.cpp)
target_link_libraries(livo_geometry_voxelizer geometry_voxel_map)
target_compile_options(livo_geometry_voxelizer PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS
  geometry_voxel_map livo_adapter_node livo_geometry_voxelizer point_cloud_conversion

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS
  scripts/livo_ready_exec.py
  scripts/livo_runtime_supervisor.py
  scripts/project_livo_map_to_2d.py
  scripts/run_livo_offline_replay.sh

  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_propagation_causal_audit test/test_propagation_causal_audit.cpp)
  target_include_directories(test_propagation_causal_audit PRIVATE include)
  add_test(NAME test_propagation_causal_audit COMMAND test_propagation_causal_audit)
  add_executable(test_causal_body_velocity test/test_causal_body_velocity.cpp)
  target_include_directories(test_causal_body_velocity PRIVATE include)
  ament_target_dependencies(test_causal_body_velocity sunhorizon_navigation_core)
  add_test(NAME test_causal_body_velocity COMMAND test_causal_body_velocity)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_livo_direct_adapter test/test_livo_direct_adapter.cpp)
  target_link_libraries(test_livo_direct_adapter livo_direct_adapter)
  ament_target_dependencies(test_livo_direct_adapter
    rclcpp sensor_msgs nav_msgs sunhorizon_navigation_interfaces sunhorizon_navigation_core)
  ament_add_gtest(point_cloud_conversion_test test/test_point_cloud_conversion.cpp)
  target_link_libraries(point_cloud_conversion_test point_cloud_conversion)
  ament_target_dependencies(point_cloud_conversion_test sensor_msgs)
  ament_add_gtest(
    measurement_continuity_test
    test/test_measurement_continuity.cpp)
  target_include_directories(measurement_continuity_test PRIVATE include)
  ament_add_gtest(
    measurement_freshness_test
    test/test_measurement_freshness.cpp)
  target_include_directories(measurement_freshness_test PRIVATE include)
  ament_add_gtest(locked_time_snapshot_test test/test_locked_time_snapshot.cpp)
  target_include_directories(locked_time_snapshot_test PRIVATE include)
  ament_add_gtest(
    candidate_admission_diagnostics_test
    test/test_candidate_admission_diagnostics.cpp)
  target_include_directories(candidate_admission_diagnostics_test PRIVATE include)
  ament_add_gtest(
    propagation_watchdog_test
    test/test_propagation_watchdog.cpp)
  target_include_directories(propagation_watchdog_test PRIVATE include)

  ament_add_gtest(
    timestamp_rate_gate_test
    test/test_timestamp_rate_gate.cpp)
  target_include_directories(timestamp_rate_gate_test PRIVATE include)

  ament_add_gtest(
    geometry_voxel_map_test
    test/test_geometry_voxel_map.cpp)
  if(TARGET geometry_voxel_map_test)
    target_link_libraries(geometry_voxel_map_test geometry_voxel_map)
  endif()

endif()

ament_package()
