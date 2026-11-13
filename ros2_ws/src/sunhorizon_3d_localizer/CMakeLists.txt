cmake_minimum_required(VERSION 3.8)
project(sunhorizon_3d_localizer)

# Existing safety/registration cores also back the exported direct runtime.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(SUNHORIZON_ENABLE_NDT_OMP
  "Build the optional tier4/ndt_omp registration backend" OFF)

option(SUNHORIZON_ENABLE_NDT_CUDA
  "Allow explicit startup selection of ndt_omp CUDA DIRECT7 computation" OFF)
if(SUNHORIZON_ENABLE_NDT_CUDA AND NOT SUNHORIZON_ENABLE_NDT_OMP)
  message(FATAL_ERROR "SUNHORIZON_ENABLE_NDT_CUDA requires SUNHORIZON_ENABLE_NDT_OMP")
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL 1.12 REQUIRED COMPONENTS common filters io kdtree registration)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sunhorizon_navigation_core REQUIRED)
find_package(Threads REQUIRED)
find_package(sunhorizon_navigation_interfaces REQUIRED)
if(SUNHORIZON_ENABLE_NDT_OMP)
  find_package(ndt_omp CONFIG REQUIRED)
  if(SUNHORIZON_ENABLE_NDT_CUDA)
    if(NOT ndt_omp_RUNTIME_ABI EQUAL 2 OR NOT ndt_omp_CUDA_ENABLED)
      message(FATAL_ERROR "CUDA requires ndt_omp runtime ABI 2 built with NDT_OMP_ENABLE_CUDA=ON")
    endif()
  endif()
endif()

add_library(sunhorizon_ndt_backend src/ndt_backend.cpp)
target_include_directories(sunhorizon_ndt_backend
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_include_directories(sunhorizon_ndt_backend SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_compile_definitions(sunhorizon_ndt_backend PUBLIC ${PCL_DEFINITIONS})
# ament_target_dependencies() uses CMake's plain target_link_libraries()
# signature, so keep this target on that signature for the optional ndt_omp
# dependency below as well.
target_link_libraries(sunhorizon_ndt_backend ${PCL_LIBRARIES})
target_compile_options(sunhorizon_ndt_backend PRIVATE -Wall -Wextra -Wpedantic)
if(SUNHORIZON_ENABLE_NDT_OMP)
  target_compile_definitions(sunhorizon_ndt_backend
    PRIVATE SUNHORIZON_ENABLE_NDT_OMP=1)
  ament_target_dependencies(sunhorizon_ndt_backend ndt_omp)
  if(SUNHORIZON_ENABLE_NDT_CUDA)
    target_compile_definitions(sunhorizon_ndt_backend PRIVATE SUNHORIZON_ENABLE_NDT_CUDA=1)
  endif()
endif()

add_library(sunhorizon_tracking_odom_logic src/tracking_odom_logic.cpp)
target_include_directories(sunhorizon_tracking_odom_logic
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_include_directories(sunhorizon_tracking_odom_logic SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_compile_options(sunhorizon_tracking_odom_logic PRIVATE -Wall -Wextra -Wpedantic)

add_library(sunhorizon_direct_map_tracking_logic src/direct_map_tracking_logic.cpp)
target_include_directories(sunhorizon_direct_map_tracking_logic
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(sunhorizon_direct_map_tracking_logic
  sunhorizon_tracking_odom_logic)
target_compile_options(sunhorizon_direct_map_tracking_logic
  PRIVATE -Wall -Wextra -Wpedantic)

add_library(sunhorizon_ndt_match_rate_gate src/ndt_match_rate_gate.cpp)
target_include_directories(sunhorizon_ndt_match_rate_gate
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(sunhorizon_ndt_match_rate_gate
  PRIVATE -Wall -Wextra -Wpedantic)

add_library(sunhorizon_ndt_fitness_failure_gate
  src/ndt_fitness_failure_gate.cpp)
target_include_directories(sunhorizon_ndt_fitness_failure_gate
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(sunhorizon_ndt_fitness_failure_gate
  PRIVATE -Wall -Wextra -Wpedantic)

add_library(sunhorizon_ndt_freshness src/ndt_freshness.cpp)
target_include_directories(sunhorizon_ndt_freshness
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(sunhorizon_ndt_freshness
  PRIVATE -Wall -Wextra -Wpedantic)

add_library(sunhorizon_ndt_result_step_gate src/ndt_result_step_gate.cpp)
target_include_directories(sunhorizon_ndt_result_step_gate
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(sunhorizon_ndt_result_step_gate
  sunhorizon_tracking_odom_logic)
target_compile_options(sunhorizon_ndt_result_step_gate
  PRIVATE -Wall -Wextra -Wpedantic)

add_library(sunhorizon_ndt_startup_alignment_gate
  src/ndt_startup_alignment_gate.cpp)
target_include_directories(sunhorizon_ndt_startup_alignment_gate
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(sunhorizon_ndt_startup_alignment_gate
  PRIVATE -Wall -Wextra -Wpedantic)

add_library(sunhorizon_ndt_prior_sync_logic src/ndt_prior_sync_logic.cpp)
target_include_directories(sunhorizon_ndt_prior_sync_logic
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_include_directories(sunhorizon_ndt_prior_sync_logic SYSTEM
  PUBLIC ${PCL_INCLUDE_DIRS})
target_compile_options(sunhorizon_ndt_prior_sync_logic
  PRIVATE -Wall -Wextra -Wpedantic)

add_library(sunhorizon_base_z_gate src/base_z_gate.cpp)
target_include_directories(sunhorizon_base_z_gate
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(sunhorizon_base_z_gate PRIVATE -Wall -Wextra -Wpedantic)

add_library(sunhorizon_scan_range_gate src/scan_range_gate.cpp)
target_include_directories(sunhorizon_scan_range_gate
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(sunhorizon_scan_range_gate
  PRIVATE -Wall -Wextra -Wpedantic)

add_library(sunhorizon_raw_scan_admission_gate
  src/raw_scan_admission_gate.cpp)
target_include_directories(sunhorizon_raw_scan_admission_gate
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(sunhorizon_raw_scan_admission_gate
  PRIVATE -Wall -Wextra -Wpedantic)

add_library(sunhorizon_source_map_bounds_filter
  src/source_map_bounds_filter.cpp)
target_include_directories(sunhorizon_source_map_bounds_filter
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_include_directories(sunhorizon_source_map_bounds_filter SYSTEM
  PUBLIC ${PCL_INCLUDE_DIRS})
target_compile_definitions(sunhorizon_source_map_bounds_filter
  PUBLIC ${PCL_DEFINITIONS})
target_link_libraries(sunhorizon_source_map_bounds_filter ${PCL_LIBRARIES})
target_compile_options(sunhorizon_source_map_bounds_filter
  PRIVATE -Wall -Wextra -Wpedantic)

add_library(sunhorizon_ndt_match_audit src/ndt_match_audit.cpp)
target_include_directories(sunhorizon_ndt_match_audit
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(sunhorizon_ndt_match_audit
  PRIVATE -Wall -Wextra -Wpedantic)

add_library(sunhorizon_ndt_registration_quality
  src/ndt_registration_quality.cpp)
target_include_directories(sunhorizon_ndt_registration_quality
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_include_directories(sunhorizon_ndt_registration_quality SYSTEM
  PUBLIC ${PCL_INCLUDE_DIRS})
target_compile_definitions(sunhorizon_ndt_registration_quality
  PUBLIC ${PCL_DEFINITIONS})
target_link_libraries(sunhorizon_ndt_registration_quality ${PCL_LIBRARIES})
target_compile_options(sunhorizon_ndt_registration_quality
  PRIVATE -Wall -Wextra -Wpedantic)

add_library(sunhorizon_local_map_target_logic src/local_map_target_logic.cpp)
target_include_directories(sunhorizon_local_map_target_logic
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(sunhorizon_local_map_target_logic
  PRIVATE -Wall -Wextra -Wpedantic)

add_executable(ndt_offline_evaluator src/ndt_offline_evaluator.cpp)
target_include_directories(ndt_offline_evaluator SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(ndt_offline_evaluator PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(ndt_offline_evaluator
  sunhorizon_ndt_backend ${PCL_LIBRARIES})

add_executable(sunhorizon_ndt_localizer_node src/ndt_localizer_node.cpp)
add_library(sunhorizon_ndt_direct_runtime SHARED src/ndt_localizer_node.cpp)
target_compile_definitions(sunhorizon_ndt_direct_runtime PRIVATE SUNHORIZON_NDT_NO_MAIN=1)
target_include_directories(sunhorizon_ndt_direct_runtime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
foreach(ndt_owner sunhorizon_ndt_localizer_node sunhorizon_ndt_direct_runtime)
target_include_directories(${ndt_owner} PRIVATE include)
target_include_directories(${ndt_owner} SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(${ndt_owner} PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(${ndt_owner}
  sunhorizon_direct_map_tracking_logic sunhorizon_local_map_target_logic
  sunhorizon_base_z_gate
  sunhorizon_ndt_backend
  sunhorizon_ndt_fitness_failure_gate
  sunhorizon_ndt_freshness
  sunhorizon_ndt_prior_sync_logic
  sunhorizon_ndt_match_audit
  sunhorizon_ndt_match_rate_gate
  sunhorizon_ndt_result_step_gate
  sunhorizon_ndt_startup_alignment_gate
  sunhorizon_ndt_registration_quality
  sunhorizon_raw_scan_admission_gate sunhorizon_scan_range_gate
  sunhorizon_source_map_bounds_filter
  sunhorizon_tracking_odom_logic ${PCL_LIBRARIES} Threads::Threads OpenSSL::Crypto)
ament_target_dependencies(${ndt_owner}
  diagnostic_msgs geometry_msgs nav_msgs rclcpp rcl_interfaces sensor_msgs
  sunhorizon_navigation_interfaces sunhorizon_navigation_core)
endforeach()
# The ROS-only public header has no PCL/core target dependency. Keep the private
# implementation libraries out of the installed CMake interface.
set_property(TARGET sunhorizon_ndt_direct_runtime PROPERTY INTERFACE_LINK_LIBRARIES "")
target_compile_options(sunhorizon_ndt_direct_runtime PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS sunhorizon_ndt_direct_runtime EXPORT export_ndt_direct_runtime
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(export_ndt_direct_runtime HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs nav_msgs rclcpp sensor_msgs sunhorizon_navigation_interfaces)

foreach(target ndt_offline_evaluator sunhorizon_ndt_localizer_node)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
endforeach()

install(TARGETS sunhorizon_tracking_odom_logic sunhorizon_direct_map_tracking_logic
  sunhorizon_base_z_gate
  sunhorizon_ndt_backend
  sunhorizon_local_map_target_logic sunhorizon_ndt_match_audit
  sunhorizon_ndt_registration_quality
  sunhorizon_ndt_fitness_failure_gate sunhorizon_ndt_freshness
  sunhorizon_ndt_result_step_gate
  sunhorizon_ndt_startup_alignment_gate
  sunhorizon_ndt_prior_sync_logic
  sunhorizon_ndt_match_rate_gate
  sunhorizon_raw_scan_admission_gate sunhorizon_scan_range_gate
  sunhorizon_source_map_bounds_filter
  ndt_offline_evaluator
  sunhorizon_ndt_localizer_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_executable(test_ndt_pending_clouds test/test_ndt_pending_clouds.cpp)
  target_include_directories(test_ndt_pending_clouds PRIVATE include)
  target_link_libraries(test_ndt_pending_clouds sunhorizon_raw_scan_admission_gate)
  add_test(NAME test_ndt_pending_clouds COMMAND test_ndt_pending_clouds)
  ament_add_gtest(test_ndt_direct_runtime test/test_ndt_direct_runtime.cpp)
  if(TARGET test_ndt_direct_runtime)
    target_include_directories(test_ndt_direct_runtime SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
    target_link_libraries(test_ndt_direct_runtime sunhorizon_ndt_direct_runtime)
    ament_target_dependencies(test_ndt_direct_runtime
      rclcpp sensor_msgs geometry_msgs nav_msgs sunhorizon_navigation_interfaces)
  endif()
  ament_add_gtest(test_ndt_backend test/test_ndt_backend.cpp)
  if(TARGET test_ndt_backend)
    target_include_directories(test_ndt_backend PRIVATE include)
    target_link_libraries(test_ndt_backend sunhorizon_ndt_backend)
    if(SUNHORIZON_ENABLE_NDT_OMP)
      target_compile_definitions(test_ndt_backend PRIVATE SUNHORIZON_TEST_NDT_OMP=1)
      ament_target_dependencies(test_ndt_backend ndt_omp)
    endif()
  endif()
  ament_add_gtest(test_tracking_odom_logic test/test_tracking_odom_logic.cpp)
  if(TARGET test_tracking_odom_logic)
    target_include_directories(test_tracking_odom_logic PRIVATE include)
    target_link_libraries(test_tracking_odom_logic sunhorizon_tracking_odom_logic)
  endif()
  ament_add_gtest(test_direct_map_tracking_logic
    test/test_direct_map_tracking_logic.cpp)
  if(TARGET test_direct_map_tracking_logic)
    target_include_directories(test_direct_map_tracking_logic PRIVATE include)
    target_link_libraries(test_direct_map_tracking_logic
      sunhorizon_direct_map_tracking_logic)
  endif()
  ament_add_gtest(test_ndt_match_rate_gate test/test_ndt_match_rate_gate.cpp)
  if(TARGET test_ndt_match_rate_gate)
    target_include_directories(test_ndt_match_rate_gate PRIVATE include)
    target_link_libraries(test_ndt_match_rate_gate
      sunhorizon_ndt_match_rate_gate)
  endif()
  ament_add_gtest(test_ndt_fitness_failure_gate
    test/test_ndt_fitness_failure_gate.cpp)
  if(TARGET test_ndt_fitness_failure_gate)
    target_include_directories(test_ndt_fitness_failure_gate PRIVATE include)
    target_link_libraries(test_ndt_fitness_failure_gate
      sunhorizon_ndt_fitness_failure_gate)
  endif()
  ament_add_gtest(test_ndt_freshness test/test_ndt_freshness.cpp)
  if(TARGET test_ndt_freshness)
    target_include_directories(test_ndt_freshness PRIVATE include)
    target_link_libraries(test_ndt_freshness sunhorizon_ndt_freshness)
  endif()
  ament_add_gtest(test_ndt_result_step_gate test/test_ndt_result_step_gate.cpp)
  if(TARGET test_ndt_result_step_gate)
    target_include_directories(test_ndt_result_step_gate PRIVATE include)
    target_link_libraries(test_ndt_result_step_gate
      sunhorizon_ndt_result_step_gate)
  endif()
  ament_add_gtest(test_ndt_startup_alignment_gate
    test/test_ndt_startup_alignment_gate.cpp)
  if(TARGET test_ndt_startup_alignment_gate)
    target_include_directories(test_ndt_startup_alignment_gate PRIVATE include)
    target_link_libraries(test_ndt_startup_alignment_gate
      sunhorizon_ndt_startup_alignment_gate)
  endif()
  ament_add_gtest(test_ndt_prior_sync_logic
    test/test_ndt_prior_sync_logic.cpp)
  if(TARGET test_ndt_prior_sync_logic)
    target_include_directories(test_ndt_prior_sync_logic PRIVATE include)
    target_link_libraries(test_ndt_prior_sync_logic
      sunhorizon_ndt_prior_sync_logic)
  endif()
  ament_add_gtest(test_base_z_gate test/test_base_z_gate.cpp)
  if(TARGET test_base_z_gate)
    target_include_directories(test_base_z_gate PRIVATE include)
    target_link_libraries(test_base_z_gate sunhorizon_base_z_gate)
  endif()
  ament_add_gtest(test_scan_range_gate test/test_scan_range_gate.cpp)
  if(TARGET test_scan_range_gate)
    target_include_directories(test_scan_range_gate PRIVATE include)
    target_link_libraries(test_scan_range_gate sunhorizon_scan_range_gate)
  endif()
  ament_add_gtest(test_raw_scan_admission_gate
    test/test_raw_scan_admission_gate.cpp)
  if(TARGET test_raw_scan_admission_gate)
    target_include_directories(test_raw_scan_admission_gate PRIVATE include)
    target_link_libraries(test_raw_scan_admission_gate
      sunhorizon_raw_scan_admission_gate)
  endif()
  ament_add_gtest(test_source_map_bounds_filter
    test/test_source_map_bounds_filter.cpp)
  if(TARGET test_source_map_bounds_filter)
    target_include_directories(test_source_map_bounds_filter PRIVATE include)
    target_link_libraries(test_source_map_bounds_filter
      sunhorizon_source_map_bounds_filter)
  endif()
  ament_add_gtest(test_ndt_match_audit test/test_ndt_match_audit.cpp test/test_ndt_worker_control.cpp)
  if(TARGET test_ndt_match_audit)
    target_include_directories(test_ndt_match_audit PRIVATE include)
    target_link_libraries(test_ndt_match_audit sunhorizon_ndt_match_audit Threads::Threads)
  endif()
  ament_add_gtest(test_ndt_registration_quality
    test/test_ndt_registration_quality.cpp)
  if(TARGET test_ndt_registration_quality)
    target_include_directories(test_ndt_registration_quality PRIVATE include)
    target_link_libraries(test_ndt_registration_quality
      sunhorizon_ndt_registration_quality)
  endif()
  ament_add_gtest(test_local_map_target_logic
    test/test_local_map_target_logic.cpp)
  if(TARGET test_local_map_target_logic)
    target_include_directories(test_local_map_target_logic PRIVATE include)
    target_link_libraries(test_local_map_target_logic
      sunhorizon_local_map_target_logic)
  endif()
endif()

ament_package()
