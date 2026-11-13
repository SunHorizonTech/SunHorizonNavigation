cmake_minimum_required(VERSION 3.16)
project(SunHorizonNavigationOffline LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
enable_testing()
find_package(Threads REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(core "${CMAKE_CURRENT_SOURCE_DIR}/ros2_ws/src/sunhorizon_navigation_core")
set(coverage "${CMAKE_CURRENT_SOURCE_DIR}/ros2_ws/src/sunhorizon_obstacle_coverage")
add_library(navigation_core
  ${core}/src/geometry_recovery.cpp
  ${core}/src/localization_evidence.cpp
  ${core}/src/localization_gate.cpp
  ${core}/src/navigation_state_machine.cpp
  ${core}/src/platform_evidence.cpp
  ${core}/src/projection_grid.cpp
  ${core}/src/rgbd_validation.cpp
  ${core}/src/safety_gate.cpp
  ${core}/src/visual_pose.cpp)
target_include_directories(navigation_core PUBLIC ${core}/include)
target_compile_options(navigation_core PRIVATE -Wall -Wextra -Wpedantic)
add_library(coverage_logic
  ${coverage}/src/coverage_evaluator.cpp
  ${coverage}/src/region_depth_support.cpp
  ${coverage}/src/region_support_tracker.cpp
  ${coverage}/src/dynamic_self_geometry.cpp
  ${coverage}/src/dynamic_self_geometry_cuda_contract.cpp
  ${coverage}/src/spatial_visibility_fusion.cpp)
target_include_directories(coverage_logic PUBLIC ${coverage}/include)
target_compile_options(coverage_logic PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(coverage_logic PUBLIC navigation_core Threads::Threads)
add_executable(core_tests tests/core_tests.cpp)
target_link_libraries(core_tests PRIVATE navigation_core Threads::Threads)
target_compile_options(core_tests PRIVATE -UNDEBUG)
add_test(NAME core_tests COMMAND core_tests)
foreach(test spatial_evidence_tracker dynamic_self_geometry spatial_visibility_fusion
    rgbd_fault_reference region_upstream_deadline_policy processed_observation_diagnostics
    belly_health_policy region_support_tracker region_depth_support)
  add_executable(test_${test} ${coverage}/test/test_${test}.cpp)
  target_link_libraries(test_${test} PRIVATE coverage_logic)
  target_compile_options(test_${test} PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
  add_test(NAME test_${test} COMMAND test_${test})
endforeach()
add_test(NAME map_bundle COMMAND ${Python3_EXECUTABLE} -B -m unittest discover
  -s ${CMAKE_CURRENT_SOURCE_DIR}/tests -p test_map_bundle.py -v)
set(diag "${CMAKE_CURRENT_SOURCE_DIR}/ros2_ws/src/sunhorizon_navigation_diagnostics")
add_test(NAME diagnostics COMMAND ${Python3_EXECUTABLE} -B -m unittest discover
  -s ${diag}/test -p test_*.py -v)
set_tests_properties(diagnostics PROPERTIES ENVIRONMENT "PYTHONPATH=${diag}")
