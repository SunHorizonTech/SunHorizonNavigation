cmake_minimum_required(VERSION 3.8)
project(sunhorizon_perception_runtime)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(orocos_kdl REQUIRED)
set(dependencies tf2_geometry_msgs rclcpp rcl_interfaces rmw sensor_msgs geometry_msgs std_msgs diagnostic_msgs tf2 tf2_ros urdf kdl_parser
  sunhorizon_camera_runtime sunhorizon_navigation_core sunhorizon_navigation_interfaces
  sunhorizon_obstacle_coverage sunhorizon_rgbd_adapter sunhorizon_platform_adapter_casbot02)
foreach(dependency ${dependencies})
  find_package(${dependency} REQUIRED)
endforeach()
add_library(perception_runtime
  src/classified_obstacle_frame.cpp
  src/near_ground_clearance.cpp
  src/ground_reference_owner.cpp
  src/ground_classification_snapshot.cpp
  src/perception_engine.cpp src/joint_kinematics.cpp src/joint_ingress.cpp
  src/runtime_config.cpp src/rgbd_health.cpp src/runtime.cpp)
target_include_directories(perception_runtime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  ${orocos_kdl_INCLUDE_DIRS})
ament_target_dependencies(perception_runtime ${dependencies})
target_link_libraries(perception_runtime
  sunhorizon_camera_runtime::camera_runtime
  sunhorizon_platform_adapter_casbot02::live_joint_state_bridge
  sunhorizon_rgbd_adapter::fused_depth_self_filter
  sunhorizon_obstacle_coverage::spatial_visibility_fusion
  sunhorizon_rgbd_adapter::region_depth_producer
  ${orocos_kdl_LIBRARIES} Threads::Threads OpenSSL::Crypto nlohmann_json::nlohmann_json)
target_compile_options(perception_runtime PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS perception_runtime EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY contract/ DESTINATION share/${PROJECT_NAME}/contract)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies} orocos_kdl OpenSSL Threads nlohmann_json)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(test_body_geometry_backend test/test_body_geometry_backend.cpp)
  target_include_directories(test_body_geometry_backend PRIVATE include)
  add_test(NAME test_body_geometry_backend COMMAND test_body_geometry_backend)
  add_executable(test_body_role_dispatch test/test_body_role_dispatch.cpp)
  target_include_directories(test_body_role_dispatch PRIVATE include)
  ament_target_dependencies(test_body_role_dispatch sunhorizon_camera_runtime)
  target_link_libraries(test_body_role_dispatch Threads::Threads)
  add_test(NAME test_body_role_dispatch COMMAND test_body_role_dispatch)
  set_tests_properties(test_body_role_dispatch PROPERTIES TIMEOUT 15)
  add_executable(test_body_pair_witness test/test_body_pair_witness.cpp)
  add_test(NAME test_body_pair_witness COMMAND test_body_pair_witness)
  add_executable(test_chest_candidate_history test/test_chest_candidate_history.cpp)
  add_test(NAME test_chest_candidate_history COMMAND test_chest_candidate_history)
  add_test(NAME test_joint_startup_contract
    COMMAND ${Python3_EXECUTABLE} -B ${CMAKE_CURRENT_SOURCE_DIR}/test/test_joint_startup_contract.py)
  foreach(test joint_kinematics joint_ingress_contract runtime_boundary attended_geometry)
    ament_add_gtest(test_${test} test/test_${test}.cpp)
    target_link_libraries(test_${test} perception_runtime)
  endforeach()
  foreach(test body_replay_snapshot perception_engine rgbd_health runtime_config collision_height_query classified_obstacle_frame ground_reference_owner near_ground_clearance)
    add_executable(test_${test} test/test_${test}.cpp)
    target_link_libraries(test_${test} perception_runtime)
    add_test(NAME test_${test} COMMAND test_${test})
  endforeach()
  add_test(NAME test_perception_self_support_only COMMAND test_perception_engine --self-support-only)
endif()
ament_package()
