cmake_minimum_required(VERSION 3.8)
project(sunhorizon_airy_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_library(airy_adapter_runtime SHARED src/airy_adapter_node.cpp)
target_include_directories(airy_adapter_runtime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(airy_adapter_runtime
  diagnostic_msgs
  rclcpp
  sensor_msgs
  tf2
)
target_compile_options(airy_adapter_runtime PRIVATE -Wall -Wextra -Wpedantic)

add_executable(airy_adapter_node src/airy_adapter_main.cpp)
target_link_libraries(airy_adapter_node airy_adapter_runtime)
install(TARGETS airy_adapter_runtime EXPORT export_airy_adapter_runtime
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS airy_adapter_node DESTINATION lib/${PROJECT_NAME})
ament_export_targets(export_airy_adapter_runtime HAS_LIBRARY_TARGET)
ament_export_dependencies(diagnostic_msgs rclcpp sensor_msgs tf2)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_direct_adapter test/test_direct_adapter.cpp)
  target_link_libraries(test_direct_adapter airy_adapter_runtime)
  add_test(NAME test_direct_adapter COMMAND test_direct_adapter)
  add_executable(test_imu_normalization test/test_imu_normalization.cpp)
  target_include_directories(test_imu_normalization PRIVATE include)
  ament_target_dependencies(test_imu_normalization sensor_msgs tf2)
  target_compile_options(test_imu_normalization PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_imu_normalization COMMAND test_imu_normalization)

  add_executable(test_header_time_rebaser test/test_header_time_rebaser.cpp)
  target_include_directories(test_header_time_rebaser PRIVATE include)
  target_compile_options(test_header_time_rebaser PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_header_time_rebaser COMMAND test_header_time_rebaser)

  add_executable(test_qos_history test/test_qos_history.cpp)
  target_include_directories(test_qos_history PRIVATE include)
  target_compile_options(test_qos_history PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_qos_history COMMAND test_qos_history)
endif()

ament_package()
