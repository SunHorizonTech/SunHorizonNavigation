cmake_minimum_required(VERSION 3.8)
project(sunhorizon_sensor_rate_probe)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(sensor_rate_probe src/sensor_rate_probe_node.cpp)
target_compile_features(sensor_rate_probe PUBLIC cxx_std_17)
target_include_directories(sensor_rate_probe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(sensor_rate_probe rclcpp sensor_msgs)

install(TARGETS sensor_rate_probe
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rate_tracker test/test_rate_tracker.cpp)
  target_compile_features(test_rate_tracker PUBLIC cxx_std_17)
  target_include_directories(test_rate_tracker PUBLIC include)
endif()

ament_package()
