cmake_minimum_required(VERSION 3.8)
project(sunhorizon_map_projector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sunhorizon_navigation_core REQUIRED)

add_executable(map_projector_node src/map_projector_node.cpp)
target_include_directories(map_projector_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(map_projector_node ${PCL_LIBRARIES} OpenSSL::Crypto)
ament_target_dependencies(map_projector_node
  rclcpp std_srvs sunhorizon_navigation_core)
target_compile_options(map_projector_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS map_projector_node DESTINATION lib/${PROJECT_NAME})

ament_package()
