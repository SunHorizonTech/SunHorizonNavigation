cmake_minimum_required(VERSION 3.8)
project(sunhorizon_localization_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sunhorizon_navigation_interfaces REQUIRED)
find_package(sunhorizon_navigation_core REQUIRED)
find_package(sunhorizon_3d_localizer REQUIRED)
find_package(Threads REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(localization_manager_logic src/localization_manager_logic.cpp)
target_include_directories(localization_manager_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(localization_manager_logic
  builtin_interfaces geometry_msgs nav_msgs sunhorizon_navigation_interfaces
  tf2 tf2_geometry_msgs)
target_compile_options(localization_manager_logic PRIVATE -Wall -Wextra -Wpedantic)

add_library(localization_manager_runtime SHARED src/localization_manager_node.cpp)
target_link_libraries(localization_manager_runtime PUBLIC localization_manager_logic Threads::Threads)
ament_target_dependencies(localization_manager_runtime PUBLIC
  geometry_msgs nav_msgs rclcpp std_srvs sunhorizon_navigation_interfaces tf2_ros
  visualization_msgs sunhorizon_navigation_core sunhorizon_3d_localizer)
target_compile_options(localization_manager_runtime PRIVATE -Wall -Wextra -Wpedantic)

add_executable(localization_manager_node src/localization_manager_main.cpp)
target_link_libraries(localization_manager_node localization_manager_runtime)
ament_target_dependencies(localization_manager_node rclcpp)
set_target_properties(localization_manager_logic PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(TARGETS localization_manager_logic localization_manager_runtime
  EXPORT export_localization_manager_logic
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS localization_manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_localization_manager_logic HAS_LIBRARY_TARGET)
ament_export_dependencies(
  builtin_interfaces geometry_msgs nav_msgs sunhorizon_navigation_interfaces
  tf2 tf2_geometry_msgs rclcpp std_srvs tf2_ros visualization_msgs sunhorizon_navigation_core sunhorizon_3d_localizer Threads)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_localization_manager_logic
    test/test_localization_manager_logic.cpp)
  target_link_libraries(test_localization_manager_logic localization_manager_logic)
  ament_target_dependencies(test_localization_manager_logic
    builtin_interfaces geometry_msgs nav_msgs sunhorizon_navigation_interfaces
    tf2 tf2_geometry_msgs)
  ament_add_gtest(test_localization_direct_runtime test/test_localization_direct_runtime.cpp)
  target_link_libraries(test_localization_direct_runtime localization_manager_runtime)
  ament_target_dependencies(test_localization_direct_runtime rclcpp tf2 tf2_geometry_msgs sunhorizon_3d_localizer)

endif()

ament_package()
