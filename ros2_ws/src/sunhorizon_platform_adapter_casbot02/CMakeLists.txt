cmake_minimum_required(VERSION 3.8)
project(sunhorizon_platform_adapter_casbot02)

include(CTest)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(crb_ros_msg REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sunhorizon_navigation_interfaces REQUIRED)
find_package(sunhorizon_navigation_core REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(joint_log_evidence src/joint_log_evidence.cpp)
target_include_directories(joint_log_evidence PUBLIC include)
ament_target_dependencies(joint_log_evidence sunhorizon_navigation_core)
target_compile_options(joint_log_evidence PRIVATE -Wall -Wextra -Wpedantic)

add_library(live_joint_state_bridge src/live_joint_state_bridge.cpp)
target_include_directories(live_joint_state_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(live_joint_state_bridge PRIVATE -Wall -Wextra -Wpedantic)

add_executable(live_joint_state_bridge_node src/live_joint_state_bridge_node.cpp)
target_include_directories(live_joint_state_bridge_node PRIVATE include)
ament_target_dependencies(live_joint_state_bridge_node rclcpp sensor_msgs std_msgs)
target_link_libraries(live_joint_state_bridge_node live_joint_state_bridge)
target_compile_options(live_joint_state_bridge_node PRIVATE -Wall -Wextra -Wpedantic)

add_executable(live_joint_state_source_capture src/live_joint_state_source_capture.cpp)
ament_target_dependencies(live_joint_state_source_capture rclcpp sensor_msgs)
target_compile_options(live_joint_state_source_capture PRIVATE -Wall -Wextra -Wpedantic)

add_executable(platform_adapter_node src/platform_adapter_node.cpp)
target_include_directories(platform_adapter_node PRIVATE include)
ament_target_dependencies(platform_adapter_node
  diagnostic_msgs geometry_msgs rclcpp sensor_msgs std_msgs sunhorizon_navigation_core
  sunhorizon_navigation_interfaces tf2 tf2_geometry_msgs tf2_ros)
# The vendor crb_ros_msg export includes its Python generator target.  The Orin
# image was built with a cross-compilation sysroot and that target retains an
# invalid absolute libpython path.  This C++ node only needs the generated C++
# headers and C++ type support, so link that target explicitly instead of
# importing every exported crb_ros_msg library via ament_target_dependencies.
target_link_libraries(platform_adapter_node
  crb_ros_msg::crb_ros_msg__rosidl_typesupport_cpp joint_log_evidence)
target_compile_options(platform_adapter_node PRIVATE -Wall -Wextra -Wpedantic)

add_executable(vendor_velocity_adapter_node src/vendor_velocity_adapter_node.cpp)
target_include_directories(vendor_velocity_adapter_node PRIVATE include)
ament_target_dependencies(vendor_velocity_adapter_node
  geometry_msgs rclcpp std_srvs sunhorizon_navigation_interfaces)
target_compile_options(vendor_velocity_adapter_node PRIVATE -Wall -Wextra -Wpedantic)

add_executable(ethercat_evidence_node src/ethercat_evidence_node.cpp)
ament_target_dependencies(ethercat_evidence_node rclcpp std_msgs sunhorizon_navigation_core)
target_compile_options(ethercat_evidence_node PRIVATE -Wall -Wextra -Wpedantic)

add_executable(joint_log_evidence_node src/joint_log_evidence_node.cpp)
target_include_directories(joint_log_evidence_node PRIVATE include)
ament_target_dependencies(joint_log_evidence_node diagnostic_msgs rclcpp sensor_msgs std_msgs)
target_link_libraries(joint_log_evidence_node joint_log_evidence)
target_compile_options(joint_log_evidence_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS
  platform_adapter_node vendor_velocity_adapter_node ethercat_evidence_node
  joint_log_evidence joint_log_evidence_node
  live_joint_state_bridge_node live_joint_state_source_capture
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS live_joint_state_bridge
  EXPORT export_live_joint_state_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_include_directories(include)
ament_export_targets(export_live_joint_state_bridge HAS_LIBRARY_TARGET)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY contract launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_joint_log_evidence test/test_joint_log_evidence.cpp)
  target_include_directories(test_joint_log_evidence PRIVATE include)
  target_link_libraries(test_joint_log_evidence joint_log_evidence)
  target_compile_options(test_joint_log_evidence PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME joint_log_evidence COMMAND test_joint_log_evidence)
  add_executable(test_vendor_velocity_contract test/test_vendor_velocity_contract.cpp)
  target_include_directories(test_vendor_velocity_contract PRIVATE include)
  target_compile_options(test_vendor_velocity_contract PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME vendor_velocity_contract COMMAND test_vendor_velocity_contract)
  add_executable(test_live_joint_state_bridge test/test_live_joint_state_bridge.cpp)
  target_include_directories(test_live_joint_state_bridge PRIVATE include)
  target_link_libraries(test_live_joint_state_bridge live_joint_state_bridge)
  target_compile_options(test_live_joint_state_bridge PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME live_joint_state_bridge COMMAND test_live_joint_state_bridge)
  add_executable(test_mode_request_lifecycle test/test_mode_request_lifecycle.cpp)
  target_include_directories(test_mode_request_lifecycle PRIVATE include)
  target_compile_options(test_mode_request_lifecycle PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME mode_request_lifecycle COMMAND test_mode_request_lifecycle)
  add_executable(test_evidence_timeout_contract test/test_evidence_timeout_contract.cpp)
  target_include_directories(test_evidence_timeout_contract PRIVATE include)
  target_compile_options(test_evidence_timeout_contract PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME evidence_timeout_contract COMMAND test_evidence_timeout_contract)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME platform_launch_joint_owner
    COMMAND ${Python3_EXECUTABLE} -B ${CMAKE_CURRENT_SOURCE_DIR}/test/test_platform_launch_joint_owner.py)
  add_test(
    NAME vendor_velocity_adapter_contract
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_vendor_velocity_adapter_contract.py)
  add_test(
    NAME platform_adapter_mode_request_contract
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_platform_adapter_mode_request_contract.py)
endif()

ament_package()
