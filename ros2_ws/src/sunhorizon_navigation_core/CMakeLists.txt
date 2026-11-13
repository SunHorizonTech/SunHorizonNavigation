cmake_minimum_required(VERSION 3.8)
project(sunhorizon_navigation_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME}
  src/geometry_recovery.cpp
  src/localization_evidence.cpp
  src/localization_gate.cpp
  src/navigation_state_machine.cpp
  src/platform_evidence.cpp
  src/projection_grid.cpp
  src/rgbd_validation.cpp
  src/safety_gate.cpp
  src/visual_pose.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Threads REQUIRED)
  ament_add_gtest(test_latest_observer test/test_latest_observer.cpp)
  target_link_libraries(test_latest_observer ${PROJECT_NAME} Threads::Threads)
  ament_add_gtest(test_navigation_core test/test_navigation_core.cpp)
  target_link_libraries(test_navigation_core ${PROJECT_NAME})
  ament_add_gtest(test_geometry_recovery test/test_geometry_recovery.cpp)
  target_link_libraries(test_geometry_recovery ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
