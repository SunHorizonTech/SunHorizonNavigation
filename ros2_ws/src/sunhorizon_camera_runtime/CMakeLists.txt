cmake_minimum_required(VERSION 3.8)
project(sunhorizon_camera_runtime)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)

option(SUNHORIZON_CAMERA_RUNTIME_BUILD_DRIVER
  "Build the direct OrbbecSDK camera runtime executable" ON)
option(SUNHORIZON_CAMERA_RUNTIME_REQUIRE_SDK
  "Fail configuration when the direct camera runtime is enabled but OrbbecSDK is absent" OFF)
set(ORBBEC_SDK_ROOT "" CACHE PATH
  "Root of an official OrbbecSDK distribution containing include/ and lib/")

set(_build_camera_driver ${SUNHORIZON_CAMERA_RUNTIME_BUILD_DRIVER})
if(_build_camera_driver)
  if(ORBBEC_SDK_ROOT)
    file(REAL_PATH "${ORBBEC_SDK_ROOT}" _orbbec_root)
    string(FIND "${_orbbec_root}" "/workspace/embed_fusion" _forbidden_sdk_index)
    if(NOT _forbidden_sdk_index EQUAL -1)
      message(FATAL_ERROR
        "ORBBEC_SDK_ROOT must reference an independently packaged official SDK, not /workspace/embed_fusion")
    endif()
    find_package(OrbbecSDK CONFIG QUIET
      PATHS "${_orbbec_root}" "${_orbbec_root}/lib"
      NO_DEFAULT_PATH)
  else()
    find_package(OrbbecSDK CONFIG QUIET)
  endif()

  if(NOT OrbbecSDK_FOUND OR NOT TARGET ob::OrbbecSDK)
    if(SUNHORIZON_CAMERA_RUNTIME_REQUIRE_SDK)
      message(FATAL_ERROR
        "Official OrbbecSDK not found. Set ORBBEC_SDK_ROOT or CMAKE_PREFIX_PATH.")
    endif()
    message(WARNING
      "Official OrbbecSDK not found; camera runtime executable is omitted, contract tests remain available")
    set(_build_camera_driver OFF)
  endif()
endif()

if(_build_camera_driver)
  foreach(_location_property
      IMPORTED_LOCATION IMPORTED_LOCATION_RELEASE IMPORTED_LOCATION_RELWITHDEBINFO)
    get_target_property(_orbbec_location ob::OrbbecSDK ${_location_property})
    if(_orbbec_location)
      string(FIND "${_orbbec_location}" "/workspace/embed_fusion" _forbidden_location_index)
      if(NOT _forbidden_location_index EQUAL -1)
        message(FATAL_ERROR
          "ob::OrbbecSDK resolves into /workspace/embed_fusion, which is forbidden")
      endif()
    endif()
  endforeach()

  add_library(camera_runtime SHARED src/camera_runtime_node.cpp)
  add_executable(sunhorizon_camera_runtime_node src/camera_runtime_main.cpp)
  add_executable(camera_profile_probe src/camera_profile_probe.cpp)
  add_executable(camera_static_capture src/camera_static_capture.cpp)
  add_executable(camera_exposure_cap_capture src/camera_exposure_cap_capture.cpp)
  target_compile_features(camera_exposure_cap_capture PUBLIC cxx_std_17)
  target_compile_options(camera_exposure_cap_capture PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(camera_exposure_cap_capture PRIVATE include)
  target_link_libraries(camera_exposure_cap_capture ob::OrbbecSDK)
  target_compile_features(camera_static_capture PUBLIC cxx_std_17)
  target_compile_options(camera_static_capture PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(camera_static_capture ob::OrbbecSDK)
  target_compile_features(camera_runtime PUBLIC cxx_std_17)
  target_compile_options(camera_runtime PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_features(sunhorizon_camera_runtime_node PUBLIC cxx_std_17)
  target_compile_features(camera_profile_probe PUBLIC cxx_std_17)
  target_compile_options(sunhorizon_camera_runtime_node PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(camera_profile_probe PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(camera_runtime PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(camera_runtime PRIVATE ob::OrbbecSDK Threads::Threads)
  target_link_libraries(sunhorizon_camera_runtime_node camera_runtime)
  target_link_libraries(camera_profile_probe ob::OrbbecSDK)
  ament_target_dependencies(camera_runtime PUBLIC
    diagnostic_msgs geometry_msgs rclcpp sensor_msgs tf2_ros)
  ament_target_dependencies(sunhorizon_camera_runtime_node
    diagnostic_msgs geometry_msgs rclcpp sensor_msgs tf2_ros)
  install(TARGETS sunhorizon_camera_runtime_node DESTINATION lib/${PROJECT_NAME})
  install(TARGETS camera_profile_probe DESTINATION lib/${PROJECT_NAME})
  install(TARGETS camera_static_capture DESTINATION lib/${PROJECT_NAME})
  install(TARGETS camera_exposure_cap_capture DESTINATION lib/${PROJECT_NAME})
else()
  # Keep the public contract linkable on development hosts without the SDK.
  # The factory fails explicitly; it never substitutes a DDS subscription path.
  add_library(camera_runtime SHARED src/camera_runtime_unavailable.cpp)
  target_compile_features(camera_runtime PUBLIC cxx_std_17)
  target_include_directories(camera_runtime PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(camera_runtime rclcpp sensor_msgs)
endif()
set_target_properties(camera_runtime PROPERTIES OUTPUT_NAME sunhorizon_camera_runtime)
install(TARGETS camera_runtime EXPORT export_camera_runtime
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(export_camera_runtime HAS_LIBRARY_TARGET)
ament_export_dependencies(diagnostic_msgs geometry_msgs rclcpp sensor_msgs tf2_ros)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS internal_optical_tf_contract.py DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_context_startup test/test_context_startup.cpp)
  target_compile_features(test_context_startup PUBLIC cxx_std_17)
  target_compile_options(test_context_startup PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(test_context_startup PRIVATE include)
  add_test(NAME test_context_startup COMMAND test_context_startup)

  add_executable(test_capture_thread_affinity test/test_capture_thread_affinity.cpp)
  target_compile_features(test_capture_thread_affinity PUBLIC cxx_std_17)
  target_compile_options(test_capture_thread_affinity PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(test_capture_thread_affinity PRIVATE include)
  target_link_libraries(test_capture_thread_affinity Threads::Threads)
  add_test(NAME test_capture_thread_affinity COMMAND test_capture_thread_affinity)
  add_executable(test_exposure_cap_transaction test/test_exposure_cap_transaction.cpp)
  target_compile_features(test_exposure_cap_transaction PUBLIC cxx_std_17)
  target_compile_options(test_exposure_cap_transaction PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(test_exposure_cap_transaction PRIVATE include)
  add_test(NAME test_exposure_cap_transaction COMMAND test_exposure_cap_transaction)
  set_tests_properties(test_exposure_cap_transaction PROPERTIES TIMEOUT 20)
  add_executable(test_ae_target_transaction test/test_ae_target_transaction.cpp)
  target_compile_features(test_ae_target_transaction PUBLIC cxx_std_17)
  target_compile_options(test_ae_target_transaction PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(test_ae_target_transaction PRIVATE include)
  add_test(NAME test_ae_target_transaction COMMAND test_ae_target_transaction)
  set_tests_properties(test_ae_target_transaction PROPERTIES TIMEOUT 20)
  add_executable(test_latest_frame_worker test/test_latest_frame_worker.cpp)
  target_compile_features(test_latest_frame_worker PUBLIC cxx_std_17)
  target_compile_options(test_latest_frame_worker PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(test_latest_frame_worker PRIVATE include)
  target_link_libraries(test_latest_frame_worker Threads::Threads)
  add_test(NAME test_latest_frame_worker COMMAND test_latest_frame_worker)
  set_tests_properties(test_latest_frame_worker PROPERTIES TIMEOUT 20)
  add_executable(test_stereo_ir_observation test/test_stereo_ir_observation.cpp)
  target_compile_features(test_stereo_ir_observation PUBLIC cxx_std_17)
  target_compile_options(test_stereo_ir_observation PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(test_stereo_ir_observation PRIVATE include)
  add_test(NAME test_stereo_ir_observation COMMAND test_stereo_ir_observation)

  add_executable(test_exact_stereo_pairing test/test_exact_stereo_pairing.cpp)
  target_compile_features(test_exact_stereo_pairing PUBLIC cxx_std_17)
  target_compile_options(test_exact_stereo_pairing PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(test_exact_stereo_pairing PRIVATE include)
  add_test(NAME test_exact_stereo_pairing COMMAND test_exact_stereo_pairing)

  add_executable(test_optional_shadow test/test_optional_shadow.cpp)
  target_compile_features(test_optional_shadow PUBLIC cxx_std_17)
  target_compile_options(test_optional_shadow PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(test_optional_shadow PRIVATE include)
  add_test(NAME test_optional_shadow COMMAND test_optional_shadow)

  add_executable(test_stereo_quality_workspace test/test_stereo_quality_workspace.cpp)
  target_compile_features(test_stereo_quality_workspace PUBLIC cxx_std_17)
  target_compile_options(test_stereo_quality_workspace PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(test_stereo_quality_workspace PRIVATE include)
  add_test(NAME test_stereo_quality_workspace COMMAND test_stereo_quality_workspace)

  add_executable(test_stereo_depth_quality test/test_stereo_depth_quality.cpp)
  target_compile_features(test_stereo_depth_quality PUBLIC cxx_std_17)
  target_compile_options(test_stereo_depth_quality PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(test_stereo_depth_quality PRIVATE include)
  add_test(NAME test_stereo_depth_quality COMMAND test_stereo_depth_quality)

  add_executable(test_runtime_contract test/test_runtime_contract.cpp)
  target_compile_features(test_runtime_contract PUBLIC cxx_std_17)
  target_compile_options(test_runtime_contract PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(test_runtime_contract PRIVATE include)
  add_test(NAME test_runtime_contract COMMAND test_runtime_contract)

  add_executable(test_callback_timing test/test_callback_timing.cpp)
  target_compile_features(test_callback_timing PUBLIC cxx_std_17)
  target_compile_options(test_callback_timing PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(test_callback_timing PRIVATE include)
  add_test(NAME test_callback_timing COMMAND test_callback_timing)

  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME test_source_contract
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_source_contract.py)
endif()

ament_package()
