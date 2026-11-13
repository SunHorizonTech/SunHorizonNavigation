cmake_minimum_required(VERSION 3.8)
project(sunhorizon_navigation_interfaces)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/LocalizationComparisonStatus.msg"
  "msg/LocalizationStatus.msg"
  "msg/NavigationStatus.msg"
  "msg/NdtMeasurement.msg"
  "msg/ObstacleCoverageStatus.msg"
  "msg/ObstacleCoverageDecision.msg"
  "msg/SpatialSourceEvidence.msg"
  "msg/SpatialCoverageEvidence.msg"
  "msg/ObstacleCoverageSpatialDecision.msg"
  "msg/PlatformSafetyState.msg"
  "msg/RgbdStatus.msg"
  "msg/VelocitySupervisorStatus.msg"
  "msg/VelocityCommandEvidence.msg"
  "msg/VisualOdometryStatus.msg"
)

set(srv_files
  "srv/Cancel.srv"
  "srv/GetStatus.srv"
  "srv/LoadMap.srv"
  "srv/Localize.srv"
  "srv/PlanTo.srv"
  "srv/SaveMap.srv"
  "srv/StartMapping.srv"
  "srv/StopMapping.srv"
  "srv/ValidateMap.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  "action/NavigateTo.action"
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
