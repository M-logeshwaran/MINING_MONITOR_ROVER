cmake_minimum_required(VERSION 3.8)
project(drillpulse_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/RoverTelemetry.msg"
  "msg/OdometryDiagnostics.msg"
  "msg/MissionState.msg"
  "msg/MissionCommand.msg"
  "msg/HazardEvent.msg"
  "msg/EnvironmentState.msg"
  "msg/RecoveryCommand.msg"
  "msg/RecoveryStatus.msg"
  "msg/SafetyStatus.msg"
  "msg/NavigationStatus.msg"
  "msg/LinkStatus.msg"
  "msg/JobAssignment.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs)
ament_package()
