cmake_minimum_required(VERSION 3.8)
project(care_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message files
set(msg_files
  "msg/RadarTarget.msg"
  "msg/RadarTargets.msg"
  "msg/SystemStatus.msg"
  "msg/SafetyZone.msg"
  "msg/SafetyCommand.msg"
  "msg/DeviceInfo.msg"
)

# Service files
set(srv_files
  "srv/SetSafetyZone.srv"
  "srv/ConfigController.srv"
  "srv/GetSystemStatus.srv"
  "srv/EmergencyStop.srv"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
