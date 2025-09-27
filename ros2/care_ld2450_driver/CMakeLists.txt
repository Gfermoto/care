cmake_minimum_required(VERSION 3.8)
project(care_ld2450_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Custom message definitions
set(msg_files
  "msg/RadarTarget.msg"
  "msg/RadarTargets.msg"
  "msg/SafetyStatus.msg"
)

set(srv_files
  "srv/SetSafetyZone.srv"
  "srv/GetRadarConfig.srv"
)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs
)

# Include directories
include_directories(include)

# Create library
add_library(${PROJECT_NAME}_lib
  src/ld2450_driver.cpp
  src/radar_target.cpp
  src/safety_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Create executable
add_executable(ld2450_driver_node src/ld2450_driver_node.cpp)
target_link_libraries(ld2450_driver_node ${PROJECT_NAME}_lib)

ament_target_dependencies(ld2450_driver_node
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install
install(TARGETS
  ld2450_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Export
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
