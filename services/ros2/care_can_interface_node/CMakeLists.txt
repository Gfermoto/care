cmake_minimum_required(VERSION 3.8)
project(care_can_interface_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(care_common REQUIRED)

add_executable(care_can_interface_node src/care_can_interface_node.cpp)
ament_target_dependencies(care_can_interface_node 
  rclcpp 
  std_msgs
  care_common
)

install(TARGETS
  care_can_interface_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
