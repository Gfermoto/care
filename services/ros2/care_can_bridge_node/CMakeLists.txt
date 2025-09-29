cmake_minimum_required(VERSION 3.8)
project(care_can_bridge_node)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/can_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
