cmake_minimum_required(VERSION 3.8)
project(care_can_bridge_node)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/can_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Enable ament_lint when testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
