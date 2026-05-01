cmake_minimum_required(VERSION 3.8)
project(vx01_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(vx01_msgs REQUIRED)

# Install Python nodes
ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/victim_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME victim_detector_node
)

install(
  PROGRAMS
    scripts/terrain_classifier_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME terrain_classifier_node
)

install(
  PROGRAMS
    scripts/mapping_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mapping_node
)

install(
  PROGRAMS
    scripts/mission_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mission_manager_node
)

install(
  PROGRAMS
    scripts/mode_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mode_manager_node
)

# Install config
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY models/
  DESTINATION lib/${PROJECT_NAME}/models
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
