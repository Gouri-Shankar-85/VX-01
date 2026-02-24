cmake_minimum_required(VERSION 3.8)
project(vx01_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(vx01_hexapod_locomotion REQUIRED)

# ── C++ walk node ─────────────────────────────────────────────────────────────
add_executable(hexapod_walk_node
  scripts/hexapod_walk_node.cpp
)

ament_target_dependencies(hexapod_walk_node
  rclcpp
  rclcpp_action
  trajectory_msgs
  control_msgs
  builtin_interfaces
  vx01_hexapod_locomotion
)

install(TARGETS hexapod_walk_node
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install config and scripts directories ────────────────────────────────────
install(
  DIRECTORY
    config
    scripts
  DESTINATION share/${PROJECT_NAME}/
)

# ── Python scripts ────────────────────────────────────────────────────────────
install(
  PROGRAMS
    scripts/test_single_leg.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()