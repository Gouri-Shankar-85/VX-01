cmake_minimum_required(VERSION 3.8)
project(vx01_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies (add to find_package block) ─────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vx01_hexapod_locomotion REQUIRED)   # your locomotion library

# ── Walk node executable ──────────────────────────────────────────
add_executable(hexapod_walk_node
    src/hexapod_walk_node.cpp
)

target_include_directories(hexapod_walk_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(hexapod_walk_node
    rclcpp
    rclcpp_action
    control_msgs
    trajectory_msgs
    geometry_msgs
    vx01_hexapod_locomotion
)

install(TARGETS hexapod_walk_node
    DESTINATION lib/${PROJECT_NAME}
)

# ── Teleop script ─────────────────────────────────────────────────
install(PROGRAMS
    scripts/hexapod_teleop_key.py
    DESTINATION lib/${PROJECT_NAME}
)

# ── Install config and scripts directories ────────────────────────────────────
install(
  DIRECTORY
    config
    scripts
    src
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()