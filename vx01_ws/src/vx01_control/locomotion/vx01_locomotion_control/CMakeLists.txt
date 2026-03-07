cmake_minimum_required(VERSION 3.14)
project(vx01_locomotion_control)

# ── C++17 ─────────────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

# ── ROS 2 dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake             REQUIRED)
find_package(rclcpp                  REQUIRED)
find_package(rclcpp_action           REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(trajectory_msgs         REQUIRED)
find_package(control_msgs            REQUIRED)
find_package(sensor_msgs             REQUIRED)
find_package(action_msgs             REQUIRED)
find_package(builtin_interfaces      REQUIRED)

# ── Hexapod locomotion library ────────────────────────────────────────────────
find_package(vx01_hexapod_locomotion REQUIRED)

# ── Generate Walk.action interface ────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
    "action/Walk.action"
    DEPENDENCIES
        builtin_interfaces
        action_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# ── Executable ────────────────────────────────────────────────────────────────
add_executable(locomotion_node
    src/locomotion_action_server.cpp
    src/locomotion_node.cpp
)

target_include_directories(locomotion_node
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Link generated Walk action typesupport
rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(locomotion_node "${cpp_typesupport_target}")

ament_target_dependencies(locomotion_node
    rclcpp
    rclcpp_action
    trajectory_msgs
    control_msgs
    sensor_msgs
    vx01_hexapod_locomotion
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS locomotion_node
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
    DESTINATION include)

install(DIRECTORY launch config scripts
    DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
    scripts/send_walk_goal.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()