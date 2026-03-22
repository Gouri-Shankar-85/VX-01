cmake_minimum_required(VERSION 3.8)
project(vx01_locomotion_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(vx01_hexapod_locomotion REQUIRED)

include_directories(include)

add_executable(hexapod_forward_walk_node
  src/hexapod_forward_walk_node.cpp
)

ament_target_dependencies(hexapod_forward_walk_node
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  vx01_hexapod_locomotion
)

add_executable(hexapod_turn_node
  src/hexapod_turn_node.cpp
)

ament_target_dependencies(hexapod_turn_node
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  vx01_hexapod_locomotion
)

install(TARGETS
  hexapod_forward_walk_node
  hexapod_turn_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()