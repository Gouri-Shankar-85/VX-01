cmake_minimum_required(VERSION 3.8)
project(vx01_mode_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vx01_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(mode_manager_node src/mode_manager_node.cpp)
ament_target_dependencies(mode_manager_node rclcpp vx01_msgs trajectory_msgs)

install(TARGETS
  mode_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
