cmake_minimum_required(VERSION 3.8)
project(vx01_aerial_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vx01_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

add_executable(aerial_controller_node src/aerial_controller_node.cpp)
ament_target_dependencies(aerial_controller_node rclcpp geometry_msgs std_msgs vx01_msgs mavros_msgs)

install(TARGETS
  aerial_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
