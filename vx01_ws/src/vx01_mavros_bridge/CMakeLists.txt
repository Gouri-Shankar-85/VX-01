cmake_minimum_required(VERSION 3.8)
project(vx01_mavros_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(vx01_msgs REQUIRED)

# Add library
add_library(${PROJECT_NAME} SHARED
  src/flight_mode_manager.cpp
  src/motor_commander.cpp
  src/telemetry_handler.cpp
  src/mavros_bridge.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  mavros
  mavros_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  vx01_msgs
)

# Add executable
add_executable(mavros_bridge_node
  src/mavros_bridge.cpp
)

target_include_directories(mavros_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(mavros_bridge_node
  ${PROJECT_NAME}
)

ament_target_dependencies(mavros_bridge_node
  rclcpp
  mavros
  mavros_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  vx01_msgs
)

# Install
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME} mavros_bridge_node
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  mavros
  mavros_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  vx01_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()