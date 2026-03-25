cmake_minimum_required(VERSION 3.8)
project(vx01_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate messages services and actions
rosidl_generate_interfaces(${PROJECT_NAME}

  # Messages
  "msg/HexapodState.msg"
  "msg/DroneState.msg"
  "msg/HybridMode.msg"
  "msg/MotorThrust.msg"
  "msg/UnderwaterState.msg"
  "msg/Victim.msg"   
  "msg/VictimArray.msg"         
  "msg/MissionState.msg"    
  "msg/Terrain.msg"         
  "msg/Walkability.msg"      
  "msg/RobotMode.msg"       
  "msg/FlightStatus.msg"     
  "msg/HexapodStatus.msg"   
  "msg/Exploration.msg"      

  # Services
  "srv/SwitchMode.srv"
  "srv/Calibrate.srv"

  # Actions
  "action/Navigate.action"
  "action/Dive.action"

  DEPENDENCIES
  std_msgs
  geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
