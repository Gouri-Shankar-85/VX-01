cmake_minimum_required(VERSION 3.8)
project(vx01_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install (
  DIRECTORY
    config 
    scripts
  DESTINATION share/${PROJECT_NAME}/
)

# Add to vx01_bringup/CMakeLists.txt
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
