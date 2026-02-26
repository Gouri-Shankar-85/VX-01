cmake_minimum_required(VERSION 2.8)
project(ascamera_demo)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS -pthread)

add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE "RELEASE")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
# SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O3 -Wall -g -fno-inline -ggdb")

set(CMAKE_SYSTEM_PROCESSOR "x86_64-win-msvc")

set(OPENCV "ON")
if(${OPENCV} MATCHES "ON")
message("support opencv")
add_definitions(-DCFG_OPENCV_ON)
include_directories(${CMAKE_SOURCE_DIR}/libs/include/opencv)
endif()

# Specify additional locations of header files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs/include)
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${CMAKE_SOURCE_DIR}/libs/lib/${CMAKE_SYSTEM_PROCESSOR}/)

# add to be built executable files
add_executable(ascamera ./src/main.cpp ./src/Demo.cpp ./src/CameraSrv.cpp ./src/Camera.cpp)

if(${OPENCV} MATCHES "ON")
target_link_libraries(ascamera AngstrongCameraSdk opencv_world3412)
else()
target_link_libraries(ascamera AngstrongCameraSdk)

endif()
