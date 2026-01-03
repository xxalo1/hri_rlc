cmake_minimum_required(VERSION 3.16)
project(ros_utils_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT COMMAND ament_target_dependencies)
  message(FATAL_ERROR "ros_utils_cpp must be built from within an ament_cmake package (ament_target_dependencies not found).")
endif()

find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rlc_interfaces REQUIRED)

add_library(ros_utils_cpp
  src/time.cpp
  src/msg_conv.cpp
)

target_include_directories(ros_utils_cpp
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ros_utils_cpp
  std_msgs
  builtin_interfaces
  sensor_msgs
  control_msgs
  rlc_interfaces
)

target_link_libraries(ros_utils_cpp
  Eigen3::Eigen
)

