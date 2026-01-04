cmake_minimum_required(VERSION 3.8)
project(rlc_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rlc_interfaces REQUIRED)

ament_python_install_package(rlc_utils)

# Compatibility headers for the in-progress migration from `ros_utils(_cpp)` to
# `rlc_utils`. This keeps source/header include directives stable while you
# rearrange the on-disk header layout.
set(_rlc_utils_generated_include_dir "${CMAKE_CURRENT_BINARY_DIR}/generated_include")
file(MAKE_DIRECTORY "${_rlc_utils_generated_include_dir}/rlc_utils")
file(WRITE "${_rlc_utils_generated_include_dir}/rlc_utils/checks.hpp"
  "#pragma once\n\n#include \"rlc_utils_cpp/ros_utils_cpp/checks.hpp\"\n")
file(WRITE "${_rlc_utils_generated_include_dir}/rlc_utils/msg_conv.hpp"
  "#pragma once\n\n#include \"rlc_utils_cpp/ros_utils_cpp/msg_conv.hpp\"\n")
file(WRITE "${_rlc_utils_generated_include_dir}/rlc_utils/time.hpp"
  "#pragma once\n\n#include \"rlc_utils_cpp/ros_utils_cpp/time.hpp\"\n")
file(WRITE "${_rlc_utils_generated_include_dir}/rlc_utils/types.hpp"
  "#pragma once\n\n#include \"rlc_utils_cpp/ros_utils_cpp/types.hpp\"\n")

file(MAKE_DIRECTORY "${_rlc_utils_generated_include_dir}/ros_utils_cpp")
file(WRITE "${_rlc_utils_generated_include_dir}/ros_utils_cpp/time.hpp"
  "#pragma once\n\n#include \"rlc_utils_cpp/ros_utils_cpp/time.hpp\"\n")

add_library(rlc_utils_cpp
  src/msg_conv.cpp
  src/time.cpp
)
add_library(rlc_utils::rlc_utils_cpp ALIAS rlc_utils_cpp)

target_compile_features(rlc_utils_cpp PUBLIC cxx_std_20)

set_source_files_properties(src/time.cpp
  PROPERTIES
    COMPILE_DEFINITIONS "ros_utils_cpp=rlc_utils"
)

target_include_directories(rlc_utils_cpp
  PUBLIC
    $<BUILD_INTERFACE:${_rlc_utils_generated_include_dir}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(rlc_utils_cpp
  PUBLIC
    Eigen3::Eigen
    builtin_interfaces::builtin_interfaces__rosidl_generator_cpp
    control_msgs::control_msgs__rosidl_generator_cpp
    geometry_msgs::geometry_msgs__rosidl_generator_cpp
    sensor_msgs::sensor_msgs__rosidl_generator_cpp
    trajectory_msgs::trajectory_msgs__rosidl_generator_cpp
    rlc_interfaces::rlc_interfaces__rosidl_generator_cpp
)

install(TARGETS rlc_utils_cpp
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY "${_rlc_utils_generated_include_dir}/"
  DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3
  builtin_interfaces
  control_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  rlc_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
