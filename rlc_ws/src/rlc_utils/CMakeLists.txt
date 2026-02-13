cmake_minimum_required(VERSION 3.8)
project(rlc_utils LANGUAGES CXX)

# Package level default (recommended in ROS workspaces)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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

# C++ library
add_library(rlc_utils_lib
  src/msg_conv.cpp
  src/time_util.cpp
)

# Nice name inside this build tree
add_library(rlc_utils::utils ALIAS rlc_utils_lib)

# Make the exported installed target name be rlc_utils::utils
set_target_properties(rlc_utils_lib PROPERTIES EXPORT_NAME utils)

# Exported contract: consumers compiling against your headers will use C++20
target_compile_features(rlc_utils_lib PUBLIC cxx_std_20)

target_include_directories(rlc_utils_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Warnings: keep private (do not impose on downstream)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(rlc_utils_lib PRIVATE -Wall -Wextra -Wpedantic)
endif()

ament_target_dependencies(rlc_utils_lib
  builtin_interfaces
  control_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  rlc_interfaces
)

target_link_libraries(rlc_utils_lib Eigen3::Eigen)

install(TARGETS rlc_utils_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET NAMESPACE rlc_utils::)
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
