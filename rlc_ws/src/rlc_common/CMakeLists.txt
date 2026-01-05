cmake_minimum_required(VERSION 3.8)
project(rlc_common LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rlc_interfaces REQUIRED)

ament_python_install_package(rlc_common)

add_library(rlc_common_lib
  src/endpoints.cpp
)

# Build-tree convenience and installed/exported name
add_library(rlc_common::common ALIAS rlc_common_lib)
set_target_properties(rlc_common_lib PROPERTIES EXPORT_NAME common)

target_compile_features(rlc_common_lib PUBLIC cxx_std_20)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(rlc_common_lib PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_include_directories(rlc_common_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

ament_target_dependencies(rlc_common_lib
  ament_index_cpp
  control_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  rlc_interfaces
)

install(TARGETS rlc_common_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET NAMESPACE rlc_common::)
ament_export_dependencies(
  ament_index_cpp
  control_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  rlc_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
