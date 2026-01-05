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

add_library(rlc_utils_cpp
  src/msg_conv.cpp
  src/time_util.cpp
)
add_library(rlc_utils::rlc_utils_cpp ALIAS rlc_utils_cpp)

target_compile_features(rlc_utils_cpp PUBLIC cxx_std_17)

target_include_directories(rlc_utils_cpp
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rlc_utils_cpp
  builtin_interfaces
  control_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  rlc_interfaces
)

target_link_libraries(rlc_utils_cpp PUBLIC Eigen3::Eigen)

install(TARGETS rlc_utils_cpp
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

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
