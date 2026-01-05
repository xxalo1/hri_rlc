cmake_minimum_required(VERSION 3.8)
project(rlc_controller_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)

find_package(rlc_interfaces REQUIRED)
find_package(rlc_common REQUIRED)
find_package(rlc_robot_models REQUIRED)

add_executable(controller_node_cpp
  src/main.cpp
  src/controller_node.cpp
)

target_compile_features(controller_node_cpp PRIVATE cxx_std_20)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(controller_node_cpp PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_include_directories(controller_node_cpp
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

ament_target_dependencies(controller_node_cpp
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
  control_msgs
  rlc_interfaces
)

target_link_libraries(controller_node_cpp
  PRIVATE
    rlc_common::common
    rlc_robot_models::models
)

install(TARGETS controller_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
