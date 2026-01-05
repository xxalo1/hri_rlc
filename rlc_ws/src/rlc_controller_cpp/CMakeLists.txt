cmake_minimum_required(VERSION 3.8)
project(rlc_controller_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rlc_interfaces REQUIRED)
find_package(rlc_common REQUIRED)
find_package(rlc_robot_models REQUIRED)

# rbt_core_cpp is vendored/exported via rlc_robot_models, but its exported target
# depends on these imported targets.
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(gen3_controller_node
  src/main.cpp
  src/controller_node.cpp
)

target_compile_features(gen3_controller_node PUBLIC cxx_std_17)

target_include_directories(gen3_controller_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gen3_controller_node
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
  control_msgs
  rlc_interfaces
  rlc_common
  rlc_robot_models
)

target_link_libraries(gen3_controller_node
  rlc_common::rlc_common
  rlc_robot_models::rlc_robot_models_cpp
  rlc_robot_models::rbt_core_cpp
)

install(TARGETS gen3_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
