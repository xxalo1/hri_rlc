cmake_minimum_required(VERSION 3.8)
project(rlc_planner LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
include(GNUInstallDirs)

find_package(Eigen3 REQUIRED)

find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tesseract_monitoring REQUIRED)
find_package(rlc_scene_bridge REQUIRED)

# Non-ament (CMake) dependencies
find_package(rbt_planning REQUIRED CONFIG)
find_package(trajopt REQUIRED CONFIG)
find_package(tesseract_environment REQUIRED CONFIG)

add_library(rlc_planner_plugin SHARED
  src/trajopt_interface.cpp
  src/trajopt_planner_manager.cpp
  src/trajopt_planning_context.cpp
)

target_compile_features(rlc_planner_plugin PUBLIC cxx_std_20)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(rlc_planner_plugin PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_include_directories(rlc_planner_plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

ament_target_dependencies(rlc_planner_plugin
  rclcpp
  pluginlib
  moveit_core
  moveit_msgs
  tesseract_monitoring
  rlc_scene_bridge
)

target_link_libraries(rlc_planner_plugin
  Eigen3::Eigen
  rbt_planning::rbt_planning
  trajopt::trajopt
  tesseract::tesseract_environment
  rlc_scene_bridge::scene_bridge
)

pluginlib_export_plugin_description_file(moveit_core rlc_planner_plugins.xml)

install(TARGETS rlc_planner_plugin
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES rlc_planner_plugins.xml
  DESTINATION share/${PROJECT_NAME}
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
