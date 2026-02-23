cmake_minimum_required(VERSION 3.8)
project(rlc_executive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/core/config_loader.cpp
  src/core/runtime_context.cpp
  src/bt/bt_executor_node.cpp
  src/bt/tree_runner.cpp
  src/execution/moveit_executor.cpp
  src/moveit/move_group_client.cpp
  src/state/state_buffer.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    ament_index_cpp
    behaviortree_cpp
    builtin_interfaces
    geometry_msgs
    moveit_msgs
    rclcpp
    rclcpp_action
    sensor_msgs
    tf2
    tf2_ros
)

target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp::yaml-cpp)

add_executable(rlc_bt_executor
  src/bt/main.cpp
)

ament_target_dependencies(rlc_bt_executor
  rclcpp
)

target_link_libraries(rlc_bt_executor ${PROJECT_NAME})

add_library(rlc_executive_nodes SHARED
  src/skills/testing/always_success.cpp
)

ament_target_dependencies(rlc_executive_nodes
  behaviortree_cpp
)

target_compile_definitions(rlc_executive_nodes PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS rlc_bt_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS rlc_executive_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY bt_trees/
  DESTINATION share/${PROJECT_NAME}/bt_trees
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY plugins/
  DESTINATION share/${PROJECT_NAME}/plugins
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp
  behaviortree_cpp
  builtin_interfaces
  geometry_msgs
  moveit_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  tf2
  tf2_ros
  yaml-cpp
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
