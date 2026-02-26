cmake_minimum_required(VERSION 3.22)
project(rlc_executive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

# Core library (no BT leaf nodes here)
add_library(${PROJECT_NAME} SHARED
  src/core/config_loader.cpp
  src/core/runtime_context.cpp
  src/bt/bt_executor_node.cpp
  src/bt/tree_runner.cpp
  src/state/state_buffer.cpp
  src/moveit/move_group_client.cpp
  src/execution/moveit_executor.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

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
    yaml_cpp_vendor
)

target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp::yaml-cpp)

# BT plugin library (all BT leaf nodes live here)
file(GLOB_RECURSE RLC_BT_NODE_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/bt_nodes/*.cpp"
)

add_library(rlc_executive_bt_nodes SHARED
  ${RLC_BT_NODE_SOURCES}
)

target_compile_features(rlc_executive_bt_nodes PUBLIC cxx_std_20)

target_include_directories(rlc_executive_bt_nodes
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rlc_executive_bt_nodes
  PUBLIC
    behaviortree_cpp
    rclcpp
    rclcpp_action
    geometry_msgs
    moveit_msgs
    sensor_msgs
)

# Most of your BT nodes use RuntimeContext, MoveGroupClient, MoveItExecutor, etc.
target_link_libraries(rlc_executive_bt_nodes
  PUBLIC
    ${PROJECT_NAME}
)

target_compile_definitions(rlc_executive_bt_nodes PRIVATE BT_PLUGIN_EXPORT)

# Executable
add_executable(rlc_bt_executor
  src/bt/main.cpp
)

ament_target_dependencies(rlc_bt_executor rclcpp)
target_link_libraries(rlc_bt_executor ${PROJECT_NAME})

# Install
install(TARGETS
  ${PROJECT_NAME}
  rlc_executive_bt_nodes
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS rlc_bt_executor
  DESTINATION lib/${PROJECT_NAME}
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
  yaml_cpp_vendor
  yaml-cpp
)

ament_package()