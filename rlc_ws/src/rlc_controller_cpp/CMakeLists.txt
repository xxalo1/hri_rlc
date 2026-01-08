cmake_minimum_required(VERSION 3.8)
project(rlc_controller_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)

find_package(rbt_core_cpp REQUIRED CONFIG)

find_package(rlc_interfaces REQUIRED)
find_package(rlc_common REQUIRED)
find_package(rlc_utils REQUIRED)

option(BUILD_LEGACY_NODE "Build legacy rclcpp controller node" OFF)

add_library(rlc_controller_cpp_controller SHARED
  src/realtime_controller.cpp
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(rlc_controller_cpp_controller PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_compile_features(rlc_controller_cpp_controller PRIVATE cxx_std_20)

target_include_directories(rlc_controller_cpp_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rlc_controller_cpp_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rlc_utils
)

target_link_libraries(rlc_controller_cpp_controller
    rbt_core_cpp::rbt_core_cpp
    rlc_utils::utils
)

if(BUILD_LEGACY_NODE)
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
  rlc_common
  rlc_utils
  )

  target_link_libraries(controller_node_cpp
    rlc_common::common
    rbt_core_cpp::rbt_core_cpp
    rlc_utils::utils
  )
endif()

pluginlib_export_plugin_description_file(controller_interface
  rlc_controller_cpp_plugins.xml)

install(TARGETS rlc_controller_cpp_controller
  DESTINATION lib
)

if(BUILD_LEGACY_NODE)
  install(TARGETS controller_node_cpp
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES rlc_controller_cpp_plugins.xml README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
