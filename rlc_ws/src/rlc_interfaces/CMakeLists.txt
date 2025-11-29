cmake_minimum_required(VERSION 3.8)
project(rlc_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointEffortCmd.msg"
  "msg/CartesianState.msg"
  "msg/JointStateSim.msg"
  "msg/PlannedTrajectory.msg"
  "srv/SetControllerGains.srv"
  "srv/PlanTrajectory.srv"
  "srv/ExecuteTrajectory.srv"
  DEPENDENCIES std_msgs geometry_msgs trajectory_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
