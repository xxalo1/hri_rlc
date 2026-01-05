cmake_minimum_required(VERSION 3.8)
project(rlc_robot_models)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(RBT_CORE_CPP_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/rbt_core_cpp")
if(NOT EXISTS "${RBT_CORE_CPP_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "Expected rbt_core_cpp at ${RBT_CORE_CPP_SOURCE_DIR}")
endif()
add_subdirectory("${RBT_CORE_CPP_SOURCE_DIR}" rbt_core_cpp)

ament_python_install_package(rlc_robot_models)

add_library(rlc_robot_models_cpp
  src/kinova_gen3.cpp
)

target_compile_features(rlc_robot_models_cpp PUBLIC cxx_std_17)

target_include_directories(rlc_robot_models_cpp
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rlc_robot_models_cpp 
  ament_index_cpp
)

target_link_libraries(rlc_robot_models_cpp
  rbt_core_cpp::rbt_core_cpp
)

install(TARGETS rlc_robot_models_cpp rbt_core_cpp
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY "${RBT_CORE_CPP_SOURCE_DIR}/include/" DESTINATION include)


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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  ament_index_cpp
  pinocchio
  Eigen3
)

ament_package()
