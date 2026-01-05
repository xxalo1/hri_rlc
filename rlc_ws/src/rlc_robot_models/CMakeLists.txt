cmake_minimum_required(VERSION 3.8)
project(rlc_robot_models LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)

# External, non-ROS library installed into your prefix
find_package(rbt_core_cpp REQUIRED CONFIG)

ament_python_install_package(rlc_robot_models)

add_library(rlc_robot_models_lib
  src/kinova_gen3.cpp
)

add_library(rlc_robot_models::models ALIAS rlc_robot_models_lib)
set_target_properties(rlc_robot_models_lib PROPERTIES EXPORT_NAME models)

target_compile_features(rlc_robot_models_lib PUBLIC cxx_std_20)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(rlc_robot_models_lib PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_include_directories(rlc_robot_models_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

ament_target_dependencies(rlc_robot_models_lib
  ament_index_cpp
)

# Link to your external library
target_link_libraries(rlc_robot_models_lib PUBLIC rbt_core_cpp::rbt_core_cpp)

install(TARGETS rlc_robot_models_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET NAMESPACE rlc_robot_models::)
ament_export_dependencies(
  ament_index_cpp
  rbt_core_cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
