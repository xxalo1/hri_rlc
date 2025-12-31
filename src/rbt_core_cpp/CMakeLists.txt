cmake_minimum_required(VERSION 3.16)
project(rbt_core_cpp LANGUAGES CXX)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Default to Release for performance
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Export compile_commands.json for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Pinocchio and Eigen
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(rbt_core_cpp
  src/dynamics.cpp
)

target_include_directories(rbt_core_cpp
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(rbt_core_cpp
  PUBLIC
    pinocchio::pinocchio
    Eigen3::Eigen
)

target_compile_options(rbt_core_cpp PRIVATE -Wall -Wextra -Wpedantic)
