cmake_minimum_required(VERSION 3.16)
project(rbt_core_cpp VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Default policy for this package (helps consistency for any future targets)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Dependencies (exported as part of the public interface via target_link_libraries PUBLIC)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(rbt_core_cpp
  src/dynamics.cpp
  src/controller.cpp
  src/robot.cpp
)
add_library(rbt_core_cpp::rbt_core_cpp ALIAS rbt_core_cpp)

# Public contract: consumers compile with C++20 when they use your headers
target_compile_features(rbt_core_cpp PUBLIC cxx_std_20)

target_include_directories(rbt_core_cpp
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(rbt_core_cpp
  PUBLIC
    pinocchio::pinocchio
    Eigen3::Eigen
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(rbt_core_cpp PRIVATE -Wall -Wextra -Wpedantic)
endif()

set_target_properties(rbt_core_cpp PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  EXPORT_NAME rbt_core_cpp
)

# --------------------------
# Install: library + headers
# --------------------------

install(TARGETS rbt_core_cpp
  EXPORT rbt_core_cppTargets
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# --------------------------
# Package config (find_package)
# --------------------------

# Export targets file
install(EXPORT rbt_core_cppTargets
  NAMESPACE rbt_core_cpp::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rbt_core_cpp
)

# Generate and install the config + version files
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/rbt_core_cppConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/rbt_core_cppConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rbt_core_cpp
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/rbt_core_cppConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/rbt_core_cppConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/rbt_core_cppConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rbt_core_cpp
)
