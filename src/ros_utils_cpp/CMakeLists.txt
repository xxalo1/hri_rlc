cmake_minimum_required(VERSION 3.16)
project(ros_utils_cpp VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)

find_package(Eigen3 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rlc_interfaces REQUIRED)

add_library(ros_utils_cpp
  src/time.cpp
  src/msg_conv.cpp
)
add_library(ros_utils_cpp::ros_utils_cpp ALIAS ros_utils_cpp)

target_compile_features(ros_utils_cpp PUBLIC cxx_std_20)

target_include_directories(ros_utils_cpp
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(ros_utils_cpp
  PUBLIC
    Eigen3::Eigen
    builtin_interfaces::builtin_interfaces__rosidl_generator_cpp
    sensor_msgs::sensor_msgs__rosidl_generator_cpp
    control_msgs::control_msgs__rosidl_generator_cpp
    rlc_interfaces::rlc_interfaces__rosidl_generator_cpp
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(ros_utils_cpp PRIVATE -Wall -Wextra -Wpedantic)
endif()

set_target_properties(ros_utils_cpp PROPERTIES POSITION_INDEPENDENT_CODE ON)

include(CMakePackageConfigHelpers)

install(TARGETS ros_utils_cpp
  EXPORT ros_utils_cppTargets
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT ros_utils_cppTargets
  NAMESPACE ros_utils_cpp::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ros_utils_cpp
)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ros_utils_cppConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/ros_utils_cppConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ros_utils_cpp
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/ros_utils_cppConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/ros_utils_cppConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/ros_utils_cppConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ros_utils_cpp
)
