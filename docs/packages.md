# CMakeLists Professional Rules (ROS 2 ament, target based)

This document is a project standard for writing `CMakeLists.txt` files in this repository. It is written for ROS 2 packages using `ament_cmake`, but most rules also apply to pure CMake projects.

## 1. The mental model

1. CMake configures a build. It generates build files for Ninja or Make.
2. A target is what CMake builds.
   1. Executable target created by `add_executable(...)`
   2. Library target created by `add_library(...)`
   3. Interface target created by `add_library(name INTERFACE)` for header only requirements
3. You attach build rules to targets. Avoid global flags when possible.
4. Headers are compile time. Libraries are link time.
5. If your public headers include a dependency header, that dependency is part of your public interface.

## 2. File structure and order

Recommended order in `CMakeLists.txt`:

1. `cmake_minimum_required(...)`
2. `project(... LANGUAGES CXX)`
3. Project defaults (C++ standard and extensions)
4. `find_package(ament_cmake REQUIRED)` and other `find_package(...)`
5. Python install rules, if any
6. Define targets (`add_library`, `add_executable`)
7. Attach target properties (compile features, includes, options, dependencies)
8. Install rules
9. Export rules
10. Testing rules
11. `ament_package()` last

## 3. C++ standard policy

For ROS 2 workspaces, set the default per package, then also enforce per exported target.

At package scope:

```cmake
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
```

On exported libraries:

```cmake
target_compile_features(my_lib PUBLIC cxx_std_20)
```

Why both.
1. The package default makes new targets consistent.
2. The target requirement is the public contract for downstream consumers.

## 4. Target based rules

### 4.1 Prefer target based commands

Use these on targets.
1. `target_include_directories`
2. `target_compile_options`
3. `target_compile_definitions`
4. `target_link_libraries`
5. `target_compile_features`

Avoid global settings like `add_compile_options(...)` unless you truly want every target to inherit it.

### 4.2 PUBLIC, PRIVATE, INTERFACE

Always choose visibility for each requirement.

1. PRIVATE
   Only this target needs it to build.
2. PUBLIC
   This target needs it, and anything linking to it also needs it.
3. INTERFACE
   This target does not compile code. Dependents need it.

Rule of thumb for dependencies.
1. If a dependency is included in public headers under `include/`, it must be PUBLIC.
2. If it is only used in `.cpp`, it should be PRIVATE.

Example.

```cmake
target_link_libraries(my_lib PUBLIC Eigen3::Eigen)
target_compile_options(my_lib PRIVATE -Wall -Wextra -Wpedantic)
```

### 4.3 Build interface and install interface for includes

For installed packages, include paths differ between the build tree and install tree. Always use this pattern for libraries with installed headers.

```cmake
target_include_directories(my_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
```

## 5. Dependencies in ROS 2 ament

### 5.1 `find_package`

For each dependency you use in CMake, add a `find_package(dep REQUIRED)`.

### 5.2 `ament_target_dependencies`

For ROS 2 packages that provide headers and linking flags, attach them to the target.

```cmake
ament_target_dependencies(my_lib
  rclcpp
  sensor_msgs
  control_msgs
)
```

This is the standard way to express ROS dependencies for a target.

### 5.3 Imported targets

When a dependency provides a CMake target like `Eigen3::Eigen`, prefer it.

```cmake
target_link_libraries(my_lib PUBLIC Eigen3::Eigen)
```

## 6. Names and exported targets

### 6.1 Internal target name vs public name

You may keep an internal target name and export a stable public name.

Example.
1. Internal: `rlc_utils_lib`
2. Public: `rlc_utils::utils`

To make the installed exported target be `rlc_utils::utils`, do both.

```cmake
add_library(rlc_utils_lib ...)
set_target_properties(rlc_utils_lib PROPERTIES EXPORT_NAME utils)
```

Optional convenience during development in the same build tree.

```cmake
add_library(rlc_utils::utils ALIAS rlc_utils_lib)
```

Important.
1. Aliases are not exported by default.
2. The exported name is controlled by `EXPORT_NAME` and the export namespace.

## 7. Install rules

### 7.1 Install targets

Install the compiled target and attach an export set.

```cmake
install(TARGETS my_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
```

### 7.2 Install headers

```cmake
install(DIRECTORY include/ DESTINATION include)
```

## 8. Export rules for downstream packages

Downstream packages typically do this.

```cmake
find_package(my_pkg REQUIRED)
target_link_libraries(their_target PRIVATE my_pkg::some_target)
```

To support that, export targets and dependencies.

```cmake
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET NAMESPACE my_pkg::)
ament_export_include_directories(include)
ament_export_dependencies(
  Eigen3
  sensor_msgs
)
```

Meaning of `ament_export_dependencies`.
1. It declares the package level dependency chain for downstream `find_package`.
2. Use it for dependencies that are part of your public interface, especially when your public headers include them.

## 9. Warnings and hygiene

Attach warnings to your targets as PRIVATE.

```cmake
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(my_lib PRIVATE -Wall -Wextra -Wpedantic)
endif()
```

Avoid exporting warning flags to downstream consumers.

## 10. Python in a mixed package

Python install is not a CMake link target. It is installed as a package module.

```cmake
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(my_python_pkg)
```

You do not normally create a CMake target name for Python.

## 11. Minimal template for a mixed Python plus C++ ROS 2 library package

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_pkg LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_python_install_package(my_pkg)

add_library(my_pkg_lib
  src/a.cpp
)
set_target_properties(my_pkg_lib PROPERTIES EXPORT_NAME utils)
add_library(my_pkg::utils ALIAS my_pkg_lib)

target_compile_features(my_pkg_lib PUBLIC cxx_std_20)

target_include_directories(my_pkg_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(my_pkg_lib
  sensor_msgs
)

target_link_libraries(my_pkg_lib PUBLIC Eigen3::Eigen)

install(TARGETS my_pkg_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET NAMESPACE my_pkg::)
ament_export_dependencies(Eigen3 sensor_msgs)

ament_package()
```

## 12. Review checklist

Before merging a `CMakeLists.txt`, confirm:
1. Targets exist for every library and executable.
2. Public headers live under `include/` and are installed.
3. Build interface and install interface include dirs are correct.
4. Dependencies included in public headers are PUBLIC on the target.
5. Warnings are PRIVATE.
6. Targets are installed and exported.
7. `ament_package()` is the last line.

