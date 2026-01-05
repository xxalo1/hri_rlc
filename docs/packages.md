# Repo Packaging Standard (ROS 2 and standalone CMake)

These rules define how packages are structured and exported in this repository. They apply to humans and automation.

## 1. Package types and required files

1. ROS 2 CMake package (C++ libraries and or nodes)  
   Required: `package.xml`, `CMakeLists.txt`
2. ROS 2 Python package  
   Required: `package.xml`, `setup.cfg`, `setup.py` (or equivalent)
3. ROS 2 mixed package (C++ plus installable Python module)  
   Required: `package.xml`, `CMakeLists.txt`, `setup.cfg`, `setup.py`
4. ROS 2 description only package (URDF, meshes, xacro, config)  
   Required: `package.xml`, `CMakeLists.txt`
5. Standalone CMake library (non ROS), installed to repo `/_install`  
   Required: `CMakeLists.txt`, install rules, `*Config.cmake` generation

## 2. CMake rules (ament_cmake)

### 2.1 Standard layout
Recommended order:
1. `cmake_minimum_required`
2. `project(... LANGUAGES CXX)`
3. C++ standard defaults
4. `find_package(...)`
5. Targets
6. Target properties and dependencies
7. Install
8. Export
9. Tests
10. `ament_package()` last

### 2.2 C++ standard
Repository standard is C++20.

At package scope:
```cmake
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
```

Exported libraries must also declare the contract:
```cmake
target_compile_features(<pkg>_lib PUBLIC cxx_std_20)
```

### 2.3 Target based configuration only
Use `target_*` commands. Avoid global flags.

Preferred:
```cmake
target_include_directories
target_compile_options
target_link_libraries
target_compile_features
```

Avoid:
```cmake
add_compile_options(...)
set(CMAKE_CXX_FLAGS ...)
```

Warnings belong on the target and are PRIVATE:
```cmake
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(tgt PRIVATE -Wall -Wextra -Wpedantic)
endif()
```

### 2.4 PUBLIC vs PRIVATE
A dependency is PUBLIC if it is required by public headers under `include/`.

Rule:
1. dependency headers appear in installed headers ⇒ PUBLIC
2. dependency used only in `.cpp` ⇒ PRIVATE

### 2.5 Include directories
Use build and install interfaces:
```cmake
target_include_directories(tgt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
```

### 2.6 Naming and exports
Libraries use an internal name ending with `_lib` and export a namespaced target.

Pattern:
1. internal target: `<pkg>_lib`
2. exported target: `<pkg>::<component>`

Implementation:
```cmake
add_library(my_pkg_lib src/a.cpp)
add_library(my_pkg::utils ALIAS my_pkg_lib)
set_target_properties(my_pkg_lib PROPERTIES EXPORT_NAME utils)
```

Export:
```cmake
install(TARGETS my_pkg_lib EXPORT export_${PROJECT_NAME} ...)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET NAMESPACE my_pkg::)
ament_export_dependencies(...)
```

### 2.7 Nodes (executables)
Executables install to `lib/${PROJECT_NAME}` and are not exported as libraries:
```cmake
add_executable(my_node src/main.cpp)
ament_target_dependencies(my_node rclcpp)
install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})
```

### 2.8 Description only packages
Do not add compiler flags or targets. Only install data:
```cmake
find_package(ament_cmake REQUIRED)
install(DIRECTORY robots/ DESTINATION share/${PROJECT_NAME})
ament_package()
```

## 3. `package.xml` rules (ROS 2)

1. `package.xml` is required for every ROS package.
2. Build type must be declared.

CMake packages:
```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<export><build_type>ament_cmake</build_type></export>
```

Python packages:
```xml
<buildtool_depend>ament_python</buildtool_depend>
<export><build_type>ament_python</build_type></export>
```

Mixed packages:
```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<build_depend>ament_cmake_python</build_depend>
<export><build_type>ament_cmake</build_type></export>
```

3. Dependencies must match usage. Default to `<depend>` unless there is a strong reason to split build and exec.

Example:
```xml
<depend>rclcpp</depend>
<depend>rlc_common</depend>
<depend>rlc_robot_models</depend>
```

4. Test tools go under `<test_depend>`.

## 4. Python packaging rules (`setup.cfg`, `setup.py`)

Include Python packaging metadata whenever the package installs an importable module or provides console scripts.

Minimal `setup.py`:
```python
from setuptools import setup
setup()
```

Minimal `setup.cfg`:
```ini
[metadata]
name = rlc_common
version = 0.0.0

[options]
packages = find:
zip_safe = True

[options.entry_points]
console_scripts =
  example_node = rlc_common.scripts.example_node:main
```

If the package contains no installable Python module, omit `setup.cfg` and `setup.py`.

## 5. Standalone non ROS CMake libraries (installed to `/_install`)

1. Standalone libraries must install targets, headers, and CMake package config files.
2. The `*Config.cmake` must pull in dependencies via `find_dependency(...)` so consumers only need `find_package(<lib> CONFIG REQUIRED)`.

`Config.cmake.in` pattern:
```cmake
@PACKAGE_INIT@
include(CMakeFindDependencyMacro)
find_dependency(Eigen3)
include("${CMAKE_CURRENT_LIST_DIR}/<lib>Targets.cmake")
```

Consumption from ROS workspace:
```bash
export CMAKE_PREFIX_PATH="$PWD/_install:${CMAKE_PREFIX_PATH}"
colcon build --symlink-install
```

## 6. Avoid list (mandatory)

1. Do not use global `add_compile_options(...)` in packages that export libraries.
2. Do not export or install standalone libraries from inside a ROS package. Use `find_package(<lib> CONFIG REQUIRED)`.
3. Do not link against internal target names. Link only to exported namespaced targets like `pkg::component`.
4. Do not copy another library's headers into a different package's `include/`.
5. Do not list dependencies in `ament_export_dependencies(...)` unless they are part of the public interface or required for downstream `find_package` correctness.
