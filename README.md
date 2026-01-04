# HRI RLC

Authors:
- Hasan Al Thobaiti
- Rashed Albalawi

Core libs live in `src/` and the ROS 2 workspace lives in `rlc_ws/`.

## Setup

### Python
```bash
pip install -e .
```

### C++ libs
Build + install `rbt_core_cpp` into the repo-local prefix:
```bash
cmake -S src/rbt_core_cpp -B build/rbt_core_cpp \
  -DCMAKE_INSTALL_PREFIX="$PWD/_install" \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build/rbt_core_cpp -j
cmake --install build/rbt_core_cpp
```

### ROS 2 (Jazzy) workspace (`rlc_ws/`)
```bash
source /opt/ros/jazzy/setup.bash
export CMAKE_PREFIX_PATH="$PWD/_install:$CMAKE_PREFIX_PATH"

cd rlc_ws
colcon build --symlink-install --install-base ../_install/ros \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

source ../_install/ros/setup.bash
ros2 launch rlc_bringup plotjuggler.launch.py
```
