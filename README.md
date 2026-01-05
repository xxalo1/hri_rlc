# HRI RLC

Authors:
- Hasan Al Thobaiti
- Rashed Albalawi

Core libs live in `src/` and the ROS 2 workspace lives in `rlc_ws/`.

## Setup

All commands below assume a POSIX shell (e.g., bash) and are run from the repository root.

### Prerequisites

- Python >= 3.10 (ROS 2 Jazzy uses Python 3.12)
- ROS 2 Jazzy: https://docs.ros.org/en/jazzy/Installation.html
- `colcon`: https://colcon.readthedocs.io/en/released/user/installation.html
- Pinocchio (C++ + Python bindings): https://stack-of-tasks.github.io/pinocchio/download.html
- MuJoCo (Python): https://mujoco.readthedocs.io/en/stable/python.html
- PyTorch: https://pytorch.org/get-started/locally/
- CMake >= 3.16, a C++17 compiler, and Eigen3
- PlotJuggler (optional): https://github.com/facontidavide/PlotJuggler

### Clone
```bash
git clone https://github.com/xxalo1/hri_rlc.git
cd hri_rlc
```

### Python packages
```bash
python3 -m pip install -U pip
python3 -m pip install numpy pyyaml mujoco
python3 -m pip install torch  # see PyTorch link above for CUDA wheels
python3 -m pip install -e .
```

### C++ libs
Build + install `rbt_core_cpp` into the repo-local prefix:
```bash
cmake -S src/rbt_core_cpp -B build/rbt_core_cpp \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build/rbt_core_cpp --parallel
cmake --install build/rbt_core_cpp --prefix _install
```

### ROS 2 (Jazzy) workspace (`rlc_ws/`)
```bash
source /opt/ros/jazzy/setup.bash
export CMAKE_PREFIX_PATH="$PWD/_install:$CMAKE_PREFIX_PATH"
cd rlc_ws
colcon build --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

source install/setup.bash
ros2 launch rlc_bringup plotjuggler.launch.py
```

If PlotJuggler is not installed:
```bash
ros2 launch rlc_bringup plotjuggler.launch.py start_plotjuggler:=false
```
