# HRI RLC

Authors:
- Hasan Al Thobaiti
- Rashed Albalawi

Core libs live in `src/` and the ROS 2 workspace lives in `rlc_ws/`.

## Setup

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


### Third party (repos)

```bash
python3 deps/scripts/lock_repos.py
vcs import --recursive < ./deps/manifests/dependency.lock.repos
```

### Apt packages
```bash
sudo apt update
sudo apt install -y \
  libqt-advanced-docking-system-dev \
  qtbase5-dev qttools5-dev libqt5svg5-dev \
  libvtk9-dev libvtk9-qt-dev

rosdep install --from-paths rlc_ws/src src --ignore-src -r -y --rosdistro jazzy
```

### Build
```bash
source /opt/ros/jazzy/setup.bash
export CMAKE_PREFIX_PATH=/opt/ros/jazzy/opt/gz_sim_vendor:/opt/ros/jazzy/opt/gz_plugin_vendor:/opt/ros/jazzy/opt/gz_transport_vendor:$CMAKE_PREFIX_PATH

export COLCON_DEFAULTS_FILE=$PWD/colcon.defaults.yaml

colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

export GZ_SIM_SYSTEM_PLUGIN_PATH="$(ros2 pkg prefix gz_ros2_control)/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$(ros2 pkg prefix kortex_description)/share:${GZ_SIM_RESOURCE_PATH}"

```
