# SO-101 Repository Dependency Install

This document summarizes the dependency installation commands required to run this repository on another PC.

Target environment:

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3

## 1. ROS Package Installation

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-realsense2-camera \
  ros-jazzy-xacro \
  ros-jazzy-tf2-ros \
  ros-jazzy-rviz2
```

## 2. Python Package Installation

On Ubuntu 24.04, append `--break-system-packages` as shown below.

```bash
python3 -m pip install -U pip --break-system-packages
python3 -m pip install --break-system-packages \
  numpy \
  opencv-python \
  matplotlib \
  ultralytics \
  pyrealsense2
```

## 3. `pymoveit2` Installation

First, try installing with the command below.

```bash
python3 -m pip install --break-system-packages pymoveit2
```

If this fails, you will need to install `pymoveit2` from source according to your environment.

## 4. Workspace Build

After cloning the repository:

```bash
cd /path/to/so101_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## 5. Usage Examples

Launch the MoveIt environment:

```bash
ros2 launch so101_moveit_config demo.launch.py
```

To also launch the coordinate-based user node:

```bash
ros2 launch so101_pick_place pick_place.launch.py
```

## Notes

- `pyrealsense2` may not install via pip directly depending on your system environment.
- `pymoveit2` may also require source installation instead of pip depending on your environment.
- YOLO model files (`.pt`) are not included in the repository and must be prepared separately on the target PC.

