# Hardware
So-101 Follower Arm
Realsense D435i Camera
Objects in COCO dataset (YOLO)
Camera Mount: https://github.com/TheRobotStudio/SO-ARM100/tree/main/Optional/Wrist_Cam_Mount_RealSense_D435

# Environment
Ubuntu 24.04.5
ROS2 Jazzy
Python3 

# Environment Preparation
## Environment and location 
cd SO101_YOLO_Pick
conda deactivate

## Install ROS
sudo apt update
sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-pymoveit2 \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-xacro \
  ros-jazzy-tf2-ros \
  ros-jazzy-rviz2 \
  ros-jazzy-rmw-cyclonedds-cpp

## Install Python package
python3 -m pip install -U pip --break-system-packages
python3 -m pip install --break-system-packages \
  numpy \
  opencv-python \
  matplotlib \
  ultralytics \
  pyrealsense2

## Install C++ package
sudo apt install librange-v3-dev
sudo apt install libserial-dev

# Run
## Build workspace
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash

## Test robot connection
ls /dev/ttyACM*
lsusb | grep -i intel

## Launch 
ros2 launch so101_pick_place pick_place.launch.py

Note: 
default usb_port:/dev/ttyACM0, for/dev/ttyACM1, run: 
ros2 launch so101_pick_place pick_place.launch.py usb_port:=/dev/ttyACM1

# Pick 
Open a new terminal
## ROS2 and workspace
source /opt/ros/jazzy/setup.bash
source /home/sean/Unitree_Go2/so101_pick/install/setup.bash
## When the object is detected, send pick command
ros2 topic pub --times 3 --rate 2 /pick_place_cmd std_msgs/String "data: 'pick'"


# Modify Object:
1. modify target_classes in 
src/so101_pick_place/config/pick_place.yaml
2. Build 
source /opt/ros/jazzy/setup.bash
colcon build --packages-select so101_pick_place

