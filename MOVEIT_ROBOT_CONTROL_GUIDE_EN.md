# SO-101 MoveIt Coordinate Control Guide

## Reference Launch

This document is based on the following launch file.

- [demo.launch.py](/home/sst/so101_ws/src/so101_moveit_config/launch/demo.launch.py)

Run it as follows.

```bash
source /opt/ros/jazzy/setup.bash
source /home/sst/so101_ws/install/setup.bash
ros2 launch so101_moveit_config demo.launch.py
```

However, there is an important distinction.

- [demo.launch.py](/home/sst/so101_ws/src/so101_moveit_config/launch/demo.launch.py)
  - Launches the MoveIt environment
  - Does not include a node that receives pose messages and triggers movement
- [pick_place.launch.py](/home/sst/so101_ws/src/so101_pick_place/launch/pick_place.launch.py)
  - Launches the MoveIt environment together with the user control node
  - Required if you want to connect coordinate publishing to actual robot movement in this workspace

In short, the roles break down as follows.

- `demo.launch.py`
  - Starts the MoveIt server and environment
- `pick_place.launch.py`
  - Starts the MoveIt server and environment + connects pose input to actual movement

## What `demo.launch.py` Actually Launches

[demo.launch.py](/home/sst/so101_ws/src/so101_moveit_config/launch/demo.launch.py) does not launch many nodes directly. Instead, it includes the following launch file.

- [follower_moveit_demo.launch.py](/home/sst/so101_ws/src/so101_bringup/launch/follower_moveit_demo.launch.py)

So when `demo.launch.py` runs, what actually comes up is:

- `ros2_control` bringup
- `robot_state_publisher`
- Controller spawner
- `move_group`
- MoveIt RViz

## Why the Robot Moved Even Though `ros2_control` Seemed Absent

The short answer is that `ros2_control` was not absent — **it was already running alongside everything else.**----------------------------------------------------------------------------------

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



Reason:

- [demo.launch.py](/home/sst/so101_ws/src/so101_moveit_config/launch/demo.launch.py)
  - Includes [follower_moveit_demo.launch.py](/home/sst/so101_ws/src/so101_bringup/launch/follower_moveit_demo.launch.py)
- [follower_moveit_demo.launch.py](/home/sst/so101_ws/src/so101_bringup/launch/follower_moveit_demo.launch.py)
  - In turn includes [follower_split.launch.py](/home/sst/so101_ws/src/so101_bringup/launch/follower_split.launch.py)
- Inside that bringup, the actual hardware, `controller_manager`, `arm_trajectory_controller`, and `gripper_controller` are all brought up

So MoveIt did not move the robot on its own.

- MoveIt plans and sends the trajectory
- The actual joint execution is handled by the `ros2_control` controllers

## Coordinate Reference Frame

Target poses in MoveIt should be specified relative to `base_link`.

That means:

- `x`: forward direction of the robot
- `y`: left/right of the robot
- `z`: height of the robot

For example:

```yaml
position:
  x: 0.28
  y: 0.03
  z: -0.06
```

This means moving the end-effector to position `(0.28, 0.03, -0.06)` relative to the `base_link` origin.

## MoveIt Named States

The MoveIt SRDF defines a set of pre-configured named states.

Reference file:

- [so101_arm.srdf](/home/sst/so101_ws/src/so101_moveit_config/config/so101_arm.srdf)

### manipulator group

- `zero`
  - Reference pose with all joints near zero
- `pick`
  - Basic pick pose with wrist_flex raised
- `rest`
  - A folded storage-like pose
- `extended`
  - A pose with the arm stretched outward

### gripper group

- `open`
  - Gripper open
- `closed`
  - Gripper closed

In summary, the arm has the following named states:

- `zero`
- `pick`
- `rest`
- `extended`

And the gripper has the following named states:

- `open`
- `closed`

These values are fixed joint states defined in the SRDF.

## How the Robot Moves When Given a Coordinate

The concept in MoveIt is straightforward.

1. Define the target end-effector pose
2. Find feasible joint values via IK
3. Run planning
4. Execute the trajectory
5. The controller drives the actual motors

So the answer to "where does the robot go when given a coordinate" is:

- The input coordinate is interpreted as the target end-effector pose relative to `base_link`
- MoveIt computes a joint path to reach that pose
- The controller executes that path

## Required Environment Setup

```bash
source /opt/ros/jazzy/setup.bash
source /home/sst/so101_ws/install/setup.bash
```

After a fresh build:

```bash
cd /home/sst/so101_ws
colcon build --packages-select so101_moveit_config so101_bringup so101_pick_place
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## How to Run

Launch MoveIt with the actual control environment:

```bash
ros2 launch so101_moveit_config demo.launch.py
```

Once running, the following are available:

- MoveIt RViz
- `move_group`
- arm / gripper controllers

## Which Launch File to Use

### 1. When Only the MoveIt Environment Is Needed

Use:

- [demo.launch.py](/home/sst/so101_ws/src/so101_moveit_config/launch/demo.launch.py)

Command:

```bash
ros2 launch so101_moveit_config demo.launch.py
```

What you can do:

- Run MoveIt
- Use RViz
- Call the MoveIt API directly from code

What you cannot do:

- Automatically trigger movement by publishing a pose message

So `demo.launch.py` is purely for bringing up the MoveIt environment.

### 2. When You Need Actual Movement via Coordinate Publishing

Use:

- [pick_place.launch.py](/home/sst/so101_ws/src/so101_pick_place/launch/pick_place.launch.py)

Command:

```bash
ros2 launch so101_pick_place pick_place.launch.py
```

What you can do:

- Run MoveIt
- Receive pose input
- Internal user node calls MoveIt
- Actually move the arm

So **if you want the robot to actually move by publishing coordinates**, use `pick_place.launch.py`.

## Important Distinction

`demo.launch.py` itself does not provide an interface where publishing a coordinate topic immediately triggers movement.

With the default MoveIt configuration alone, the following do not happen automatically:

- Subscribing to a specific coordinate topic
- Moving immediately upon receiving a coordinate

MoveIt is fundamentally used in one of two ways:

1. Specifying a goal pose directly in the RViz MotionPlanning plugin
2. Feeding a target pose via the MoveIt API from separate user code

In other words, the default MoveIt interface is not topic-based commanding — it works by feeding a goal pose or named state through the API.

## What MoveIt Actually Needs as Input

What MoveIt ultimately requires is the target end-effector pose.

- Reference frame: `base_link`
- Position: `(x, y, z)`
- Orientation: quaternion `(x, y, z, w)`

Example pose:

```yaml
frame_id: base_link
position:
  x: 0.28
  y: 0.03
  z: -0.06
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
```

When this pose is fed into MoveIt, it will attempt to move the end-effector to that position and orientation.

## Pose Input Format Used in This Workspace

In this workspace, a separate user node wraps MoveIt, and the coordinate format fed into that node is a `PoseStamped` message.

- Topic: `/detected_object_pose`
- Type: `geometry_msgs/msg/PoseStamped`
- Reference frame: `base_link`

To send a coordinate, use the following format:

```bash
ros2 topic pub --once /detected_object_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {
    position: {x: 0.28, y: 0.03, z: -0.06},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

Field meanings:

- `position.x`
  - Distance in the forward direction relative to the robot
- `position.y`
  - Left/right position relative to the robot
- `position.z`
  - Height relative to the robot
- `orientation`
  - The absolute orientation the end-effector must reach

So a single `PoseStamped` message is all that is needed to specify a coordinate input.

## Recommended Usage

This document treats **API / code-based control** as the primary method, rather than RViz interaction.

The role of `demo.launch.py` is limited to:

- Starting the MoveIt server
- Starting `move_group`
- Starting the controllers

For actual coordinate input, the correct approach is one of the following:

1. Calling the MoveIt API
2. Setting a pose target from a user node
3. Calling a named state

In practice, **driving the robot from code** is more important than operating it through RViz.

## API / Code-Based Control

### 1. Moving to a Pose Target

The core concept is:

- Reference frame: `base_link`
- Target: end-effector pose
- Input: `position + quaternion`

In code, you simply feed MoveIt the following values:

```python
position = [0.28, 0.03, -0.06]
quat_xyzw = [0.0, 0.0, 0.0, 1.0]
```

MoveIt then:

1. Computes IK
2. Plans the trajectory
3. Executes the trajectory

### 2. Moving to a Named State

MoveIt also supports calling named states directly instead of specifying coordinates.

For the current SO-101 setup:

- manipulator
  - `zero`
  - `pick`
  - `rest`
  - `extended`
- gripper
  - `open`
  - `closed`

So from code, you can issue commands such as:

- Move the arm to `pick`
- Open the gripper to `open`

## Message-Based vs. API-Based Control

The key distinction here is:

- MoveIt default
  - No built-in topic subscriber interface for coordinate commands
- MoveIt in practice
  - API called from code
  - Or a separate control node subscribes to a topic and calls the API

So "publishing a coordinate directly to `move_group` via a message" is not the default behavior.

To use message-based control, you always need one of the following internally:

1. A user node that subscribes to the topic
2. That node calls the MoveIt API on behalf of the message

So even when an external message is sent, the actual movement is always triggered by a MoveIt API call inside the code.

## Minimal Code Example

The following is a conceptual example.

```python
from pymoveit2 import MoveIt2

moveit2 = MoveIt2(
    node=node,
    joint_names=[
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
    ],
    base_link_name="base_link",
    end_effector_name="gripper_frame_link",
    group_name="manipulator",
)

moveit2.move_to_pose(
    position=[0.28, 0.03, -0.06],
    quat_xyzw=[0.0, 0.0, 0.0, 1.0],
)
```

The key takeaway is:

- Launch the MoveIt environment with `demo.launch.py`
- Call the MoveIt API from separate code
- Control the robot by feeding a pose or named state

## To Control via Messages

If you want message-based control, the structure always looks like this:

1. Define a custom topic
2. A user node subscribes to that topic
3. The received coordinate is converted into MoveIt API input
4. MoveIt planning and execution are triggered

So messages do **not directly move the robot** — they serve as **input to the user code that calls the MoveIt API**.

## Simplest Usage

The simplest way to use `demo.launch.py` is through RViz.

1. Launch MoveIt

```bash
ros2 launch so101_moveit_config demo.launch.py
```

2. Set the goal pose in the RViz MotionPlanning panel

3. Click `Plan` then `Execute`

So if you are working purely with `demo.launch.py`, the basic way to specify a coordinate is by setting the goal pose in RViz.

## Notes

- All coordinates must be specified relative to `base_link`
- Quaternion values are also absolute
- `demo.launch.py` itself does not include a coordinate topic subscriber
- Topic-based coordinate movement requires a separate user node
- In practice, API / code-based control is more important than RViz interaction

## One-Line Summary

Running [demo.launch.py](/home/sst/so101_ws/src/so101_moveit_config/launch/demo.launch.py) actually brings up `ros2_control + MoveIt` together. Once running, MoveIt is ready to accept a target end-effector pose relative to `base_link` and perform planning and execution. In practice, the key is to control the robot by feeding a target pose or named state via separate code or API calls.
