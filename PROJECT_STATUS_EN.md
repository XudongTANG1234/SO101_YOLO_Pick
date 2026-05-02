# SO-101 Pick & Place Project Documentation

## Project Overview

A **Pick & Place** system that combines an Intel RealSense D435i camera and YOLO object detection with the SO-101 5-DOF robot arm to automatically pick up detected objects.

- **Platform**: ROS2 Jazzy / Ubuntu 24.04
- **Motion Planning**: MoveIt2 + OMPL (KDL IK solver, position-only)
- **Hardware Interface**: ros2_control + Feetech STS3215 servos
- **Perception**: YOLOv8x + Intel RealSense D435i (direct pyrealsense2 control)
- **MoveIt Python API**: pymoveit2 v4.2.0 (used in async mode)
- **GPU**: CUDA support (YOLO inference acceleration)

---

## Directory Structure

```
~/so101_ws/
└── src/
    ├── so101_description/          # URDF/Xacro robot model + STL meshes
    ├── so101_moveit_config/        # MoveIt2 configuration (SRDF, IK, OMPL, joint limits)
    ├── so101_bringup/              # Launch files, controllers, camera, teleop configuration
    ├── feetech_ros2_driver/        # Feetech STS3215 servo ros2_control hardware interface
    └── so101_pick_place/           # [New] YOLO perception + MoveIt2 Pick & Place
        ├── package.xml
        ├── setup.py / setup.cfg
        ├── resource/so101_pick_place
        ├── config/
        │   ├── pick_place.yaml             # Node parameter configuration
        │   └── follower_joints.yaml        # Servo calibration (6 joints)
        ├── launch/
        │   ├── perception.launch.py        # detect_and_locate standalone launch
        │   └── pick_place.launch.py        # Full pipeline (MoveIt + perception + pick)
        └── so101_pick_place/
            ├── __init__.py
            ├── detect_and_locate.py        # YOLO + RealSense → 3D position / normal / grip angle
            └── pick_place_node.py          # pymoveit2-based pick sequence controller
```

---

## Robot Specifications

### SO-101 Arm

| Item | Value |
|------|-------|
| DOF | 5 (manipulator) + 1 (gripper) = 6 |
| Joints | shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll |
| Gripper | gripper (open: 1.5 rad, closed: -0.16 rad) |
| Link Chain | base_link → shoulder → upper_arm → lower_arm → wrist → gripper → gripper_frame |
| IK Solver | KDL (position_only_ik: true) |
| Planning Groups | `manipulator` (5-DOF), `gripper` (1-DOF) |
| Servos | Feetech STS3215 × 6 |
| Communication | USB Serial (/dev/ttyACM0) |

### Joint Limits

| Joint | Range (rad) | Max Velocity | Max Acceleration |
|-------|-------------|--------------|-----------------|
| shoulder_pan | ±1.92 | 3.14 rad/s | 1.0 rad/s² |
| shoulder_lift | ±1.92 | 3.14 rad/s | 1.0 rad/s² |
| elbow_flex | ±1.69 | 3.14 rad/s | 1.0 rad/s² |
| wrist_flex | ±1.66 | 3.14 rad/s | 1.0 rad/s² |
| wrist_roll | ±2.84 | 3.14 rad/s | 1.0 rad/s² |
| gripper | -0.16 ~ 1.5 | 3.14 rad/s | 1.0 rad/s² |

### Predefined Poses (SRDF group_state)

| Name | Description | Joint Values |
|------|-------------|-------------|
| zero | All joints at 0 | [0, 0, 0, 0, 0] |
| pick | Grasp-ready pose | [0, 0, 0, 1.5708, 0] |
| rest | Safe standby pose | [0, -1.57, 1.57, 0.75, 0] |
| extended | Fully extended | [0, 1.57, -1.57, 0, 0] |

---

## Package Details

### 1. so101_description

Package that defines the physical model of the robot.

- **urdf/so101_arm.urdf.xacro**: Main URDF (variant: leader/follower)
- **urdf/so101_arm_common.xacro**: 5-DOF arm kinematics + link/joint definitions
- **urdf/end_effectors/**: Gripper definitions per leader/follower variant
- **urdf/ros2_control/**: Feetech hardware interface xacro
- **meshes/**: 20 STL files (visual/collision geometry)
- **launch/display*.launch.py**: For URDF visualization

### 2. so101_moveit_config

MoveIt2 motion planning configuration.

| Config File | Role |
|-------------|------|
| so101_arm.srdf | Planning groups, named poses, collision disabling, end-effector definition |
| kinematics.yaml | KDL solver, position_only_ik: true, timeout: 1.0s |
| joint_limits.yaml | Joint velocity/acceleration limits (default scale 0.5) |
| ompl_planning.yaml | OMPL planner configuration (RRTConnect, RRTstar, PRM, etc.) |
| moveit_controllers.yaml | MoveIt → ros2_control controller mapping |

**Key Launch Files**:
- `move_group.launch.py`: MoveIt server
- `demo.launch.py`: Mock hardware demo

### 3. so101_bringup

Integrated launch for hardware bringup, teleop, camera, recording, etc.

| Launch File | Purpose |
|-------------|---------|
| follower.launch.py | Follower standalone bringup |
| follower_split.launch.py | Split controllers (arm + gripper separated) |
| follower_moveit_demo.launch.py | Follower + MoveIt2 + RViz (used by pick_place) |
| teleop.launch.py | Leader → Follower teleoperation |
| cameras.launch.py | Multi-camera (RealSense, USB, GigE, V4L2) |
| recording_session.launch.py | LeRobot episode recording |

**Controller Config** (`config/ros2_control/`):
- follower_split_controllers.yaml: arm_trajectory_controller + gripper_controller

### 4. feetech_ros2_driver

ros2_control hardware interface for Feetech STS3215 servos (C++).

- Controls 6 servos via USB Serial communication
- Position command / Position+Velocity state interface
- PID, offset, and range configurable via joint config YAML

### 5. so101_pick_place (New Implementation)

Integrated Pick & Place package combining YOLO object detection and MoveIt2 motion planning.

- **Build Type**: ament_python
- **Dependencies**: rclpy, geometry_msgs, std_msgs, sensor_msgs, so101_bringup (exec), so101_moveit_config (exec)
- **Entry Points**:
  - `detect_and_locate` → `so101_pick_place.detect_and_locate:main`
  - `pick_place` → `so101_pick_place.pick_place_node:main`
- **data_files**: launch/*.py, config/*.yaml → installed to share

---

## Node Details

### Node 1: detect_and_locate

**File**: `so101_pick_place/detect_and_locate.py`
**Run**: `ros2 run so101_pick_place detect_and_locate`

#### Function
Acquires RGB+Depth from the RealSense D435i camera, detects objects with YOLO,
and computes and publishes 3D position + surface normal vector + grip angle.

- RealSense: 640×480 @ 30fps (RGB + Depth), aligned depth
- Depth filtering: center 20×20 ROI → median value, valid range 0.1~3.0m
- OpenCV visualization window: press `q` to shut down the node
- Camera + inference runs in a separate thread (`_camera_loop`), independent of ROS spin

#### Pipeline

```
RealSense D435i (separate thread: _camera_loop)
    ↓ align (depth → color frame alignment)
RGB Frame + Aligned Depth Frame
    ↓
YOLO object detection (GPU/CPU, confidence threshold filter)
    ↓ target_classes filter → highest confidence selection (best_det)
    ↓ Bounding box center (cx, cy)
Depth center ROI median → z_m (meters)
    ↓ Back-projection to 3D via camera intrinsics (x_cam, y_cam)
    ↓ Camera → World transform (R @ p + t)
/detected_object_pose (PoseStamped, frame_id='world')
/detected_surface_normal (Vector3Stamped)  ← only when normal is available
/detected_grip_angle (Float64)
```

- Log output throttled to once per second (`throttle_duration_sec=1.0`)
- When multiple target classes are present, only the one with the highest confidence is used

#### Camera → World Coordinate Transform

```
R_cam = Ry(pitch) @ Rx(-90°) @ Rz(-90°)    # Camera optical frame → world frame
P_world = R_cam @ P_camera + T              # T = [cam_x, cam_y, cam_z]
```

Current camera mount parameters:
- cam_x = 0.08m (8cm in front of base)
- cam_y = 0.03m (3cm to the left of base)
- cam_z = -0.01m (1cm below base)
- cam_pitch = 15.0° (tilted 15° downward)

#### Surface Normal Vector Computation
- Extracts valid 3D points from a 50×50 depth patch around the object center (2-pixel stride sampling, minimum 10 points required)
- Computes normal vector via SVD plane fitting (vector of the smallest singular value)
- Corrects normal in camera optical frame to point toward the camera (z < 0)
- Rotates to world frame (no translation)
- Result: vector pointing from the object surface toward the camera (approach by moving in the -normal direction)

#### Grip Angle Computation
- 2D PCA on valid depth pixels within the bounding box
- arctan2 of the major axis → grip_angle
- Short-axis alignment (+90°) + coordinate correction (-90°) cancel out → angle used as-is
- Normalized to -π/2 ~ +π/2 range (exploits gripper symmetry, prevents 180° jumps)

#### Key Methods
- `_rotation_matrix_to_quaternion(R)`: 3×3 rotation matrix → (x,y,z,w) quaternion (module-level function)
- `_build_transform(cam_x, cam_y, cam_z, pitch_deg)`: Builds camera optical → world transform (`self.R`, `self.t`)
- `cam_to_world(x_cam, y_cam, z_cam)`: Camera coordinates → world coordinates (`R @ p + t`)
- `_compute_surface_normal(depth_image, cx, cy, patch_size=25)`: SVD normal computation
- `_compute_grip_angle(depth_image, det)`: 2D PCA grip angle computation
- `_normal_to_quaternion(normal)`: Normal → orientation quaternion (**in use** — included in pose message; ignored by pick_place_node since position-only IK is used)
- `_grasp_orientation(normal, grip_angle)`: Normal + grip angle → full quaternion (**currently unused**, available for future 6-DOF use)
- `_camera_loop()`: RealSense + YOLO + publish + OpenCV visualization loop running in a separate thread
- `destroy_node()`: Stops RealSense pipeline + closes OpenCV windows

#### Executor
- `rclpy.spin(node)` — single-threaded (sufficient since the camera loop runs as a separate daemon thread)

#### Visualization
- YOLO bounding box + class/confidence display
- Blue arrow: surface normal vector
- Red arrow: grip direction (0° = upward)
- 3D coordinate text overlay

#### Published Topics

| Topic | Type | Content |
|-------|------|---------|
| `/detected_object_pose` | PoseStamped | Object 3D position (world frame) |
| `/detected_grip_angle` | Float64 | wrist_roll angle (rad) |
| `/detected_surface_normal` | Vector3Stamped | Surface normal vector (world frame) |

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| model | yolov8x.pt | YOLO model file |
| target_classes | ['sports ball'] | Detection target classes |
| confidence | 0.5 | Detection confidence threshold |
| device | cuda | Inference device |
| cam_x | 0.08 | Camera X position (m) |
| cam_y | 0.03 | Camera Y position (m) |
| cam_z | -0.01 | Camera Z position (m) |
| cam_pitch | 15.0 | Camera pitch angle (°) |

---

### Node 2: pick_place

**File**: `so101_pick_place/pick_place_node.py`
**Run**: `ros2 run so101_pick_place pick_place`

#### Function
Moves the robot arm via MoveIt2 using object coordinates received from detect_and_locate,
and executes a gripper-controlled pick sequence.

#### pymoveit2 Async Architecture

pymoveit2's sync methods (`plan()`, `execute()`, `wait_until_executed()`, `compute_ik()`)
internally call `rclpy.spin_once()`, which conflicts with MultiThreadedExecutor.
To work around this, **all calls are implemented using the async version + time.sleep polling**.

```
[Previous - not usable]
moveit2.plan()                → internally calls rclpy.spin_once() → Executor conflict
moveit2.wait_until_executed() → internally calls rclpy.spin_once() → callback interruption

[Current - Async approach]
compute_ik_async()  → future → time.sleep polling → get_compute_ik_result()
plan_async()        → future → time.sleep polling → get_trajectory()
execute()           → async send (no spin_once call)
                    → poll __is_motion_requested / __is_executing flags
```

#### MoveIt2 Configuration

| Item | Value |
|------|-------|
| group_name | `manipulator` |
| base_link | `base_link` |
| end_effector | `gripper_frame_link` |
| use_move_group_action | `False` (service interface, prevents action client conflict) |
| allowed_planning_time | 10.0 seconds |
| num_planning_attempts | 10 |
| gripper_command_action | `follower/gripper_controller/gripper_cmd` |
| gripper open/close | 1.5 / -0.16 rad |

#### Executor
- `MultiThreadedExecutor` + `ReentrantCallbackGroup` (required for pymoveit2 async callback handling)
- detect_and_locate uses single-thread (`rclpy.spin`); only pick_place uses multi-thread

#### Home Pose
- `PICK_CONFIG = [0.0, 0.0, 0.0, 1.5708, 0.0]` — same as SRDF "pick" state

#### Concurrent Command Prevention
- `is_busy` flag: ignores new commands while a command is executing (`Busy, ignoring command`)
- Automatically logs `Ready.` after command completion
- Commands other than `home` are ignored when no object has been detected (`No object detected yet`)

#### Key Internal Methods

**Callbacks:**
| Callback | Topic | Role |
|----------|-------|------|
| `_pose_cb` | `/detected_object_pose` | Updates latest_pose |
| `_grip_cb` | `/detected_grip_angle` | Updates latest_grip_angle |
| `_normal_cb` | `/detected_surface_normal` | Updates latest_normal |
| `_cmd_cb` | `/pick_place_cmd` | Routes commands (plan/execute/go/pick/home) |

**Async Helpers:**
| Method | Role |
|--------|------|
| `_solve_ik(x, y, z, quat, apply_grip_angle)` | Solves IK → returns joint config (5-second timeout) |
| `_plan_joints_async(joint_positions)` | OMPL planning → returns trajectory |
| `_execute_and_wait(trajectory)` | Executes trajectory + waits for completion (30-second timeout) |
| `_move_joints(joint_positions)` | Integrated plan + execute |
| `_move_to_position(x, y, z, quat)` | Full flow: IK → plan → execute |
| `_plan_to_position(x, y, z, quat)` | IK → plan only (no execute), viewable in RViz |
| `_get_target()` | Extracts (x,y,z) and (qx,qy,qz,qw) from latest_pose |

**Command Handlers:**
| Method | Command | Role |
|--------|---------|------|
| `_plan_to_target()` | `plan` | Applies pre_grasp_offset then calls _plan_to_position |
| `_execute_last_plan()` | `execute` | _last_joint_goal → _move_joints (re-plan + execute) |
| `_move_to_target()` | `go` | Normal standoff → target approach (2 steps) |
| `_execute_pick()` | `pick` | Standoff → gripper open 80% → approach (3 steps) |
| `_go_home()` | `home` | _move_joints to PICK_CONFIG pose |

#### Commands (String topic → /pick_place_cmd)

| Command | Behavior | Current Status |
|---------|----------|----------------|
| `plan` | IK + OMPL planning to detected coordinates (no execution) | Working |
| `execute` | Execute the last plan result | Working |
| `go` | Normal-direction 5cm standoff → target approach (2 steps) | Working (standoff removal needed) |
| `pick` | Standoff → gripper 80% open → target approach (3 steps) | Partial (standoff removal needed, IK failure on approach, gripper close not implemented) |
| `home` | Return to PICK_CONFIG pose | Working |

#### Grip Angle Application
- Receives `/detected_grip_angle` → directly mapped to wrist_roll (5th joint, index 4)
- Overwrites the IK result's wrist_roll with the grip angle
- Applied only when non-zero (controllable via `apply_grip_angle` parameter)

#### Logging
- Object position log: `throttle_duration_sec=2.0` (once every 2 seconds)
- Command receive/execute/complete logs: no throttle (always printed)

#### Subscribed Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/detected_object_pose` | PoseStamped | Target position |
| `/detected_grip_angle` | Float64 | wrist_roll angle |
| `/detected_surface_normal` | Vector3Stamped | Approach direction |
| `/pick_place_cmd` | String | Commands |

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| pre_grasp_offset | 0.05 | Pre-grasp Z offset (m) |
| max_velocity | 0.3 | Maximum velocity scale |
| max_acceleration | 0.3 | Maximum acceleration scale |

#### Topic Remapping (set in launch)
```
/joint_states               → /follower/joint_states
/robot_description          → /follower/robot_description
/robot_description_semantic → /follower/robot_description_semantic
```

---

## Launch Files

### pick_place.launch.py (Full Pipeline)

```
pick_place.launch.py
    ├── follower_moveit_demo.launch.py (from so101_bringup)
    │   ├── follower_split.launch.py
    │   │   ├── robot_state_publisher
    │   │   ├── ros2_control_node (Feetech driver)
    │   │   ├── arm_trajectory_controller (spawner)
    │   │   ├── gripper_controller (spawner)
    │   │   └── joint_state_broadcaster (spawner)
    │   ├── move_group (MoveIt2 planning server)
    │   └── rviz2
    ├── detect_and_locate (YOLO + RealSense)
    └── pick_place (MoveIt2 pick sequence)
```

### How to Run

```bash
cd ~/so101_ws
colcon build --symlink-install
source install/setup.bash

# Full pipeline (real hardware)
ros2 launch so101_pick_place pick_place.launch.py

# Mock hardware test
ros2 launch so101_pick_place pick_place.launch.py hardware_type:=mock

# Perception only
ros2 launch so101_pick_place perception.launch.py

# Send commands (--times 3 ensures reliable delivery)
ros2 topic pub --times 3 --rate 2 /pick_place_cmd std_msgs/String "data: 'go'"
ros2 topic pub --times 3 --rate 2 /pick_place_cmd std_msgs/String "data: 'pick'"
ros2 topic pub --times 3 --rate 2 /pick_place_cmd std_msgs/String "data: 'home'"
ros2 topic pub --times 3 --rate 2 /pick_place_cmd std_msgs/String "data: 'plan'"
ros2 topic pub --times 3 --rate 2 /pick_place_cmd std_msgs/String "data: 'execute'"
```

---

## Completed Work

### 1. Workspace Setup
- [x] Created so101_pick_place package (ament_python)
- [x] Wrote package.xml, setup.py, setup.cfg
- [x] Integrated existing packages (description, moveit_config, bringup, feetech_driver)

### 2. detect_and_locate Node Implementation
- [x] YOLO + RealSense D435i integration
- [x] Camera → World coordinate transform (numpy-based)
- [x] Applied physically measured camera mount parameters (cam_x=0.08, cam_y=0.03, cam_z=-0.01, cam_pitch=15.0)
- [x] Surface normal vector computation (SVD plane fitting)
- [x] Grip angle computation (2D PCA → direct wrist_roll mapping)
- [x] Grip angle normalization to -π/2 ~ +π/2 (prevents 180° jumps)
- [x] YOLO visualization (bounding box, normal arrow, grip arrow, 3D coordinates)
- [x] YOLO model upgraded yolov8n → yolov8x (significant accuracy improvement)
- [x] Separate-thread camera pipeline (non-blocking ROS callbacks)

### 3. pick_place_node Implementation
- [x] Migrated to pymoveit2 async architecture (resolved rclpy.spin_once conflict)
- [x] Implemented compute_ik_async + plan_async + execute (async send)
- [x] Added 5-second IK timeout (prevents infinite wait)
- [x] Added 30-second execution timeout
- [x] Direct grip angle → wrist_roll mapping (no quaternion used)
- [x] Normal vector-based approach direction
- [x] Implemented plan / execute / go / pick / home commands
- [x] Implemented gripper 80% open action
- [x] Set up MultiThreadedExecutor + ReentrantCallbackGroup
- [x] use_move_group_action=False (using service interface)

### 4. Major Issues Resolved
- [x] pymoveit2 sync methods → MultiThreadedExecutor conflict: resolved by switching to async
- [x] Camera coordinate sign inversion: fixed with measured values
- [x] Grip angle wrist_roll out of range: normalized to -π/2 ~ +π/2
- [x] Grip angle 180° instability: resolved by normalization
- [x] IK timeout hang: added 5-second limit
- [x] Publish order bug (UnboundLocalError): fixed order
- [x] `ros2 topic pub --once` message not received: recommend `--times 3 --rate 2`

---

## Remaining Work (TODO)

### Priority 1: Target Position/Orientation Accuracy

IK currently fails at some positions, and z values are sometimes computed as negative (below base).
The accuracy of the camera → world transform needs to be improved.

- [ ] **Camera calibration refinement**: Currently using manually measured values → automated calibration or more precise measurement
- [ ] **z-coordinate validation**: Analyze and fix cases where the object is computed below the base (z < 0)
- [ ] **IK failure analysis**: Debug cases where IK fails at reachable positions
  - Current log: standoff position IK succeeds, actual target position IK times out
  - z=-0.092 and similar negative z values may exceed KDL solver reach
- [ ] **Remove 5cm standoff**: Update `_move_to_target` (go) and `_execute_pick` (pick) to use detected coordinates directly without standoff approach
- [ ] **Orientation refinement**: Currently using position-only IK → full orientation planning possible when upgrading to 6-DOF
- [ ] **Coordinate stabilization**: YOLO bounding box jitter causes coordinate instability → consider applying moving average or Kalman filter
- [ ] **Custom YOLO model training**: Improve detection stability with a model trained for specific objects

### Priority 2: Complete the Pick Sequence

Currently implemented pick sequence:
```
Step 1/3: Move to standoff position (5cm back along normal direction)
Step 2/3: Open gripper 80% (gripper.move_to_position(1.17))
Step 3/3: Approach target position (move along normal direction)
=== Pick Complete ===
```

**Note**: In Step 2, `gripper.wait_until_executed()` is a pymoveit2 sync method that
internally calls `rclpy.spin_once()`. It is currently working but carries a potential conflict risk.

Remaining items:
- [ ] **Gripper close**: Close gripper after arriving at target position
- [ ] **Lift**: Raise along the z-axis after closing the gripper
- [ ] **Move to place position**: Move to the designated place location
- [ ] **Gripper open (release)**: Open gripper at place position to release object
- [ ] **Return home**: Return to home pose after placing
- [ ] **Full pick & place sequence**: Integrated command for pick → lift → place → release → home
- [ ] **Place position specification**: Decide on method — fixed coordinates / topic / parameter

### Priority 3: Stability and Enhancements

- [ ] **Error recovery**: Return to safe pose on motion failure
- [ ] **Sequential operation**: Pick & place multiple objects in sequence
- [ ] **Collision avoidance**: Add table/obstacles to MoveIt2 planning scene
- [ ] **6-DOF upgrade**: Full orientation planning when transitioning to a 6-DOF robot
- [ ] **Force/Torque feedback**: Gripper force control to prevent object damage

---

## Topic Map

```
[detect_and_locate]
    ├──pub→ /detected_object_pose     (PoseStamped)
    ├──pub→ /detected_grip_angle      (Float64)
    └──pub→ /detected_surface_normal  (Vector3Stamped)

[pick_place]
    ├──sub← /detected_object_pose
    ├──sub← /detected_grip_angle
    ├──sub← /detected_surface_normal
    ├──sub← /pick_place_cmd           (String: plan/execute/go/pick/home)
    │
    ├──srv→ /compute_ik               (MoveIt2 IK service)
    ├──srv→ /plan_kinematic_path      (MoveIt2 planning service)
    └──act→ /execute_trajectory       (MoveIt2 execution action)

[move_group]  (MoveIt2)
    ├──sub← /follower/joint_states
    ├──sub← /follower/robot_description
    └──sub← /follower/robot_description_semantic

[ros2_control_node]  (Feetech driver)
    ├──pub→ /follower/joint_states
    ├──sub← arm_trajectory_controller commands
    └──sub← gripper_controller commands
```

---

## Config / Launch Parameter Mismatch (Note)

**The values actually applied come from what is directly specified in the launch file.** `config/pick_place.yaml` exists for reference only and is overridden by the launch file.

| Parameter | config/pick_place.yaml (old) | pick_place.launch.py (actual) | perception.launch.py (old) |
|-----------|------------------------------|-------------------------------|---------------------------|
| model | yolov8n.pt | **yolov8x.pt** | yolov8n.pt |
| cam_x | 0.05 | **0.08** | 0.05 |
| cam_y | 0.03 | 0.03 | 0.0 |
| cam_z | -0.09 | **-0.01** | -0.09 |
| cam_pitch | 0.0 | **15.0** | 0.0 |
| pre_grasp_offset | 0.05 | **0.0** | - |

> **TODO**: Update default values in `config/pick_place.yaml` and `perception.launch.py` to the latest measured values

---

## Python Dependencies

| Package | Purpose | Installation |
|---------|---------|-------------|
| pymoveit2 | MoveIt2 Python API | `apt: ros-jazzy-pymoveit2` |
| ultralytics | YOLOv8 | `pip install ultralytics` |
| pyrealsense2 | Intel RealSense SDK | `pip install pyrealsense2` |
| opencv-python | Visualization / image processing | `pip install opencv-python` |
| numpy | Math / matrix operations | `pip install numpy` |

---

## Known Issues

| Issue | Status | Description |
|-------|--------|-------------|
| IK timeout | Unresolved | KDL IK times out after 5 seconds at z < 0 positions. Reachability needs verification |
| 5cm standoff not removed | Unresolved | go/pick commands still include the 5cm standoff approach |
| Coordinate instability | Partially resolved | Coordinate jitter from YOLO bbox instability. Improved with yolov8x but not fully resolved |
| `--once` message not received | Workaround | `ros2 topic pub --once` intermittently drops messages. Use `--times 3 --rate 2` |
| Gripper close not implemented | Not implemented | pick sequence has no gripper close after approach |
| pymoveit2 `motion_suceeded` typo | Note | Internal pymoveit2 variable is named `motion_suceeded` (typo). Used as-is in code |
| config/launch parameter mismatch | Unresolved | `pick_place.yaml` and `perception.launch.py` default values are outdated |
| detect_and_locate code default mismatch | Unresolved | `declare_parameter` defaults in code (e.g. cam_x=0.05) differ from launch values (no runtime impact since launch overrides) |

---

## Reference: pymoveit2 Async Pattern

pymoveit2 v4.2.0 sync methods internally call `rclpy.spin_once()` and cannot be used
with a MultiThreadedExecutor. Use the following pattern instead:

```python
# Solve IK
future = moveit2.compute_ik_async(position=[x, y, z], quat_xyzw=quat)
while not future.done():
    time.sleep(0.01)
joint_state = moveit2.get_compute_ik_result(future)

# Planning
future = moveit2.plan_async(joint_positions=joint_positions)
while not future.done():
    time.sleep(0.01)
trajectory = moveit2.get_trajectory(future)

# Execution (async send - no spin_once call)
moveit2.execute(trajectory)
while moveit2._MoveIt2__is_motion_requested or moveit2._MoveIt2__is_executing:
    time.sleep(0.05)
success = moveit2.motion_suceeded  # note the typo
```

**Important notes**:
- `use_move_group_action=False` is required (uses service interface)
- `moveit2._MoveIt2__is_executing` is private variable access (name mangling)
- ReentrantCallbackGroup + MultiThreadedExecutor combination is required

---

## Recent Test Log Summary (2026-03-25)

**Test**: Attempted to pick a sports ball using the `pick` command

```
Object at (0.266, 0.030, -0.024)    ← Detected object coordinates (z negative = below base)

Command: "pick"
=== Pick Start ===
Step 1/3: Standoff (0.216, 0.027, -0.014)     ← 5cm back along normal direction
  → IK solved: [-0.147, -0.005, 1.584, -0.883, -0.073] (wrist_roll=-4°)
  → Plan found, executing...
  → Arrived.                                    ← standoff succeeded

Step 2/3: Open gripper 80% (1.17)             ← gripper open succeeded

Step 3/3: Approach (0.265, 0.030, -0.023)     ← actual target position
  → IK timeout!                                ← IK 5-second timeout failure
  → Approach failed.
=== Pick Complete ===
```

**Analysis**:
- Standoff position (z=-0.014) IK succeeds; target position (z=-0.023) IK fails
- Suspected to exceed reach as z decreases further
- However, the user considers this "a position the arm should be able to reach"
- **Possible causes**: camera z-coordinate transform issue, KDL solver limitation, or the joint state after moving to standoff affecting the next IK solve

---

## Development History Summary

1. **Workspace setup**: Created new SO-101 workspace from openarm_moveit
2. **detect_and_locate port**: Adapted openarm version for SO-101
3. **pick_place_node implementation**: Migrated pymoveit2 sync → async (core architecture change)
4. **Camera calibration**: Fixed sign inversion, applied measured values, added pitch
5. **Normal vector + grip angle**: Implemented SVD plane fitting + 2D PCA
6. **Grip angle separation**: Changed from quaternion integration to direct wrist_roll mapping
7. **Visualization improvements**: Normal/grip arrows, coordinate overlay
8. **YOLO model upgrade**: yolov8n → yolov8x (significant accuracy improvement)
9. **IK stability**: Added timeout, implemented busy guard
10. **Current**: Debugging IK failures + working on standoff removal
