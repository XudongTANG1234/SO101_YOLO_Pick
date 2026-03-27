# SO-101 Pick & Place 프로젝트 문서

## 프로젝트 개요

SO-101 5-DOF 로봇 팔에 RealSense D435i 카메라와 YOLO 객체 인식을 결합하여,
감지된 물체를 자동으로 집어올리는 **Pick & Place** 시스템.

- **플랫폼**: ROS2 Jazzy / Ubuntu 24.04
- **모션 플래닝**: MoveIt2 + OMPL (KDL IK solver, position-only)
- **하드웨어 인터페이스**: ros2_control + Feetech STS3215 서보
- **인식**: YOLOv8x + Intel RealSense D435i (pyrealsense2 직접 제어)
- **MoveIt Python API**: pymoveit2 v4.2.0 (async 방식으로 사용)
- **GPU**: CUDA 지원 (YOLO 추론 가속)

---

## 디렉토리 구조

```
~/so101_ws/
└── src/
    ├── so101_description/          # URDF/Xacro 로봇 모델 + STL 메시
    ├── so101_moveit_config/        # MoveIt2 설정 (SRDF, IK, OMPL, 조인트 제한)
    ├── so101_bringup/              # Launch, 컨트롤러, 카메라, 텔레옵 설정
    ├── feetech_ros2_driver/        # Feetech STS3215 서보 ros2_control 하드웨어 인터페이스
    └── so101_pick_place/           # [신규] YOLO 인식 + MoveIt2 Pick & Place
        ├── package.xml
        ├── setup.py / setup.cfg
        ├── resource/so101_pick_place
        ├── config/
        │   ├── pick_place.yaml             # 노드 파라미터 설정
        │   └── follower_joints.yaml        # 서보 캘리브레이션 (6 joints)
        ├── launch/
        │   ├── perception.launch.py        # detect_and_locate 단독 실행
        │   └── pick_place.launch.py        # 전체 파이프라인 (MoveIt + 인식 + pick)
        └── so101_pick_place/
            ├── __init__.py
            ├── detect_and_locate.py        # YOLO + RealSense → 3D 좌표/법선/그립각도
            └── pick_place_node.py          # pymoveit2 기반 pick 시퀀스 제어
```

---

## 로봇 사양

### SO-101 Arm

| 항목 | 값 |
|------|-----|
| DOF | 5 (manipulator) + 1 (gripper) = 6 |
| 조인트 | shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll |
| 그리퍼 | gripper (열림: 1.5 rad, 닫힘: -0.16 rad) |
| 링크 체인 | base_link → shoulder → upper_arm → lower_arm → wrist → gripper → gripper_frame |
| IK Solver | KDL (position_only_ik: true) |
| 플래닝 그룹 | `manipulator` (5-DOF), `gripper` (1-DOF) |
| 서보 | Feetech STS3215 x 6 |
| 통신 | USB Serial (/dev/ttyACM0) |

### 조인트 제한

| Joint | Range (rad) | Max Velocity | Max Acceleration |
|-------|------------|--------------|-----------------|
| shoulder_pan | ±1.92 | 3.14 rad/s | 1.0 rad/s² |
| shoulder_lift | ±1.92 | 3.14 rad/s | 1.0 rad/s² |
| elbow_flex | ±1.69 | 3.14 rad/s | 1.0 rad/s² |
| wrist_flex | ±1.66 | 3.14 rad/s | 1.0 rad/s² |
| wrist_roll | ±2.84 | 3.14 rad/s | 1.0 rad/s² |
| gripper | -0.16 ~ 1.5 | 3.14 rad/s | 1.0 rad/s² |

### 미리 정의된 자세 (SRDF group_state)

| 이름 | 설명 | Joint Values |
|------|------|-------------|
| zero | 모든 조인트 0 | [0, 0, 0, 0, 0] |
| pick | 그립 준비 자세 | [0, 0, 0, 1.5708, 0] |
| rest | 안전 대기 자세 | [0, -1.57, 1.57, 0.75, 0] |
| extended | 완전 확장 | [0, 1.57, -1.57, 0, 0] |

---

## 패키지별 상세 설명

### 1. so101_description

로봇의 물리적 모델을 정의하는 패키지.

- **urdf/so101_arm.urdf.xacro**: 메인 URDF (variant: leader/follower)
- **urdf/so101_arm_common.xacro**: 5-DOF 팔 kinematics + 링크/조인트 정의
- **urdf/end_effectors/**: leader/follower별 그리퍼 정의
- **urdf/ros2_control/**: Feetech 하드웨어 인터페이스 xacro
- **meshes/**: 20개 STL 파일 (시각/충돌 지오메트리)
- **launch/display*.launch.py**: URDF 시각화용

### 2. so101_moveit_config

MoveIt2 모션 플래닝 설정.

| 설정 파일 | 역할 |
|-----------|------|
| so101_arm.srdf | 플래닝 그룹, 명명 자세, 충돌 비활성화, end-effector 정의 |
| kinematics.yaml | KDL solver, position_only_ik: true, timeout: 1.0s |
| joint_limits.yaml | 조인트 속도/가속도 제한 (기본 스케일 0.5) |
| ompl_planning.yaml | OMPL 플래너 설정 (RRTConnect, RRTstar, PRM 등) |
| moveit_controllers.yaml | MoveIt → ros2_control 컨트롤러 매핑 |

**주요 Launch**:
- `move_group.launch.py`: MoveIt 서버
- `demo.launch.py`: mock 하드웨어 데모

### 3. so101_bringup

하드웨어 구동, 텔레옵, 카메라, 녹화 등 통합 launch.

| Launch 파일 | 용도 |
|------------|------|
| follower.launch.py | Follower 단독 구동 |
| follower_split.launch.py | Split 컨트롤러 (arm + gripper 분리) |
| follower_moveit_demo.launch.py | Follower + MoveIt2 + RViz (pick_place에서 사용) |
| teleop.launch.py | Leader → Follower 텔레옵 |
| cameras.launch.py | 멀티 카메라 (RealSense, USB, GigE, V4L2) |
| recording_session.launch.py | LeRobot 에피소드 녹화 |

**Controller Config** (`config/ros2_control/`):
- follower_split_controllers.yaml: arm_trajectory_controller + gripper_controller

### 4. feetech_ros2_driver

Feetech STS3215 서보용 ros2_control 하드웨어 인터페이스 (C++).

- USB Serial 통신으로 6개 서보 제어
- Position command / Position+Velocity state interface
- Joint config YAML로 PID, offset, range 설정 가능

### 5. so101_pick_place (신규 구현)

YOLO 객체 인식 + MoveIt2 모션 플래닝 통합 Pick & Place 패키지.

- **빌드 타입**: ament_python
- **의존성**: rclpy, geometry_msgs, std_msgs, sensor_msgs, so101_bringup(exec), so101_moveit_config(exec)
- **Entry Points**:
  - `detect_and_locate` → `so101_pick_place.detect_and_locate:main`
  - `pick_place` → `so101_pick_place.pick_place_node:main`
- **data_files**: launch/*.py, config/*.yaml → share에 설치

---

## 노드별 상세 설명

### Node 1: detect_and_locate

**파일**: `so101_pick_place/detect_and_locate.py`
**실행**: `ros2 run so101_pick_place detect_and_locate`

#### 기능
RealSense D435i 카메라로 RGB+Depth를 취득하고, YOLO로 객체를 검출하여
3D 좌표 + 표면 법선 벡터 + 그립 각도를 계산, 퍼블리시.

- RealSense: 640x480 @ 30fps (RGB + Depth), aligned depth
- Depth 필터링: 중심 20x20 ROI → median 값 사용, 유효 범위 0.1~3.0m
- OpenCV 시각화 창: `q` 키로 노드 종료 가능
- 별도 스레드(`_camera_loop`)에서 카메라+추론 수행, ROS spin과 독립

#### 파이프라인

```
RealSense D435i (별도 스레드, _camera_loop)
    ↓ align (depth → color 프레임 정렬)
RGB Frame + Aligned Depth Frame
    ↓
YOLO 객체 검출 (GPU/CPU, conf 임계값 필터)
    ↓ target_classes 필터 → 최고 confidence 선택 (best_det)
    ↓ Bounding Box 중심 (cx, cy)
Depth 중심 ROI median → z_m (미터)
    ↓ 카메라 intrinsics로 3D 역투영 (x_cam, y_cam)
    ↓ Camera → World 변환 (R @ p + t)
/detected_object_pose (PoseStamped, frame_id='world')
/detected_surface_normal (Vector3Stamped)  ← 법선 있을 때만
/detected_grip_angle (Float64)
```

- 로그 출력은 `throttle_duration_sec=1.0`으로 1초에 1회 제한
- 여러 target class가 있으면 그 중 confidence가 가장 높은 것만 사용

#### 카메라 → World 좌표 변환

```
R_cam = Ry(pitch) @ Rx(-90°) @ Rz(-90°)    # 카메라 광학 프레임 → 월드 프레임
P_world = R_cam @ P_camera + T              # T = [cam_x, cam_y, cam_z]
```

현재 카메라 설치 파라미터:
- cam_x = 0.08m (베이스 앞 8cm)
- cam_y = 0.03m (베이스 왼쪽 3cm)
- cam_z = -0.01m (베이스 아래 1cm)
- cam_pitch = 15.0° (아래로 15도)

#### 표면 법선 벡터 계산
- 객체 중심 50x50 depth patch에서 유효 3D 포인트 추출 (2픽셀 간격 샘플링, 최소 10개 필요)
- SVD 평면 피팅으로 법선 벡터 계산 (가장 작은 singular value의 벡터)
- 카메라 optical frame에서 법선이 카메라를 향하도록 보정 (z < 0)
- World 프레임으로 회전 변환 (translation 없음)
- 결과: 물체 표면에서 카메라 방향으로 가리키는 벡터 (접근 시 -normal 방향으로 이동)

#### 그립 각도 계산
- Bounding box 내 유효 depth 픽셀에 대해 2D PCA
- 주축(major axis)의 arctan2 → grip_angle
- 짧은 축 정렬(+90°) + 좌표계 보정(-90°) = 상쇄 → angle 그대로 사용
- -π/2 ~ +π/2 범위로 정규화 (그리퍼 대칭 활용, 180도 점프 방지)

#### 주요 메서드
- `_rotation_matrix_to_quaternion(R)`: 3x3 회전 행렬 → (x,y,z,w) 쿼터니언 (모듈 레벨 함수)
- `_build_transform(cam_x, cam_y, cam_z, pitch_deg)`: 카메라 optical → world 변환 행렬 구성 (`self.R`, `self.t`)
- `cam_to_world(x_cam, y_cam, z_cam)`: 카메라 좌표 → world 좌표 변환 (`R @ p + t`)
- `_compute_surface_normal(depth_image, cx, cy, patch_size=25)`: SVD 법선 계산
- `_compute_grip_angle(depth_image, det)`: 2D PCA 그립 각도 계산
- `_normal_to_quaternion(normal)`: 법선 → orientation quaternion (**사용 중** — pose 메시지에 포함. 단, pick_place_node에서는 position-only IK이므로 무시됨)
- `_grasp_orientation(normal, grip_angle)`: 법선+그립각도 → full quaternion (**현재 미사용**, 향후 6-DOF에서 활용 가능)
- `_camera_loop()`: 별도 스레드에서 RealSense + YOLO + publish + OpenCV 시각화 루프
- `destroy_node()`: RealSense pipeline 정지 + OpenCV 창 정리

#### Executor
- `rclpy.spin(node)` — 싱글스레드 (카메라 루프가 별도 daemon 스레드이므로 충분)

#### 시각화
- YOLO 바운딩 박스 + 클래스/신뢰도 표시
- 파란색 화살표: 표면 법선 벡터
- 빨간색 화살표: 그립 방향 (0° = 위)
- 3D 좌표 텍스트 오버레이

#### Publish Topics

| Topic | Type | 내용 |
|-------|------|------|
| `/detected_object_pose` | PoseStamped | 객체 3D 위치 (world 프레임) |
| `/detected_grip_angle` | Float64 | wrist_roll 각도 (rad) |
| `/detected_surface_normal` | Vector3Stamped | 표면 법선 벡터 (world 프레임) |

#### Parameters

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| model | yolov8x.pt | YOLO 모델 파일 |
| target_classes | ['sports ball'] | 검출 대상 클래스 |
| confidence | 0.5 | 검출 신뢰도 임계값 |
| device | cuda | 추론 디바이스 |
| cam_x | 0.08 | 카메라 X 위치 (m) |
| cam_y | 0.03 | 카메라 Y 위치 (m) |
| cam_z | -0.01 | 카메라 Z 위치 (m) |
| cam_pitch | 15.0 | 카메라 피치 각도 (°) |

---

### Node 2: pick_place

**파일**: `so101_pick_place/pick_place_node.py`
**실행**: `ros2 run so101_pick_place pick_place`

#### 기능
detect_and_locate에서 수신한 객체 좌표로 MoveIt2를 통해 로봇 팔을 이동시키고,
그리퍼를 제어하여 물체를 집는 시퀀스를 실행.

#### pymoveit2 Async 아키텍처

pymoveit2의 sync 메서드(`plan()`, `execute()`, `wait_until_executed()`, `compute_ik()`)는
내부적으로 `rclpy.spin_once()`를 호출하여 MultiThreadedExecutor와 충돌한다.
이를 해결하기 위해 **모든 호출을 async 버전 + time.sleep 대기**로 구현했다.

```
[기존 - 사용 불가]
moveit2.plan()              → 내부에서 rclpy.spin_once() 호출 → Executor 충돌
moveit2.wait_until_executed() → 내부에서 rclpy.spin_once() 호출 → 콜백 중단

[현재 - Async 방식]
compute_ik_async()  → future → time.sleep 폴링 → get_compute_ik_result()
plan_async()        → future → time.sleep 폴링 → get_trajectory()
execute()           → async send (spin_once 미호출)
                    → __is_motion_requested / __is_executing 플래그 폴링
```

#### MoveIt2 설정

| 항목 | 값 |
|------|-----|
| group_name | `manipulator` |
| base_link | `base_link` |
| end_effector | `gripper_frame_link` |
| use_move_group_action | `False` (서비스 인터페이스, 액션 클라이언트 충돌 방지) |
| allowed_planning_time | 10.0초 |
| num_planning_attempts | 10회 |
| gripper_command_action | `follower/gripper_controller/gripper_cmd` |
| gripper open/close | 1.5 / -0.16 rad |

#### Executor
- `MultiThreadedExecutor` + `ReentrantCallbackGroup` (pymoveit2 async 콜백 처리에 필수)
- detect_and_locate는 싱글스레드(`rclpy.spin`), pick_place만 멀티스레드

#### Home 자세
- `PICK_CONFIG = [0.0, 0.0, 0.0, 1.5708, 0.0]` — SRDF "pick" state와 동일

#### 동시 명령 방지
- `is_busy` 플래그: 명령 실행 중 새 명령 무시 (`Busy, ignoring command`)
- 명령 완료 후 자동으로 `Ready.` 로그 출력
- 객체 미감지 상태에서 `home` 외 명령 무시 (`No object detected yet`)

#### 핵심 내부 메서드

**콜백:**
| 콜백 | Topic | 역할 |
|------|-------|------|
| `_pose_cb` | `/detected_object_pose` | latest_pose 업데이트 |
| `_grip_cb` | `/detected_grip_angle` | latest_grip_angle 업데이트 |
| `_normal_cb` | `/detected_surface_normal` | latest_normal 업데이트 |
| `_cmd_cb` | `/pick_place_cmd` | 명령 분기 (plan/execute/go/pick/home) |

**Async 헬퍼:**
| 메서드 | 역할 |
|--------|------|
| `_solve_ik(x, y, z, quat, apply_grip_angle)` | IK 풀기 → joint config 반환 (5초 타임아웃) |
| `_plan_joints_async(joint_positions)` | OMPL 플래닝 → trajectory 반환 |
| `_execute_and_wait(trajectory)` | trajectory 실행 + 완료 대기 (30초 타임아웃) |
| `_move_joints(joint_positions)` | plan + execute 통합 |
| `_move_to_position(x, y, z, quat)` | IK → plan → execute 전체 흐름 |
| `_plan_to_position(x, y, z, quat)` | IK → plan만 (execute 안 함), RViz에서 확인 가능 |
| `_get_target()` | latest_pose에서 (x,y,z), (qx,qy,qz,qw) 추출 |

**명령 핸들러:**
| 메서드 | 명령 | 역할 |
|--------|------|------|
| `_plan_to_target()` | `plan` | pre_grasp_offset 적용 후 _plan_to_position |
| `_execute_last_plan()` | `execute` | _last_joint_goal → _move_joints (재플래닝+실행) |
| `_move_to_target()` | `go` | 법선 standoff → 목표 접근 (2단계) |
| `_execute_pick()` | `pick` | standoff → gripper open 80% → 접근 (3단계) |
| `_go_home()` | `home` | PICK_CONFIG 자세로 _move_joints |

#### 명령어 (String topic → /pick_place_cmd)

| 명령 | 동작 | 현재 상태 |
|------|------|----------|
| `plan` | 감지 좌표로 IK + OMPL 플래닝 (실행 안 함) | 동작 |
| `execute` | 마지막 plan 결과 실행 | 동작 |
| `go` | 법선 방향 5cm standoff → 목표 접근 (2단계) | 동작 (standoff 제거 필요) |
| `pick` | standoff → 그리퍼 80% 열기 → 목표 접근 (3단계) | 부분동작 (standoff 제거 필요, 접근 시 IK 실패 이슈, gripper close 미구현) |
| `home` | PICK_CONFIG 자세로 복귀 | 동작 |

#### 그립 각도 적용
- `/detected_grip_angle` 수신 → wrist_roll (5번째 조인트, index 4)에 직접 매핑
- IK 결과의 wrist_roll을 그립 각도로 덮어쓰기
- 0이 아닌 경우에만 적용 (`apply_grip_angle` 파라미터로 제어 가능)

#### 로그
- Object at 로그: `throttle_duration_sec=2.0` (2초에 1회)
- 명령 수신/실행/완료 로그는 throttle 없음 (매번 출력)

#### Subscribe Topics

| Topic | Type | 용도 |
|-------|------|------|
| `/detected_object_pose` | PoseStamped | 목표 위치 |
| `/detected_grip_angle` | Float64 | wrist_roll 각도 |
| `/detected_surface_normal` | Vector3Stamped | 접근 방향 |
| `/pick_place_cmd` | String | 명령어 |

#### Parameters

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| pre_grasp_offset | 0.05 | pre-grasp Z 오프셋 (m) |
| max_velocity | 0.3 | 최대 속도 스케일 |
| max_acceleration | 0.3 | 최대 가속도 스케일 |

#### Topic Remapping (launch에서 설정)
```
/joint_states         → /follower/joint_states
/robot_description    → /follower/robot_description
/robot_description_semantic → /follower/robot_description_semantic
```

---

## Launch 파일

### pick_place.launch.py (전체 파이프라인)

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

### 실행 방법

```bash
cd ~/so101_ws
colcon build --symlink-install
source install/setup.bash

# 전체 파이프라인 (실제 하드웨어)
ros2 launch so101_pick_place pick_place.launch.py

# mock 하드웨어 테스트
ros2 launch so101_pick_place pick_place.launch.py hardware_type:=mock

# 인식만 테스트
ros2 launch so101_pick_place perception.launch.py

# 명령 전송 (--times 3으로 전달 안정성 확보)
ros2 topic pub --times 3 --rate 2 /pick_place_cmd std_msgs/String "data: 'go'"
ros2 topic pub --times 3 --rate 2 /pick_place_cmd std_msgs/String "data: 'pick'"
ros2 topic pub --times 3 --rate 2 /pick_place_cmd std_msgs/String "data: 'home'"
ros2 topic pub --times 3 --rate 2 /pick_place_cmd std_msgs/String "data: 'plan'"
ros2 topic pub --times 3 --rate 2 /pick_place_cmd std_msgs/String "data: 'execute'"
```

---

## 완료된 작업

### 1. 워크스페이스 구성
- [x] so101_pick_place 패키지 생성 (ament_python)
- [x] package.xml, setup.py, setup.cfg 작성
- [x] 기존 패키지 통합 (description, moveit_config, bringup, feetech_driver)

### 2. detect_and_locate 노드 구현
- [x] YOLO + RealSense D435i 통합
- [x] 카메라 → World 좌표 변환 (numpy 기반)
- [x] 카메라 설치 파라미터 실측값 적용 (cam_x=0.08, cam_y=0.03, cam_z=-0.01, cam_pitch=15.0)
- [x] 표면 법선 벡터 계산 (SVD 평면 피팅)
- [x] 그립 각도 계산 (2D PCA → wrist_roll 직접 매핑)
- [x] 그립 각도 -π/2 ~ +π/2 정규화 (180도 점프 방지)
- [x] YOLO 시각화 (바운딩 박스, 법선 화살표, 그립 화살표, 3D 좌표)
- [x] YOLO 모델 yolov8n → yolov8x 업그레이드 (정확도 대폭 향상)
- [x] 별도 스레드 카메라 파이프라인 (ROS 콜백 비차단)

### 3. pick_place_node 구현
- [x] pymoveit2 async 아키텍처 전환 (rclpy.spin_once 충돌 해결)
- [x] compute_ik_async + plan_async + execute (async send) 구현
- [x] IK 5초 타임아웃 추가 (무한 대기 방지)
- [x] 실행 30초 타임아웃 추가
- [x] 그립 각도 → wrist_roll 직접 매핑 (쿼터니언 미사용)
- [x] 법선 벡터 기반 접근 방향 구현
- [x] plan / execute / go / pick / home 명령어 구현
- [x] 그리퍼 80% 열기 동작 구현
- [x] MultiThreadedExecutor + ReentrantCallbackGroup 설정
- [x] use_move_group_action=False (서비스 인터페이스 사용)

### 4. 해결한 주요 문제들
- [x] pymoveit2 sync 메서드 → MultiThreadedExecutor 충돌: async 전환으로 해결
- [x] 카메라 좌표 부호 반전 문제: 실측값으로 수정
- [x] 그립 각도 wrist_roll 범위 초과: -π/2 ~ +π/2 정규화
- [x] 그립 각도 180도 불안정: 정규화로 해결
- [x] IK 타임아웃 hang: 5초 제한 추가
- [x] publish 순서 버그 (UnboundLocalError): 순서 수정
- [x] `ros2 topic pub --once` 메시지 미수신: `--times 3 --rate 2` 권장

---

## 남은 작업 (TODO)

### 우선순위 1: 타겟 위치/방향 정확도 고도화

현재 IK가 일부 위치에서 실패하며, z값이 음수(베이스 아래)로 계산되는 경우가 있다.
카메라 → World 변환의 정확도를 더 높여야 한다.

- [ ] **카메라 캘리브레이션 정밀화**: 현재 수동 실측값 사용 → 자동 캘리브레이션 또는 더 정확한 측정
- [ ] **z좌표 검증**: 물체가 베이스보다 아래(z < 0)로 계산되는 원인 분석 및 수정
- [ ] **IK 실패 원인 분석**: 도달 가능한 위치에서 IK가 실패하는 케이스 디버깅
  - 현재 로그: standoff 위치는 IK 성공, 실제 목표 위치는 IK 타임아웃
  - z=-0.092 등 음수 z값이 KDL solver의 도달 범위를 벗어날 가능성
- [ ] **5cm standoff 제거**: `_move_to_target`(go)과 `_execute_pick`(pick)에서 standoff 접근을 제거하고 감지된 좌표를 그대로 사용하도록 변경
- [ ] **오리엔테이션 정밀화**: 현재 position-only IK 사용 → 6-DOF 업그레이드 시 full orientation 플래닝 가능
- [ ] **좌표 안정화**: YOLO 바운딩 박스 흔들림으로 좌표가 불안정 → 이동 평균 또는 칼만 필터 적용 검토
- [ ] **커스텀 YOLO 모델 학습**: 특정 물체에 대한 학습 모델로 검출 안정성 향상

### 우선순위 2: Pick 시퀀스 완성

현재 구현된 pick 시퀀스:
```
Step 1/3: Standoff 위치로 이동 (법선 방향 5cm 뒤)
Step 2/3: 그리퍼 80% 열기 (gripper.move_to_position(1.17))
Step 3/3: 목표 위치로 접근 (법선 방향 이동)
=== Pick Complete ===
```

**주의**: Step 2에서 `gripper.wait_until_executed()`는 pymoveit2 sync 메서드로,
내부에서 `rclpy.spin_once()`를 호출한다. 현재 동작하고 있지만 잠재적 충돌 위험이 있다.

남은 구현 항목:
- [ ] **그리퍼 닫기**: 목표 위치 도착 후 그리퍼 닫기 (gripper close)
- [ ] **들어올리기 (Lift)**: 그리퍼 닫은 후 z축으로 들어올리기
- [ ] **Place 위치 이동**: 지정된 Place 위치로 이동
- [ ] **그리퍼 열기 (Release)**: Place 위치에서 그리퍼 열어 물체 놓기
- [ ] **Home 복귀**: Place 후 home 자세로 복귀
- [ ] **전체 Pick & Place 시퀀스**: pick → lift → place → release → home 통합 명령
- [ ] **Place 위치 지정 방법**: 고정 좌표 / 토픽 / 파라미터 등 결정 필요

### 우선순위 3: 안정성 및 고도화

- [ ] **에러 복구**: 모션 실패 시 안전 자세로 복귀
- [ ] **연속 동작**: 여러 물체 순차 pick & place
- [ ] **충돌 회피**: MoveIt2 planning scene에 테이블/장애물 추가
- [ ] **6-DOF 업그레이드**: SO-101 → 6-DOF 로봇 전환 시 full orientation 플래닝
- [ ] **Force/Torque 피드백**: 그리퍼 힘 제어로 물체 파손 방지

---

## 토픽 맵

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
    ├──srv→ /compute_ik               (MoveIt2 IK 서비스)
    ├──srv→ /plan_kinematic_path      (MoveIt2 플래닝 서비스)
    └──act→ /execute_trajectory       (MoveIt2 실행 액션)

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

## Config / Launch 파라미터 불일치 (주의)

**실제 적용되는 값은 launch 파일**에서 직접 지정한 값이다. config/pick_place.yaml은 참조용으로만 존재하며, launch에서 override된다.

| 파라미터 | config/pick_place.yaml (구버전) | pick_place.launch.py (실제 적용) | perception.launch.py (구버전) |
|---------|-------------------------------|--------------------------------|------------------------------|
| model | yolov8n.pt | **yolov8x.pt** | yolov8n.pt |
| cam_x | 0.05 | **0.08** | 0.05 |
| cam_y | 0.03 | 0.03 | 0.0 |
| cam_z | -0.09 | **-0.01** | -0.09 |
| cam_pitch | 0.0 | **15.0** | 0.0 |
| pre_grasp_offset | 0.05 | **0.0** | - |

> **TODO**: config/pick_place.yaml과 perception.launch.py의 기본값을 최신 실측값으로 업데이트 필요

---

## 의존성 (Python)

| 패키지 | 용도 | 설치 |
|--------|------|------|
| pymoveit2 | MoveIt2 Python API | `apt: ros-jazzy-pymoveit2` |
| ultralytics | YOLOv8 | `pip install ultralytics` |
| pyrealsense2 | Intel RealSense SDK | `pip install pyrealsense2` |
| opencv-python | 시각화/이미지 처리 | `pip install opencv-python` |
| numpy | 수학/행렬 연산 | `pip install numpy` |

---

## 알려진 이슈

| 이슈 | 상태 | 설명 |
|------|------|------|
| IK 타임아웃 | 미해결 | z < 0 위치에서 KDL IK가 5초 타임아웃 발생. 도달 가능 여부 확인 필요 |
| 5cm standoff 미제거 | 미해결 | go/pick 명령에 아직 5cm standoff 접근이 남아있음 |
| 좌표 불안정 | 부분해결 | YOLO bbox 흔들림으로 좌표 변동. yolov8x로 개선되었으나 완전하지 않음 |
| `--once` 미수신 | 우회 | `ros2 topic pub --once`가 간헐적으로 미수신. `--times 3 --rate 2` 사용 |
| 그리퍼 닫기 미구현 | 미구현 | pick 시퀀스에서 접근 후 gripper close가 아직 없음 |
| pymoveit2 motion_suceeded 오타 | 참고 | pymoveit2 내부 변수명이 `motion_suceeded` (오타). 코드에서 그대로 사용 |
| config/launch 파라미터 불일치 | 미해결 | pick_place.yaml과 perception.launch.py 기본값이 구버전 상태 |
| detect_and_locate 코드 기본값 불일치 | 미해결 | 코드 내 declare_parameter 기본값(cam_x=0.05 등)이 launch 값과 다름 (launch에서 override되므로 동작에 영향 없음) |

---

## 참고: pymoveit2 Async 패턴

pymoveit2 v4.2.0의 sync 메서드는 `rclpy.spin_once()`를 내부 호출하여
MultiThreadedExecutor와 함께 사용할 수 없다. 아래 패턴으로 우회한다:

```python
# IK 풀기
future = moveit2.compute_ik_async(position=[x, y, z], quat_xyzw=quat)
while not future.done():
    time.sleep(0.01)
joint_state = moveit2.get_compute_ik_result(future)

# 플래닝
future = moveit2.plan_async(joint_positions=joint_positions)
while not future.done():
    time.sleep(0.01)
trajectory = moveit2.get_trajectory(future)

# 실행 (async send - spin_once 미호출)
moveit2.execute(trajectory)
while moveit2._MoveIt2__is_motion_requested or moveit2._MoveIt2__is_executing:
    time.sleep(0.05)
success = moveit2.motion_suceeded  # 오타 주의
```

**주의사항**:
- `use_move_group_action=False` 필수 (서비스 인터페이스 사용)
- `moveit2._MoveIt2__is_executing`은 private 변수 접근 (name mangling)
- ReentrantCallbackGroup + MultiThreadedExecutor 조합 필수

---

## 최근 테스트 로그 요약 (2026-03-25)

**테스트**: `pick` 명령으로 sports ball 집기 시도

```
Object at (0.266, 0.030, -0.024)    ← 감지된 물체 좌표 (z가 음수 = 베이스 아래)

Command: "pick"
=== Pick Start ===
Step 1/3: Standoff (0.216, 0.027, -0.014)     ← 법선 방향 5cm 뒤
  → IK solved: [-0.147, -0.005, 1.584, -0.883, -0.073] (wrist_roll=-4°)
  → Plan found, executing...
  → Arrived.                                    ← standoff 성공

Step 2/3: Open gripper 80% (1.17)             ← 그리퍼 열기 성공

Step 3/3: Approach (0.265, 0.030, -0.023)     ← 실제 목표 위치
  → IK timeout!                                ← IK 5초 타임아웃 실패
  → Approach failed.
=== Pick Complete ===
```

**분석**:
- Standoff 위치(z=-0.014)는 IK 성공, 목표 위치(z=-0.023)는 IK 실패
- z가 더 낮아지면서 도달 범위를 벗어나는 것으로 추정
- 하지만 사용자는 "충분히 갈 수 있는 위치"라고 판단
- **가능한 원인**: 카메라 z좌표 변환 문제, KDL solver 제한, 또는 standoff에서 이동한 후의 joint 상태가 다음 IK에 영향

---

## 개발 히스토리 요약

1. **워크스페이스 구성**: openarm_moveit에서 SO-101용 새 워크스페이스 생성
2. **detect_and_locate 포팅**: openarm 버전에서 SO-101용으로 수정
3. **pick_place_node 구현**: pymoveit2 sync → async 전환 (핵심 아키텍처 변경)
4. **카메라 캘리브레이션**: 부호 반전, 실측값 적용, pitch 추가
5. **법선벡터 + 그립각도**: SVD 평면 피팅 + 2D PCA 구현
6. **그립각도 분리**: 쿼터니언 통합 → wrist_roll 직접 매핑으로 변경
7. **시각화 개선**: 법선/그립 화살표, 좌표 오버레이
8. **YOLO 모델 업그레이드**: yolov8n → yolov8x (정확도 대폭 향상)
9. **IK 안정성**: 타임아웃 추가, busy 가드 구현
10. **현재**: IK 실패 디버깅 + standoff 제거 작업 중
