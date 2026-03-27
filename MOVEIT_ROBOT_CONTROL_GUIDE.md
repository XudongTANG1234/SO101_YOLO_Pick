# SO-101 MoveIt Coordinate Control Guide

## 기준 런치

이 문서는 아래 런치 파일을 기준으로 설명한다.

- [demo.launch.py](/home/sst/so101_ws/src/so101_moveit_config/launch/demo.launch.py)

실행은 이렇게 한다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/sst/so101_ws/install/setup.bash
ros2 launch so101_moveit_config demo.launch.py
```

하지만 중요한 구분이 있다.

- [demo.launch.py](/home/sst/so101_ws/src/so101_moveit_config/launch/demo.launch.py)
  - MoveIt 환경을 띄우는 런치
  - pose 메시지를 받아 바로 이동시키는 노드는 없음
- [pick_place.launch.py](/home/sst/so101_ws/src/so101_pick_place/launch/pick_place.launch.py)
  - MoveIt 환경 + 사용자 제어 노드까지 같이 띄우는 런치
  - 현재 워크스페이스에서 좌표 publish로 실제 이동까지 연결하려면 이쪽이 필요함

즉 역할을 나누면 이렇게 보면 된다.

- `demo.launch.py`
  - MoveIt 서버/환경 실행
- `pick_place.launch.py`
  - MoveIt 서버/환경 실행 + pose 입력을 실제 이동으로 연결

## `demo.launch.py`가 실제로 띄우는 것

[demo.launch.py](/home/sst/so101_ws/src/so101_moveit_config/launch/demo.launch.py) 는 직접 노드를 많이 띄우지 않고, 아래 런치를 그대로 포함한다.

- [follower_moveit_demo.launch.py](/home/sst/so101_ws/src/so101_bringup/launch/follower_moveit_demo.launch.py)

즉 `demo.launch.py` 실행 시 실제로 같이 올라오는 건:

- `ros2_control` bringup
- `robot_state_publisher`
- controller spawner
- `move_group`
- MoveIt RViz

## `ros2_control`이 없어 보였는데 왜 움직였는가

결론부터 말하면, `ros2_control`은 없던 게 아니라 **이미 같이 실행되고 있었다.**

이유:

- [demo.launch.py](/home/sst/so101_ws/src/so101_moveit_config/launch/demo.launch.py)
  - [follower_moveit_demo.launch.py](/home/sst/so101_ws/src/so101_bringup/launch/follower_moveit_demo.launch.py) 포함
- [follower_moveit_demo.launch.py](/home/sst/so101_ws/src/so101_bringup/launch/follower_moveit_demo.launch.py)
  - 다시 [follower_split.launch.py](/home/sst/so101_ws/src/so101_bringup/launch/follower_split.launch.py) 포함
- 그 bringup 안에서 실제 하드웨어, `controller_manager`, `arm_trajectory_controller`, `gripper_controller`가 올라간다

즉 MoveIt이 혼자 로봇을 움직인 게 아니라,

- MoveIt은 trajectory를 계획하고 보내고
- 실제 관절 실행은 `ros2_control` controller가 담당한 것이다

## 좌표 기준

MoveIt에서 목표 pose는 `base_link` 기준으로 주는 걸 기준으로 보면 된다.

즉:

- `x`: 로봇 앞쪽
- `y`: 로봇 좌우
- `z`: 로봇 높이

예를 들어:

```yaml
position:
  x: 0.28
  y: 0.03
  z: -0.06
```

이면 end-effector를 `base_link` 원점 기준 `(0.28, 0.03, -0.06)` 위치로 보내려는 뜻이다.

## MoveIt Named State

MoveIt SRDF에는 미리 정의된 named state가 있다.

기준 파일:

- [so101_arm.srdf](/home/sst/so101_ws/src/so101_moveit_config/config/so101_arm.srdf)

### manipulator 그룹

- `zero`
  - 모든 관절이 0에 가까운 기준 자세
- `pick`
  - wrist_flex가 올라간 기본 pick 자세
- `rest`
  - 접힌 보관 자세에 가까운 상태
- `extended`
  - 팔을 펴는 방향의 상태

### gripper 그룹

- `open`
  - gripper 열림
- `closed`
  - gripper 닫힘

즉 MoveIt 기준으로 arm은 아래 named state를 가진다.

- `zero`
- `pick`
- `rest`
- `extended`

그리고 gripper는 아래 named state를 가진다.

- `open`
- `closed`

이 값들은 SRDF에 정의된 고정 joint state다.

## 좌표를 주면 어떻게 움직이는가

MoveIt 기준으로는 개념이 단순하다.

1. 목표 end-effector pose를 정함
2. IK로 가능한 관절값을 찾음
3. planning 수행
4. trajectory execute
5. controller가 실제 모터를 움직임

즉 “좌표를 찍으면 어디로 가는가”에 대한 답은:

- 입력한 좌표를 `base_link` 기준 end-effector 목표 pose로 해석하고
- 그 pose에 도달하도록 MoveIt이 관절 경로를 계산하고
- controller가 그 경로를 실행한다

## 필요한 환경

```bash
source /opt/ros/jazzy/setup.bash
source /home/sst/so101_ws/install/setup.bash
```

새로 빌드했다면:

```bash
cd /home/sst/so101_ws
colcon build --packages-select so101_moveit_config so101_bringup so101_pick_place
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 실행 방법

MoveIt + 실제 제어 환경 실행:

```bash
ros2 launch so101_moveit_config demo.launch.py
```

이 상태가 되면:

- MoveIt RViz 사용 가능
- `move_group` 사용 가능
- arm / gripper controller 사용 가능

## 어떤 런치를 써야 하는가

### 1. MoveIt 환경만 필요할 때

사용:

- [demo.launch.py](/home/sst/so101_ws/src/so101_moveit_config/launch/demo.launch.py)

명령:

```bash
ros2 launch so101_moveit_config demo.launch.py
```

이 경우 가능한 것은:

- MoveIt 실행
- RViz 사용
- 코드에서 MoveIt API 직접 호출

이 경우 안 되는 것은:

- pose 메시지를 publish 했을 때 자동으로 이동하는 것

즉 `demo.launch.py`는 MoveIt 환경을 띄우는 용도다.

### 2. 좌표 publish로 실제 이동까지 하려면

사용:

- [pick_place.launch.py](/home/sst/so101_ws/src/so101_pick_place/launch/pick_place.launch.py)

명령:

```bash
ros2 launch so101_pick_place pick_place.launch.py
```

이 경우 가능한 것은:

- MoveIt 실행
- pose 입력 수신
- 내부 사용자 노드가 MoveIt 호출
- 실제 arm 이동

즉 현재 워크스페이스에서 **좌표를 넣고 로봇이 실제로 움직이게 하려면** `pick_place.launch.py` 쪽을 써야 한다.

## 중요한 구분

`demo.launch.py` 자체는 "좌표를 topic으로 publish하면 바로 움직이는 인터페이스"를 제공하지 않는다.

즉 MoveIt 기본 구성만으로는:

- 특정 좌표 topic subscribe
- 좌표 수신 즉시 이동

이 자동으로 생기지 않는다.

MoveIt은 기본적으로 아래 둘 중 하나로 쓴다.

1. RViz MotionPlanning 플러그인에서 goal pose를 직접 지정
2. 별도 사용자 코드에서 MoveIt API로 목표 pose를 넣음

즉 MoveIt 기본 인터페이스는 topic 기반 명령이 아니라, goal pose 또는 named state를 API로 넣는 방식이라고 보면 된다.

## MoveIt만 기준으로 보면 필요한 입력

MoveIt이 실제로 필요한 건 결국 end-effector 목표 pose다.

- 기준 프레임: `base_link`
- position: `(x, y, z)`
- orientation: quaternion `(x, y, z, w)`

예시 pose:

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

이 pose를 MoveIt에 넣으면, MoveIt은 end-effector를 그 위치와 자세로 보내려고 한다.

## 현재 워크스페이스에서 쓰는 pose 입력 형식

이 워크스페이스에서는 별도 사용자 노드가 MoveIt을 감싸고 있고, 그 노드에 들어가는 좌표 형식은 아래 `PoseStamped` 다.

- 토픽: `/detected_object_pose`
- 타입: `geometry_msgs/msg/PoseStamped`
- 기준 프레임: `base_link`

즉 좌표를 넣는다면 아래 형식으로 넣으면 된다.

```bash
ros2 topic pub --once /detected_object_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {
    position: {x: 0.28, y: 0.03, z: -0.06},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

여기서 의미는 다음과 같다.

- `position.x`
  - 로봇 기준 앞쪽 거리
- `position.y`
  - 로봇 기준 좌우 위치
- `position.z`
  - 로봇 기준 높이
- `orientation`
  - end-effector가 도달해야 하는 절대 자세

즉 현재 좌표 입력 자체는 `PoseStamped` 한 개로 표현된다.

## 권장 사용 방식

이 문서 기준에서는 RViz보다 **API / 코드 기반 제어**를 더 중요한 사용 방식으로 본다.

즉 `demo.launch.py`의 역할은:

- MoveIt 서버 실행
- `move_group` 실행
- controller 실행

까지이고,

실제 좌표 입력은 아래 방식으로 하는 것이 맞다.

1. MoveIt API 호출
2. 사용자 노드에서 pose target 설정
3. named state 호출

즉 실사용 기준으로는 `RViz 조작`보다 `코드로 목표 pose를 넣는 방식`이 더 중요하다.

## API / 코드 기반 제어

### 1. pose target으로 이동

핵심 개념은 이것이다.

- 기준 프레임: `base_link`
- 목표: end-effector pose
- 입력: `position + quaternion`

즉 코드에서는 결국 아래 값을 MoveIt에 넣으면 된다.

```python
position = [0.28, 0.03, -0.06]
quat_xyzw = [0.0, 0.0, 0.0, 1.0]
```

그러면 MoveIt은:

1. IK 계산
2. planning
3. execute

를 수행한다.

### 2. named state로 이동

MoveIt에서는 좌표 대신 named state를 바로 호출하는 것도 가능하다.

현재 SO-101 기준:

- manipulator
  - `zero`
  - `pick`
  - `rest`
  - `extended`
- gripper
  - `open`
  - `closed`

즉 코드에서는:

- arm을 `pick`으로 보내기
- gripper를 `open`으로 보내기

같은 식의 제어도 가능하다.

## 메시지 기반 vs API 기반

여기서 중요한 구분은 이거다.

- MoveIt 기본
  - topic subscriber 기반 좌표 명령 인터페이스 없음
- MoveIt 실제 사용
  - 코드에서 API 호출
  - 또는 별도 제어 노드가 topic을 받아 API 호출

즉 "메시지로 좌표를 바로 move_group에 publish" 하는 구조가 기본은 아니다.

메시지 기반으로 쓰고 싶으면 결국 내부적으로는 아래 둘 중 하나가 필요하다.

1. 사용자 노드가 topic을 subscribe
2. 그 노드가 MoveIt API를 대신 호출

즉 외부에서 메시지를 보내더라도, 실제 이동은 결국 코드 안에서 MoveIt API를 호출해서 이루어진다.

## 최소 코드 예시 개념

아래는 개념 예시다.

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

즉 핵심은:

- `demo.launch.py`로 MoveIt 환경 실행
- 별도 코드에서 MoveIt API 호출
- pose 또는 named state를 넣어 제어

이다.

## 메시지로 제어하려면

메시지 기반 제어를 원하면, 구조는 항상 아래처럼 된다.

1. 임의의 topic 정의
2. 사용자 노드가 그 topic subscribe
3. 받은 좌표를 MoveIt API 입력으로 변환
4. MoveIt planning/execution 실행

즉 메시지는 **직접 로봇을 움직이는 것**이 아니라,
**MoveIt API를 호출하는 사용자 코드에 입력을 주는 역할**을 한다.

## 가장 단순한 사용 방식

`demo.launch.py` 기준에서 가장 단순한 사용 방식은 RViz다.

1. MoveIt 실행

```bash
ros2 launch so101_moveit_config demo.launch.py
```

2. RViz MotionPlanning 패널에서 goal pose 지정

3. `Plan` 후 `Execute`

즉 `demo.launch.py`만 기준으로 설명하면, 좌표를 주는 기본 방법은 RViz에서 goal pose를 잡는 것이다.

## 주의사항

- 좌표는 `base_link` 기준으로 넣어야 한다
- quaternion도 같이 절대값으로 넣는다
- `demo.launch.py` 자체에는 좌표 topic subscriber가 없다
- topic 기반 좌표 이동은 별도 사용자 노드가 있어야 한다
- 실사용 기준으로는 RViz보다 API / 코드 기반 제어를 중심으로 보는 것이 맞다

## 한 줄 요약

[demo.launch.py](/home/sst/so101_ws/src/so101_moveit_config/launch/demo.launch.py) 를 실행하면 실제로는 `ros2_control + MoveIt`이 같이 올라간다.  
이 상태에서 MoveIt은 `base_link` 기준 end-effector 목표 pose를 받아 planning/execution 할 준비가 된 상태가 되며, 실사용에선 별도 코드나 API 호출로 목표 pose 또는 named state를 넣어 제어하는 것이 핵심이다.
