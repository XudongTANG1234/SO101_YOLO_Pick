# SO-101 Repository Dependency Install

이 문서는 다른 PC에서 이 저장소를 실행하기 위해 필요한 의존성 설치 명령을 정리한 것이다.

기준 환경:

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3

## 1. ROS 패키지 설치

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

## 2. Python 패키지 설치

Ubuntu 24.04에서는 아래처럼 `--break-system-packages`를 붙인다.

```bash
python3 -m pip install -U pip --break-system-packages
python3 -m pip install --break-system-packages \
  numpy \
  opencv-python \
  matplotlib \
  ultralytics \
  pyrealsense2
```

## 3. `pymoveit2` 설치

먼저 아래 명령으로 설치를 시도한다.

```bash
python3 -m pip install --break-system-packages pymoveit2
```

만약 여기서 실패하면, 해당 PC 환경에 맞게 `pymoveit2`를 별도로 소스 설치해야 한다.

## 4. 워크스페이스 빌드

저장소를 받은 뒤:

```bash
cd /path/to/so101_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## 5. 실행 예시

MoveIt 환경 실행:

```bash
ros2 launch so101_moveit_config demo.launch.py
```

좌표 기반 사용자 노드까지 같이 실행하려면:

```bash
ros2 launch so101_pick_place pick_place.launch.py
```

## 주의사항

- `pyrealsense2`는 PC 환경에 따라 pip 설치가 바로 안 될 수 있다.
- `pymoveit2`도 환경에 따라 pip 대신 소스 설치가 필요할 수 있다.
- YOLO 모델 파일 (`.pt`) 은 git에 포함하지 않는 기준이면, 실행 PC에 따로 준비해야 한다.
