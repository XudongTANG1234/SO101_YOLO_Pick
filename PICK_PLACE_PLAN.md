# SO-101 Pick & Place 개선 계획

## 목표

최종 목표는 다음 3가지를 동시에 만족하는 것이다.

1. 물체의 3D 위치를 안정적이고 정확하게 추정한다.
2. grasp에 필요한 접근 방향과 그리퍼 회전각을 일관되게 계산한다.
3. perception 결과를 바탕으로 MoveIt과 그리퍼가 재현 가능한 pick 시퀀스를 수행한다.

이 문서는 구현 순서를 고정하고, 각 단계를 하나씩 검증하면서 같이 진행하기 위한 작업 기준 문서다.

---

## 작업 원칙

1. 문서보다 실제 코드와 런타임 동작을 우선한다.
2. 정확도에 직접 영향을 주는 요소부터 먼저 수정한다.
3. perception, transform, planning, execution을 분리해서 검증한다.
4. 한 번에 큰 리팩터링을 하지 않고 단계별로 측정 가능한 결과를 남긴다.
5. pick 동작은 "처음 확정된 target" 기준으로 일관되게 수행되도록 만든다.

---

## 현재 합의한 기준

### 좌표계 기준

현재 시스템의 최종 pick target은 `world`가 아니라 `base_link` 기준으로 정리한다.

이유:
- 지금 목표는 고정형 암 데모가 아니라, 향후 사족보행 로봇 위에 팔을 올린 상태까지 고려하는 것이다.
- 그 경우 grasp target은 전역 좌표계보다 로봇 본체 기준 좌표계로 표현되는 편이 더 자연스럽고 확장성이 좋다.
- 따라서 perception은 카메라 프레임에서 측정한 값을 최종적으로 `base_link` 기준 target으로 변환하는 구조를 목표로 한다.

정리:
- raw sensing frame: camera optical frame
- execution target frame: `base_link`
- `world`는 현재 필수 기준 좌표계로 두지 않는다

### Target 확정 시점

pick target은 "객체가 처음 검출된 시점" 기준이 아니라, 실제 동작 명령이 들어온 순간의 최신 perception 값을 기준으로 확정한다.

적용 대상 명령:
- `go`
- `pick`
- 이후 필요 시 `plan` 등도 동일 원칙 적용 가능

정리:
- perception 노드는 계속 최신 detection 결과를 갱신한다.
- 하지만 로봇이 실제로 사용하는 target은 동작 명령을 수신한 순간 snapshot으로 고정한다.
- 실행 중에는 perception 결과가 새로 들어와도 현재 동작의 target은 바뀌지 않는다.

이 기준은 아래와 같은 실행 흐름을 전제로 한다.

1. 물체 인식 결과를 계속 갱신한다.
2. 사용자가 `go` 또는 `pick` 명령을 보낸다.
3. 그 순간의 최신 `position`, `orientation`, `grip_angle`, `timestamp`를 snapshot으로 확정한다.
4. 이후 standoff, approach, gripper close는 그 snapshot만 사용한다.

---

## 전체 단계

### 1단계. 기준선 정리

목적:
- 현재 시스템에서 실제로 어떤 파라미터와 값이 사용되는지 하나로 정리한다.
- 문서, launch, yaml, 코드 기본값 불일치를 제거한다.
- 이후 정확도 튜닝의 기준선을 만든다.

주요 작업:
- 카메라 extrinsic 관련 파라미터 소스 단일화
- `launch`, `yaml`, 코드 default 값 불일치 제거
- topic / frame / controller 이름 재확인
- pick 시퀀스에서 어떤 pose를 기준으로 움직일지 명시

완료 기준:
- 실제 사용되는 파라미터 값이 한 군데서 관리된다.
- 런치마다 다른 카메라 위치/YOLO 기본값 혼선이 없다.
- 시스템 데이터 흐름을 한 문장으로 설명할 수 있다.

---

### 2단계. Pick 시퀀스 고정

목적:
- perception 값이 계속 바뀌더라도 pick 명령이 시작되면 고정된 target으로 끝까지 수행되게 한다.
- "맨 처음 전달된 값으로 움직인다"는 요구를 코드로 보장한다.

주요 작업:
- pick 시작 시 `pose`, `normal`, `grip_angle`, `timestamp` snapshot latch
- pick 실행 중 `latest_*` 갱신과 실행 target 분리
- `80% open -> standoff -> final approach -> close` 순서 명시
- 필요 시 retreat / home 동작 정의

완료 기준:
- pick 명령 도중 perception 값이 바뀌어도 목표가 흔들리지 않는다.
- 실행 시퀀스가 코드 상에서 명확한 단계로 나뉜다.

---

### 3단계. Camera to Base 변환 정리

목적:
- 3D 위치 오차의 가장 큰 원인인 카메라-로봇 변환을 신뢰 가능한 구조로 정리한다.

주요 작업:
- 현재 수동 `cam_x`, `cam_y`, `cam_z`, `cam_pitch` 구조 재검토
- 가능하면 TF 기반 고정 transform 구조로 이동
- calibration 가능한 구조로 정리
- perception 결과가 `base` 기준 좌표로 일관되게 나오는지 검증

완료 기준:
- camera frame에서 base frame으로의 변환 경로가 명확하다.
- extrinsic 값을 실측/보정 절차로 업데이트할 수 있다.

---

### 4단계. Perception 정확도 개선

목적:
- 단순 detection 중심점이 아니라 grasp 가능한 target pose를 안정적으로 만들 수 있게 한다.

주요 작업:
- 중심 ROI depth 계산 방식 재검토
- depth foreground / outlier 제거 개선
- surface normal 계산 안정화
- bbox 기반 grip angle 계산 안정화
- 필요 시 segmentation 또는 depth-based object mask 도입 검토

완료 기준:
- perception 노드가 물체 위치, 접근 방향, 그리퍼 회전 정보를 안정적으로 출력한다.
- debug overlay와 로그만 봐도 결과가 타당한지 판단할 수 있다.

---

### 5단계. Grasp Frame 중심 구조로 정리

목적:
- `normal`과 `grip_angle`을 따로 다루는 임시 구조를 넘어서, 최종 grasp pose 자체를 일관되게 표현한다.

주요 작업:
- 접근축, lateral 축, gripper axis를 이용한 grasp frame 정의
- rotation matrix -> quaternion 생성 구조 정리
- perception 출력 메시지 구조 재검토
- 최종적으로 planning이 사용할 grasp target 형식 고정

완료 기준:
- grasp target이 `position + orientation` 기준으로 일관되게 표현된다.
- wrist roll만 따로 덮어쓰는 임시 로직 의존도를 줄인다.

---

### 6단계. MoveIt Planning 개선

목적:
- 계산된 orientation이 실제 motion planning과 IK에서 의미 있게 반영되도록 만든다.

주요 작업:
- `position_only_ik` 유지 여부 검토
- full orientation 강제 vs 접근축 우선 전략 결정
- standoff와 final approach planning 분리
- 실패 시 fallback 전략 정의

완료 기준:
- MoveIt이 grasp orientation 또는 최소한 접근축을 실제로 반영한다.
- target까지 가는 과정이 단순 위치 이동보다 grasp 친화적으로 바뀐다.

---

### 7단계. 검증 및 튜닝 체계 추가

목적:
- 수정 후 좋아졌는지 아닌지를 감으로 판단하지 않도록 한다.

주요 작업:
- perception debug overlay 보강
- detected target / commanded target / actual motion 로그 정리
- 반복 테스트 기준 정의
- 오차 측정 포인트 정리

완료 기준:
- 각 단계 수정 후 비교 가능한 로그나 관찰 기준이 있다.
- 튜닝 방향을 객관적으로 결정할 수 있다.

---

## 추천 진행 순서

실제 작업은 아래 순서로 진행한다.

1. 기준선 정리
2. Pick 시퀀스 고정
3. Camera to Base 변환 정리
4. Perception 정확도 개선
5. Grasp Frame 중심 구조 정리
6. MoveIt Planning 개선
7. 검증 및 튜닝 체계 추가

---

## 현재 판단

현재 repo는 이미 다음 요소를 일부 갖고 있다.

- YOLO + RealSense 기반 검출
- depth 기반 3D 위치 계산
- surface normal 계산
- grip angle 계산
- MoveIt 연동

하지만 아직은 "동작하는 프로토타입"에 가깝고, 정확도 최우선 시스템으로 보려면 아래가 더 필요하다.

- 일관된 extrinsic 관리
- pick 시점 target 고정
- 더 안정적인 grasp orientation 표현
- orientation이 실제 planning에 반영되는 구조
- 반복 검증 가능한 테스트 흐름

---

## 작업 방식

이 문서를 기준으로 단계별로 하나씩 처리한다.

각 단계마다:

1. 목표를 다시 확인한다.
2. 관련 파일을 좁힌다.
3. 필요한 수정만 적용한다.
4. 확인 방법과 남은 리스크를 기록한다.
