# Fairino 방향별 Action 테스트

이 폴더 안에서만 수정하는 독립 패키지입니다. 실행 후에는 움직이지 않고 Action 요청을 기다립니다.

## 빌드 및 실행

기존 demo와 테스트 launch를 종료한 뒤:

```bash
cd ~/robot_arm
source install/setup.bash
colcon build --symlink-install --packages-select fairino_waypoint_test
source install/setup.bash
ros2 launch fairino_waypoint_test waypoint_test.launch.py start_demo:=true use_rviz:=true
```

`READY: /scan_side action waiting for side=right or left`가 나오면 다른 터미널에서:

```bash
source ~/robot_arm/install/setup.bash
ros2 action send_goal /scan_side fairino_waypoint_test/action/ScanSide "{side: right}" --feedback
```

왼쪽 요청:

```bash
ros2 action send_goal /scan_side fairino_waypoint_test/action/ScanSide "{side: left}" --feedback
```

한 요청이 끝나면 다음 요청을 보낼 수 있습니다. `right`, `left` 이외의 값과
동작 중 추가 요청은 거절합니다. ROS Action 취소는 현재 MoveIt 계획/이동이 끝난 다음
처리하며 이후 이동은 실행하지 않습니다. 즉시 정지용 명령은 아닙니다.

## Action 정의

`action/ScanSide.action`:

- Goal: `string side` (`right` 또는 `left`), `string capture_service`, `string inspection_id`
  (`capture_service`와 `inspection_id`는 plant_inspection_pkg 연동 시 사용하며 단독 이동에서는 생략)
- Result: `bool success`, `string message`, `uint32 completed_waypoints`
- Feedback: `string side`, `string stage`, `uint32 completed_waypoints`, `uint32 total_waypoints`

`stage`의 `reached: 01 left upper right` 등은 해당 기록 자세에 도착했다는 뜻입니다.
양쪽 모두 각 자세 도착 뒤 3초 정지합니다. `capture_service`가 지정된 요청은 정지 후 CaptureWaypoint 서비스를 호출하고 성공 응답을 받은 뒤 다음 자세로 이동합니다. 촬영 실패 또는 30초 응답 시간 초과 시 스캔을 중단합니다. 기본값은 빈 문자열이며 기존 단독 명령은 이동만 수행합니다.

## 방향별 기록 자세 실행

`side: right` 또는 `side: left`가 시작 신호입니다. `src/recorded_poses.hpp`에서
오른쪽은 `poses`, 왼쪽은 `left_poses`의 기록 자세를
입력 순서대로 각각 한 번 방문하고 **매 도착마다 3초 정지**합니다. 마지막 자세에서도
3초 정지한 뒤 성공을 반환하고 다음 요청을 기다립니다. 별도 ready/zero 이동은 없습니다.

원본 값은 라디안이며 `/joint_states.name` 순서 `[j1, j2, j4, j5, j3, j6]` 그대로
저장하고, 관절 이름으로 MoveIt 목표를 지정합니다. `position`의 인덱스를 j1~j6로
가정하지 않습니다. 양쪽 모두 01~09 번호로 구분합니다.

OMPL/RRTConnect로 각 구간을 계획하고 성공한 경로만 실행합니다. 계획 실패 시
동일 목표로 최대 3번 재계획합니다. 특정 자세의 계획이나
실행이 실패하면 해당 번호와 사유를 Action ABORTED로 반환합니다. 그 자세를 건너뛰거나
다른 목표로 바꾸지 않습니다. 완료 결과의 `completed_waypoints`는 도착한 자세 수입니다.

- `/display_planned_path`: 현재 구간 전체 로봇의 계획 경로 애니메이션.
- `/waypoint_test/planned_path`: 각 계획 구간에서 계산한 wrist3_link 이동선.
  한 요청의 경로를 누적 표시하며 다음 요청 시작 시 지웁니다. 실측 경로는 아닙니다.
- `/waypoint_test/markers`: 방향과 등록된 목표 위치의 번호·이름.
- `reached: 01 ...`, `holding 3 seconds: 01 ...` feedback으로 도착·정지를 구분합니다.

카메라 촬영 호출이나 촬영 완료 확인은 아직 없습니다. 정지 구간은 고정 3초입니다.
취소 요청은 현재 MoveIt 동작이 끝난 뒤 처리하며 정지 중에는 약 50ms 주기로 확인합니다.

## 실행 옵션

- `velocity_scale:=0.1`, `acceleration_scale:=0.1`: 속도·가속도 배율.
- `connection_timeout:=30.0`: MoveIt 연결 제한 시간(초).
- `use_rviz:=false`: RViz 창 없이 실행. 경로 토픽과 충돌 검사는 유지됩니다.

왼쪽 랜덤 동작은 기록 자세 실행으로 교체했습니다. 이전 랜덤 동작용
`cycles`, `seed`, `spread_deg`, `move_zero_first` 옵션은 제거했습니다.
양쪽 모두 선택한 방향의 자세를 한 번씩 방문하며 ready/zero로 추가 이동하지 않습니다.

## 이미 실행 중인 demo / 실물 로봇에 연결

MoveIt와 컨트롤러가 이미 실행 중이라면 중복 실행하지 않고:

```bash
ros2 launch fairino_waypoint_test waypoint_test.launch.py start_demo:=false use_rviz:=false
```

`start_demo` 기본값은 false입니다. 기존 demo는 mock_components/GenericSystem을
사용하므로 `use_rviz:=false`만으로 실물 제어로 전환되지 않습니다. 실물용 MoveIt와
컨트롤러를 먼저 실행해야 합니다. 주변 장애물은 PlanningScene에 반영되어야 합니다.
MoveIt 연결 실패 시 두 터미널의 ROS_DOMAIN_ID와 discovery 설정을 확인하세요.

기존 RViz에서 방향 표시를 보려면 Add → MarkerArray에서 `/waypoint_test/markers`를
선택하고 Durability Policy를 Transient Local로 설정하세요.

## 환경 장애물

서버 시작 시 `/apply_planning_scene`으로 아래 세 물체를 등록합니다.
등록에 성공해야 `/scan_side` 요청을 받을 수 있습니다. RViz의 MotionPlanning →
Scene Geometry에 표시되며 `use_rviz:=false`에서도 MoveIt 충돌 검사에 포함됩니다.

- 오른쪽 벽: zero 자세 shoulder 기준 로봇 쪽 면 x=-0.500m.
- 왼쪽 벽: 같은 기준 로봇 쪽 면 x=+0.500m.
- 벽 기본 크기: x 두께 0.020m × y 폭 1.000m × z 높이 1.000m.
  y 중앙은 0, 아래쪽은 z=0이며 두께는 로봇 바깥쪽으로 늘어납니다.
  폭·높이는 요청에 치수가 없어 임시로 정한 값입니다.
- base 박스: base_link 기준 중심 (0, 0, -0.141)m, 크기 0.440m 정육면체.
  윗면은 z=0.079m(base 상단), 아랫면은 z=-0.361m입니다.

벽 크기와 거리는 미터 단위 launch 옵션으로 변경할 수 있습니다:

```bash
ros2 launch fairino_waypoint_test waypoint_test.launch.py start_demo:=true \
  wall_distance:=0.5 wall_thickness:=0.02 wall_width:=1.0 wall_height:=1.0
```

고정 ID `scan_right_wall`, `scan_left_wall`, `scan_base_box`를 사용해 재실행 시
동일 물체를 갱신합니다. 다른 장애물은 변경하지 않습니다. base를 덮는 설치 구조이므로 박스와
base_link·shoulder_link 사이의 충돌만 허용합니다. CAD의 shoulder 하단은 74mm부터
시작해 base 상단과 5mm 겹칩니다. 이 두 링크 전체에 대한 박스 충돌 허용이며,
upperarm 이후 링크와 박스, 모든 링크와 벽의 충돌 검사는 유지합니다.
물체는 테스트 노드 종료 후에도 실행 중인 MoveIt PlanningScene에 남습니다.
충돌 없는 경로를 찾지 못하면 해당 자세 번호와 함께 Action 실패를 반환합니다.

기존 mock의 모든 관절 0도 초기 자세는 벽과 충돌합니다. 따라서 `start_demo:=true`는
이 패키지의 `obstacle_demo.launch.py`와 `config/initial_positions.yaml`을 사용해
mock 초기 자세를 zero `[0, -90, 0, -90, 0, 0]`로 설정합니다. 이것은 초기화 설정이며
자동 이동 명령이 아닙니다. 기존 demo에 연결하거나 실물을 사용할 때는 현재 자세가
장애물과 충돌하지 않아야 계획할 수 있습니다.

## 자세 추가

`recorded_poses.hpp`에서 오른쪽은 `poses`, 왼쪽은 `left_poses`를 수정합니다.
각각 현재 9개이며 `std::array<Pose, 9>`를 사용하므로 항목 추가 시 배열 크기도
맞춰야 합니다. Action의 총 개수는 각 배열 크기에서 자동으로 계산합니다.
주석 처리한 항목은 실행에서 제외합니다.

C++ 헤더이므로 저장 후 다음 명령으로 빌드하고 launch를 재시작하세요.

```bash
cd ~/robot_arm
colcon build --symlink-install --packages-select fairino_waypoint_test
source install/setup.bash
```

이미 실행 중인 노드는 재빌드만으로 갱신되지 않습니다. 재시작 후 해당 방향의
Action을 보내면 최신 목표 마커가 발행됩니다.

## 검증

2026-09-10: ROS 2 Humble symlink 빌드와 RViz 실행 조건 4가지 조합을 검증했습니다.
분리된 ROS_DOMAIN_ID=193 mock 환경에서 왼쪽 9개 후 오른쪽 9개를 실제 실행해
양쪽 각각의 입력 순서, 도착 횟수, 9번의 3초 정지, 누적 9개 경로 마커, 성공 결과,
중복/잘못된 요청 거절 및 취소 결과를 확인했습니다. RViz GUI 화면 자체는 직접
확인하지 않았습니다. `test/check_scan_action.py`는 격리된 mock 환경에서만 실행하세요.

## 팔 상태 토픽

Fairino5 MoveIt launch와 waypoint launch는 `/joint_states`를 `/arm/joint_states`로
리매핑합니다. demo의 동적 관절 상태는 `/arm/dynamic_joint_states`로 발행합니다.
기존 팔 bringup도 새 launch로 재시작해야 발행자와 구독자가 일치합니다.
`/scan_side`, `/start_inspection` 액션과 명령 토픽 이름은 그대로입니다.
별도 실행하는 외부 구독 노드는 `--ros-args -r /joint_states:=/arm/joint_states`를
사용하거나 구독 설정을 변경해야 합니다.

## 스캔 완료 후 대기 자세

좌/우 스캔의 모든 이동 및 촬영이 정상 완료되면 마지막으로 대기 자세를
MoveIt 충돌 검사·경로 계획을 거쳐 실행합니다. 복귀까지 성공해야 액션이 성공합니다.
대기 자세는 촬영하지 않으며 completed_waypoints/total_waypoints에 포함하지 않습니다.
피드백은 `returning to standby`, `planning: standby`, `moving: standby`,
`returned to standby` 순서입니다.

`recorded_poses.hpp`의 `standby`는 입력된 마지막 관절 샘플을 이름 기준으로 저장합니다.
`j1,j2,j4,j5,j3,j6` 순서이며 단위는 rad입니다. velocity/effort의 NaN은 사용하지 않습니다.
복귀 계획/실행 실패는 액션 실패로 보고합니다. 스캔 실패·취소·종료 요청 시에는
기존 중단 동작을 유지하여 추가 복귀 이동을 시작하지 않습니다.
이 동작은 `/scan_side`와 이를 호출하는 방향별 `/start_inspection`에 적용됩니다.

## 실제 로봇에서 실행 성공만 뜨고 움직이지 않는 경우

`ServoJ ... code:14`는 SDK의 명령 실행 실패입니다. 기존 v3.9.5 드라이버는
ServoMoveStart를 호출하지 않고 ServoJ를 전송했고, 전송 실패도 OK로 반환했습니다.
현재 Fairino5 설정은 수정한 `fairino_hardware_v3_9_5/FairinoHardwareInterface`를
명시적으로 선택합니다. 다른 버전의 동명 플러그인과 혼용하지 않습니다.

- 활성화: 실측 관절 읽기 → 명령/상태 동기화 → ServoMoveStart 성공 확인
- 구동: 100 Hz, 명령·실측 값 유효성 및 SDK 반환값 검사
- 실패: 하드웨어 ERROR 반환 및 정지. 자동 알람 해제/재시작은 하지 않음
- 완료: 컨트롤러 추종 오차 0.20 rad, 목표 오차 0.02 rad 검사
- 스캔: 새 `/arm/joint_states`에서 목표 도달을 확인한 후에만 촬영

로봇 웹 화면/티치펜던트에서 정상 통신, Enable, 자동 모드, 알람 해제 상태를
확인해야 합니다. 드라이버는 이 상태를 강제로 바꾸지 않습니다.

모든 팔/검사 launch를 종료한 뒤 빌드하고, 각 터미널에서 setup을 다시 source합니다.

```bash
cd ~/theimc_robot
colcon build --symlink-install --packages-select \
  fairino_hardware_v3_9_5 fairino5_v6_moveit2_config fairino_waypoint_test plant_inspection_pkg
source install/setup.bash
```

기존 네 터미널 흐름을 그대로 사용합니다.

```bash
# 1: 실제 하드웨어 + MoveIt. Hardware ready ... ServoMoveStart succeeded 확인
ros2 launch fairino5_v6_moveit2_config demo.launch.py use_rviz:=false
# 2: 별도 터미널. 초기 동작 확인에는 기본 0.1 속도/가속도를 사용
ros2 launch fairino_waypoint_test waypoint_test.launch.py start_demo:=false use_rviz:=false
# 3: 별도 터미널
ros2 launch plant_inspection_pkg inspection_mvp.launch.py
# 4: 별도 터미널
ros2 action send_goal /start_inspection interfaces_pkg/action/StartInspection \
  "{region: 'sj', device_number: 1, bed_id: 1, hole_id: 1, side: 'left'}" --feedback
```

서보 시작/전송이 계속 거절되면 첫 SDK 오류 코드와 로봇 알람을 확인합니다.
TF의 AGV/arm 트리 연결은 별도 장착 변환 문제이며, 임의의 identity 변환으로 연결하지 않습니다.
