# 방향별 식물 검사

`/start_inspection`의 `side`에 `left` 또는 `right`를 지정하면 Fairino의
`/scan_side`를 통해 해당 방향의 **모든 기록 자세**를 순회합니다. 각 자세에서
3초 정지한 뒤 RGB/Depth/카메라 정보를 촬영하고, 촬영 완료 응답을 받은 뒤
다음 자세로 이동합니다. 완료된 검사는 기존 HTTP 업로드/실패 시 로컬 큐 경로를 사용합니다.

`side`를 생략하면 기존 생육일수 기반 검사 경로를 유지합니다.
잘못된 방향과 `dry_run: true`에서의 방향별 검사는 거절합니다.
직접 `/scan_side`에 `{side: left}`만 보내면 기존처럼 이동만 수행합니다.

## 빌드

액션 정의가 변경되어 세 패키지를 함께 빌드해야 합니다.
기존 inspection/scan 노드를 종료한 뒤 실행하고, 관련 터미널 모두 새 환경을 source하세요.
서버를 다른 PC에서 실행한다면 그 PC의 fairino_waypoint_test도 동일 버전으로 빌드해야 합니다.

```bash
cd ~/theimc_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select interfaces_pkg fairino_waypoint_test plant_inspection_pkg
source install/setup.bash
```

## 실행

기존 MoveIt/로봇 bringup이 실행 중인 상태에서, 별도 터미널에 스캔 서버를 실행합니다.
이미 `/scan_side` 서버를 실행 중이라면 중복 실행하지 않습니다.

```bash
ros2 launch fairino_waypoint_test waypoint_test.launch.py start_demo:=false use_rviz:=false
```

다른 터미널에서 검사 노드를 실행합니다. `inspection_mvp.launch.py`는 기존처럼
검사 노드만 실행하며 로봇 bringup을 자동 시작하지 않습니다.

```bash
ros2 launch plant_inspection_pkg inspection_mvp.launch.py
```

왼쪽 검사:

```bash
ros2 action send_goal /start_inspection interfaces_pkg/action/StartInspection \
  "{region: 'sj', device_number: 1, bed_id: 1, hole_id: 1, side: 'left'}" --feedback
```

오른쪽은 `side: 'right'`로 요청합니다. `/scan_side` 명령을 별도로 보낼 필요가 없습니다.

## 동작과 제한

- 촬영 서비스는 검사 ID, 방향, 자세 순서를 검증합니다. 중복/다른 검사 촬영은 거절합니다.
- 촬영 실패 또는 스캔 서버의 촬영 응답 대기 30초 초과 시 다음 자세로 이동하지 않고 실패합니다.
- 스캔 중 기존 inspection의 팔 명령 잠금이 유지됩니다. `/start_inspection`의 취소 지원은 기존과 동일하게 제공하지 않습니다.
- 저장되는 `viewpoint_id`에는 방향과 기록 자세 이름이 들어갑니다.
- 관절/손목 자세는 Fairino의 현재 상태에서 가져옵니다. `camera_pose`는 손목 자세 근사값이며 실제 카메라 외부 보정은 적용하지 않습니다.
- 스캔 실패 시 부분 촬영 결과는 업로드하지 않습니다.

## 검증

세 패키지 빌드 후 로봇과 분리된 ROS 도메인에서 액션/서비스 연동을 검사합니다.
카메라와 이동 서버는 테스트 대역을 사용합니다.

```bash
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 python3 -m pytest -q \
  src/plant_inspection_pkg/test/test_side_scan.py
```
