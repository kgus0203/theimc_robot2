# keepout_test_pkg

Nav2 keepout filter의 RViz 표시 문제를 실제 로봇과 분리해 재현하는 패키지입니다.

## 안전 범위

- 로봇 드라이버, 센서 드라이버, localization, planner, controller를 실행하지 않습니다.
- `/cmd_vel` 또는 다른 구동 명령을 publish하는 노드가 없습니다.
- RViz에는 Goal/Initial Pose 도구를 넣지 않았습니다.
- 실행되는 노드는 map server 2개, filter info server, 독립형 global costmap,
  고정 TF, lifecycle manager, RViz뿐입니다.

## 실행

```bash
cd /home/jeff/theimc_robot
colcon build --packages-select keepout_test_pkg --symlink-install
source install/setup.bash
ros2 launch keepout_test_pkg keepout_rviz_test.launch.py
```

RViz에서 아래 세 레이어를 각각 켜고 끄면서 비교할 수 있습니다.

- `Map`: 원본 정적 지도(`/map`)
- `Global Costmap (Keepout Applied)`: keepout filter 적용 결과
  (`/costmap/costmap`; 독립 실행 바이너리의 기본 namespace)
- `Keepout Mask`: 필터 마스크(`/keepout_filter_mask`)

`config/navigation_keepout.yaml`은 Keepout Filter 뒤에 전용 Inflation Layer를
적용하여 금지구역 경계의 비용 팽창까지 확인합니다.

현재 `maps/keepout_map.yaml`은 손상된 PGM의 UTF-8 치환문자를 흰색
픽셀로 복원한 `keepout_map_recovered.pgm`을 읽습니다. 손상된
`keepout_map.pgm`과 정상 지도 복사본 `keepout_map_clean.pgm`은 비교
자료로 그대로 남겨 두었습니다. 복구 절차는
`tools/recover_keepout_pgm.py`로 재현할 수 있습니다.
