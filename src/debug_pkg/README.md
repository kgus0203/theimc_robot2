# 검사 업로드 단독 테스트

로봇, 카메라, ROS action 서버 없이 합성 RGB JPEG, 16-bit Depth PNG,
카메라 정보 JSON 및 테스트 환경값을 실제 검사와 같은 multipart 형식으로 한 번 전송합니다.
실행하면 서버에 테스트 데이터가 실제 저장될 수 있습니다. ID는 매번 고유한 `TEST_...`입니다.
로컬 재전송 큐는 사용하지 않습니다.

```bash
cd ~/theimc_robot
colcon build --symlink-install --packages-select plant_inspection_pkg debug_pkg
source install/setup.bash
ros2 run debug_pkg test_inspection_upload
```

기본 URL은 `http://192.168.0.72:8080/api/v1/inspections`이며 다음처럼 바꿀 수 있습니다.
검사 노드의 YAML 설정과는 별개입니다.

```bash
ros2 run debug_pkg test_inspection_upload \
  --upload-url http://192.168.0.72:8080/api/v1/inspections \
  --region sj --device-number 1 --bed-id 1 --hole-id 1 --timeout 10
```

`UPLOAD ACCEPTED`는 HTTP 성공 응답을 뜻하며 서버 응답 본문을 함께 출력합니다.
DB/파일의 실제 저장 여부는 서버 응답 또는 서버 기록에서 확인하세요.
실패하면 `UPLOAD FAILED`와 HTTP 오류 본문 또는 연결 오류를 출력하고 종료 코드 1을 반환합니다.
성공 종료 코드는 0입니다.
