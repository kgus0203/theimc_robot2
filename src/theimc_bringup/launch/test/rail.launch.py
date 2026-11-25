'''
카메라랑 관련된 launch는 다 여기서 
aruco, yolo 등등 ,,,
'''
import os

from launch.actions import SetEnvironmentVariable
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nvidia_site_packages = '/home/jeff/.local/lib/python3.10/site-packages/nvidia'
    cuda_library_paths = [
        f'{nvidia_site_packages}/cuda_runtime/lib',
        f'{nvidia_site_packages}/cublas/lib',
        f'{nvidia_site_packages}/cusparse/lib',
        f'{nvidia_site_packages}/cusparselt/lib',
        f'{nvidia_site_packages}/cudnn/lib',
        f'{nvidia_site_packages}/cufft/lib',
        f'{nvidia_site_packages}/curand/lib',
        f'{nvidia_site_packages}/cusolver/lib',
        f'{nvidia_site_packages}/nccl/lib',
        f'{nvidia_site_packages}/nvtx/lib',
        f'{nvidia_site_packages}/cuda_cupti/lib',
        f'{nvidia_site_packages}/cuda_nvrtc/lib',
        f'{nvidia_site_packages}/nvjitlink/lib',
    ]
    ld_library_path = ':'.join(
        cuda_library_paths + [os.environ.get('LD_LIBRARY_PATH', '')]
    )

    return LaunchDescription([
        SetEnvironmentVariable('LD_LIBRARY_PATH', ld_library_path),
        SetEnvironmentVariable('MPLCONFIGDIR', '/tmp/matplotlib'),
        SetEnvironmentVariable('YOLO_CONFIG_DIR', '/tmp/Ultralytics'),
        Node(
            package='camera_perception_pkg',
            executable='image_publisher_node',
            name='image_publisher_node',
            output='screen',
        ),

        Node(
            package='camera_perception_pkg',
            executable='yolov8_node',
            name='yolov8_node',
            output='screen',
        ),
        Node(
            package='camera_perception_pkg',
            executable='rail_info_extractor_node',
            name='rail_info_extractor_node',
            output='screen',
            parameters=[
                {
                    # 기존 레일을 잠그고 옆 레일로 순간 전환하지 않는다.
                    'stale_sec': 0.40,
                    'ema_alpha': 0.25,
                    'max_target_jump_px': 100.0,
                }
            ],
        ),
        Node(
            package='decision_making_pkg',
            executable='rail_approach_action_server_node',
            name='rail_approach_action_server_node',
            output='screen',
            parameters=[
                {
                    # 레일 인식 결과를 받는 토픽
                    # rail_info_extractor_node 같은 노드가 이 토픽으로 RailInfo 메시지를 publish해야 함
                    'rail_info_topic': '/rail_info',

                    # 레일 전용 mux 입력. /cmd_vel에는 twist_mux만 출력한다.
                    'cmd_vel_topic': '/cmd_vel_rail',

                    # 제어 주기
                    'control_rate_hz': 20.0,

                    # ALIGN 상태에서 제자리 회전 속도
                    'turn_speed': 0.035,

                    # 정렬 완료 후 연속 전진 속도
                    'forward_speed': 0.07,
                    'min_forward_speed': 0.05,
                    'min_rail_confidence': 0.45,
                    # 최종 성공 tolerance보다 엄격하게 접근 제어
                    'control_tolerance_ratio': 0.65,
                    'min_control_x_tolerance': 0.06,
                    'min_control_angle_tolerance': 1.5,
                    'near_fail_hold_sec': 1.0,

                    # rail_info가 이 시간 이상 갱신되지 않으면 즉시 정지
                    'rail_timeout_sec': 0.5,

                    # rail_start bbox 면적이 화면에서 이 비율 이상이면 가까워졌다고 판단
                    'close_bbox_area_ratio': 0.18,

                    # 성공 알림 토픽
                    'success_topic': '/rail_approach_success',

                    # 레일 진입 명령 토픽
                    'rail_command_topic': '/rail_command',
                }
            ]
        ),
    ])
