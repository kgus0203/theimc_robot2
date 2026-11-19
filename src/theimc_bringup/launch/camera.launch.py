'''
카메라랑 관련된 launch는 다 여기서
aruco, yolo 등등 ,,,
'''

from launch.actions import SetEnvironmentVariable
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # matplotlib / ultralytics 설정 파일 경로만 지정
        SetEnvironmentVariable(
            'MPLCONFIGDIR',
            '/tmp/matplotlib'
        ),

        SetEnvironmentVariable(
            'YOLO_CONFIG_DIR',
            '/tmp/Ultralytics'
        ),

        # ============================================================
        # Camera
        # ============================================================

        Node(
            package='camera_perception_pkg',
            executable='image_publisher_node',
            name='image_publisher_node',
            output='screen',
        ),

        # ============================================================
        # Rail 인식
        # ============================================================

        Node(
            package='camera_perception_pkg',
            executable='yolov26_node',
            name='yolov26_node',
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
                    # 레일 인식 결과
                    'rail_info_topic': '/rail_info',

                    # 레일 전용 mux 입력. /cmd_vel에는 twist_mux만 출력한다.
                    'cmd_vel_topic': '/cmd_vel_rail',

                    # 제어 주기
                    'control_rate_hz': 20.0,

                    # ALIGN 제자리 회전 속도
                    'turn_speed': 0.035,

                    # 연속 전진 속도
                    'forward_speed': 0.07,
                    'min_forward_speed': 0.03,
                    'min_rail_confidence': 0.45,

                    # rail_info timeout
                    'rail_timeout_sec': 0.5,

                    # rail_start bbox 근접 판단
                    'close_bbox_area_ratio': 0.18,

                    # 성공 알림
                    'success_topic': '/rail_approach_success',

                    # 레일 진입 명령
                    'rail_command_topic': '/rail_command',
                }
            ]
        ),

        # ============================================================
        # ArUco
        # ============================================================

        Node(
            package='camera_perception_pkg',
            executable='aruco_detector_node',
            name='aruco_detector_node',
            output='screen'
        ),

        Node(
            package='decision_making_pkg',
            executable='aruco_docker_node',
            name='aruco_docker_node',
            output='screen'
        ),
    ])
