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
                    # rail_start 검출이 사라지면
                    # 0.5초 안에 has_rail=false
                    'hold_sec': 0.5,

                    'ema_alpha_bbox': 0.35,
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

                    # 로봇 속도 명령
                    'cmd_vel_topic': '/cmd_vel',

                    # 제어 주기
                    'control_rate_hz': 20.0,

                    # ALIGN 제자리 회전 속도
                    'angular_speed': 0.08,

                    # 짧은 전진
                    'pulse_linear_speed': 0.08,
                    'pulse_forward_sec': 0.2,

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