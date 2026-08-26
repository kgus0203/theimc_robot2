#!/usr/bin/env python3
'''
주석 없애지 말기!~!

STM 출력
WHEEL,0.000,0.000 
IMU,3.10,-0.24,2.26

여기서 WHEEL + IMU (카메라 추가하면 nav2 위치추정이 잘 안돼서 뺌)

여기 포함되는 거
- robot agent(웹 연결용)
- STM 통신 및 데이터 처리
- 라이다
'''

import os
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='Use simulation clock if true',
    ),
    DeclareLaunchArgument(
        'enable_d435i_fusion',
        default_value='true',
        choices=['true', 'false'],
        description='Launch the D435 camera',
    ),
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_d435_fusion = LaunchConfiguration('enable_d435i_fusion')

    pkg_share_bringup = get_package_share_directory('theimc_bringup')
    pkg_share_description = get_package_share_directory('theimc_description')

    params_ydlidar = PathJoinSubstitution(
        [pkg_share_bringup, 'params', 'theimc_ydlidar.yaml']
    )
    params_scan_filter = PathJoinSubstitution(
        [pkg_share_bringup, 'params', 'vehicle_scan_filter.yaml']
    )
    params_ekf = PathJoinSubstitution(
        [pkg_share_bringup, 'params', 'ekf.yaml']
    )
    params_twist_mux = PathJoinSubstitution(
        [pkg_share_bringup, 'params', 'twist_mux_params.yaml']
    )

    urdf_file = os.path.join(
        pkg_share_description,
        'urdf',
        'theimc.urdf',
    )
    # URDF에서 base_link collision box 크기 읽기
    urdf_root = ET.parse(urdf_file).getroot()

    base_link = urdf_root.find("./link[@name='base_link']")
    if base_link is None:
        raise RuntimeError("base_link was not found in URDF")

    collision_box = base_link.find("./collision/geometry/box")
    if collision_box is None:
        raise RuntimeError("base_link collision box was not found in URDF")

    body_size = [
        float(value)
        for value in collision_box.attrib["size"].split()
    ]

    body_length = body_size[0]
    body_width = body_size[1]

    half_length = body_length / 2.0
    half_width = body_width / 2.0

    vehicle_polygon = (
        f"[[-{half_length}, -{half_width}], "
        f"[-{half_length}, {half_width}], "
        f"[{half_length}, {half_width}], "
        f"[{half_length}, -{half_width}]]"
    )
    with open(urdf_file, 'r', encoding='utf-8') as urdf_stream:
        robot_description_content = urdf_stream.read()

    motor_drive_cmd = Node(
        package='theimc_bringup',
        executable='bringup_node',
        name='bringup_node',
        output='screen',
    )

    # LiDAR raw scan: /scan_raw
    ydlidar_cmd = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_ydlidar],
        remappings=[
            ('scan', '/scan_raw'),
        ],
    )

    # Filtered scan: /scan
    scan_filter_cmd = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_filter_chain',
        output='screen',
        emulate_tty=True,
        parameters=[
            params_scan_filter,
            {
                'filter1.params.polygon': vehicle_polygon,
            },
        ],
        remappings=[
            ('scan', '/scan_raw'),
            ('scan_filtered', '/scan'),
        ],
    )

    twist_mux_cmd = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[params_twist_mux],
        remappings=[
            ('cmd_vel_out', '/cmd_vel'), 
        ]
    )

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description_content},
        ],
    )

    robot_agent_cmd = Node(
        package='theimc_bringup',
        executable='robot_agent_node',
        name='robot_agent',
        output='screen',
        emulate_tty=True,
    )

    # Intel RealSense D435 driver.
    # Existing URDF publishes base_link -> camera_link.
    # The RealSense driver publishes camera_link -> internal sensor frames.
    realsense_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py',
            )
        ),
        condition=IfCondition(enable_d435_fusion),
        launch_arguments={
            'camera_namespace': '',
            'camera_name': 'camera',
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'unite_imu_method': '0',
            'enable_sync': 'true',
            'depth_module.depth_profile': '848x480x30',
            'rgb_camera.color_profile': '848x480x30',
            # 'depth_module.visual_preset': '3',  # 현재 드라이버에서 미지원
            # 'depth_module.emitter_enabled': '1',
            # 'depth_module.enable_auto_exposure': 'true',   
            # 'depth_module.laser_power': '360',
            'disparity_filter.enable': 'false',
            'spatial_filter.enable': 'false',
            # 'spatial_filter.filter_magnitude': '1',
            # 'spatial_filter.filter_smooth_alpha': '0.25',
            # 'spatial_filter.filter_smooth_delta': '3',
            'decimation_filter.enable': 'false',
            'decimation_filter.filter_magnitude': '2',
            'align_depth.enable': 'true',
            'hole_filling_filter.enable': 'false',
            'clip_distance': '4.0',
            'temporal_filter.enable': 'false',
            # 'temporal_filter.filter_smooth_alpha': '0.6',  # 현재 드라이버에서 미지원
            # 'temporal_filter.filter_smooth_delta': '15',  # 현재 드라이버에서 미지원
            'publish_tf': 'true',
            'pointcloud.enable': 'true',
            'initial_reset': 'true',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # RTAB RGB-D visual odometry (currently disabled).
    # It publishes only /camera/odom. It must not publish odom TF because
    # robot_localization is the single odom -> base_footprint TF publisher.
    # rgbd_odometry_cmd = Node(
    #     package='rtabmap_odom',
    #     executable='rgbd_odometry',
    #     name='d435_rgbd_odometry',
    #     output='screen',
    #     arguments=['--ros-args', '--log-level', 'warn'],
    #     condition=IfCondition(enable_d435_fusion),
    #     parameters=[
    #         {
    #             'use_sim_time': use_sim_time,
    #             'frame_id': 'base_footprint',
    #             'odom_frame_id': 'odom',
    #             'publish_tf': False,
    #             'subscribe_depth': True,
    #             'subscribe_odom_info': True,
    #             'approx_sync': True,
    #             'approx_sync_max_interval': 0.01,
    #             'queue_size': 10,
    #             'sync_queue_size': 10,
    #             'wait_imu_to_init': False,
    #         }
    #     ],
    #     remappings=[
    #         ('rgb/image', '/camera/color/image_raw'),
    #         ('rgb/camera_info', '/camera/color/camera_info'),
    #         (
    #             'depth/image',
    #             '/camera/aligned_depth_to_color/image_raw',
    #         ),
    #         ('odom', '/camera/odom'),
    #     ],
    # )

    # EKF publishes /odom and odom TF from wheel odometry and IMU.
    ekf_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            params_ekf,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('odometry/filtered', '/odom'),
        ],
    )

    launch_description = LaunchDescription(ARGUMENTS)
    launch_description.add_action(robot_state_publisher_cmd)
    launch_description.add_action(twist_mux_cmd)
    launch_description.add_action(motor_drive_cmd)
    launch_description.add_action(ydlidar_cmd)
    launch_description.add_action(scan_filter_cmd)
    launch_description.add_action(robot_agent_cmd)
    #launch_description.add_action(realsense_cmd)
    # launch_description.add_action(rgbd_odometry_cmd)  # RTAB odometry disabled
    launch_description.add_action(ekf_cmd)

    return launch_description
