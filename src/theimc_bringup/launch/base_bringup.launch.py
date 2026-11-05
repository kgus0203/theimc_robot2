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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
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
    fastdds_profile = os.path.join(
        pkg_share_bringup,
        'params',
        'fastdds_udp_only.xml',
    )

    urdf_file = os.path.join(
        pkg_share_description,
        'urdf',
        'theimc.urdf',
    )
    # URDF에서 base_link collision box 크기 읽기
    urdf_root = ET.parse(urdf_file).getroot()

    base_link = urdf_root.find("./link[@name='agv_base_link']")
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
        # Do not reset Beckhoff faults automatically on ROS launch.
        # Inspect the alarm and recover explicitly using /agv/reset_faults.
        parameters=[{'auto_reset_faults_on_start': False}],
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
        name='agv_robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description_content},
        ],
        remappings=[
            ('robot_description', '/agv/robot_description'),
        ],
    )

    robot_agent_cmd = Node(
        package='theimc_bringup',
        executable='robot_agent_node',
        name='robot_agent',
        output='screen',
        emulate_tty=True,
    )

    # Beckhoff ADS driver
    #
    # Verified network / ADS configuration:
    #   Jetson IP        : 192.168.0.50
    #   Local AMS Net ID : 192.168.0.50.1.2
    #   Beckhoff IP      : 192.168.0.20
    #   Remote AMS Net ID: 192.168.0.20.1.1
    #   PLC ADS port     : 851
    #
    # Axis mapping confirmed by vendor:
    #   Axis 1 / 2 = drive wheels
    #   Axis 3     = rail motor
    #
    # Wheel dimensions:
    #   diameter            = 0.2032 m (8 inch)
    #   center-to-center    = 0.930 m
    #   rail wheel diameter = 0.130 m (already handled by PLC)
    #
    # NOTE:
    # left/right physical Axis 1/2 mapping and direction signs still
    # need one low-speed field test. If one side runs opposite,
    # change only the corresponding *_wheel_sign to -1.0.
    agv_ads_driver_cmd = Node(
        package='agv_ads_driver',
        executable='agv_node',
        name='agv_ads_driver',
        output='screen',
        # Long enough for verified shutdown before ROS launch escalates to SIGTERM.
        sigterm_timeout='15',
        sigkill_timeout='5',
        parameters=[
            {
                # Real Beckhoff ADS mode
                'mock_mode': False,

                # Axis mapping
                'left_wheel_axis': 1,
                'right_wheel_axis': 2,
                'rail_axis': 3,

                # ROS positive-direction correction
                'left_wheel_sign': 1.0,
                'right_wheel_sign': 1.0,
                'rail_sign': 1.0,

                # Actual drive-wheel geometry
                'wheel_separation_m': 0.930,
                'wheel_diameter_m': 0.2032,

                # ROS m/s -> Beckhoff mm/s
                'velocity_scale': 1000.0,

                # Beckhoff MoveVelocity parameters
                'wheel_ramp_ms': 1000.0,
                'rail_ramp_ms': 1000.0,
                'jerk_percent': 100.0,
                'rail_direction_stop_hold_ms': 200.0,

                # Command watchdogs
                'cmd_vel_timeout_sec': 10.0,  # requested ROS watchdog; independent PLC watchdog required
                'rail_cmd_timeout_sec': 10.0,  # requested ROS watchdog; independent PLC watchdog required
                # ADS motion command serialization (conservative ramp+margin).
                # Does not replace checking PLC MoveVelocity_Status/Stop_Status.
                'ads_command_settle_extra_ms': 150.0,
                'wheel_move_command_min_interval_ms': 100.0,
                'stop_suppress_velocity_mps': 0.002,
                'stopped_velocity_threshold_mps': 0.01,
                'wheel_stop_pending_timeout_sec': 4.0,


                # Do not resend nearly-identical velocity commands
                'wheel_command_epsilon_mps': 0.005,
                'rail_command_epsilon_mps': 0.001,

                # Normal bringup: enable axes once at startup,
                # stop + power-disable them during driver shutdown.
                # Controlled manual operation. Do not start with an active PLC motion job.
                'power_enable_on_start': True,  # OPERATION: require safe PLC idle before enabling wheels
                'diagnostic_read_only': False,  # OPERATION: allow ADS motor writes after PLC interlocks
                # On shutdown Stop then PowerDisable only if axes report confirmed standstill.
                'power_disable_on_shutdown': True,  # each axis: STOP -> verified OFF
                'shutdown_poweroff_timeout_sec': 7.0,
                # Keep strict NC/FB interlocks; do not treat speed=0 as NotMoving.
                'startup_power_transition_timeout_sec': 5.0,
                'allow_simultaneous_drive': False,
                'auto_reset_axis_fault': False,  # operator-initiated reset only
                # Vendor policy: on StopMove failure, request PowerDisable once and latch motion.
                # Verify coast/brake behavior under site safety controls before using on hardware.
                'disable_on_stop_failure': True,
                'require_power_restore_approval': True,  # Reset does not automatically PowerEnable
                'log_axis_monitor': False,
                'axis_monitor_period_sec': 0.5,
                'diagnostic_ads_poll_period_sec': 0.5,
                'ads_slow_read_warn_ms': 100.0,

                # Suppress repetitive normal-motion logs.
                'log_motion_commands': False,  # reduce repetitive 20 Hz log output
            }
        ],
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
    # Avoid stale FastDDS shared-memory locks after forced ROS process exits.
    launch_description.add_action(SetEnvironmentVariable(
        name='FASTRTPS_DEFAULT_PROFILES_FILE',
        value=fastdds_profile,
    ))
    launch_description.add_action(robot_state_publisher_cmd)
    launch_description.add_action(twist_mux_cmd)
    launch_description.add_action(motor_drive_cmd)
    launch_description.add_action(ydlidar_cmd)
    launch_description.add_action(scan_filter_cmd)
    launch_description.add_action(robot_agent_cmd)
    #launch_description.add_action(realsense_cmd)
    # launch_description.add_action(rgbd_odometry_cmd)  # RTAB odometry disabled
    launch_description.add_action(ekf_cmd)
    launch_description.add_action(agv_ads_driver_cmd)


    return launch_description
