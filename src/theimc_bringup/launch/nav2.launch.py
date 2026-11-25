import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, SetParameter, SetRemap
from launch.actions import GroupAction


ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='Use simulation (Gazebo) clock if true'
    ),
    DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        choices=['True', 'False'],
        description='Run Nav2 nodes in a component container if true'
    ),
]


def generate_launch_description():

    pkg_share_navigation2 = get_package_share_directory('theimc_navigation2')
    pkg_share_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_share_bringup = get_package_share_directory('theimc_bringup')

    # ---------------------------------------------------------
    # Path 설정
    # ---------------------------------------------------------
    params_nav2_map = PathJoinSubstitution([
        pkg_share_navigation2,
        'maps',
        'map.yaml'
    ])

    params_nav2 = PathJoinSubstitution([
        pkg_share_navigation2,
        'params',
        'theimc_nav2_params_mppi.yaml'
    ])

    keepout_map_yaml = os.path.join(
        pkg_share_navigation2,
        'maps',
        'keepout_map.yaml'
    )

    fastdds_profile = os.path.join(
        pkg_share_bringup,
        'params',
        'fastdds_udp_only.xml'
    )

    launch_nav2_bringup = PathJoinSubstitution([
        pkg_share_nav2_bringup,
        'launch',
        'bringup_launch.py'
    ])

    # ---------------------------------------------------------
    # Launch Config
    # ---------------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_composition = LaunchConfiguration('use_composition')

    # ---------------------------------------------------------
    # Declare Launch Args
    # ---------------------------------------------------------
    dla_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=params_nav2_map,
        description='Full path to map yaml file to load'
    )

    dla_params_nav2_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_nav2,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    # ---------------------------------------------------------
    # Keepout Filter Mask Map Server
    # ---------------------------------------------------------
    filter_mask_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': keepout_map_yaml,
            'topic_name': '/keepout_filter_mask',
            'frame_id': 'map'
        }]
    )

    # ---------------------------------------------------------
    # Costmap Filter Info Server
    # ---------------------------------------------------------
    costmap_filter_info_server_cmd = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'filter_info_topic': '/costmap_filter_info',
            'type': 0,
            'mask_topic': '/keepout_filter_mask',
            'base': 0.0,
            'multiplier': 1.0
        }]
    )

    lifecycle_manager_keepout = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_keepout',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            # Keep this shorter than launch's SIGINT/SIGTERM escalation window
            # so the manager can finish cleanup before launch sends SIGKILL.
            'bond_timeout': 4.0,
            'bond_respawn_max_duration': 10.0,
            'node_names': [
                'filter_mask_server',
                'costmap_filter_info_server',
            ],
        }]
    )

    # ---------------------------------------------------------
    # Nav2 Bringup
    #
    # 선언된 map / params_file 인자를 그대로 Nav2 bringup에 전달
    # ---------------------------------------------------------
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_nav2_bringup]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'true',
            'use_composition': use_composition,
        }.items()
    )
    nav2_remapping_group = GroupAction(
        actions=[
            SetParameter(name='bond_timeout', value=4.0),
            SetParameter(name='bond_respawn_max_duration', value=10.0),
            SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),
            SetRemap(src='cmd_vel', dst='/cmd_vel_nav'),
            nav2_bringup_launch
        ]
    )

    # ---------------------------------------------------------
    # Last Pose Manager
    #
    # /amcl_pose를 저장하고, 다음 부팅 때 /initialpose로 자동 publish
    # Nav2/AMCL이 실행된 뒤 동작해야 하므로 TimerAction으로 지연 실행
    # ---------------------------------------------------------
    last_pose_manager_node = Node(
        package='theimc_bringup',
        executable='last_pose_manager_node',
        name='last_pose_manager_node',
        output='screen',
        parameters=[{
            'pose_file': '/home/jeff/theimc_robot/config/last_pose.yaml',
            'auto_publish_initial_pose': True,
            'publish_delay_sec': 5.0,
            'save_interval_sec': 2.0,
            'max_xy_covariance': 1.0,
            'max_yaw_covariance': 1.0,
        }]
    )

    # ---------------------------------------------------------
    # Launch Description
    # ---------------------------------------------------------
    ld = LaunchDescription(ARGUMENTS)

    # FastDDS shared-memory port files can remain corrupted after forced exits.
    # Use UDP for this launch so Nav2 communication does not depend on SHM locks.
    ld.add_action(SetEnvironmentVariable(
        name='FASTRTPS_DEFAULT_PROFILES_FILE',
        value=fastdds_profile,
    ))

    ld.add_action(dla_map_yaml_cmd)
    ld.add_action(dla_params_nav2_cmd)

    # Keepout mask와 filter 정보를 먼저 활성화한다.
    ld.add_action(filter_mask_server_cmd)
    ld.add_action(costmap_filter_info_server_cmd)
    ld.add_action(lifecycle_manager_keepout)

    # Keepout 토픽이 준비된 뒤 Nav2를 실행한다.
    ld.add_action(
        TimerAction(
            period=10.0,
            actions=[nav2_remapping_group]
        )
    )

    # Nav2/AMCL 실행 후 last pose manager 실행
    ld.add_action(
        TimerAction(
            period=15.0,
            actions=[last_pose_manager_node]
        )
    )
    return ld
