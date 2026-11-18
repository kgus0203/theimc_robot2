"""Launch a display-only keepout-filter reproduction without robot I/O."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('keepout_test_pkg')
    map_yaml = os.path.join(package_share, 'maps', 'map.yaml')
    mask_yaml = os.path.join(package_share, 'maps', 'keepout_map.yaml')
    params_file = os.path.join(
        package_share, 'config', 'navigation_keepout.yaml'
    )
    rviz_file = os.path.join(
        package_share, 'rviz', 'keepout_test.rviz'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            choices=['true', 'false'],
        ),
        # Keep ROS logs writable in restricted/remote sessions.
        SetEnvironmentVariable('ROS_LOG_DIR', '/tmp/keepout_test_pkg_logs'),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='keepout_test_map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,
                'topic_name': '/map',
                'frame_id': 'map',
            }],
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='keepout_test_filter_mask_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': mask_yaml,
                'topic_name': '/keepout_filter_mask',
                'frame_id': 'map',
            }],
        ),
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='keepout_test_filter_info_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'filter_info_topic': '/costmap_filter_info',
                'type': 0,
                'mask_topic': '/keepout_filter_mask',
                'base': 0.0,
                'multiplier': 1.0,
            }],
        ),
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        # A test-only transform lets the standalone costmap initialize.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='keepout_test_static_tf',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'map',
                '--child-frame-id', 'keepout_test_base',
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='keepout_test_lifecycle_manager',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                # The standalone costmap node lives at /costmap/costmap and
                # does not form the standard Nav2 server bond name. Lifecycle
                # transitions still run; setting 0 disables only bond checks.
                'bond_timeout': 0.0,
                'node_names': [
                    'keepout_test_map_server',
                    'keepout_test_filter_mask_server',
                    'keepout_test_filter_info_server',
                    'costmap/costmap',
                ],
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='keepout_test_rviz',
            output='screen',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(launch_rviz),
        ),
    ])
