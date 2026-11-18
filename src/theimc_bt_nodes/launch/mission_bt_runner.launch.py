from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'bt_xml',
            default_value=PathJoinSubstitution([
                FindPackageShare('theimc_bt_nodes'), 'bt_xml', 'theimc_bt.xml'
            ]),
        ),
        DeclareLaunchArgument(
            'rails_yaml',
            default_value=PathJoinSubstitution([
                FindPackageShare('theimc_bt_nodes'), 'config', 'rails.yaml'
            ]),
        ),
        DeclareLaunchArgument(
            'rail_approach_yaml',
            default_value=PathJoinSubstitution([
                FindPackageShare('theimc_bt_nodes'), 'config', 'rail_approach.yaml'
            ]),
        ),
        DeclareLaunchArgument(
            'start_rail_approach',
            default_value='false',
            description='Set true only when another launch is not already running the RailApproach action server',
        ),
        DeclareLaunchArgument(
            'rail_approach_executable',
            default_value='rail_approach_action_server_node',
            description='Must match decision_making_pkg setup.py console_scripts entry point',
        ),
        Node(
            package='theimc_bt_nodes',
            executable='mission_bt_runner',
            name='mission_bt_runner',
            output='screen',
            parameters=[{
                'bt_xml': LaunchConfiguration('bt_xml'),
                'rails_yaml': LaunchConfiguration('rails_yaml'),
            }],
        ),
        Node(
            package='decision_making_pkg',
            executable=LaunchConfiguration('rail_approach_executable'),
            name='simple_rail_approach_action_node',
            output='screen',
            parameters=[LaunchConfiguration('rail_approach_yaml')],
            condition=IfCondition(LaunchConfiguration('start_rail_approach')),
        ),
    ])

