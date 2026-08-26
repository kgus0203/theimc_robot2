from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("plant_inspection_pkg"), "config", "inspection_mvp.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=config),
            Node(
                package="plant_inspection_pkg",
                executable="inspection_manager",
                name="inspection_manager",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            ),
        ]
    )
