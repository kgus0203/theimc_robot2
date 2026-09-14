"""Attach to an existing MoveIt system, optionally starting the stock demo."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    share = Path(get_package_share_directory("fairino_waypoint_test"))
    config = MoveItConfigsBuilder(
        "fairino5_v6_robot", package_name="fairino5_v6_moveit2_config"
    ).to_moveit_configs()
    args = [
        DeclareLaunchArgument("start_demo", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("wall_distance", default_value="0.5"),
        DeclareLaunchArgument("wall_thickness", default_value="0.02"),
        DeclareLaunchArgument("wall_width", default_value="1.0"),
        DeclareLaunchArgument("wall_height", default_value="1.0"),
        DeclareLaunchArgument("connection_timeout", default_value="30.0"),
        DeclareLaunchArgument("velocity_scale", default_value="0.1"),
        DeclareLaunchArgument("acceleration_scale", default_value="0.1"),
    ]
    params = {name: ParameterValue(LaunchConfiguration(name), value_type=float)
              for name in ("wall_distance", "wall_thickness", "wall_width", "wall_height",
                           "connection_timeout", "velocity_scale", "acceleration_scale")}
    return LaunchDescription(args + [
        LogInfo(msg=["start_demo=", LaunchConfiguration("start_demo"),
                     ". With start_demo=false, an existing MoveIt demo/hardware bringup is required."]),
        # IncludeLaunchDescription sets arguments in the current context. Isolate
        # the demo's use_rviz=false so it cannot disable our own RViz below.
        GroupAction(
            scoped=True,
            condition=IfCondition(LaunchConfiguration("start_demo")),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(share / "launch/obstacle_demo.launch.py")),
                launch_arguments={"use_rviz": "false"}.items(),
            )],
        ),
        Node(package="rviz2", executable="rviz2", arguments=["-d", str(share / "config/test.rviz")],
             condition=IfCondition(LaunchConfiguration("use_rviz")),
             parameters=[config.robot_description, config.robot_description_semantic,
                         config.robot_description_kinematics]),
        Node(package="fairino_waypoint_test", executable="waypoint_test", output="screen",
             parameters=[config.to_dict(), params]),
    ])
