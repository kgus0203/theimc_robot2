from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config")
        .to_moveit_configs()
    )

    return LaunchDescription(
        [
            SetRemap(src='/joint_states', dst='/arm/joint_states'),
            Node(
                package="fairino5_v6_moveit2_config",
                executable="tcp_waypoint_action",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    {
                        "group_name": "fairino5_v6_group",
                        "tcp_link": "wrist3_link",
                        "reference_frame": "base_link",
                        "command_topic": "/arm_command",
                        "start_command": "START",
                        "velocity_scaling": 0.2,
                        "acceleration_scaling": 0.2,
                        "eef_step": 0.01,
                        "jump_threshold": 0.0,
                        "min_path_fraction": 0.95,
                        "rectangle_points": [
                            0.35,
                            -0.05,
                            0.34,
                            0.35,
                            0.05,
                            0.34,
                            0.35,
                            0.05,
                            0.28,
                            0.35,
                            -0.05,
                            0.28,
                        ],
                        "use_current_orientation": True,
                        "target_orientation_xyzw": [1.0, 0.0, 0.0, 1.0],
                    },
                ],
            )
        ]
    )
