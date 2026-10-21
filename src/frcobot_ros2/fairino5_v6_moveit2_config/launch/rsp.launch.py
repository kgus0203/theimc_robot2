from launch import LaunchDescription
from launch_ros.actions import SetRemap
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config").to_moveit_configs()
    return LaunchDescription([
        SetRemap(src='/joint_states', dst='/arm/joint_states'),
        SetRemap(src='/dynamic_joint_states', dst='/arm/dynamic_joint_states'),
        *generate_rsp_launch(moveit_config).entities,
    ])
