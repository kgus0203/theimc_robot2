"""Stock mock demo with an initial pose that fits between the side walls."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    share = Path(get_package_share_directory('fairino_waypoint_test'))
    config = (MoveItConfigsBuilder('fairino5_v6_robot',
                                  package_name='fairino5_v6_moveit2_config')
              .robot_description(mappings={
                  'initial_positions_file': str(share / 'config/initial_positions.yaml')})
              .to_moveit_configs())
    return generate_demo_launch(config)
