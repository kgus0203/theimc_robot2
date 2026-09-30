import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('theimc_description')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'theimc.rviz')
    urdf_file = os.path.join(pkg_share, 'urdf', 'theimc.urdf')
    use_sim_time = LaunchConfiguration('use_sim_time')


    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    dla_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_file]
    )

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 
                     'robot_description': robot_description_content}]
    )

    joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(dla_use_sim_time)
    # ld.add_action(rviz_cmd)
    ld.add_action(robot_state_publisher_cmd)

    return ld

