import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('theimc_bringup')

    rviz_config = os.path.join(
        bringup_share,
        'rviz',
        'theimc_bringup.rviz',
    )
    fastdds_profile = os.path.join(
        bringup_share,
        'params',
        'fastdds_udp_only.xml',
    )

    return LaunchDescription([
        # Humble RViz/Ogre can crash in the Map GLSL shader on some laptop
        # OpenGL drivers.  Mesa software rendering is slower but avoids that
        # driver-specific crash and keeps the shared RViz config unchanged.
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            fastdds_profile,
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
