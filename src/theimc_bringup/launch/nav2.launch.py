import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, SetRemap
from launch.actions import GroupAction


ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='Use simulation (Gazebo) clock if true'
    ),
]


def generate_launch_description():

    pkg_share_navigation2 = get_package_share_directory('theimc_navigation2')
    pkg_share_nav2_bringup = get_package_share_directory('nav2_bringup')

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

    launch_nav2_bringup = PathJoinSubstitution([
        pkg_share_nav2_bringup,
        'launch',
        'bringup_launch.py'
    ])

    # ---------------------------------------------------------
    # Launch Config
    # ---------------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')

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
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'yaml_filename': keepout_map_yaml,
                'topic_name': '/keepout_filter_mask',
                'frame_id': 'map'
            }
        ]
    )

    # ---------------------------------------------------------
    # Costmap Filter Info Server
    # ---------------------------------------------------------
    costmap_filter_info_server_cmd = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'filter_info_topic': '/costmap_filter_info',
                'type': 0,
                'mask_topic': '/keepout_filter_mask',
                'base': 0.0,
                'multiplier': 1.0
            }
        ]
    )

    # ---------------------------------------------------------
    # Keepout lifecycle 직접 처리
    #
    # 노드가 ROS graph에 보일 때까지 기다린 다음
    # unconfigured -> configure
    # inactive -> activate
    # 순서로 처리
    # ---------------------------------------------------------
    bringup_keepout_filters_cmd = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            '''
            echo "[KEEP_OUT] Waiting for /filter_mask_server..."
            until ros2 node list | grep -qx "/filter_mask_server"; do
              sleep 0.5
            done

            echo "[KEEP_OUT] Waiting for /costmap_filter_info_server..."
            until ros2 node list | grep -qx "/costmap_filter_info_server"; do
              sleep 0.5
            done

            echo "[KEEP_OUT] Configure /filter_mask_server if needed..."
            STATE=$(ros2 lifecycle get /filter_mask_server 2>/dev/null | awk '{print $1}')
            echo "[KEEP_OUT] /filter_mask_server state: $STATE"
            if [ "$STATE" = "unconfigured" ]; then
              ros2 lifecycle set /filter_mask_server configure
              sleep 1.0
            fi

            echo "[KEEP_OUT] Activate /filter_mask_server if needed..."
            STATE=$(ros2 lifecycle get /filter_mask_server 2>/dev/null | awk '{print $1}')
            echo "[KEEP_OUT] /filter_mask_server state: $STATE"
            if [ "$STATE" = "inactive" ]; then
              ros2 lifecycle set /filter_mask_server activate
              sleep 1.0
            fi

            echo "[KEEP_OUT] Configure /costmap_filter_info_server if needed..."
            STATE=$(ros2 lifecycle get /costmap_filter_info_server 2>/dev/null | awk '{print $1}')
            echo "[KEEP_OUT] /costmap_filter_info_server state: $STATE"
            if [ "$STATE" = "unconfigured" ]; then
              ros2 lifecycle set /costmap_filter_info_server configure
              sleep 1.0
            fi

            echo "[KEEP_OUT] Activate /costmap_filter_info_server if needed..."
            STATE=$(ros2 lifecycle get /costmap_filter_info_server 2>/dev/null | awk '{print $1}')
            echo "[KEEP_OUT] /costmap_filter_info_server state: $STATE"
            if [ "$STATE" = "inactive" ]; then
              ros2 lifecycle set /costmap_filter_info_server activate
              sleep 1.0
            fi

            echo "[KEEP_OUT] Final states:"
            ros2 lifecycle get /filter_mask_server
            ros2 lifecycle get /costmap_filter_info_server
            '''
        ],
        output='screen'
    )

    # ---------------------------------------------------------
    # Nav2 Bringup
    #
    # 여기서는 LaunchConfiguration('map')을 쓰지 않고
    # params_nav2_map을 직접 넘겨서
    # launch configuration 'map' does not exist 에러를 피함
    # ---------------------------------------------------------
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_nav2_bringup]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': params_nav2_map,
            'params_file': params_nav2,
            'autostart': 'true'
        }.items()
    )
    nav2_remapping_group = GroupAction(
        actions=[
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

    ld.add_action(dla_map_yaml_cmd)
    ld.add_action(dla_params_nav2_cmd)

    # 1. Keepout filter 서버 실행
    ld.add_action(filter_mask_server_cmd)
    ld.add_action(costmap_filter_info_server_cmd)

    # 2. 노드 뜨는 것 기다렸다가 configure/activate
    ld.add_action(
        TimerAction(
            period=2.0,
            actions=[bringup_keepout_filters_cmd]
        )
    )

    # 3. Keepout 서버가 active 된 뒤 Nav2 실행
    ld.add_action(
        TimerAction(
            period=10.0,
            actions=[nav2_remapping_group]
        )
    )
    
    # 4. Nav2/AMCL 실행 후 last pose manager 실행
    ld.add_action(
        TimerAction(
            period=15.0,
            actions=[last_pose_manager_node]
        )
    )
    return ld
