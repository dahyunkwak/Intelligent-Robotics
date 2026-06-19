import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


# ─────────────────────────────────────────────────────────────
#  타이밍 상수 (WSL2 환경에 맞게 조정)
# ─────────────────────────────────────────────────────────────
GAZEBO_STARTUP_DELAY = 12.0
NAV2_STARTUP_DELAY   = 22.0
GT_POSE_DELAY        = 25.0


def generate_launch_description():

    pkg_path          = get_package_share_directory('campus_delivery_robot')
    turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_dir  = get_package_share_directory('nav2_bringup')

    world_file  = os.path.join(pkg_path, 'worlds', 'campus_world_v2.world')
    nav2_params = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
    map_file    = os.path.join(pkg_path, 'maps',   'campus_map.yaml')
    urdf_file   = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf', 'turtlebot3_burger.urdf'
    )
    robot_sdf   = os.path.join(
        turtlebot3_gazebo, 'models', 'turtlebot3_burger', 'model.sdf'
    )

    try:
        robot_desc = xacro.process_file(urdf_file).toxml()
    except Exception as e:
        raise RuntimeError(
            f"[LAUNCH] ❌ URDF xacro 처리 실패: {e}\n"
            f"  파일: {urdf_file}"
        )

    return LaunchDescription([

        SetEnvironmentVariable('TURTLEBOT3_MODEL',      'burger'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),

        # 1단계: Gazebo 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch', 'gzserver.launch.py'
                )
            ),
            launch_arguments={'world': world_file}.items(),
        ),

        # 2단계: 로봇 스폰 + RSP
        TimerAction(
            period=GAZEBO_STARTUP_DELAY,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'turtlebot3_burger',
                        '-file',   robot_sdf,
                        '-x', '0.0',
                        '-y', '-18.0',
                        '-z', '0.01',
                    ],
                    output='screen',
                ),
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{
                        'robot_description': robot_desc,
                        'use_sim_time': True,
                    }],
                ),
            ],
        ),

        # 3단계: Nav2
        TimerAction(
            period=NAV2_STARTUP_DELAY,
            actions=[
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[{
                        'use_sim_time':  True,
                        'yaml_filename': map_file,
                    }],
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'autostart':    True,
                        'node_names':   ['map_server'],
                    }],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            nav2_bringup_dir, 'launch', 'navigation_launch.py'
                        )
                    ),
                    launch_arguments={
                        'use_sim_time': 'True',
                        'params_file':  nav2_params,
                        'autostart':    'True',
                    }.items(),
                ),
            ],
        ),

        # 4단계: Ground-truth 위치 퍼블리셔
        TimerAction(
            period=GT_POSE_DELAY,
            actions=[
                Node(
                    package='campus_delivery_robot',
                    executable='gt_pose_pub',
                    name='gt_pose_publisher',
                    output='screen',
                    parameters=[{'use_sim_time': True}],
                ),
            ],
        ),
    ])
