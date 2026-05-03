import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('campus_delivery_robot')
    world_file = os.path.join(pkg_path, 'worlds', 'campus.world')
    turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),

        # Gazebo 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch', 'gazebo.launch.py'
                )
            ]),
            launch_arguments={'world': world_file}.items(),
        ),

        # 5초 딜레이 후 스폰 (Gazebo 완전히 뜰 때까지 대기)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'turtlebot3_burger',
                        '-file', os.path.join(
                            turtlebot3_gazebo,
                            'models', 'turtlebot3_burger', 'model.sdf'
                        ),
                        '-x', '0.0',
                        '-y', '0.0',
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
                        'robot_description': open(
                            os.path.join(
                                get_package_share_directory('turtlebot3_description'),
                                'urdf', 'turtlebot3_burger.urdf'
                            )
                        ).read()
                    }],
                ),
            ]
        ),
    ])
