"""Launch the known-map TurtleBot3 core-task navigation environment."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('omokai_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')
    turtlebot_navigation_share = get_package_share_directory(
        'turtlebot3_navigation2'
    )

    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')
    map_file = os.path.join(turtlebot_navigation_share, 'map', 'map.yaml')
    params_file = os.path.join(
        turtlebot_navigation_share, 'param', 'waffle_pi.yaml'
    )
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'yaml_filename': map_file},
        convert_types=True,
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'gui': gui,
            'use_sim_time': use_sim_time,
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': '',
            'params_file': configured_params,
            'use_sim_time': use_sim_time,
            'autostart': 'True',
            'use_composition': 'False',
        }.items(),
    )

    initial_pose = Node(
        package='omokai_bringup',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'x': x_pose,
                'y': y_pose,
                'yaw': yaw,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('gui', default_value='false'),
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument('x_pose', default_value='-2.0'),
            DeclareLaunchArgument('y_pose', default_value='-0.5'),
            DeclareLaunchArgument('yaw', default_value='0.0'),
            simulation,
            TimerAction(period=5.0, actions=[navigation]),
            TimerAction(period=7.0, actions=[initial_pose]),
        ]
    )
