"""Launch TurtleBot3 localization and navigation against a saved SLAM map."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('omokai_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')
    turtlebot_navigation_share = get_package_share_directory(
        'turtlebot3_navigation2'
    )

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')
    params_file = os.path.join(
        turtlebot_navigation_share,
        'param',
        'waffle_pi.yaml',
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
            'x_pose': spawn_x,
            'y_pose': spawn_y,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': 'False',
            'map': map_file,
            'params_file': configured_params,
            'use_sim_time': use_sim_time,
            'autostart': 'True',
            'use_composition': 'False',
        }.items(),
    )

    initial_pose = Node(
        package='omokai_bringup',
        executable='initial_pose_publisher',
        name='saved_map_initial_pose',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'x': initial_x,
                'y': initial_y,
                'yaw': initial_yaw,
            }
        ],
    )

    rviz_view = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            os.path.join(bringup_share, 'rviz', 'slam_navigation.rviz'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('gui', default_value='false'),
            DeclareLaunchArgument('rviz', default_value='false'),
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument(
                'map',
                default_value=EnvironmentVariable('OMOKAI_MAP_FILE'),
            ),
            DeclareLaunchArgument('spawn_x', default_value='-2.0'),
            DeclareLaunchArgument('spawn_y', default_value='-0.5'),
            DeclareLaunchArgument('initial_x', default_value='0.0'),
            DeclareLaunchArgument('initial_y', default_value='0.0'),
            DeclareLaunchArgument('initial_yaw', default_value='0.0'),
            simulation,
            TimerAction(period=5.0, actions=[navigation]),
            TimerAction(period=7.0, actions=[initial_pose, rviz_view]),
        ]
    )
