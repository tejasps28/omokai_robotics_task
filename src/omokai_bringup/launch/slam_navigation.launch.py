"""Launch TurtleBot3 with online SLAM and Nav2 navigation."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('omokai_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')
    turtlebot_navigation_share = get_package_share_directory(
        'turtlebot3_navigation2'
    )

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    params_file = os.path.join(
        turtlebot_navigation_share,
        'param',
        'waffle_pi.yaml',
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

    navigation_with_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': 'True',
            'map': '',
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'True',
            'use_composition': 'False',
        }.items(),
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
            DeclareLaunchArgument('map', default_value=''),
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument('x_pose', default_value='-2.0'),
            DeclareLaunchArgument('y_pose', default_value='-0.5'),
            simulation,
            TimerAction(period=5.0, actions=[navigation_with_slam]),
            TimerAction(period=7.0, actions=[rviz_view]),
        ]
    )
