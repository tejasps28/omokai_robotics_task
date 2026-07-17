"""Launch TurtleBot3 Waffle Pi in Gazebo Harmonic with optional GUI.

The launch composition follows the Apache-2.0 ROBOTIS TurtleBot3 Jazzy
simulation launch structure while separating server and client for a genuine
headless mode. See docs/sources.md.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    turtlebot_share = get_package_share_directory('turtlebot3_gazebo')
    turtlebot_launch = os.path.join(turtlebot_share, 'launch')
    ros_gz_share = get_package_share_directory('ros_gz_sim')

    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    verbosity = EnvironmentVariable('GZ_VERBOSITY', default_value='2')
    # Keep the acceptance world self-contained.  The upstream TurtleBot3 world
    # references Fuel-hosted Ground Plane and Sun models, which makes a clean
    # Docker start depend on network access and a warm Gazebo cache.
    world = os.path.join(
        get_package_share_directory('omokai_bringup'),
        'worlds',
        'task1_turtlebot3.world',
    )

    resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(turtlebot_share, 'models'),
    )

    server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v', verbosity, ' ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-g -v', verbosity],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(gui),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot_launch, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot_launch, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        resource_path,
        server,
        client,
        robot_state_publisher,
        spawn_robot,
    ])
