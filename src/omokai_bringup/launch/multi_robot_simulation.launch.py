"""Launch three isolated TurtleBot3 Waffle Pi robots in Gazebo Harmonic."""

from __future__ import annotations

import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from omokai_bringup.multi_robot_model import render_namespaced_sdf
from omokai_bringup.multi_robot_nav import namespace_nav_parameters
from omokai_bringup.test_zone_map import generate_test_zone_map
from omokai_fleet.model import ROBOT_IDS
from omokai_fleet.scenario import DOCKING_POSES
import yaml


ROBOTS = tuple(
    (
        robot_id.value,
        str(pose.x),
        str(pose.y),
        str(pose.yaw),
    )
    for robot_id, pose in zip(ROBOT_IDS, DOCKING_POSES)
)


def _robot_actions(context, model_path: str, urdf_path: str):
    source = Path(model_path).read_text(encoding='utf-8')
    description = Path(urdf_path).read_text(encoding='utf-8')
    generated_directory = Path(
        tempfile.mkdtemp(prefix='omokai-multi-robot-')
    )
    actions = []

    for robot_name, x_pose, y_pose, yaw_pose in ROBOTS:
        sdf_path = generated_directory / f'{robot_name}.sdf'
        sdf_path.write_text(
            render_namespaced_sdf(source, robot_name),
            encoding='utf-8',
        )
        actions.extend(
            [
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    namespace=robot_name,
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[
                        {
                            'use_sim_time': True,
                            'robot_description': description,
                            'frame_prefix': f'{robot_name}/',
                        }
                    ],
                    remappings=[
                        ('/tf', 'tf'),
                        ('/tf_static', 'tf_static'),
                    ],
                ),
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name=f'spawn_{robot_name}',
                    arguments=[
                        '-name',
                        robot_name,
                        '-file',
                        str(sdf_path),
                        '-x',
                        x_pose,
                        '-y',
                        y_pose,
                        '-z',
                        '0.01',
                        '-Y',
                        yaw_pose,
                    ],
                    output='screen',
                ),
            ]
        )

    return actions


def _navigation_actions(
    context,
    nav2_launch_path: str,
    params_path: str,
    map_path: str,
):
    parameters = yaml.safe_load(Path(params_path).read_text(encoding='utf-8'))
    demo_max_speed = float(
        os.environ.get('OMOKAI_DEMO_MAX_SPEED_MPS', '0.18')
    )
    if demo_max_speed > 0.26:
        controller = parameters['controller_server']['ros__parameters'][
            'FollowPath'
        ]
        controller['max_vel_x'] = demo_max_speed
        controller['max_speed_xy'] = demo_max_speed
        smoother = parameters['velocity_smoother']['ros__parameters']
        smoother['max_velocity'][0] = demo_max_speed
        smoother['min_velocity'][0] = -demo_max_speed
    generated_directory = Path(
        tempfile.mkdtemp(prefix='omokai-multi-nav-')
    )
    actions = []

    for robot_name, x_pose, y_pose, yaw_pose in ROBOTS:
        robot_params_path = generated_directory / f'{robot_name}.yaml'
        robot_params_path.write_text(
            yaml.safe_dump(
                namespace_nav_parameters(parameters, robot_name),
                sort_keys=False,
            ),
            encoding='utf-8',
        )
        actions.extend(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch_path),
                    launch_arguments={
                        'namespace': robot_name,
                        'use_namespace': 'True',
                        'slam': 'False',
                        'map': map_path,
                        'use_sim_time': 'True',
                        'params_file': str(robot_params_path),
                        'autostart': 'True',
                        'use_composition': 'True',
                        'use_respawn': 'False',
                    }.items(),
                ),
                Node(
                    package='omokai_bringup',
                    executable='initial_pose_publisher',
                    namespace=robot_name,
                    name='initial_pose_publisher',
                    output='screen',
                    parameters=[
                        {
                            'use_sim_time': True,
                            'x': float(x_pose),
                            'y': float(y_pose),
                            'yaw': float(yaw_pose),
                            'timeout_sec': 90.0,
                        }
                    ],
                ),
            ]
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('omokai_bringup')
    turtlebot_share = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    nav2_share = get_package_share_directory('nav2_bringup')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    verbosity = EnvironmentVariable('GZ_VERBOSITY', default_value='2')
    world = os.path.join(
        bringup_share,
        'worlds',
        'multi_robot_test_zone.world',
    )
    model_path = os.path.join(
        turtlebot_share,
        'models',
        'turtlebot3_waffle_pi',
        'model.sdf',
    )
    urdf_path = os.path.join(
        turtlebot_share,
        'urdf',
        'turtlebot3_waffle_pi.urdf',
    )
    nav2_launch_path = os.path.join(
        nav2_share,
        'launch',
        'bringup_launch.py',
    )
    nav2_params_path = os.path.join(
        nav2_share,
        'params',
        'nav2_multirobot_params_all.yaml',
    )
    map_path = generate_test_zone_map(
        os.path.join(
            bringup_share,
            'models',
            'test_zone',
            'model.sdf',
        )
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
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='multi_robot_bridge',
        arguments=[
            '--ros-args',
            '-p',
            'config_file:=' + os.path.join(
                bringup_share,
                'config',
                'multi_robot_bridge.yaml',
            ),
        ],
        output='screen',
    )
    tf_aggregator = Node(
        package='omokai_bringup',
        executable='fleet_tf_aggregator',
        name='fleet_tf_aggregator',
        output='screen',
    )
    traffic_manager = Node(
        package='omokai_bringup',
        executable='fleet_traffic_manager',
        name='fleet_traffic_manager',
        output='screen',
    )
    formation_controller = Node(
        package='omokai_fleet',
        executable='formation_controller',
        name='fleet_formation_controller',
        output='screen',
    )
    rviz_view = Node(
        package='rviz2',
        executable='rviz2',
        name='fleet_rviz',
        arguments=[
            '-d',
            os.path.join(bringup_share, 'rviz', 'multi_robot.rviz'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('gui', default_value='false'),
            DeclareLaunchArgument('rviz', default_value='false'),
            AppendEnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH',
                os.path.join(turtlebot_share, 'models'),
            ),
            AppendEnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH',
                os.path.join(bringup_share, 'models'),
            ),
            server,
            client,
            bridge,
            tf_aggregator,
            traffic_manager,
            formation_controller,
            TimerAction(
                period=3.0,
                actions=[
                    OpaqueFunction(
                        function=_robot_actions,
                        args=[model_path, urdf_path],
                    )
                ],
            ),
            TimerAction(
                period=8.0,
                actions=[
                    OpaqueFunction(
                        function=_navigation_actions,
                        args=[
                            nav2_launch_path,
                            nav2_params_path,
                            map_path,
                        ],
                    )
                ],
            ),
            TimerAction(period=10.0, actions=[rviz_view]),
        ]
    )
