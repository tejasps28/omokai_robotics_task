"""Launch the isolated RGB-D vision test with one controlled human actor."""

from __future__ import annotations

import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from omokai_bringup.vision_robot_model import render_vision_sdf
from omokai_bringup.vision_robot_model import render_vision_urdf
from omokai_bringup.vision_scene import ROBOT_START
from omokai_bringup.vision_scene import ROBOT_START_YAW_RAD
from omokai_bringup.vision_scene import RED_ACTOR_START
from omokai_bringup.vision_scene import STATIONARY_ACTOR_YAW_RAD
from omokai_bringup.vision_scene import WHITE_ACTOR_START


def _robot_actions(context, model_path: str, urdf_path: str):
    generated_directory = Path(tempfile.mkdtemp(prefix='omokai-vision-'))
    sdf_path = generated_directory / 'robot1.sdf'
    sdf_path.write_text(
        render_vision_sdf(
            Path(model_path).read_text(encoding='utf-8'), 'robot1'
        ),
        encoding='utf-8',
    )
    description = render_vision_urdf(
        Path(urdf_path).read_text(encoding='utf-8')
    )
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='robot1',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                    'robot_description': description,
                    'frame_prefix': 'robot1/',
                }
            ],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_robot1',
            arguments=[
                '-name', 'robot1',
                '-file', str(sdf_path),
                '-x', str(ROBOT_START[0]),
                '-y', str(ROBOT_START[1]),
                # The office carpet/floor top is above z=0.01. Spawn clear of
                # it and let physics settle the wheels onto the surface.
                '-z', '0.10',
                # The stock Waffle Pi camera optical axis points opposite the
                # actor placement in this dedicated vision world.
                '-Y', str(ROBOT_START_YAW_RAD),
            ],
            output='screen',
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('omokai_bringup')
    turtlebot_share = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    actor_share = get_package_share_directory('gazebo_ros_actor_plugin')
    actor_prefix = get_package_prefix('gazebo_ros_actor_plugin')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    actor_motion = LaunchConfiguration('actor_motion')
    red_actor = LaunchConfiguration('red_actor')
    verbosity = EnvironmentVariable('GZ_VERBOSITY', default_value='2')
    world = os.path.join(bringup_share, 'worlds', 'vision_rgbd.world')
    model_path = os.path.join(
        turtlebot_share, 'models', 'turtlebot3_waffle_pi', 'model.sdf'
    )
    urdf_path = os.path.join(
        turtlebot_share, 'urdf', 'turtlebot3_waffle_pi.urdf'
    )
    red_actor_path = os.path.join(
        bringup_share, 'models', 'DoctorFemaleWalkRed', 'model.sdf'
    )
    moving_actor_path = os.path.join(
        bringup_share,
        'models',
        'DoctorFemaleWalkScenario',
        'moving.sdf',
    )
    stationary_actor_path = os.path.join(
        bringup_share,
        'models',
        'DoctorFemaleWalkScenario',
        'stationary.sdf',
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
        name='vision_bridge',
        arguments=[
            '--ros-args',
            '-p',
            'config_file:='
            + os.path.join(bringup_share, 'config', 'vision_bridge.yaml'),
        ],
        output='screen',
    )
    visualization = Node(
        package='omokai_bringup',
        executable='vision_visualization',
        name='vision_visualization',
        output='screen',
    )
    detector = Node(
        package='omokai_perception',
        executable='person_detector',
        name='person_detector',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'confidence_threshold': 0.30,
                'nms_threshold': 0.50,
                'max_inference_hz': 5.0,
                'confirmation_frames': 2,
                'attribute_minimum_ratio': 0.02,
                'attribute_ambiguity_margin': 0.015,
            }
        ],
    )
    follower = Node(
        package='omokai_following',
        executable='vision_follower',
        name='vision_follower',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'artifact_root': '/data/artifacts',
                'target_coat_color': 'white',
                'desired_standoff_m': 1.20,
                'maximum_linear_mps': 0.38,
                'maximum_angular_rps': 0.50,
                'bearing_gain': 1.0,
                'observation_timeout_sec': 0.75,
                'reacquisition_sweep_period_sec': 2.0,
                'initial_search_angular_rps': 0.40,
                'reacquisition_timeout_sec': 10.0,
            }
        ],
    )
    actor_patrol = Node(
        package='omokai_bringup',
        executable='vision_actor_patrol',
        name='vision_actor_patrol',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(actor_motion),
    )
    moving_actor_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_moving_white_coat_actor',
        arguments=[
            '-name', 'vision_target',
            '-file', moving_actor_path,
            '-x', str(WHITE_ACTOR_START[0]),
            '-y', str(WHITE_ACTOR_START[1]),
            '-z', '0',
            '-Y', str(STATIONARY_ACTOR_YAW_RAD),
        ],
        output='screen',
        condition=IfCondition(actor_motion),
    )
    stationary_actor_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_stationary_white_coat_actor',
        arguments=[
            '-name', 'vision_target',
            '-file', stationary_actor_path,
            '-x', str(WHITE_ACTOR_START[0]),
            '-y', str(WHITE_ACTOR_START[1]),
            '-z', '0',
            '-Y', str(STATIONARY_ACTOR_YAW_RAD),
        ],
        output='screen',
        condition=UnlessCondition(actor_motion),
    )
    red_actor_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_red_coat_actor',
        arguments=[
            '-name', 'vision_distractor',
            '-file', red_actor_path,
            '-x', str(RED_ACTOR_START[0]),
            '-y', str(RED_ACTOR_START[1]),
            '-z', '0',
            '-Y', str(STATIONARY_ACTOR_YAW_RAD),
        ],
        output='screen',
        condition=IfCondition(red_actor),
    )
    rviz_view = Node(
        package='rviz2',
        executable='rviz2',
        name='vision_rviz',
        arguments=[
            '-d', os.path.join(bringup_share, 'rviz', 'vision_rgbd.rviz')
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('gui', default_value='false'),
            DeclareLaunchArgument('rviz', default_value='false'),
            DeclareLaunchArgument(
                'actor_motion',
                default_value=EnvironmentVariable(
                    'OMOKAI_ACTOR_MOTION', default_value='false'
                ),
            ),
            DeclareLaunchArgument(
                'red_actor',
                default_value=EnvironmentVariable(
                    'OMOKAI_RED_ACTOR', default_value='false'
                ),
            ),
            # Set one explicit search path before either Gazebo process starts.
            # AppendEnvironmentVariable did not reach the nested GUI launch in
            # Gazebo Harmonic, leaving the server camera able to render actors
            # while the GUI reported "failed to create drawable".
            SetEnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH',
                os.pathsep.join(
                    [
                        os.path.join(turtlebot_share, 'models'),
                        os.path.join(actor_share, 'config', 'skins'),
                        os.path.join(bringup_share, 'models'),
                    ]
                ),
            ),
            SetEnvironmentVariable(
                'GZ_SIM_SYSTEM_PLUGIN_PATH',
                os.pathsep.join(
                    [
                        os.path.join(actor_prefix, 'lib'),
                        os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
                    ]
                ),
            ),
            server,
            client,
            bridge,
            visualization,
            detector,
            follower,
            TimerAction(
                period=2.0,
                actions=[moving_actor_spawn, stationary_actor_spawn],
            ),
            # Wait until the dynamically spawned plugin has subscribed before
            # publishing the first path command.
            TimerAction(period=5.0, actions=[actor_patrol]),
            TimerAction(period=2.0, actions=[red_actor_spawn]),
            TimerAction(
                period=3.0,
                actions=[
                    OpaqueFunction(
                        function=_robot_actions,
                        args=[model_path, urdf_path],
                    )
                ],
            ),
            TimerAction(period=7.0, actions=[rviz_view]),
        ]
    )
