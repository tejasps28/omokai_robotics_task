from glob import glob
import os
from setuptools import find_packages, setup


package_name = 'omokai_bringup'


def recursive_data_files(source_root: str):
    """Install a model collection while retaining its relative layout."""

    entries = []
    for directory, _, filenames in os.walk(source_root):
        if not filenames:
            continue
        destination = os.path.join('share', package_name, directory)
        entries.append(
            (
                destination,
                [os.path.join(directory, filename) for filename in filenames],
            )
        )
    return entries

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        (
            'share/' + package_name + '/models/test_zone',
            glob('models/test_zone/*.*'),
        ),
        (
            'share/' + package_name + '/models/test_zone/meshes',
            glob('models/test_zone/meshes/*'),
        ),
        (
            'share/' + package_name + '/models/vision_obstacle',
            glob('models/vision_obstacle/*.*'),
        ),
        (
            'share/' + package_name + '/models/office_small_collection',
            glob('models/office_small_collection/*.*'),
        ),
    ]
    + recursive_data_files('models/DoctorFemaleWalkRed')
    + recursive_data_files('models/DoctorFemaleWalkScenario'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tejas',
    maintainer_email='tejasps28@gmail.com',
    description='Simulation bringup and foundation checks for the Omokai ground robot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'foundation_smoke_test = omokai_bringup.foundation_smoke_test:main',
            'fleet_tf_aggregator = omokai_bringup.tf_aggregator:main',
            'fleet_traffic_manager = omokai_bringup.traffic_manager:main',
            'initial_pose_publisher = omokai_bringup.initial_pose_publisher:main',
            'saved_map_verifier = omokai_bringup.saved_map_runner:main',
            'task1_mission_runner = omokai_bringup.task1_mission_runner:main',
            'vision_visualization = omokai_bringup.vision_visualization:main',
            'vision_actor_patrol = omokai_bringup.vision_actor_patrol:main',
            'vision_live_validation = omokai_bringup.vision_live_validation:main',
        ],
    },
)
