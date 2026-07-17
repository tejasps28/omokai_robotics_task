from glob import glob
from setuptools import find_packages, setup


package_name = 'omokai_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tejas',
    maintainer_email='tejasps28@gmail.com',
    description='Simulation bringup and foundation checks for the Omokai ground robot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'foundation_smoke_test = omokai_bringup.foundation_smoke_test:main',
            'initial_pose_publisher = omokai_bringup.initial_pose_publisher:main',
            'task1_mission_runner = omokai_bringup.task1_mission_runner:main',
        ],
    },
)
