from setuptools import find_packages, setup


package_name = 'omokai_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['data/*.json']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'omokai-mission-preview = omokai_mission.preview:main',
        ],
    },
    zip_safe=False,
    maintainer='Tejas',
    maintainer_email='tejasps28@gmail.com',
    description='Mission validation, route compilation, and planner abstraction for the Omokai core pipeline.',
    license='Apache-2.0',
)
