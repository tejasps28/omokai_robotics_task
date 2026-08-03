from setuptools import find_packages, setup


package_name = 'omokai_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tejas',
    maintainer_email='tejasps28@gmail.com',
    description='Local object detection for the Omokai vision challenge.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'person_detector = omokai_perception.person_detector:main',
        ],
    },
)
