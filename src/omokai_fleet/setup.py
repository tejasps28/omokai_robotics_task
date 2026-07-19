from setuptools import find_packages, setup


package_name = 'omokai_fleet'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Tejas',
    maintainer_email='tejasps28@gmail.com',
    description='Deterministic models and coordination for a robot fleet.',
    license='Apache-2.0',
)
