from setuptools import find_packages, setup


package_name = 'omokai_interfaces'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['schema/*.json']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tejas',
    maintainer_email='tejasps28@gmail.com',
    description='Versioned mission-domain contracts for the Omokai core pipeline.',
    license='Apache-2.0',
)
