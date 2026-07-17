from setuptools import find_packages, setup


package_name = 'omokai_pipeline'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Tejas',
    maintainer_email='tejasps28@gmail.com',
    description='Application-layer orchestration for the Omokai core mission pipeline.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'omokai-pipeline-preview = omokai_pipeline.preview:main',
            'omokai-operator = omokai_pipeline.operator_cli:main',
        ],
    },
)
