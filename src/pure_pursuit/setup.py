from setuptools import find_packages, setup

package_name = 'pure_pursuit'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/pure_pursuit_launch.py',
            'launch/waypoint_recorder_launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/pure_pursuit.yaml',
            'config/waypoint_recorder.yaml',
        ]),
        ('share/' + package_name + '/waypoints', [
            'waypoints/example_stadium_raw.csv',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racerbotcar-2',
    maintainer_email='bryanmaubc@gmail.com',
    description='Map-based race controller: pure pursuit over a curvature-aware velocity profile, '
                'plus tools to record and pace a racing line.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit_node = pure_pursuit.pure_pursuit_node:main',
            'waypoint_recorder_node = pure_pursuit.waypoint_recorder_node:main',
            'generate_velocity_profile = pure_pursuit.generate_velocity_profile:main',
        ],
    },
)
