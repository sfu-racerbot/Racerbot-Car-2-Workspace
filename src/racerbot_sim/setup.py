from setuptools import find_packages, setup

package_name = 'racerbot_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/sim_bringup_launch.py',
            'launch/sim_auto_map_race_launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racerbotcar-2',
    maintainer_email='bryanmaubc@gmail.com',
    description=(
        'Headless F1TENTH Gym simulator exposed as ROS2 topics, so the real '
        'driving stack can be run and validated end to end without the car'),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gym_bridge_node = racerbot_sim.gym_bridge_node:main',
            'sim_joy_node = racerbot_sim.sim_joy_node:main',
        ],
    },
)
