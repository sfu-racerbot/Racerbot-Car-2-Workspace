from setuptools import find_packages, setup


package_name = 'imu_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/imu_bridge_launch.py',
            'launch/imu_ekf_shadow_launch.py',
            'launch/vehicle_state_launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/imu_bridge.yaml',
            'config/ekf_shadow.yaml',
            'config/vehicle_state.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racerbotcar-2',
    maintainer_email='bryanmaubc@gmail.com',
    description='Calibrated IMU topics from the VESC IMU and an optional BNO085, '
                'fused speed and measured steering angle (no /drive publisher).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_bridge_node = imu_bridge.imu_bridge_node:main',
            'vehicle_state_node = imu_bridge.vehicle_state_node:main',
        ],
    },
)
