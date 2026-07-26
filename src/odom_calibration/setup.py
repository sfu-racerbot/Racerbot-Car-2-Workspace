from setuptools import find_packages, setup


package_name = 'odom_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/odom_calibration_launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/odom_calibration.yaml',
        ]),
        ('share/' + package_name + '/web', [
            'web/index.html',
            'web/wizard.js',
            'web/style.css',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racerbotcar-2',
    maintainer_email='bryanmaubc@gmail.com',
    description='Read-only guided odometry and steering calibration wizard.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration_node = odom_calibration.calibration_node:main',
        ],
    },
)
