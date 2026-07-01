from setuptools import find_packages, setup

package_name = 'web_dashboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/web_dashboard_launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/web_dashboard.yaml',
        ]),
        ('share/' + package_name + '/web', [
            'web/index.html',
            'web/dashboard.js',
            'web/style.css',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racerbotcar-2',
    maintainer_email='bryanmaubc@gmail.com',
    description="Live browser dashboard of the car's map, LIDAR scan, and pose.",
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_node = web_dashboard.dashboard_node:main',
        ],
    },
)
