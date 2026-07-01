from setuptools import find_packages, setup

package_name = 'gap_follow'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gap_follow_launch.py']),
        ('share/' + package_name + '/config', ['config/gap_follow.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racerbotcar-2',
    maintainer_email='bryanmaubc@gmail.com',
    description='Reactive follow-the-gap baseline autonomous driving algorithm for the race car',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gap_follow_node = gap_follow.gap_follow_node:main',
        ],
    },
)
