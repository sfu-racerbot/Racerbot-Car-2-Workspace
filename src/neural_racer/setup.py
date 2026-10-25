from setuptools import find_packages, setup

package_name = 'neural_racer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/neural_racer_launch.py']),
        ('share/' + package_name + '/config', ['config/neural_racer.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racerbotcar-1',
    maintainer_email='bryanmaubc@gmail.com',
    description='Learned racing policy inside a deterministic safety wrapper, with the LB deadman',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'neural_racer_node = neural_racer.neural_racer_node:main',
            'approve_model = neural_racer.approve_model:main',
        ],
    },
)
