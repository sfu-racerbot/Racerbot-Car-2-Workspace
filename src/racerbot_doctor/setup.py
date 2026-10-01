from setuptools import find_packages, setup

package_name = 'racerbot_doctor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racerbotcar-2',
    maintainer_email='bryanmaubc@gmail.com',
    description='Read-only self-check of the car hardware and the bringup layer.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'doctor = racerbot_doctor.cli:main',
        ],
    },
)
