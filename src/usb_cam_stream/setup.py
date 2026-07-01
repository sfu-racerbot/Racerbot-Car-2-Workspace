from setuptools import find_packages, setup

package_name = 'usb_cam_stream'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/usb_cam_stream_launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/usb_cam_stream.yaml',
        ]),
        ('share/' + package_name + '/web', [
            'web/index.html',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racerbotcar-2',
    maintainer_email='bryanmaubc@gmail.com',
    description='Live MJPEG video stream from a USB webcam, served over plain HTTP.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_stream_node = usb_cam_stream.camera_stream_node:main',
        ],
    },
)
