from glob import glob

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
        # Globbed, NOT listed by hand. These were an explicit list until
        # adding web/panels.js to the page and forgetting to add it here
        # too -- which installs a dashboard whose index.html asks for a
        # script that was never copied, so the browser 404s it and every
        # feature in that file silently does not exist. Nothing about the
        # page looks broken; the missing behaviour just never appears.
        # A glob cannot forget. test_packaging.py holds the line.
        ('share/' + package_name + '/web',
            glob('web/*.html') + glob('web/*.js') + glob('web/*.css')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racerbotcar-2',
    maintainer_email='bryanmaubc@gmail.com',
    description="Read-only live dashboard with vehicle telemetry and camera overlay.",
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_node = web_dashboard.dashboard_node:main',
        ],
    },
)
