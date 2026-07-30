from setuptools import find_packages, setup

package_name = 'race_diagnostics'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/record_run.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/race_diagnostics.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racerbotcar-2',
    maintainer_email='bryanmaubc@gmail.com',
    description='Read-only run recorder and post-run analyzer for the driving stack.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'race_diag_node = race_diagnostics.race_diag_node:main',
            'filter_log = race_diagnostics.filter_log:main',
            'summarize_run = race_diagnostics.summarize_run:main',
        ],
    },
)
