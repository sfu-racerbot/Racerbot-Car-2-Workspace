from setuptools import find_packages, setup

package_name = 'drive_intent'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Shipped so the racerbot_a/racerbot_b C++ nodes can include the
        # same schema this workspace's Python nodes publish, rather than
        # each team re-deriving the field names from the docs.
        ('include/' + package_name, ['include/drive_intent/drive_intent.hpp']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racerbotcar-2',
    maintainer_email='bryanmaubc@gmail.com',
    description='Shared schema and trajectory prediction for /drive_intent: '
                'what a driving algorithm is trying to do, and why',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
