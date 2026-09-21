from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'rby1_navigation'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            os.path.join('share', package_name),
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='physrobotics4',
    maintainer_email='physrobotics4@example.com',
    description='Odometry closed-loop navigation for RBY1',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node = rby1_navigation.navigation_node:main',
        ],
    },
)
