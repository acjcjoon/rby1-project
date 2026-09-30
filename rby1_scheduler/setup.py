from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'rby1_scheduler'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'mock_server'), glob('mock_server/*')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=False,
    maintainer='RB-Y1 Developer',
    maintainer_email='user@example.com',
    description='Priority scheduler and PyQt operator UI for RB-Y1 laboratory automation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'scheduler_node = rby1_scheduler.scheduler_node:main',
        'scheduler_ui = rby1_scheduler.ui.main:main',
    ]},
)
