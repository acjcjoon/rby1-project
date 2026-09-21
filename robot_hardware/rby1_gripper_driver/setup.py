from glob import glob

from setuptools import find_packages, setup


package_name = 'rby1_gripper_driver'


setup(
    name=package_name,
    version='0.4.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RB-Y1 Developer',
    maintainer_email='user@example.com',
    description=(
        'ROS 2 driver for real and Isaac-simulated RBY1 grippers.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_driver = rby1_gripper_driver.driver_node:main',
            'gripper_debug_controller = '
            'rby1_gripper_driver.debug_controller:main',
        ],
    },
)
