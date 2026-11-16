from glob import glob
from os.path import isfile
from setuptools import find_packages, setup

PACKAGE = 'rby1_vslam'
SHELL_SCRIPTS = [
    'recorder.sh',
    'run_d435i_host.sh',
    'run_isaac_vslam_docker.sh',
    'run_mobile_base.sh',
    'run_navigation.sh',
    'run_nvblox_docker.sh',
    'run_nvblox_rviz.sh',
    'run_handheld_nvblox.sh',
]

setup(
    name=PACKAGE,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml', 'README.md']),
        ('share/' + PACKAGE + '/launch', glob('launch/*.launch.py')),
        ('share/' + PACKAGE + '/config',
         glob('config/*.yaml') + glob('config/*.xml') + glob('config/*.rviz')),
        ('share/' + PACKAGE + '/data', glob('data/*.yaml')),
        ('share/' + PACKAGE + '/docs', [path for path in glob('docs/*.md') if isfile(path)]),
        ('share/' + PACKAGE + '/scripts', SHELL_SCRIPTS),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RBY1 project',
    maintainer_email='maintainer@example.com',
    description='RBY1 station navigation with wheel odometry and stationary visual correction',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'tracking_monitor = rby1_vslam.tracking_monitor:main',
        'median_filter = rby1_vslam.median_filter:main',
        'pose_adapter = rby1_vslam.pose_adapter:main',
        'map_origin_initializer = rby1_vslam.map_origin_initializer:main',
        'localization_tf = rby1_vslam.localization_tf:main',
        'wheel_odom_adapter = rby1_vslam.wheel_odom_adapter:main',
        'nav2_gate = rby1_vslam.nav2_gate:main',
        'map_tool = rby1_vslam.map_tool:main',
        'nvblox_map_tool = rby1_vslam.nvblox_map_tool:main',
        'waypoint_ui = rby1_vslam.waypoint_ui:main',
        'mobile_base_qt = rby1_vslam.mobile_base_qt:main',
    ]},
)
