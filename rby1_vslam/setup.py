from glob import glob
from setuptools import find_packages, setup

PACKAGE = 'rby1_vslam'

setup(
    name=PACKAGE,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml', 'README.md']),
        ('share/' + PACKAGE + '/launch', glob('launch/*.launch.py')),
        ('share/' + PACKAGE + '/config', glob('config/*.yaml') + glob('config/*.xml')),
        ('share/' + PACKAGE + '/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RBY1 project',
    maintainer_email='maintainer@example.com',
    description='Humble UPC / Jazzy LAB visual SLAM bridge and Nav2 integration',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'bridge_node = rby1_vslam.bridge_node:main',
        'pose_adapter = rby1_vslam.pose_adapter:main',
        'localization_tf = rby1_vslam.localization_tf:main',
        'nav2_gate = rby1_vslam.nav2_gate:main',
        'map_tool = rby1_vslam.map_tool:main',
    ]},
)
