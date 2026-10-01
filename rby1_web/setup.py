from glob import glob
from setuptools import find_packages, setup

setup(
    name='rby1_web', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/rby1_web']),
        ('share/rby1_web', ['package.xml', 'README.md']),
        ('share/rby1_web/launch', glob('launch/*.launch.py')),
    ],
    package_data={'rby1_web': ['static/*.html']},
    install_requires=['setuptools'], zip_safe=False,
    maintainer='RB-Y1 Developer', maintainer_email='user@example.com',
    description='Headless mobile-base web operator for UPC.', license='Apache-2.0',
    entry_points={'console_scripts': [
        'mobile_base_web = rby1_web.web_node:main',
        'operator_web = rby1_web.web_node:main',
    ]},
)
