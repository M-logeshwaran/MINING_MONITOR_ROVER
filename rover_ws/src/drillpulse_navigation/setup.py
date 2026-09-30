import os
from glob import glob
from setuptools import find_packages, setup
package_name = 'drillpulse_navigation'
setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DrillPulse Team',
    maintainer_email='loki@drillpulse.local',
    description='Nav2 configuration and SLAM/AMCL launch integration for DrillPulse',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)