import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'drillpulse_autonomous'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DrillPulse Team',
    maintainer_email='drillpulse@rover.local',
    description='ROS2 autonomous navigation and vision package for DrillPulse Mini Rover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_controller = drillpulse_autonomous.autonomous_controller:main',
            'point_navigation = drillpulse_autonomous.point_navigation_mode:main_cli',
        ],
    },
)
