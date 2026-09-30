import os
from glob import glob
from setuptools import find_packages, setup
package_name = 'drillpulse_odometry'
setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DrillPulse Team',
    maintainer_email='loki@drillpulse.local',
    description='Odometry estimation and threshold consistency package for DrillPulse Rover',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_odometry_node = drillpulse_odometry.wheel_odometry_node:main',
            'imu_odometry_node = drillpulse_odometry.imu_odometry_node:main',
            'threshold_consistency_node = drillpulse_odometry.threshold_consistency_node:main',
            'odometry_manager_node = drillpulse_odometry.odometry_manager_node:main',
        ],
    },
)