import os
from glob import glob
from setuptools import find_packages, setup
package_name = 'drillpulse_mission'
setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DrillPulse Team',
    maintainer_email='loki@drillpulse.local',
    description='Mission manager and frontier exploration for DrillPulse',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager_node = drillpulse_mission.mission_manager_node:main',
            'frontier_exploration_node = drillpulse_mission.frontier_exploration_node:main',
        ],
    },
)