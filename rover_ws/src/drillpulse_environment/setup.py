import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'drillpulse_environment'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DrillPulse Team',
    maintainer_email='rover@drillpulse.local',
    description='DrillPulse Mine Rover drillpulse_environment',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'environment_monitor_node = drillpulse_environment.environment_monitor_node:main'
        ],
    },
)
