import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'drillpulse_safety'

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
    description='DrillPulse Mine Rover drillpulse_safety',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_monitor_node = drillpulse_safety.safety_monitor_node:main'
        ],
    },
)
