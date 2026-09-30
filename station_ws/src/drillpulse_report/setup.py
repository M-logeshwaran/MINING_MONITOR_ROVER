from setuptools import find_packages, setup

package_name = 'drillpulse_report'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DrillPulse Team',
    maintainer_email='rover@drillpulse.local',
    description='Automated Mission, Hazard, and Environmental Incident Reporting System for DrillPulse Mine Rover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'report_generator_node = drillpulse_report.report_generator_node:main',
        ],
    },
)
