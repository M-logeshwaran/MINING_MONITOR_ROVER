from setuptools import find_packages, setup
package_name = 'drillpulse_imu'
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
    description='SparkFun ICM-20948 IMU driver node with gyro calibration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sparkfun_imu_node = drillpulse_imu.sparkfun_imu_node:main',
        ],
    },
)