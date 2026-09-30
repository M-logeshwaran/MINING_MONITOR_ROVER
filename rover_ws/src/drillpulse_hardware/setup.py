from setuptools import find_packages, setup
package_name = 'drillpulse_hardware'
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
    description='Hardware bridges for STM32 encoders, Arduino motors, and linear actuators',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stm32_encoder_bridge = drillpulse_hardware.stm32_encoder_bridge:main',
            'motor_telemetry_bridge = drillpulse_hardware.motor_telemetry_bridge:main',
            'actuator_bridge = drillpulse_hardware.actuator_bridge:main',
        ],
    },
)