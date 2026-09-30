from setuptools import find_packages, setup

package_name = 'drillpulse_transport'

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
    maintainer_email='loki@drillpulse.local',
    description='LoRa mesh command and telemetry transport package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lora_transceiver_node = drillpulse_transport.lora_transceiver_node:main',
        ],
    },
)
