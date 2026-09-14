from setuptools import find_packages, setup

package_name = 'drillpulse_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vicky',
    maintainer_email='vicky06237@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          'joystick_sender = drillpulse_control.joystick_sender:main',
          'virtual_odometry = drillpulse_control.virtual_odometry:main',
          'map_visualizer = drillpulse_control.map_visualizer:main',
	  'joystick = drillpulse_control.drillpulse_rover_node:main',
          'central = drillpulse_control.cam_node:main',
        ],
    },
)
