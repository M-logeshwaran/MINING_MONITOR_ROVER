import os
from glob import glob
from setuptools import find_packages, setup

package_name = "rover_dashboard"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(include=["rover_dashboard", "rover_dashboard.*"], exclude=["test"]),
    package_data={package_name: ["web/*", "web/**/*"]},
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "flask", "flask-socketio", "python-socketio"],
    zip_safe=False,
    maintainer="Vicky",
    maintainer_email="you@example.com",
    description="Web-based liquid-glass dashboard for mine rover telemetry.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dashboard = rover_dashboard.dashboard:main",
            "thermal = rover_dashboard.thermal:main",
        ],
    },
)
