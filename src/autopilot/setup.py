from glob import glob

from setuptools import find_packages, setup

package_name = "autopilot"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="autoboatvt",
    maintainer_email="autoboat@vt.edu",
    description="Contains all of the ros2 nodes necessary to run the autopilot software and telemetry software",
    license="http://www.apache.org/licenses/LICENSE-2.0",
    entry_points={
        'console_scripts': [
            'sailboat_autopilot = autopilot.sailboat_autopilot_node:main',
            'motorboat_autopilot = autopilot.motorboat_autopilot_node:main',
            'telemetry = autopilot.telemetry_node:main',
            'pathfinding = autopilot.sailboat_pathfinding_node:main',
            'obstacles = autopilot.sailboat_obstacles_node:main'
        ],
    },
)