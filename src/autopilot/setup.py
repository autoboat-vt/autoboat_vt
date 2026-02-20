from setuptools import find_packages, setup
from glob import glob


package_name = "autopilot"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="animated",
    maintainer_email="73669160+ChrisNassif@users.noreply.github.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sailboat_autopilot = autopilot.sailboat_autopilot_node:main",
            "motorboat_autopilot = autopilot.motorboat_autopilot_node:main",
            "telemetry = autopilot.telemetry_node:main",
        ],
    },
)
