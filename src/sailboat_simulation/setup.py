from glob import glob

from setuptools import find_packages, setup

package_name = "sailboat_simulation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("lib/python3.10/site-packages/" + package_name, glob("wind_data/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="animated",
    maintainer_email="73669160+ChrisNassif@users.noreply.github.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["simulation = sailboat_simulation.simulation_node:main"],
    },
)
