from glob import glob

from setuptools import find_packages, setup

package_name = "sailboat_simulation"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("lib/python3.10/site-packages/" + package_name, glob("wind_data/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="autoboatvt",
    maintainer_email="autoboat@vt.edu",
    description="Contains a ros2 compatible version of the usv sim lsa sailboat simulation",
    license="http://www.apache.org/licenses/LICENSE-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["simulation = sailboat_simulation.simulation_node:main"],
    },
)
