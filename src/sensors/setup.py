from setuptools import find_packages, setup

package_name = "sensors"

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
    maintainer="autoboatvt",
    maintainer_email="autoboat@vt.edu",
    description="Contains a library of autoboat compatible sensor drivers",
    license="http://www.apache.org/licenses/LICENSE-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gps = sensors.gps_publisher:main",
            "wind_sensor = sensors.wind_sensor_publisher:main",
            "rc = sensors.rc_publisher:main",
            "jetson_stats = sensors.jetson_stats_publisher:main",
            "rc_emulator_keyboard = sensors.rc_emulator_keyboard_publisher:main",
        ],
    },
)
