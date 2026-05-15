from setuptools import find_packages, setup

package_name = "drivers"

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
    description="Contains a library of autoboat compatible drivers",
    license="http://www.apache.org/licenses/LICENSE-2.0",
    entry_points={
        "console_scripts": [
            "gps = drivers.gps.gps:main",
            "wind_sensor = drivers.wind_sensor.wind_sensor:main",
            "rc = drivers.rc.rc:main",
            "jetson_stats = drivers.jetson.jetson_stats:main",
            "rc_keyboard_emulator = drivers.rc.rc_keyboard_emulator:main",
            "vesc = drivers.motor_controller.vesc:main",
        ],
    },
)
