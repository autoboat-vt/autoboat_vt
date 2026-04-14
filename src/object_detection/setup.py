from setuptools import find_packages, setup

package_name = "object_detection"

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
    description="Contains nodes that perform object detection/ localization on marine objects such as boats and buoys",
    license="http://www.apache.org/licenses/LICENSE-2.0",
    entry_points={
        "console_scripts": [
            # 'object_detection = object_detection.buoy_detection_node:main'
            "object_detection = object_detection.deepstream_buoy_detection_node:main",
            'cam_corder = object_detection.camera_recorder_node:main'
        ],
    },
)