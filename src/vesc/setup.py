from setuptools import find_packages, setup

package_name = "vesc"

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
    description="Contains an autoboat compatible driver for any VESC based motor controller",
    license="http://www.apache.org/licenses/LICENSE-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["vesc = vesc.vesc_publisher:main"],
    },
)
