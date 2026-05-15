from setuptools import find_packages, setup

package_name = 'simulation_transform'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer="autoboatvt",
    maintainer_email="autoboat@vt.edu",
    description="Transforms the data that the simulation natively outputs into formats/ units/ standards the autopilot can easily understand",
    license="http://www.apache.org/licenses/LICENSE-2.0",
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "simulation_transform = simulation_transform.motorboat_simulation_transform_node:main"
        ],
    },
)
