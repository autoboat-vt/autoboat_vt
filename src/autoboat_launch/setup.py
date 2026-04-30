import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'autoboat_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('autoboat_launch', '*launch.[pxy][yma]*')))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='animated',
    maintainer_email='autoboat@vt.edu',
    description='Launch files for AutoBoat VT',
    license='http://www.apache.org/licenses/LICENSE-2.0',
    extras_require={},
    entry_points={
        'console_scripts': [
        ],
    },
)
