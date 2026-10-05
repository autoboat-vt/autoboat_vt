import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'simulation_models'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='autoboatvt',
    maintainer_email='autoboat@vt.edu',
    description='SDF models and resources for the VT AutoBoat simulation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
