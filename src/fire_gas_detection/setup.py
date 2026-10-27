import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'fire_gas_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='treesa',
    maintainer_email='treesajohnny345@gmail.com',
    description='Fire and gas detection and alert node',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'detection_node = fire_gas_detection.detection_node:main',
        ],
    },
)
