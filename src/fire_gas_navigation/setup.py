
from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'fire_gas_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anavadhya',
    maintainer_email='anavadhya@todo.todo',
    description='Waypoint navigation for the fire and gas detection robot',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'waypoint_navigator = fire_gas_navigation.waypoint_navigator:main',
        ],
    },
)

