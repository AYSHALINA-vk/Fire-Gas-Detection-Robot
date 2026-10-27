from setuptools import setup
from glob import glob
import os

package_name = 'fire_gas_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'maps'),
            glob('maps/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lenovo',
    maintainer_email='user@example.com',
    description='Custom A* path planning and robot path following',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astar_planner = fire_gas_navigation.astar_planner:main',
            'astar_node = fire_gas_navigation.astar_node:main',
            'path_follower = fire_gas_navigation.path_follower:main',
        ],
    },
)
