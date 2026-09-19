from setuptools import find_packages, setup

package_name = 'fire_gas_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='treesa',
    maintainer_email='your@email.com',
    description='Simulated fire and gas sensors',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_simulator = fire_gas_sensors.sensor_simulator:main',
        ],
    },
)
