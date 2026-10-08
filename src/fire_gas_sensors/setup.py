from setuptools import setup

package_name = 'fire_gas_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='treesa',
    maintainer_email='treesa@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_simulator = fire_gas_sensors.sensor_simulator:main',
            'distance_sensor_simulator = fire_gas_sensors.distance_sensor_simulator:main',
        ],
    },
)
