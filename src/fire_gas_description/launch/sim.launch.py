import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('fire_gas_description')

    world_file = os.path.join(
        pkg_share,
        'worlds',
        'warehouse.world'
    )

    urdf_file = os.path.join(
        pkg_share,
        'urdf',
        'robot.urdf'
    )

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                '/opt/ros/humble/share/gazebo_ros',
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_file
        }.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description}
        ],
        output='screen'
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'fire_gas_robot',
            '-x', '0',
            '-y', '0',
            '-z', '0.2'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot
    ])
