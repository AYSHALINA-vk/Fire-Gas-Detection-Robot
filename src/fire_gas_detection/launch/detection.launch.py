from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fire_gas_detection',
            executable='detection_node',
            name='detection_node',
            output='screen',
            parameters=[{
                'flame_threshold': 20.0,
                'gas_threshold': 20.0,
            }],
        ),
    ])
