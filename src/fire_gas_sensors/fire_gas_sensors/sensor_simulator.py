import math
import random

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry


class SensorSimulator(Node):

    def __init__(self):
        super().__init__('sensor_simulator')

        self.robot_x = 0.0
        self.robot_y = 0.0

        # Hazard positions in Gazebo
        self.fire_x = 2.5
        self.fire_y = 1.5

        self.gas_x = -2.5
        self.gas_y = -1.5

        self.flame_pub = self.create_publisher(
            Float32,
            '/flame_reading',
            10
        )

        self.gas_pub = self.create_publisher(
            Float32,
            '/gas_reading',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.publish_sensor_data
        )

        self.get_logger().info('Sensor simulator started')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def distance(self, x1, y1, x2, y2):
        return math.sqrt(
            (x1 - x2) ** 2 +
            (y1 - y2) ** 2
        )

    def publish_sensor_data(self):

        fire_distance = self.distance(
            self.robot_x,
            self.robot_y,
            self.fire_x,
            self.fire_y
        )

        gas_distance = self.distance(
            self.robot_x,
            self.robot_y,
            self.gas_x,
            self.gas_y
        )

        # Distance-based sensor model
        flame = 100.0 / (1.0 + fire_distance ** 2)
        gas = 100.0 / (1.0 + gas_distance ** 2)

        # Simulated sensor noise
        flame += random.uniform(-2.0, 2.0)
        gas += random.uniform(-2.0, 2.0)

        # Prevent negative readings
        flame = max(0.0, flame)
        gas = max(0.0, gas)

        flame_msg = Float32()
        flame_msg.data = float(flame)

        gas_msg = Float32()
        gas_msg.data = float(gas)

        self.flame_pub.publish(flame_msg)
        self.gas_pub.publish(gas_msg)


def main(args=None):

    rclpy.init(args=args)

    node = SensorSimulator()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
