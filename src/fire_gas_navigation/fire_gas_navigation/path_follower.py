import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, TransformException


class PathFollower(Node):

    def __init__(self):
        super().__init__('path_follower')

        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('lookahead_distance', 0.25)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_gain', 1.5)
        self.declare_parameter('goal_tolerance', 0.12)

        self.robot_frame = self.get_parameter(
            'robot_frame').value
        self.lookahead = float(self.get_parameter(
            'lookahead_distance').value)
        self.linear_speed = float(self.get_parameter(
            'linear_speed').value)
        self.angular_gain = float(self.get_parameter(
            'angular_gain').value)
        self.goal_tolerance = float(self.get_parameter(
            'goal_tolerance').value)

        self.path = []
        self.goal = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self)

        self.path_sub = self.create_subscription(
            Path, '/planned_path', self.path_callback, 10)

        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(
            0.1, self.follow_path)

        self.get_logger().info(
            'Path follower ready; waiting for /planned_path')

    def path_callback(self, msg):
        self.path = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]

        self.goal = self.path[-1] if self.path else None

        if self.path:
            self.get_logger().info(
                f'Received path with {len(self.path)} points')
        else:
            self.stop_robot()

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def follow_path(self):
        if not self.path or self.goal is None:
            self.stop_robot()
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                self.robot_frame,
                rclpy.time.Time()
            )
        except TransformException:
            self.stop_robot()
            return

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y

        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        goal_distance = math.hypot(
            self.goal[0] - robot_x,
            self.goal[1] - robot_y
        )

        if goal_distance <= self.goal_tolerance:
            self.stop_robot()
            self.path = []
            self.goal = None
            self.get_logger().info('Goal reached!')
            return

        target = self.path[-1]

        for point in self.path:
            distance = math.hypot(
                point[0] - robot_x,
                point[1] - robot_y
            )
            if distance >= self.lookahead:
                target = point
                break

        target_angle = math.atan2(
            target[1] - robot_y,
            target[0] - robot_x
        )

        angle_error = math.atan2(
            math.sin(target_angle - yaw),
            math.cos(target_angle - yaw)
        )

        cmd = Twist()
        cmd.angular.z = max(
            -1.0,
            min(1.0, self.angular_gain * angle_error)
        )

        if abs(angle_error) > 0.6:
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = self.linear_speed * max(
                0.2, 1.0 - abs(angle_error)
            )

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
