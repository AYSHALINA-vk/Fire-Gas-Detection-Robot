#!/usr/bin/env python3
import math
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(nav, x, y, yaw_deg):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    yaw = math.radians(yaw_deg)
    p.pose.orientation.z = math.sin(yaw / 2)
    p.pose.orientation.w = math.cos(yaw / 2)
    return p


def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.setInitialPose(make_pose(nav, 0.0, 0.0, 0.0))
    nav.waitUntilNav2Active()

    points = [
        (2.5, 0.5, 90),     # near fire (fire is at 2.5, 1.5)
        (3.5, -3.0, 180),   # east side
        (-2.5, -0.5, -90),  # near gas (gas is at -2.5, -1.5)
        (-3.5, 3.0, 0),     # north-west
        (0.0, 0.0, 0),      # home
    ]
    waypoints = [make_pose(nav, *p) for p in points]

    nav.followWaypoints(waypoints)
    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        if fb:
            print(f'Heading to waypoint {fb.current_waypoint + 1}/{len(waypoints)}')

    result = nav.getResult()
    print({TaskResult.SUCCEEDED: 'Patrol complete',
           TaskResult.CANCELED: 'Canceled',
           TaskResult.FAILED: 'Failed'}.get(result, 'Unknown'))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
