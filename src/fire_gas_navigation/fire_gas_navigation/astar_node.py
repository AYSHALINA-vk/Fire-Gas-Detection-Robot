import heapq
import math
import os
import yaml
import numpy as np

from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path

from tf2_ros import Buffer, TransformListener, TransformException


class AStarNode(Node):

    def __init__(self):
        super().__init__('astar_node')

        self.declare_parameter(
            'map_yaml',
            os.path.expanduser(
                '~/fire_gas/maps/fire_robot_map.yaml'
            )
        )
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')

        self.map_frame = self.get_parameter(
            'map_frame'
        ).value

        self.robot_frame = self.get_parameter(
            'robot_frame'
        ).value

        # Map data
        self.map_data = None
        self.free_cells = None
        self.width = 0
        self.height = 0
        self.resolution = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_yaw = 0.0

        # Create the map publisher BEFORE loading the map.
        # Transient-local durability lets late subscribers
        # receive the most recently published map.
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            map_qos
        )

        self.path_pub = self.create_publisher(
            Path,
            '/planned_path',
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # TF is used to determine the robot's current position.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # Load the saved occupancy map.
        map_yaml = os.path.expanduser(
            self.get_parameter('map_yaml').value
        )

        self.load_map(map_yaml)

        self.get_logger().info(
            'Custom A* planner is ready. '
            'Send a goal on /goal_pose.'
        )

    # --------------------------------------------------
    # LOAD MAP
    # --------------------------------------------------

    def load_map(self, yaml_path):

        if not os.path.isfile(yaml_path):
            raise FileNotFoundError(
                f'Map YAML not found: {yaml_path}'
            )

        with open(yaml_path, 'r') as file:
            config = yaml.safe_load(file)

        image_path = config['image']

        # Resolve relative image paths from the YAML directory.
        if not os.path.isabs(image_path):
            image_path = os.path.join(
                os.path.dirname(yaml_path),
                image_path
            )

        if not os.path.isfile(image_path):
            raise FileNotFoundError(
                f'Map image not found: {image_path}'
            )

        image = Image.open(image_path).convert('L')

        # OccupancyGrid row zero is the bottom row.
        gray = np.flipud(
            np.asarray(image, dtype=np.uint8)
        )

        self.resolution = float(config['resolution'])

        origin = config['origin']
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])
        self.origin_yaw = float(origin[2])

        negate = int(config.get('negate', 0))
        occupied_thresh = float(
            config.get('occupied_thresh', 0.65)
        )
        free_thresh = float(
            config.get('free_thresh', 0.25)
        )

        # Convert grayscale pixels into occupancy probabilities.
        if negate == 0:
            probability = (255.0 - gray) / 255.0
        else:
            probability = gray / 255.0

        self.height, self.width = gray.shape

        # Start with unknown cells. Unknown cells are blocked
        # during planning to avoid paths through uncertain areas.
        self.map_data = np.full(
            (self.height, self.width),
            -1,
            dtype=np.int8
        )

        free = probability < free_thresh
        occupied = probability > occupied_thresh

        self.map_data[free] = 0
        self.map_data[occupied] = 100

        self.free_cells = (self.map_data == 0)

        self.publish_map()

        self.get_logger().info(
            f'Map loaded: {self.width} x {self.height} cells, '
            f'resolution={self.resolution:.3f} m/cell'
        )

    # --------------------------------------------------
    # PUBLISH MAP
    # --------------------------------------------------

    def publish_map(self):

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height

        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0

        msg.info.origin.orientation.z = math.sin(
            self.origin_yaw / 2.0
        )
        msg.info.origin.orientation.w = math.cos(
            self.origin_yaw / 2.0
        )

        # Rows are already ordered bottom-to-top.
        msg.data = self.map_data.flatten().tolist()

        self.map_pub.publish(msg)

    # --------------------------------------------------
    # WORLD COORDINATES <-> GRID COORDINATES
    # --------------------------------------------------

    def world_to_grid(self, world_x, world_y):

        dx = world_x - self.origin_x
        dy = world_y - self.origin_y

        # Rotate world coordinates into the map's local frame.
        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)

        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy

        col = int(math.floor(local_x / self.resolution))
        row = int(math.floor(local_y / self.resolution))

        if not (
            0 <= col < self.width
            and 0 <= row < self.height
        ):
            return None

        return (row, col)

    def grid_to_world(self, cell):

        row, col = cell

        local_x = (col + 0.5) * self.resolution
        local_y = (row + 0.5) * self.resolution

        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)

        world_x = (
            self.origin_x + c * local_x - s * local_y
        )
        world_y = (
            self.origin_y + s * local_x + c * local_y
        )

        return world_x, world_y

    # --------------------------------------------------
    # A* ALGORITHM
    # --------------------------------------------------

    def heuristic(self, a, b):
        return math.hypot(
            a[0] - b[0],
            a[1] - b[1]
        )

    def astar(self, start, goal):

        if not self.free_cells[start]:
            self.get_logger().error(
                'The robot start position is not in a known '
                'free cell.'
            )
            return None

        if not self.free_cells[goal]:
            self.get_logger().error(
                'The goal is occupied, unknown, or outside '
                'the known free space.'
            )
            return None

        # Eight possible movement directions.
        directions = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)),
            (1, 1, math.sqrt(2)),
        ]

        open_set = []
        counter = 0

        heapq.heappush(
            open_set,
            (self.heuristic(start, goal), counter, start)
        )

        came_from = {}
        g_score = {start: 0.0}
        closed_set = set()

        while open_set:

            _, _, current = heapq.heappop(open_set)

            if current in closed_set:
                continue

            if current == goal:
                result = [current]

                while current in came_from:
                    current = came_from[current]
                    result.append(current)

                result.reverse()
                return result

            closed_set.add(current)

            row, col = current

            for dr, dc, move_cost in directions:

                nr = row + dr
                nc = col + dc
                neighbor = (nr, nc)

                if not (
                    0 <= nr < self.height
                    and 0 <= nc < self.width
                ):
                    continue

                if not self.free_cells[nr, nc]:
                    continue

                # Prevent diagonal movement through obstacle corners.
                if dr != 0 and dc != 0:
                    if not self.free_cells[row + dr, col]:
                        continue

                    if not self.free_cells[row, col + dc]:
                        continue

                if neighbor in closed_set:
                    continue

                tentative_g = (
                    g_score[current] + move_cost
                )

                if tentative_g < g_score.get(
                    neighbor, float('inf')
                ):

                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g

                    f_score = (
                        tentative_g
                        + self.heuristic(neighbor, goal)
                    )

                    counter += 1

                    heapq.heappush(
                        open_set,
                        (f_score, counter, neighbor)
                    )

        self.get_logger().error(
            'A* could not find a path to the goal.'
        )

        return None

    # --------------------------------------------------
    # RECEIVE GOAL AND PLAN PATH
    # --------------------------------------------------

    def goal_callback(self, msg):

        goal_frame = msg.header.frame_id

        if goal_frame and goal_frame != self.map_frame:
            self.get_logger().error(
                f'Goal frame is "{goal_frame}", but the planner '
                f'expects "{self.map_frame}". Send the goal '
                f'in the map frame.'
            )
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time()
            )

        except TransformException as ex:
            self.get_logger().error(
                f'Cannot find transform '
                f'{self.map_frame} -> {self.robot_frame}: {ex}'
            )
            return

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y

        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y

        start_cell = self.world_to_grid(robot_x, robot_y)
        goal_cell = self.world_to_grid(goal_x, goal_y)

        if start_cell is None:
            self.get_logger().error(
                'Robot position lies outside the saved map.'
            )
            return

        if goal_cell is None:
            self.get_logger().error(
                'Goal lies outside the saved map.'
            )
            return

        self.get_logger().info(
            f'Planning from {start_cell} to {goal_cell}'
        )

        cells = self.astar(start_cell, goal_cell)

        if cells is None:
            # Publish an empty path so an old path is not followed.
            empty_path = Path()
            empty_path.header.frame_id = self.map_frame
            empty_path.header.stamp = (
                self.get_clock().now().to_msg()
            )
            self.path_pub.publish(empty_path)
            return

        self.publish_path(cells)

    # --------------------------------------------------
    # PUBLISH PLANNED PATH
    # --------------------------------------------------

    def publish_path(self, cells):

        path_msg = Path()
        path_msg.header.frame_id = self.map_frame
        path_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        for cell in cells:

            world_x, world_y = self.grid_to_world(cell)

            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.header.stamp = path_msg.header.stamp

            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0

            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

        self.get_logger().info(
            f'A* found a path with {len(cells)} cells. '
            'Published on /planned_path.'
        )


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:
        node = AStarNode()
        rclpy.spin(node)

    except (FileNotFoundError, KeyError, ValueError) as ex:
        print(f'Failed to load or configure map: {ex}')

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
