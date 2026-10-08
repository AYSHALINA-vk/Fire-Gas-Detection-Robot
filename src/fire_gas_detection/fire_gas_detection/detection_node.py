import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')

        self.declare_parameter('flame_threshold', 20.0)
        self.declare_parameter('gas_threshold', 20.0)

        self.flame = None
        self.gas = None
        self.last_state = None

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.declare_parameter('flame_threshold', 20.0)
        self.declare_parameter('gas_threshold', 20.0)
        self.declare_parameter('hysteresis', 3.0)  # Drop below threshold - hysteresis to clear
        self.declare_parameter('consecutive_required', 3)  # Debounce count

        self.flame = None
        self.gas = None
        self.last_state = 'NORMAL'
        self.current_state = 'NORMAL'
        
        # Counters for debouncing
        self.candidate_state = 'NORMAL'
        self.consecutive_count = 0

        self.create_subscription(Float32, '/flame_reading', self.flame_cb, 10)
        self.create_subscription(Float32, '/gas_reading', self.gas_cb, 10)
        self.pub = self.create_publisher(String, '/alert', 10)
        self.create_timer(0.5, self.check)
        self.get_logger().info('Detection node started')

        self.create_timer(0.5, self.check)
        self.get_logger().info('Detection node started with hysteresis & debouncing')


    def flame_cb(self, msg):
        self.flame = msg.data

    def gas_cb(self, msg):
        self.gas = msg.data

    def check(self):
        if self.flame is None or self.gas is None:
            return

        ft = self.get_parameter('flame_threshold').value
        gt = self.get_parameter('gas_threshold').value


        fire = self.flame >= ft
        gas = self.gas >= gt

        if fire and gas:
            state = 'BOTH'
        elif fire:
            state = 'FIRE'
        elif gas:
            state = 'GAS'
        else:
            state = 'NORMAL'

        t = self.get_clock().now().nanoseconds / 1e9
        msg = String()
        msg.data = (f'{state} | Flame: {self.flame:.2f} | '
                    f'Gas: {self.gas:.2f} | Time: {t:.2f}')
        self.pub.publish(msg)

        if state != self.last_state:
            self.get_logger().info(f'State changed: {state}')
            self.last_state = state


        hys = self.get_parameter('hysteresis').value
        required = self.get_parameter('consecutive_required').value

        # Apply hysteresis based on current stable state
        fire_high = self.flame >= (ft if self.current_state not in ['FIRE', 'BOTH'] else ft - hys)
        gas_high = self.gas >= (gt if self.current_state not in ['GAS', 'BOTH'] else gt - hys)

        if fire_high and gas_high:
            new_raw_state = 'BOTH'
        elif fire_high:
            new_raw_state = 'FIRE'
        elif gas_high:
            new_raw_state = 'GAS'
        else:
            new_raw_state = 'NORMAL'

        # Debounce logic: require N consecutive matching readings
        if new_raw_state == self.candidate_state:
            self.consecutive_count += 1
        else:
            self.candidate_state = new_raw_state
            self.consecutive_count = 1

        if self.consecutive_count >= required:
            self.current_state = self.candidate_state

        t = self.get_clock().now().nanoseconds / 1e9
        msg = String()
        msg.data = (f'{self.current_state} | Flame: {self.flame:.2f} | '
                    f'Gas: {self.gas:.2f} | Time: {t:.2f}')
        self.pub.publish(msg)

        if self.current_state != self.last_state:
            self.get_logger().info(f'State changed: {self.current_state}')
            self.last_state = self.current_state

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
