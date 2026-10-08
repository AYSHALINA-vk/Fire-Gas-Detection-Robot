import math
import random
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class DistanceSensorSimulator(Node):
    def __init__(self):
        super().__init__('sensor_simulator')
        
        # Hazard positions in the Gazebo WORLD frame (metres)
        self.declare_parameter('fire_x', 5.0)
        self.declare_parameter('fire_y', 3.0)
        self.declare_parameter('gas_x', -4.0)
        self.declare_parameter('gas_y', -2.0)
        
        # Robot spawn pose in the world (odom starts at 0,0 at spawn)
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        
        # Reading model
        self.declare_parameter('baseline', 10.0)
        self.declare_parameter('peak', 90.0)     # added at distance 0
        self.declare_parameter('sigma', 1.5)     # falloff width (m)
        self.declare_parameter('noise', 1.2)     # std dev
        self.declare_parameter('rate_hz', 5.0)
        
        self.x = 0.0
        self.y = 0.0
        
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.flame_pub = self.create_publisher(Float32, '/flame_reading', 10)
        self.gas_pub = self.create_publisher(Float32, '/gas_reading', 10)
        
        period = 1.0 / self.get_parameter('rate_hz').value
        self.create_timer(period, self.publish)
        self.get_logger().info('Distance-based sensor simulator started')

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.x = p.x + self.get_parameter('spawn_x').value
        self.y = p.y + self.get_parameter('spawn_y').value

    def reading(self, hx, hy):
        base = self.get_parameter('baseline').value
        peak = self.get_parameter('peak').value
        sigma = self.get_parameter('sigma').value
        noise = self.get_parameter('noise').value
        
        d = math.hypot(self.x - hx, self.y - hy)
        value = base + peak * math.exp(-(d * d) / (2.0 * sigma * sigma))
        return max(0.0, value + random.gauss(0.0, noise))

    def publish(self):
        gp = self.get_parameter
        f = Float32()
        g = Float32()
        f.data = float(self.reading(gp('fire_x').value, gp('fire_y').value))
        g.data = float(self.reading(gp('gas_x').value, gp('gas_y').value))
        
        self.flame_pub.publish(f)
        self.gas_pub.publish(g)

def main(args=None):
    rclpy.init(args=args)
    node = DistanceSensorSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
