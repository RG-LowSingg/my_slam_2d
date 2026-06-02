import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class CameraIntrusionGuardNode(Node):
    def __init__(self):
        super().__init__('camera_intrusion_guard')
        
        self.declare_parameter('intrusion_distance', 1.0)
        self.declare_parameter('intrusion_angle_range', 60.0)
        self.declare_parameter('confirm_frames', 3)
        self.declare_parameter('clear_frames', 10)
        self.declare_parameter('decel_rate', 0.1)

        self.intrusion_distance = self.get_parameter('intrusion_distance').value
        self.intrusion_angle_range = self.get_parameter('intrusion_angle_range').value
        self.confirm_frames = self.get_parameter('confirm_frames').value
        self.clear_frames = self.get_parameter('clear_frames').value
        self.decel_rate = self.get_parameter('decel_rate').value

        self.state = 'MONITORING'
        self.intrusion_count = 0
        self.clear_count = 0

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/gemini335/depth/scan',
            self.scan_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.stop_timer = None
        self.current_goal = None

    def scan_callback(self, msg):
        import math
        angle = msg.angle_min
        intrusion_detected = False
        half_angle = math.radians(self.intrusion_angle_range / 2.0)
        
        for r in msg.ranges:
            if -half_angle <= angle <= half_angle:
                if not math.isinf(r) and not math.isnan(r) and r < self.intrusion_distance:
                    intrusion_detected = True
                    break
            angle += msg.angle_increment

        if intrusion_detected:
            self.clear_count = 0
            self.intrusion_count += 1
            if self.intrusion_count >= self.confirm_frames and self.state == 'MONITORING':
                self.state = 'INTRUDING'
                self.get_logger().warn('Intrusion detected! Stopping...')
                if self.stop_timer is None:
                    self.stop_timer = self.create_timer(self.decel_rate, self.stop_timer_callback)
                # Trigger the first stop immediately
                self.stop_timer_callback()
        else:
            if self.state == 'INTRUDING':
                self.clear_count += 1
                if self.clear_count >= self.clear_frames:
                    self.state = 'MONITORING'
                    self.intrusion_count = 0
                    self.clear_count = 0
                    self.get_logger().info('Path clear, resuming...')
                    if self.stop_timer:
                        self.stop_timer.cancel()
                        self.stop_timer = None
            else:
                self.intrusion_count = 0

    def stop_timer_callback(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CameraIntrusionGuardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
