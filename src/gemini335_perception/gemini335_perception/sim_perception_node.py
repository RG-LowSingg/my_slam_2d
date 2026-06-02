#!/usr/bin/env python3
"""
sim_perception_node.py — Gemini 335 Sim Perception Node

Dedicated node for Gazebo simulation that converts standard depth map (32FC1)
into a pseudo 2D LaserScan without requiring hardware SDKs.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Header
from cv_bridge import CvBridge

from gemini335_perception.depth_to_scan_core import depth_image_to_scan

class SimPerceptionNode(Node):
    def __init__(self):
        super().__init__('sim_perception_node')
        
        # Parameters
        self.declare_parameter('scan_height_min_ratio', 0.1)
        self.declare_parameter('scan_height_max_ratio', 0.9)
        self.declare_parameter('enable_sim_noise', False)
        self.declare_parameter('frame_id_scan', 'gemini335_camera_link')
        self.declare_parameter('topic_scan', 'gemini335/depth/scan')

        # Read parameters
        self.scan_height_min_ratio = self.get_parameter('scan_height_min_ratio').value
        self.scan_height_max_ratio = self.get_parameter('scan_height_max_ratio').value
        self.enable_sim_noise = self.get_parameter('enable_sim_noise').value
        self.frame_id_scan = self.get_parameter('frame_id_scan').value

        # Publishers
        self.pub_scan = self.create_publisher(
            LaserScan, self.get_parameter('topic_scan').value, 10)

        # Subscriptions
        self.sub_info = self.create_subscription(
            CameraInfo, '/gemini335/depth/camera_info', self._info_callback, 10)
        self.sub_depth = self.create_subscription(
            Image, '/gemini335/depth/image_raw', self._depth_callback, 10)

        # Internal state
        self.fx = None
        self.cx = None
        self.bridge = CvBridge()
        
        self.get_logger().info('Sim Perception Node started.')

    def _info_callback(self, msg):
        # Extract intrinsics from K matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.fx = msg.k[0]
        self.cx = msg.k[2]

    def _depth_callback(self, msg):
        if self.fx is None or self.cx is None:
            return  # Wait for camera info

        try:
            # Gazebo depth is usually 32FC1 in meters
            depth_m = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            
            # Optional noise injection for realism
            if self.enable_sim_noise:
                noise = np.random.normal(0, 0.002, depth_m.shape).astype(np.float32)
                depth_m = np.clip(depth_m + noise, 0, None)

            # Core calculation
            result = depth_image_to_scan(
                depth_m, self.fx, self.cx,
                height_min_ratio=self.scan_height_min_ratio,
                height_max_ratio=self.scan_height_max_ratio,
            )

            # Build and publish LaserScan
            scan = LaserScan()
            scan.header = Header(frame_id=self.frame_id_scan, stamp=msg.header.stamp)
            scan.angle_min = result.angle_min
            scan.angle_max = result.angle_max
            scan.angle_increment = result.angle_increment
            scan.time_increment = 0.0
            scan.scan_time = 1.0 / 10.0 # Assuming ~10Hz from Gazebo sensor
            scan.range_min = 0.1
            scan.range_max = 8.0
            scan.ranges = result.ranges

            self.pub_scan.publish(scan)
            
        except Exception as e:
            self.get_logger().error(f'Error processing depth: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SimPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
