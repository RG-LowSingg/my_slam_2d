import pytest
import sys
from unittest.mock import MagicMock

# Mock rclpy and nav2_msgs
mock_rclpy = MagicMock()
class MockNode:
    def __init__(self, name):
        self.name = name
    def declare_parameter(self, name, value):
        pass
    def get_parameter(self, name):
        m = MagicMock()
        defaults = {
            'intrusion_distance': 1.0,
            'intrusion_angle_range': 60.0,
            'confirm_frames': 3,
            'clear_frames': 10,
            'decel_rate': 0.1
        }
        m.value = defaults.get(name)
        return m
    def create_subscription(self, *args, **kwargs):
        return MagicMock()
    def create_publisher(self, *args, **kwargs):
        return MagicMock()
    def create_timer(self, *args, **kwargs):
        return MagicMock()
    def get_logger(self):
        return MagicMock()

mock_rclpy.node.Node = MockNode
sys.modules['rclpy'] = mock_rclpy
sys.modules['rclpy.node'] = mock_rclpy.node

mock_action = MagicMock()
sys.modules['rclpy.action'] = mock_action

mock_sensor_msgs = MagicMock()
sys.modules['sensor_msgs'] = mock_sensor_msgs
sys.modules['sensor_msgs.msg'] = mock_sensor_msgs.msg

mock_geometry_msgs = MagicMock()
sys.modules['geometry_msgs'] = mock_geometry_msgs
sys.modules['geometry_msgs.msg'] = mock_geometry_msgs.msg

mock_nav2_msgs = MagicMock()
sys.modules['nav2_msgs'] = mock_nav2_msgs
sys.modules['nav2_msgs.action'] = mock_nav2_msgs.action

from gemini335_perception.camera_intrusion_guard_node import CameraIntrusionGuardNode

def test_initial_state_is_monitoring():
    node = CameraIntrusionGuardNode()
    assert node.state == 'MONITORING'
    assert node.intrusion_count == 0

def test_detects_intrusion():
    node = CameraIntrusionGuardNode()
    msg = mock_sensor_msgs.msg.LaserScan()
    msg.angle_min = -1.0
    msg.angle_max = 1.0
    msg.angle_increment = 0.1
    # 21 beams. Middle beam is at 0 rad.
    # Set all ranges to infinity initially
    msg.ranges = [float('inf')] * 21
    
    # Put an obstacle at 0.5m in front (0 rad), which is within intrusion_distance (1.0m)
    msg.ranges[10] = 0.5
    
    node.scan_callback(msg)
    assert node.intrusion_count == 1
    
    node.scan_callback(msg)
    assert node.intrusion_count == 2
    
    # 3rd frame confirms intrusion
    node.scan_callback(msg)
    assert node.state == 'INTRUDING'
    assert node.cmd_vel_pub.publish.called
