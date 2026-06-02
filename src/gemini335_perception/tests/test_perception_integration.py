"""回归测试：验证重构后 perception_node 的 LaserScan 输出。"""
import pytest
import numpy as np
import math
# mock rclpy and ros msgs to avoid needing a ros environment
import sys
from unittest.mock import MagicMock

# Create a mock for rclpy and its components
mock_rclpy = MagicMock()
class MockNode:
    def __init__(self, *args, **kwargs):
        pass
    def get_logger(self):
        return MagicMock()
    def declare_parameter(self, *args, **kwargs):
        pass
    def get_parameter(self, name):
        m = MagicMock()
        m.value = None
        return m
    def create_publisher(self, *args, **kwargs):
        return MagicMock()
    def get_clock(self):
        m = MagicMock()
        m.now.return_value.to_msg.return_value = None
        return m

mock_rclpy.node.Node = MockNode
sys.modules['rclpy'] = mock_rclpy
sys.modules['rclpy.node'] = mock_rclpy.node

# Create mock for sensor_msgs
mock_sensor_msgs = MagicMock()
class MockHeader:
    def __init__(self, frame_id='', stamp=None):
        self.frame_id = frame_id
        self.stamp = stamp
class MockLaserScan:
    def __init__(self):
        self.header = MockHeader()
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0
        self.time_increment = 0.0
        self.scan_time = 0.0
        self.range_min = 0.0
        self.range_max = 0.0
        self.ranges = []

mock_sensor_msgs.msg.Header = MockHeader
mock_sensor_msgs.msg.LaserScan = MockLaserScan
sys.modules['sensor_msgs'] = mock_sensor_msgs
sys.modules['sensor_msgs.msg'] = mock_sensor_msgs.msg
sys.modules['std_msgs'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()
sys.modules['std_msgs.msg'].Header = MockHeader

# Create mock for pyorbbecsdk
mock_ob = MagicMock()
sys.modules['pyorbbecsdk'] = mock_ob

# Create mock for cv_bridge
mock_cv_bridge = MagicMock()
sys.modules['cv_bridge'] = mock_cv_bridge

from gemini335_perception.perception_node import PerceptionNode
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

class DummyDepthFrame:
    def __init__(self, data, width, height, scale):
        self.data = data
        self._width = width
        self._height = height
        self._scale = scale
        
    def get_data(self):
        return self.data
        
    def get_width(self):
        return self._width
        
    def get_height(self):
        return self._height
        
    def get_depth_scale(self):
        return self._scale

def test_extract_laser_scan_msg_fields(monkeypatch):
    monkeypatch.setattr(PerceptionNode, "__init__", lambda self: None)
    node = PerceptionNode()
    node.scan_height_min_ratio = 0.1
    node.scan_height_max_ratio = 0.9
    node.frame_id_scan = "test_scan_frame"
    node.color_fps = 30
    node.camera_params = None
    node._make_header = lambda frame_id, stamp: Header(frame_id=frame_id, stamp=stamp)

    # 480x640 uint16 depth map, 2000mm (2.0m)
    depth_np = np.full((480, 640), 2000, dtype=np.uint16)
    depth_frame = DummyDepthFrame(depth_np.tobytes(), 640, 480, 1.0)
    
    stamp = MagicMock()
    
    scan_msg = node._extract_laser_scan(depth_frame, stamp, is_aligned=True)
    
    assert isinstance(scan_msg, LaserScan)
    assert scan_msg.header.frame_id == "test_scan_frame"
    assert len(scan_msg.ranges) == 640
    assert all(math.isclose(r, 2.0, rel_tol=1e-5) for r in scan_msg.ranges)
