"""Tests for sim_perception_node — Gazebo depth → LaserScan conversion."""
import pytest
import numpy as np
import math

import sys
from unittest.mock import MagicMock

# mock rclpy
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
        m.value = False if 'noise' in name else 'mock_frame'
        if name == 'scan_height_min_ratio': m.value = 0.1
        if name == 'scan_height_max_ratio': m.value = 0.9
        return m
    def create_publisher(self, *args, **kwargs):
        return MagicMock()
    def create_subscription(self, *args, **kwargs):
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
class MockImage:
    def __init__(self):
        self.header = MockHeader()
class MockCameraInfo:
    def __init__(self):
        self.k = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]

mock_sensor_msgs.msg.Header = MockHeader
mock_sensor_msgs.msg.LaserScan = MockLaserScan
mock_sensor_msgs.msg.Image = MockImage
mock_sensor_msgs.msg.CameraInfo = MockCameraInfo
sys.modules['sensor_msgs'] = mock_sensor_msgs
sys.modules['sensor_msgs.msg'] = mock_sensor_msgs.msg

sys.modules['std_msgs'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()
sys.modules['std_msgs.msg'].Header = MockHeader

# Create mock for cv_bridge
mock_cv_bridge = MagicMock()
sys.modules['cv_bridge'] = mock_cv_bridge

from gemini335_perception.sim_perception_node import SimPerceptionNode

def test_camera_info_intrinsics_extraction():
    node = SimPerceptionNode()
    
    info_msg = MockCameraInfo()
    info_msg.k = [320.0, 0.0, 319.5, 0.0, 320.0, 239.5, 0.0, 0.0, 1.0]
    
    node._info_callback(info_msg)
    
    assert node.fx == 320.0
    assert node.cx == 319.5

def test_float32_depth_to_scan_conversion():
    node = SimPerceptionNode()
    node.fx = 320.0
    node.cx = 320.0
    
    # Mock cv_bridge returning float32 numpy array
    depth_m = np.full((480, 640), 2.5, dtype=np.float32)
    node.bridge.imgmsg_to_cv2.return_value = depth_m
    
    # Capture published message
    published_msg = None
    def mock_publish(msg):
        nonlocal published_msg
        published_msg = msg
    node.pub_scan.publish = mock_publish
    
    msg = MockImage()
    node._depth_callback(msg)
    
    assert published_msg is not None
    assert isinstance(published_msg, MockLaserScan)
    assert len(published_msg.ranges) == 640
    assert all(math.isclose(r, 2.5, rel_tol=1e-5) for r in published_msg.ranges)

def test_sim_noise_injection():
    node = SimPerceptionNode()
    node.fx = 320.0
    node.cx = 320.0
    node.enable_sim_noise = True
    
    depth_m = np.full((480, 640), 2.5, dtype=np.float32)
    node.bridge.imgmsg_to_cv2.return_value = depth_m
    
    published_msg = None
    def mock_publish(msg):
        nonlocal published_msg
        published_msg = msg
    node.pub_scan.publish = mock_publish
    
    msg = MockImage()
    node._depth_callback(msg)
    
    ranges = np.array(published_msg.ranges)
    # The variance should be > 0 due to noise
    assert np.var(ranges) > 0
    # The mean should be close to 2.5
    assert math.isclose(np.mean(ranges), 2.5, abs_tol=0.1)
