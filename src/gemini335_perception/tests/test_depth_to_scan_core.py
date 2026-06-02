"""Tests for depth_to_scan_core — pure algorithm, no ROS/SDK dependency."""
import numpy as np
import math
import pytest
from gemini335_perception.depth_to_scan_core import depth_image_to_scan

def test_uniform_depth_returns_correct_range_count(uniform_depth_image, dummy_intrinsic):
    result = depth_image_to_scan(uniform_depth_image, dummy_intrinsic['fx'], dummy_intrinsic['cx'])
    assert len(result.ranges) == 640
    assert all(math.isclose(r, 2.0, rel_tol=1e-5) for r in result.ranges)

def test_zero_depth_treated_as_inf(uniform_depth_image, dummy_intrinsic):
    uniform_depth_image[:, 320] = 0.0
    result = depth_image_to_scan(uniform_depth_image, dummy_intrinsic['fx'], dummy_intrinsic['cx'])
    assert math.isinf(result.ranges[640 - 1 - 320]) # Remember ranges are reversed

def test_nan_depth_treated_as_inf(uniform_depth_image, dummy_intrinsic):
    uniform_depth_image[:, 100] = np.nan
    result = depth_image_to_scan(uniform_depth_image, dummy_intrinsic['fx'], dummy_intrinsic['cx'])
    assert math.isinf(result.ranges[640 - 1 - 100])

def test_height_slice_uses_minimum_distance(dummy_intrinsic):
    depth = np.full((480, 640), 3.0, dtype=np.float32)
    # Put a closer object in the middle of the slice
    depth[240, 200] = 1.0
    result = depth_image_to_scan(depth, dummy_intrinsic['fx'], dummy_intrinsic['cx'])
    
    expected_index = 640 - 1 - 200
    assert math.isclose(result.ranges[expected_index], 1.0, rel_tol=1e-5)

def test_angle_range_matches_camera_fov():
    depth = np.full((100, 320), 2.0, dtype=np.float32)
    fx = 160.0 # simple 90 degree fov when w=320 (cx=160, w=320, atan(160/160)=pi/4)
    cx = 160.0
    result = depth_image_to_scan(depth, fx, cx)
    
    assert math.isclose(result.angle_max, math.pi / 4, rel_tol=1e-2)
    assert math.isclose(result.angle_min, -math.pi / 4, rel_tol=1e-2)

def test_ranges_reversed_for_ros_convention(dummy_intrinsic):
    depth = np.full((480, 640), 3.0, dtype=np.float32)
    depth[:, 0] = 1.0 # Left side of image
    depth[:, 639] = 2.0 # Right side of image
    
    result = depth_image_to_scan(depth, dummy_intrinsic['fx'], dummy_intrinsic['cx'])
    
    assert math.isclose(result.ranges[-1], 1.0, rel_tol=1e-5) # Left side becomes last index
    assert math.isclose(result.ranges[0], 2.0, rel_tol=1e-5) # Right side becomes first index

def test_out_of_range_clamped(uniform_depth_image, dummy_intrinsic):
    uniform_depth_image[:, 10] = 0.05 # < range_min (0.1)
    uniform_depth_image[:, 20] = 10.0 # > range_max (8.0)
    
    result = depth_image_to_scan(uniform_depth_image, dummy_intrinsic['fx'], dummy_intrinsic['cx'])
    
    assert math.isinf(result.ranges[640 - 1 - 10])
    assert math.isinf(result.ranges[640 - 1 - 20])
