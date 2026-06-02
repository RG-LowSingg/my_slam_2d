import numpy as np
import math
from dataclasses import dataclass

@dataclass
class ScanResult:
    """核心算法的纯数据输出，不含任何 ROS 消息类型。"""
    ranges: list[float]        # 距离数组 (米), 从 angle_min 到 angle_max
    angle_min: float           # 最小角度 (rad)
    angle_max: float           # 最大角度 (rad)
    angle_increment: float     # 角度增量 (rad)

def depth_image_to_scan(
    depth_meters: np.ndarray,
    fx: float,
    cx: float,
    height_min_ratio: float = 0.1,
    height_max_ratio: float = 0.9,
    range_min: float = 0.1,
    range_max: float = 8.0,
) -> ScanResult:
    """
    将深度图的指定高度切片转换为 2D 激光扫描数据。
    """
    h, w = depth_meters.shape

    row_min = int(h * height_min_ratio)
    row_max = int(h * height_max_ratio)
    row_min = max(0, min(h - 1, row_min))
    row_max = max(row_min + 1, min(h, row_max))

    # Slice the specified vertical frustum
    depth_slice = depth_meters[row_min:row_max, :].copy()
    
    # Treat 0, NaN, and values outside of range as inf
    invalid_mask = (depth_slice <= 0) | np.isnan(depth_slice)
    depth_slice[invalid_mask] = np.inf

    # Get the minimum depth value in each column
    scan_data_raw = np.min(depth_slice, axis=0)
    
    # Clamp to inf if outside valid range
    out_of_range_mask = (scan_data_raw < range_min) | (scan_data_raw > range_max)
    scan_data_raw[out_of_range_mask] = np.inf

    # Reverse ranges so index 0 is the right side of the image (negative angle)
    ranges = scan_data_raw[::-1].tolist()

    # Calculate angles
    angle_min = math.atan2(cx - (w - 1), fx)
    angle_max = math.atan2(cx, fx)
    angle_increment = (angle_max - angle_min) / max(1, (w - 1))

    return ScanResult(
        ranges=ranges,
        angle_min=angle_min,
        angle_max=angle_max,
        angle_increment=angle_increment
    )
