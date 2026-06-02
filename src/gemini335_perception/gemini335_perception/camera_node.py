#!/usr/bin/env python3
"""
camera_node.py — Gemini 335 Camera ROS2 Node

Publishes:
  - /gemini335/color/image_raw   (sensor_msgs/Image, BGR8)
  - /gemini335/color/camera_info  (sensor_msgs/CameraInfo)
  - /gemini335/depth/image_raw   (sensor_msgs/Image, 16UC1, mm)
  - /gemini335/depth/camera_info  (sensor_msgs/CameraInfo)

Uses pyorbbecsdk to capture frames with HwD2C/SwD2C alignment.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

import pyorbbecsdk as ob


class CameraNode(Node):
    """ROS2 node that publishes RGB and Depth images from Gemini 335."""

    def __init__(self):
        super().__init__('gemini335_camera_node')

        # Declare parameters
        self.declare_parameter('color_width', 640)
        self.declare_parameter('color_height', 480)
        self.declare_parameter('color_fps', 30)
        self.declare_parameter('color_format', 'mjpg')
        self.declare_parameter('enable_depth', True)
        self.declare_parameter('depth_align_mode', 'hw')
        self.declare_parameter('frame_id_color', 'gemini335_color_optical_frame')
        self.declare_parameter('frame_id_depth', 'gemini335_depth_optical_frame')
        self.declare_parameter('topic_color_image', 'gemini335/color/image_raw')
        self.declare_parameter('topic_color_info', 'gemini335/color/camera_info')
        self.declare_parameter('topic_depth_image', 'gemini335/depth/image_raw')
        self.declare_parameter('topic_depth_info', 'gemini335/depth/camera_info')

        # Read parameters
        self.color_w = self.get_parameter('color_width').value
        self.color_h = self.get_parameter('color_height').value
        self.color_fps = self.get_parameter('color_fps').value
        self.color_fmt_str = self.get_parameter('color_format').value
        self.enable_depth = self.get_parameter('enable_depth').value
        self.align_mode_str = self.get_parameter('depth_align_mode').value
        self.frame_id_color = self.get_parameter('frame_id_color').value
        self.frame_id_depth = self.get_parameter('frame_id_depth').value

        # Publishers
        color_topic = self.get_parameter('topic_color_image').value
        color_info_topic = self.get_parameter('topic_color_info').value
        self.pub_color = self.create_publisher(Image, color_topic, 10)
        self.pub_color_info = self.create_publisher(CameraInfo, color_info_topic, 10)

        if self.enable_depth:
            depth_topic = self.get_parameter('topic_depth_image').value
            depth_info_topic = self.get_parameter('topic_depth_info').value
            self.pub_depth = self.create_publisher(Image, depth_topic, 10)
            self.pub_depth_info = self.create_publisher(CameraInfo, depth_info_topic, 10)
        else:
            self.pub_depth = None
            self.pub_depth_info = None

        # Camera state
        self.pipeline = None
        self.color_format = None
        self.camera_params = None

        # Initialize camera
        self._init_camera()

        # Timer for frame capture (match camera FPS)
        period = 1.0 / self.color_fps
        self.timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            f'Camera node started: {self.color_w}x{self.color_h}@{self.color_fps}fps, '
            f'depth={self.enable_depth}, align={self.align_mode_str}'
        )

    def _init_camera(self):
        """Initialize Orbbec camera pipeline."""
        ctx = ob.Context()
        device_list = ctx.query_devices()
        if device_list.get_count() == 0:
            self.get_logger().error('No Orbbec device found!')
            raise RuntimeError('No Orbbec device found')

        device = device_list.get_device_by_index(0)
        info = device.get_device_info()
        self.get_logger().info(f'Device: {info.get_name()} (SN: {info.get_serial_number()})')

        self.pipeline = ob.Pipeline(device)
        config = ob.Config()

        # Color profile
        color_profiles = self.pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
        fmt_map = {
            'mjpg': ob.OBFormat.MJPG,
            'rgb': ob.OBFormat.RGB,
            'bgr': ob.OBFormat.BGR,
        }
        target_fmt = fmt_map.get(self.color_fmt_str.lower(), ob.OBFormat.MJPG)

        color_profile = None
        for fmt in [target_fmt, ob.OBFormat.MJPG, ob.OBFormat.RGB, ob.OBFormat.BGR]:
            try:
                color_profile = color_profiles.get_video_stream_profile(
                    self.color_w, self.color_h, fmt, self.color_fps
                )
                self.color_format = fmt
                break
            except Exception:
                continue

        if color_profile is None:
            color_profile = color_profiles.get_default_video_stream_profile()
            self.color_format = color_profile.as_video_stream_profile().get_format()
            self.get_logger().warn(f'Using default color profile, format: {self.color_format.name}')

        config.enable_stream(color_profile)

        # Depth with D2C
        if self.enable_depth:
            align_map = {
                'hw': ob.OBAlignMode.HW_MODE,
                'sw': ob.OBAlignMode.SW_MODE,
                'off': ob.OBAlignMode.DISABLE,
            }
            target_align = align_map.get(self.align_mode_str.lower(), ob.OBAlignMode.HW_MODE)

            depth_profile = None
            actual_align = ob.OBAlignMode.DISABLE

            for mode in [target_align, ob.OBAlignMode.HW_MODE, ob.OBAlignMode.SW_MODE]:
                if mode == ob.OBAlignMode.DISABLE:
                    continue
                try:
                    d2c_list = self.pipeline.get_d2c_depth_profile_list(color_profile, mode)
                    if d2c_list.get_count() > 0:
                        depth_profile = d2c_list.get_stream_profile_by_index(0)
                        actual_align = mode
                        break
                except Exception:
                    continue

            if depth_profile is not None:
                config.enable_stream(depth_profile)
                config.set_align_mode(actual_align)
                self.get_logger().info(f'Depth: D2C align={actual_align.name}')
            else:
                config.enable_stream(ob.OBSensorType.DEPTH_SENSOR)
                self.get_logger().warn('D2C not available, using raw depth')

        # Get camera parameters for CameraInfo
        config.set_frame_aggregate_output_mode(ob.OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)
        self.pipeline.enable_frame_sync()
        self.pipeline.start(config)

        try:
            self.camera_params = self.pipeline.get_camera_param()
        except Exception:
            self.camera_params = None
            self.get_logger().warn('Could not get camera parameters')

    def _make_header(self, frame_id):
        """Create a std_msgs/Header with current time."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        return header

    def _make_camera_info(self, width, height, frame_id):
        """Create a basic CameraInfo message."""
        msg = CameraInfo()
        msg.header = self._make_header(frame_id)
        msg.width = width
        msg.height = height

        # Fill intrinsics from camera_params if available
        if self.camera_params is not None:
            try:
                intrinsic = self.camera_params.rgb_intrinsic if frame_id == self.frame_id_color \
                    else self.camera_params.depth_intrinsic
                msg.k = [
                    float(intrinsic.fx), 0.0, float(intrinsic.cx),
                    0.0, float(intrinsic.fy), float(intrinsic.cy),
                    0.0, 0.0, 1.0,
                ]
                msg.p = [
                    float(intrinsic.fx), 0.0, float(intrinsic.cx), 0.0,
                    0.0, float(intrinsic.fy), float(intrinsic.cy), 0.0,
                    0.0, 0.0, 1.0, 0.0,
                ]
            except Exception:
                pass

        return msg

    def _decode_color(self, frame):
        """Decode color frame to BGR numpy array."""
        data = frame.get_data()
        h, w = frame.get_height(), frame.get_width()

        if self.color_format == ob.OBFormat.MJPG:
            import cv2
            buf = np.frombuffer(data, dtype=np.uint8) if not isinstance(data, np.ndarray) else data
            image = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            return image
        elif self.color_format == ob.OBFormat.RGB:
            import cv2
            arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3) if not isinstance(data, np.ndarray) else data.reshape(h, w, 3)
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        elif self.color_format == ob.OBFormat.BGR:
            return np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3) if not isinstance(data, np.ndarray) else data.reshape(h, w, 3)
        else:
            return np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3) if not isinstance(data, np.ndarray) else data.reshape(h, w, 3)

    def _decode_depth(self, frame):
        """Decode depth frame to uint16 numpy array (mm)."""
        data = frame.get_data()
        h, w = frame.get_height(), frame.get_width()

        if isinstance(data, np.ndarray):
            if data.dtype == np.uint8:
                data = data.view(np.uint16)
            depth = data.reshape(h, w).astype(np.uint16)
        else:
            depth = np.frombuffer(data, dtype=np.uint16).reshape(h, w)

        scale = frame.get_depth_scale()
        if scale != 1.0:
            depth = (depth.astype(np.float32) * scale).astype(np.uint16)
        return depth

    def _numpy_to_image_msg(self, array, frame_id, encoding):
        """Convert numpy array to sensor_msgs/Image."""
        msg = Image()
        msg.header = self._make_header(frame_id)
        msg.height = array.shape[0]
        msg.width = array.shape[1]
        msg.encoding = encoding

        if len(array.shape) == 3:
            msg.step = array.shape[1] * array.shape[2] * array.dtype.itemsize
        else:
            msg.step = array.shape[1] * array.dtype.itemsize

        msg.is_bigendian = False
        msg.data = array.tobytes()
        return msg

    def _timer_callback(self):
        """Timer callback: capture and publish frames."""
        if self.pipeline is None:
            return

        try:
            frameset = self.pipeline.wait_for_frames(100)
        except Exception:
            return

        if frameset is None:
            return

        # Publish color
        color_frame = frameset.get_color_frame()
        if color_frame is not None:
            color_bgr = self._decode_color(color_frame)
            if color_bgr is not None:
                color_msg = self._numpy_to_image_msg(color_bgr, self.frame_id_color, 'bgr8')
                self.pub_color.publish(color_msg)

                info_msg = self._make_camera_info(
                    color_bgr.shape[1], color_bgr.shape[0], self.frame_id_color
                )
                self.pub_color_info.publish(info_msg)

        # Publish depth
        if self.enable_depth and self.pub_depth is not None:
            depth_frame = frameset.get_depth_frame()
            if depth_frame is not None:
                depth_map = self._decode_depth(depth_frame)
                depth_msg = self._numpy_to_image_msg(depth_map, self.frame_id_depth, '16UC1')
                self.pub_depth.publish(depth_msg)

                info_msg = self._make_camera_info(
                    depth_map.shape[1], depth_map.shape[0], self.frame_id_depth
                )
                self.pub_depth_info.publish(info_msg)

    def destroy_node(self):
        """Clean shutdown."""
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
            self.pipeline = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
