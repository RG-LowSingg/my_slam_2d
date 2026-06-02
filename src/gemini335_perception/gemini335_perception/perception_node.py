#!/usr/bin/env python3
"""
perception_node.py — Gemini 335 Unified Perception Node

Combined node that publishes all sensor data from a single pipeline:
  - /gemini335/color/image_raw   (sensor_msgs/Image, BGR8)
  - /gemini335/depth/image_raw   (sensor_msgs/Image, 16UC1)
  - /gemini335/imu               (sensor_msgs/Imu)
  - /gemini335/color/camera_info  (sensor_msgs/CameraInfo)
  - /gemini335/depth/camera_info  (sensor_msgs/CameraInfo)

This is the recommended node for production use since it uses a single
pipeline and avoids device contention issues.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from std_msgs.msg import Header
from cv_bridge import CvBridge

import pyorbbecsdk as ob
from gemini335_perception.depth_to_scan_core import depth_image_to_scan

class PerceptionNode(Node):
    """Combined perception node: RGB + Depth + IMU from single pipeline."""

    def __init__(self):
        super().__init__('gemini335_perception_node')
        try:
            ob.Context.set_logger_level(ob.OBLogSeverity.FATAL)
        except AttributeError:
            try:
                ob.Context.set_logger_level(ob.OBLogLevel.FATAL)
            except AttributeError:
                pass

        # Parameters
        self.declare_parameter('color_width', 640)
        self.declare_parameter('color_height', 480)
        self.declare_parameter('color_fps', 30)
        self.declare_parameter('color_format', 'mjpg')
        self.declare_parameter('enable_depth', True)
        self.declare_parameter('depth_align_mode', 'hw')
        self.declare_parameter('enable_imu', True)
        self.declare_parameter('enable_noise_filter', True)
        self.declare_parameter('enable_spatial_filter', False)
        self.declare_parameter('frame_id_color', 'gemini335_color_optical_frame')
        self.declare_parameter('frame_id_depth', 'gemini335_depth_optical_frame')
        self.declare_parameter('frame_id_scan', 'gemini335_camera_link')
        self.declare_parameter('frame_id_imu', 'gemini335_imu_frame')
        self.declare_parameter('topic_color_image', 'gemini335/color/image_raw')
        self.declare_parameter('topic_color_info', 'gemini335/color/camera_info')
        self.declare_parameter('topic_depth_image', 'gemini335/depth/image_raw')
        self.declare_parameter('topic_depth_info', 'gemini335/depth/camera_info')
        self.declare_parameter('topic_scan', 'gemini335/depth/scan')
        self.declare_parameter('topic_imu', 'gemini335/imu')

        # Advanced scan settings
        self.declare_parameter('scan_height_min_ratio', 0.1)
        self.declare_parameter('scan_height_max_ratio', 0.9)

        # Read params
        self.color_w = self.get_parameter('color_width').value
        self.color_h = self.get_parameter('color_height').value
        self.color_fps = self.get_parameter('color_fps').value
        self.color_fmt_str = self.get_parameter('color_format').value
        self.enable_depth = self.get_parameter('enable_depth').value
        self.align_str = self.get_parameter('depth_align_mode').value
        self.enable_imu = self.get_parameter('enable_imu').value
        self.enable_noise_filter = self.get_parameter('enable_noise_filter').value
        self.enable_spatial_filter = self.get_parameter('enable_spatial_filter').value
        self.frame_id_color = self.get_parameter('frame_id_color').value
        self.frame_id_depth = self.get_parameter('frame_id_depth').value
        self.frame_id_scan = self.get_parameter('frame_id_scan').value
        self.frame_id_imu = self.get_parameter('frame_id_imu').value
        self.scan_height_min_ratio = self.get_parameter('scan_height_min_ratio').value
        self.scan_height_max_ratio = self.get_parameter('scan_height_max_ratio').value

        # Publishers
        self.pub_color = self.create_publisher(
            Image, self.get_parameter('topic_color_image').value, 10)
        self.pub_color_info = self.create_publisher(
            CameraInfo, self.get_parameter('topic_color_info').value, 10)

        self.pub_depth = None
        self.pub_depth_info = None
        self.pub_scan = None
        if self.enable_depth:
            self.pub_depth = self.create_publisher(
                Image, self.get_parameter('topic_depth_image').value, 10)
            self.pub_depth_info = self.create_publisher(
                CameraInfo, self.get_parameter('topic_depth_info').value, 10)
            self.pub_scan = self.create_publisher(
                LaserScan, self.get_parameter('topic_scan').value, 10)

        self.pub_imu = None
        if self.enable_imu:
            self.pub_imu = self.create_publisher(
                Imu, self.get_parameter('topic_imu').value, 50)

        # CvBridge
        self.bridge = CvBridge()

        # State
        self.pipeline = None
        self.color_format = None
        self.camera_params = None
        self.frame_count = 0

        # Filters
        self.noise_filter = None
        self.spatial_filter = None

        self._init_pipeline()

        # Dedicated capture thread instead of 30Hz timer to drain 200Hz IMU frames
        import threading
        self.is_running = True
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

        self.get_logger().info(
            f'Perception node started: {self.color_w}x{self.color_h}@{self.color_fps}fps, '
            f'depth={self.enable_depth}, imu={self.enable_imu}, align={self.align_str}'
        )

    def _init_pipeline(self):
        """Initialize unified pipeline."""
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

        # --- Color ---
        color_profiles = self.pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
        fmt_map = {'mjpg': ob.OBFormat.MJPG, 'rgb': ob.OBFormat.RGB, 'bgr': ob.OBFormat.BGR}
        target_fmt = fmt_map.get(self.color_fmt_str.lower(), ob.OBFormat.MJPG)

        color_profile = None
        for fmt in [target_fmt, ob.OBFormat.MJPG, ob.OBFormat.RGB, ob.OBFormat.BGR]:
            try:
                color_profile = color_profiles.get_video_stream_profile(
                    self.color_w, self.color_h, fmt, self.color_fps)
                self.color_format = fmt
                break
            except Exception:
                continue

        if color_profile is None:
            color_profile = color_profiles.get_default_video_stream_profile()
            self.color_format = color_profile.as_video_stream_profile().get_format()

        config.enable_stream(color_profile)
        cvsp = color_profile.as_video_stream_profile()
        self.color_w = cvsp.get_width()
        self.color_h = cvsp.get_height()
        self.get_logger().info(
            f'Color: {self.color_w}x{self.color_h}@{cvsp.get_fps()} ({self.color_format.name})')

        # --- Depth ---
        if self.enable_depth:
            align_map = {'hw': ob.OBAlignMode.HW_MODE, 'sw': ob.OBAlignMode.SW_MODE, 'off': ob.OBAlignMode.DISABLE}
            target_align = align_map.get(self.align_str.lower(), ob.OBAlignMode.HW_MODE)

            depth_profile = None
            actual_align = ob.OBAlignMode.DISABLE
            for mode in [target_align, ob.OBAlignMode.HW_MODE, ob.OBAlignMode.SW_MODE]:
                if mode == ob.OBAlignMode.DISABLE:
                    continue
                try:
                    d2c_list = self.pipeline.get_d2c_depth_profile_list(color_profile, mode)
                    if d2c_list.get_count() > 0:
                        # Find matching fps and resolution profile
                        for i in range(d2c_list.get_count()):
                            p = d2c_list.get_stream_profile_by_index(i)
                            vp = p.as_video_stream_profile()
                            if vp.get_fps() == self.color_fps and vp.get_width() == self.color_w and vp.get_height() == self.color_h:
                                depth_profile = p
                                break
                        # If no exact resolution match, just match fps
                        if depth_profile is None:
                            for i in range(d2c_list.get_count()):
                                p = d2c_list.get_stream_profile_by_index(i)
                                if p.as_video_stream_profile().get_fps() == self.color_fps:
                                    depth_profile = p
                                    break
                        if depth_profile is None:
                            depth_profile = d2c_list.get_stream_profile_by_index(0)
                        actual_align = mode
                        break
                except Exception:
                    continue

            if depth_profile is not None:
                config.enable_stream(depth_profile)
                config.set_align_mode(actual_align)
                dvsp = depth_profile.as_video_stream_profile()
                self.get_logger().info(f'Depth: {dvsp.get_width()}x{dvsp.get_height()} align={actual_align.name}')
            else:
                config.enable_stream(ob.OBSensorType.DEPTH_SENSOR)
                self.get_logger().warn('D2C unavailable, raw depth')

        # --- IMU ---
        self.has_accel = False
        self.has_gyro = False
        if self.enable_imu:
            try:
                if device.get_sensor(ob.OBSensorType.ACCEL_SENSOR) is not None:
                    config.enable_accel_stream()
                    self.has_accel = True
            except Exception:
                pass
            try:
                if device.get_sensor(ob.OBSensorType.GYRO_SENSOR) is not None:
                    config.enable_gyro_stream()
                    self.has_gyro = True
            except Exception:
                pass
            self.get_logger().info(f'IMU: accel={self.has_accel}, gyro={self.has_gyro}')

        # Start
        self.pipeline.enable_frame_sync()
        self.pipeline.start(config)

        try:
            self.camera_params = self.pipeline.get_camera_param()
        except Exception:
            self.camera_params = None

        # Initialize SDK Filters after pipeline start
        try:
            self.noise_filter = ob.NoiseRemovalFilter()
            self.get_logger().info('NoiseRemovalFilter initialized')
        except Exception as e:
            self.get_logger().warn(f'Failed to initialize NoiseRemovalFilter: {e}')

        try:
            self.spatial_filter = ob.SpatialAdvancedFilter()
            self.get_logger().info('SpatialAdvancedFilter initialized')
        except Exception as e:
            self.get_logger().warn(f'Failed to initialize SpatialAdvancedFilter: {e}')

    def _make_header(self, frame_id, stamp=None):
        h = Header()
        h.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        h.frame_id = frame_id
        return h

    def _make_camera_info(self, w, h, frame_id, stamp):
        msg = CameraInfo()
        msg.header = self._make_header(frame_id, stamp)
        msg.width = w
        msg.height = h

        if self.camera_params is not None:
            try:
                # If depth is aligned to color, its intrinsic is the RGB intrinsic!
                is_aligned = (self.align_str.lower() in ['hw', 'sw'])
                use_rgb_intrinsic = (frame_id == self.frame_id_color) or (is_aligned and frame_id == self.frame_id_depth)
                
                intrinsic = self.camera_params.rgb_intrinsic if use_rgb_intrinsic \
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
        data = frame.get_data()
        h, w = frame.get_height(), frame.get_width()

        if self.color_format == ob.OBFormat.MJPG:
            import cv2
            try:
                buf = np.frombuffer(data, dtype=np.uint8) if not isinstance(data, np.ndarray) else data
                img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                if img is None:
                    self.get_logger().error(f"cv2.imdecode failed on buf size {len(buf)}")
                return img
            except Exception as e:
                self.get_logger().error(f"Decode error: {e}")
                return None
        elif self.color_format == ob.OBFormat.RGB:
            import cv2
            arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3) if not isinstance(data, np.ndarray) else data.reshape(h, w, 3)
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        else:
            return np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3) if not isinstance(data, np.ndarray) else data.reshape(h, w, 3)

    def _decode_depth(self, frame):
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

    def _numpy_to_image(self, arr, frame_id, encoding, stamp):
        msg = self.bridge.cv2_to_imgmsg(arr, encoding=encoding)
        msg.header = self._make_header(frame_id, stamp)
        return msg

    def _extract_laser_scan(self, depth_frame, stamp, is_aligned):
        import math
        
        data = depth_frame.get_data()
        h, w = depth_frame.get_height(), depth_frame.get_width()
        
        if isinstance(data, np.ndarray):
            if data.dtype == np.uint8:
                data = data.view(np.uint16)
            depth_raw = data.reshape(h, w)
        else:
            depth_raw = np.frombuffer(data, dtype=np.uint16).reshape(h, w)

        scale = depth_frame.get_depth_scale()
        depth_m = depth_raw.astype(np.float32) * scale / 1000.0

        if self.camera_params is not None:
            intrinsic = self.camera_params.rgb_intrinsic if is_aligned else self.camera_params.depth_intrinsic
            cx = float(intrinsic.cx)
            fx = float(intrinsic.fx)
        else:
            cx = w / 2.0
            fx = w / (2.0 * math.tan(math.radians(35.0)))
        
        result = depth_image_to_scan(
            depth_m, fx, cx,
            height_min_ratio=self.scan_height_min_ratio,
            height_max_ratio=self.scan_height_max_ratio,
        )

        scan = LaserScan()
        scan.header = self._make_header(self.frame_id_scan, stamp)
        scan.angle_min = result.angle_min
        scan.angle_max = result.angle_max
        scan.angle_increment = result.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.color_fps
        scan.range_min = 0.1
        scan.range_max = 8.0
        scan.ranges = result.ranges
        
        return scan



    def safe_publish(self, pub, msg):
        try:
            if pub is not None:
                pub.publish(msg)
        except Exception:
            pass

    def safe_get_subs(self, pub):
        try:
            return pub.get_subscription_count() > 0 if pub else False
        except Exception:
            return False

    def _capture_loop(self):
        while rclpy.ok() and self.is_running and self.pipeline is not None:
            try:
                frameset = self.pipeline.wait_for_frames(100)
            except Exception:
                continue

            if frameset is None:
                continue

            self.frame_count += 1
            
            # Unified timestamp for all frames in this frameset to allow accurate PointCloud2 generation
            current_stamp = self.get_clock().now().to_msg()

            # --- Color ---
            if self.safe_get_subs(self.pub_color) or self.safe_get_subs(self.pub_color_info):
                color_frame = frameset.get_color_frame()
                if color_frame is not None:
                    bgr = self._decode_color(color_frame)
                    if bgr is not None:
                        self.safe_publish(self.pub_color, self._numpy_to_image(bgr, self.frame_id_color, 'bgr8', current_stamp))
                        self.pub_color_info.publish(
                            self._make_camera_info(bgr.shape[1], bgr.shape[0], self.frame_id_color, current_stamp))

            # --- Depth ---
            if self.enable_depth and self.pub_depth is not None:
                depth_frame = frameset.get_depth_frame()
                if depth_frame is not None:
                    depth_subs = self.safe_get_subs(self.pub_depth)
                    info_subs = self.safe_get_subs(self.pub_depth_info)
                    scan_subs = self.safe_get_subs(self.pub_scan)
                    
                    if depth_subs or scan_subs or info_subs:
                        # Apply SDK software filters directly to the depth frame before decoding
                        if self.enable_noise_filter and self.noise_filter is not None:
                            try:
                                filtered_frame = self.noise_filter.process(depth_frame)
                                if filtered_frame is not None:
                                    depth_frame = filtered_frame
                            except Exception:
                                pass
                                
                        if self.enable_spatial_filter and self.spatial_filter is not None:
                            try:
                                filtered_frame = self.spatial_filter.process(depth_frame)
                                if filtered_frame is not None:
                                    depth_frame = filtered_frame
                            except Exception:
                                pass

                        is_aligned = self.align_str.lower() in ['hw', 'sw']

                        if depth_subs or info_subs:
                            depth = self._decode_depth(depth_frame)
                            
                            # Ensure depth resolution perfectly matches color resolution when aligned
                            if is_aligned and (depth.shape[1] != self.color_w or depth.shape[0] != self.color_h):
                                # Center-crop or center-pad to preserve optical center (cx, cy)
                                new_depth = np.zeros((self.color_h, self.color_w), dtype=depth.dtype)
                                h_in, w_in = depth.shape
                                h_out, w_out = self.color_h, self.color_w
                                
                                y_in = max(0, (h_in - h_out) // 2)
                                x_in = max(0, (w_in - w_out) // 2)
                                y_out = max(0, (h_out - h_in) // 2)
                                x_out = max(0, (w_out - w_in) // 2)
                                
                                crop_h, crop_w = min(h_in, h_out), min(w_in, w_out)
                                new_depth[y_out:y_out+crop_h, x_out:x_out+crop_w] = depth[y_in:y_in+crop_h, x_in:x_in+crop_w]
                                depth = new_depth

                            # If aligned, the depth image belongs to the color optical frame!
                            actual_depth_frame_id = self.frame_id_color if is_aligned else self.frame_id_depth
                            
                            if depth_subs:
                                self.safe_publish(self.pub_depth, self._numpy_to_image(depth, actual_depth_frame_id, '16UC1', current_stamp))
                            if info_subs:
                                self.pub_depth_info.publish(
                                    self._make_camera_info(depth.shape[1], depth.shape[0], actual_depth_frame_id, current_stamp))

                        if scan_subs:
                            scan_msg = self._extract_laser_scan(depth_frame, current_stamp, is_aligned)
                            if scan_msg is not None:
                                self.safe_publish(self.pub_scan, scan_msg)

            # --- IMU ---
            if self.pub_imu is not None:
                imu_msg = Imu()
                imu_msg.header = self._make_header(self.frame_id_imu, current_stamp)
                imu_msg.orientation_covariance = [-1.0] + [0.0] * 8
                has_imu = False

                if self.has_accel:
                    try:
                        af_raw = frameset.get_frame_by_type(ob.OBFrameType.ACCEL_FRAME)
                        if af_raw is not None:
                            af = af_raw.as_accel_frame()
                            imu_msg.linear_acceleration.x = float(af.get_x())
                            imu_msg.linear_acceleration.y = float(af.get_y())
                            imu_msg.linear_acceleration.z = float(af.get_z())
                            imu_msg.linear_acceleration_covariance = [0.0] * 9
                            has_imu = True
                    except Exception:
                        pass

                if self.has_gyro:
                    try:
                        gf_raw = frameset.get_frame_by_type(ob.OBFrameType.GYRO_FRAME)
                        if gf_raw is not None:
                            gf = gf_raw.as_gyro_frame()
                            imu_msg.angular_velocity.x = float(gf.get_x())
                            imu_msg.angular_velocity.y = float(gf.get_y())
                            imu_msg.angular_velocity.z = float(gf.get_z())
                            imu_msg.angular_velocity_covariance = [0.0] * 9
                            has_imu = True
                    except Exception:
                        pass

                if has_imu:
                    self.safe_publish(self.pub_imu, imu_msg)

            # Periodic log
            if self.frame_count % 300 == 0:
                self.get_logger().info(f'Published {self.frame_count} framesets')

    def destroy_node(self):
        self.is_running = False
        if self.capture_thread is not None:
            self.capture_thread.join(timeout=2.0)
        # Avoid calling pipeline.stop() to prevent SDK deadlock on shutdown
        self.pipeline = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
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
